#!/usr/bin/env python
# import __future__
import rospy
import math
from astar_nav.srv import *
import tf
from tf.transformations import euler_from_quaternion, rotation_from_matrix, rotation_matrix
from tf.transformations import quaternion_from_euler
from tf import TransformListener
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from itertools import chain
import numpy as np
import csv
pi = math.pi

class pose():
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other): 
            if not isinstance(other, pose):
                # don't attempt to compare against unrelated types
                return NotImplemented

            return self.x == other.x and self.y == other.y

    def print_pose(self):
        print "(x,y,theta) = ",(self.x, self.y, self.theta)

class pathTracker():
    def __init__(self):
        self.pose_sub = rospy.Subscriber("/global_filter", Odometry, self.poseCallback)
        # self.pose_sub = rospy.Subscriber("/global_filter", Odometry, self.poseCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
        self.rviz_point = rospy.Publisher('/rolling_window', PoseWithCovarianceStamped, queue_size=10)
        self.posearray_pub = rospy.Publisher('path_theta', PoseArray, queue_size=10)
        self.main_feedback_pub = rospy.Publisher('/plan_state', Int32MultiArray, queue_size=10)
        self.tf_listener = tf.TransformListener()

        # pos feedback from localization
        self.curPos = pose(0,0,0)
        self.curPos_quaternion = []
        self.goalPos = pose(0,0,0)
        self.path = Path()
        self.globalPath = []
        self.xy_tolerance = 0.025
        self.theta_tolerance = 0.05
        self.linear_velocity = 0.3
        self.angular_velocity = 0.3
        self.last_localgoal = pose(0,0,0)
        self.k_theta = 0.3
        self.rotate_direction = 1
        self.rotate_saturation = 0.4
        # pure pursuit
        self.d_lookahead = 0.8
        self.d_lookahead_const = 0.5
        self.d_lookahead_K = 0.01
        self.d_lookahead_max = 1
        self.init_goal = pose(0,0,0)
        self.localgoal = pose(0,0,0)
        self.startTime = 0
        self.d_t = 0.1

    def start(self):
        while 1:
            print("waiting for tf transform !")
            if self.tf_listener.canTransform('base_footprint', 'map', rospy.Time()):
                break
        # find the closest point on global path, set as first local goal
        min = 1000000000000000
        for i in self.globalPath:
            d = self.distance(self.curPos, i)
            if d < min:
                min = d
                self.init_goal = i
        self.localgoal = self.init_goal
        self.controller()

    def poseCallback(self, data):
        self.curPos.x = data.pose.pose.position.x
        self.curPos.y = data.pose.pose.position.y
        self.curPos_quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(self.curPos_quaternion)
        self.curPos.theta = self.theta_convert(yaw)

    def goalCallback(self, goalmsg):
        del self.globalPath[:]
        quaternion = [goalmsg.pose.orientation.x, goalmsg.pose.orientation.y,
                      goalmsg.pose.orientation.z, goalmsg.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = self.theta_convert(yaw)
        self.goalPos = pose(goalmsg.pose.position.x, goalmsg.pose.position.y, yaw)
        print("goal received ! Heading toward "), round(goalmsg.pose.position.x, 4), round(goalmsg.pose.position.y, 4), math.degrees(round((yaw), 4))
        globalPath = self.path_client(self.curPos, self.goalPos)

        for i in globalPath.path.poses:
            (roll, pitch, yaw) = euler_from_quaternion([i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w])
            self.globalPath.append(pose(i.pose.position.x, i.pose.position.y, yaw))

        self.path_theta_distribution(self.globalPath)
        self.start()

    def path_theta_distribution(self, path):
        # path.reverse()
        init_theta = self.curPos.theta
        goal_theta = self.goalPos.theta
        # print ("init theta", init_theta)
        # print ("goal theta", goal_theta)
        init_vec_x = math.cos(init_theta)
        init_vec_y = math.sin(init_theta)
        goal_vec_x = math.cos(goal_theta)
        goal_vec_y = math.sin(goal_theta)
        # rotate vector
        tf_angle = self.theta_convert(init_theta - pi/2)
        tf_goal_vec_x = goal_vec_x * math.cos(-tf_angle) + goal_vec_y * -math.sin(-tf_angle)
        tf_goal_vec_y = goal_vec_x * math.sin(-tf_angle) + goal_vec_y * math.cos(-tf_angle)
        if tf_goal_vec_x > 0:
            self.rotate_direction = -1
        else:
            self.rotate_direction = 1

        theta_err = math.acos(init_vec_x * goal_vec_x + init_vec_y * goal_vec_y)
        delta_theta = self.rotate_direction * theta_err/(len(path)-1)
        # print ("theta_err"), theta_err
        # print ("delta theta"), delta_theta
        # print ("len path"), len(path)
        for i in range(len(path)):
            # if i == 0:
                # print("path",i), path[i].theta
            if i != 0:
                path[i].theta = self.theta_convert(path[i-1].theta + delta_theta)
                # print("path", i), path[i].theta
        
        pose_arr = PoseArray()
        pose_arr.header.frame_id = "map"
        pose_arr.header.stamp = rospy.get_rostime()
        poses = []
        for i in path:
            pose = Pose()
            pose.position.x = i.x
            pose.position.y = i.y
            pose.position.z = 0
            quaternion = quaternion_from_euler(0, 0, self.theta_convert(i.theta))
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            poses.append(pose)
        pose_arr.poses = poses
        self.posearray_pub.publish(pose_arr)

    def path_client(self, curPos, goalPos):
        cur_pos = Pose2D()
        goal_pos = Pose2D()
        cur_pos.x = curPos.x
        cur_pos.y = curPos.y
        cur_pos.theta = curPos.theta
        goal_pos.x = goalPos.x
        goal_pos.y = goalPos.y
        goal_pos.theta = goalPos.theta
        rospy.wait_for_service('path')
        try:
            path = rospy.ServiceProxy('path', astar_controller)
            respond_path = path(cur_pos, goal_pos)
            return respond_path
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def theta_convert(self, input):
        # convert rad domain to [-pi, pi]
        if input >=0:
            input = math.fmod(input, 2*pi)
            if input > pi:
                input -= 2*pi
                output = input
            else:
                output = input
        else:
            input *= -1
            input = math.fmod(input, 2*pi)
            if input > pi:
                input -= 2*pi
                output = input*-1
            else:
                output = input*-1
        return output

    def find_localgoal(self, cur_center, R, globalPath):
        # find the localgoal in interval [a, b], then interpolate
        # k = 1
        # lastk = 0
        # a = pose(0,0,0)
        # b = pose(1000,1000,0)
        # count = 0
        # for i in globalPath:
        #     if count == 1:
        #         lastk = 0 
        #     lastk = k
        #     d = self.distance(i, cur_center)
        #     if d >= R:
        #         k = 1
        #     else:
        #         k = 0

        #     deltak = k - lastk
        #     if deltak == 1:
        #         b = i
        #         break
        #     count +=1
        
        # # while circle has no intersection with path, choose the closest point as b
        # if b not in globalPath:
        #     min = 1000000000000000
        #     for i in globalPath:
        #         d = self.distance(cur_center, i)
        #         if d < min:
        #             min = d
        #             b = i

        # a = globalPath[globalPath.index(b)-1]
        # dis = [self.distance(a, cur_center), self.distance(b, cur_center)]
        # x = [a.x, b.x]
        # y = [a.y, b.y]
        # localgoal_x = np.interp(R, dis, x)
        # localgoal_y = np.interp(R, dis, y)    
        # localgoal = pose(localgoal_x, localgoal_y, a.theta)

        min = 1000000000000000
        if self.last_localgoal not in globalPath:
            self.last_localgoal = globalPath[0]

        for i in range(globalPath.index(self.last_localgoal), len(globalPath)):
            dist = self.distance(globalPath[i], cur_center)
            if dist >= R:
                min_idx = i
                min_dist = dist
                break
        localgoal = self.globalPath[min_idx]
        self.last_localgoal = localgoal
        # print("local goal")
        # localgoal.print_pose()
        self.point_publish(localgoal)
        # target = (self.path[min_idx, 0], self.path[min_idx, 1])
        return localgoal

    def distance(self, curPos, goalPos):
        dis = math.sqrt(pow((curPos.x - goalPos.x),2) + pow((curPos.y - goalPos.y),2))
        return dis

    def xy_goalReached(self,curPos, goalPos, tol):
        if self.distance(curPos, goalPos) < tol:
            return True
        else:
            return False

    def theta_error(self,theta1, theta2):
        curPos_vx = math.cos(theta1)
        curPos_vy = math.sin(theta1)
        goalPos_vx = math.cos(theta2)
        goalPos_vy = math.sin(theta2)
        theta_err = math.acos(curPos_vx * goalPos_vx + curPos_vy * goalPos_vy)
        return theta_err

    def theta_goalReached(self,curPos, goalPos):   
        curPos_vx = math.cos(curPos.theta)
        curPos_vy = math.sin(curPos.theta)
        goalPos_vx = math.cos(goalPos.theta)
        goalPos_vy = math.sin(goalPos.theta)
        theta_err = math.acos(curPos_vx * goalPos_vx + curPos_vy * goalPos_vy)
        if abs(theta_err) < self.theta_tolerance:
            return True
        else:
            return False

    def controller(self):
        curGoal = self.localgoal
        rate = rospy.Rate(50)
        reached_target_range = 0
        xy_goalreached = 0
        theta_goalreached = 0
        main_count = 0
        main_feedback = Int32MultiArray()

        while (not xy_goalreached or not theta_goalreached) and not rospy.is_shutdown():            
            print("current subgoal :"), (curGoal.x, curGoal.y, curGoal.theta)
            if self.distance(self.goalPos, self.curPos) < self.d_lookahead + 0.05 or reached_target_range == 1:
                curGoal = self.goalPos
                self.linear_velocity *= 0.5
                reached_target_range = 1
            else:
                curGoal = self.find_localgoal(self.curPos, self.d_lookahead, self.globalPath)

            # transform local goal to base frame
            curGoal_base_x = math.cos(-self.curPos.theta)*(curGoal.x-self.curPos.x) - math.sin(-self.curPos.theta)*(curGoal.y-self.curPos.y)
            curGoal_base_y = math.sin(-self.curPos.theta)*(curGoal.x-self.curPos.x) + math.cos(-self.curPos.theta)*(curGoal.y-self.curPos.y)
            direct_angle = math.atan2(curGoal_base_y, curGoal_base_x)
            vel_x = self.linear_velocity * math.cos(direct_angle)
            vel_y = self.linear_velocity * math.sin(direct_angle)
            theta_err = self.theta_error(self.curPos.theta, curGoal.theta)
            vel_w = self.rotate_direction * self.k_theta * theta_err
            
            if self.xy_goalReached(self.curPos, self.goalPos, self.xy_tolerance):
                print("xy_goalReached")
                xy_goalreached = 1
            if self.theta_goalReached(self.curPos, self.goalPos):
                print("theta_goalReached")
                theta_goalreached = 1
            if xy_goalreached:
                vel_x = 0
                vel_y = 0
            if theta_goalreached:
                vel_w = 0

            self.vel_publish(vel_x, vel_y, vel_w)
            main_count +=1
            main_feedback.data = [0,0,main_count]
            self.main_feedback_pub.publish(main_feedback)
            rate.sleep()

        print("goal reached !!!!!!")
        xy_goalreached = 1
        self.vel_publish(0, 0, 0)
        main_count +=1
        main_feedback.data = [1,0,main_count]
        self.main_feedback_pub.publish(main_feedback)

    def vel_publish(self, vx, vy, w):
        if w > 0.5:
            w = 0.5
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        if w > self.rotate_saturation:
            w = self.rotate_saturation
        msg.angular.z = w
        # print("current v, w ="), v, w
        self.vel_pub.publish(msg)

    def point_publish(self, point):
        pos_msg = PoseWithCovarianceStamped()
        pos_msg.header.frame_id = "map"
        pos_msg.header.stamp = rospy.get_rostime()
        pos_msg.pose.pose.position.x = point.x
        pos_msg.pose.pose.position.y = point.y
        pos_msg.pose.pose.position.z = 0
        q = quaternion_from_euler(0,0,point.theta)
        pos_msg.pose.pose.orientation.x = q[0]
        pos_msg.pose.pose.orientation.y = q[1]
        pos_msg.pose.pose.orientation.z = q[2]
        pos_msg.pose.pose.orientation.w = q[3]
        pos_msg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
        self.rviz_point.publish(pos_msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous = True)
    rate = rospy.Rate(10)
    path_tracker = pathTracker()
    # path_tracker.start()
    rospy.spin()
