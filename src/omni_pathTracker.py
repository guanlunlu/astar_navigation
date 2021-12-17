#!/usr/bin/env python
# import __future__
import rospy
import math
from astar_nav.srv import *
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf import TransformListener
from tf import TransformListener
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
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
        # self.pose_sub = rospy.Subscriber("/odom", Odometry, self.poseCallback)
        # self.pose_sub = rospy.Subscriber("/global_filter", Odometry, self.poseCallback)
        self.pose_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.poseCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
        self.rviz_point = rospy.Publisher('/rolling_window', PoseWithCovarianceStamped, queue_size=10)
        self.pathpub = rospy.Publisher('/path_visualize', Path, queue_size=10)
        self.tf_listener = tf.TransformListener()
        # pos feedback from localization
        self.curPos = pose(0,0,0)
        self.curPos_quaternion = []
        self.goalPos = pose(0,0,0)
        self.path = Path()
        self.globalPath = []
        self.last_localgoal = pose(0,0,0)

        # Path tracking parameter
        self.control_freqeuncy = 20
        # Linear Parameter
        self.k_p = 0.8
        self.d_lookahead = 0.2
        self.max_linear_velocity = 0.5
        self.xy_tolerance = 0.02
        # Angular Parameter
        self.max_angular_velocity = 0.4
        self.k_theta = 1
        self.theta_tolerance = 0.05
        self.init_goal = pose(0,0,0)
        self.localgoal = pose(0,0,0)
        self.startTime = 0
        self.d_t = 0.1
        self.mission_count =0

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
        self.rviz_pathshow(self.globalPath)
        self.start()

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
        min = 1000000000000000

        if self.last_localgoal not in globalPath:
            self.last_localgoal = globalPath[0]

        for i in range(globalPath.index(self.last_localgoal), len(globalPath)):
            dist = self.distance(globalPath[i], cur_center)
            # dist = (x - self.path[i, 0])**2 + (y - self.path[i, 1])
            if dist >= R:
                min_idx = i
                min_dist = dist
                break
        localgoal = self.globalPath[min_idx]
        self.last_localgoal = localgoal
        self.point_publish(localgoal)
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
        print("current subgoal :"), [curGoal.x, curGoal.y]
        # self.startTime = rospy.Time.now().to_sec()
        rate = rospy.Rate(self.control_freqeuncy)
        reached_target_range = 0
        xy_goalreached = 0

        while not self.xy_goalReached(self.curPos, self.goalPos, self.xy_tolerance) and not rospy.is_shutdown():            
            linear_vel = self.distance(self.curPos, self.goalPos) * self.k_p
            if linear_vel > self.max_linear_velocity:
                linear_vel = self.max_linear_velocity
            if self.distance(self.goalPos, self.curPos) < self.d_lookahead + 0.1 or reached_target_range == 1:
                curGoal = self.goalPos
                reached_target_range = 1
            else:
                curGoal = self.find_localgoal(self.curPos, self.d_lookahead, self.globalPath)
            curGoal_base_x = math.cos(-self.curPos.theta)*(curGoal.x-self.curPos.x) - math.sin(-self.curPos.theta)*(curGoal.y-self.curPos.y)
            curGoal_base_y = math.sin(-self.curPos.theta)*(curGoal.x-self.curPos.x) + math.cos(-self.curPos.theta)*(curGoal.y-self.curPos.y)
            direct_angle = math.atan2(curGoal_base_y, curGoal_base_x)
            vel_x = linear_vel * math.cos(direct_angle)
            vel_y = linear_vel * math.sin(direct_angle)
            self.vel_publish(vel_x, vel_y, 0)
            rate.sleep()
        print("xy goal reached")
        xy_goalreached = 1
        self.vel_publish(0, 0, 0)

        while xy_goalreached and not self.theta_goalReached(self.curPos, self.goalPos) and not rospy.is_shutdown():
            vel_w = self.k_theta * self.theta_convert(self.goalPos.theta - self.curPos.theta)
            if vel_w > self.max_angular_velocity:
                vel_w = self.max_angular_velocity
            # # print("w : "), angular_vel
            self.vel_publish(0,0,vel_w)
            rate.sleep()
        print("theta goal reached"), math.degrees(self.curPos.theta)
        self.vel_publish(0,0,0)

    def vel_publish(self, vx, vy, w):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
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

    def rviz_pathshow(self, astar_output):
        # visualize astar
        pathmsg = Path()
        pathmsg.header.frame_id = "map"
        pathmsg.header.stamp = rospy.Time.now()
        for i in astar_output:
            pose = PoseStamped()
            pose.pose.position.x = i.x
            pose.pose.position.y = i.y
            pose.pose.position.z = 0
            quaternion = quaternion_from_euler(0, 0, i.theta)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            pathmsg.poses.append(pose)
        self.pathpub.publish(pathmsg)
      
if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous = True)
    rate = rospy.Rate(10)
    path_tracker = pathTracker()
    # path_tracker.start()
    rospy.spin()
