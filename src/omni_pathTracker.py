#!/usr/bin/env python
# import __future__
import rospy
import math
from astar_nav.srv import *
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf import TransformListener
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from itertools import chain
import numpy as np
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
        print ("(x,y,theta) = "),(self.x, self.y, self.theta)

class pathTracker():
    def __init__(self):
        # self.pose_sub = rospy.Subscriber("/odom", Odometry, self.poseCallback)
        # self.pose_sub = rospy.Subscriber("/global_filter", Odometry, self.poseCallback)
        self.pose_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.poseCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
        self.rviz_localgoal = rospy.Publisher('/local_goal', PoseWithCovarianceStamped, queue_size=10)
        self.rviz_robotlocus = rospy.Publisher('/robot_locus', Path, queue_size=10)
        self.pathpub = rospy.Publisher('/path_visualize', Path, queue_size=10)
        self.tf_listener = tf.TransformListener()

        # Robot position feedback from localization
        self.curPos = pose(0,0,0)
        self.curPos_quaternion = []
        self.robot_locus = []

        # Goal request from Main and Path received from Astar node
        self.goalPos = pose(0,0,0)
        self.path = Path()
        self.globalPath = []
        self.last_localgoal = pose(0,0,0)

        # Path tracking parameter
        self.control_freqeuncy = 20

        # Rolling Window Parameter
        self.d_lookahead = 0.2

        # Linear Parameter
        self.k_p = 0.8
        self.max_linear_velocity = 0.5
        self.linear_acceleration = 0.3
        self.linear_brake_distance = 0.3
        self.xy_tolerance = 0.02

        # Angular Parameter
        self.max_angular_velocity = 0.4
        self.angular_acceleration = 0.3
        self.angular_brake_distance = 0.2
        self.k_theta = 1
        self.theta_tolerance = 0.03

    def start(self):
        while 1:
            print("waiting for tf transform ...")
            if self.tf_listener.canTransform('base_footprint', 'map', rospy.Time()):
                print("tf canTransform")
                break
        # find the closest point on global path, set as first local goal
        min = 1000000000000000
        init_local_goal = pose(0,0,0)
        for i in self.globalPath:
            d = self.distance(self.curPos, i)
            if d < min:
                min = d
                init_local_goal = i

        self.controller(init_local_goal)

    def poseCallback(self, data):
        # position data achieved from localization node
        self.curPos.x = data.pose.pose.position.x
        self.curPos.y = data.pose.pose.position.y
        self.curPos_quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(self.curPos_quaternion)
        self.curPos.theta = self.theta_convert(yaw)
        self.robot_locus.append(self.curPos)

    def goalCallback(self, goalmsg):
        del self.globalPath[:]
        del self.robot_locus[:]
        quaternion = [goalmsg.pose.orientation.x, goalmsg.pose.orientation.y,
                      goalmsg.pose.orientation.z, goalmsg.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = self.theta_convert(yaw)
        self.goalPos = pose(goalmsg.pose.position.x, goalmsg.pose.position.y, yaw)
        print("=================================================================")
        print("goal received ! Heading toward "), (round(goalmsg.pose.position.x, 4), round(goalmsg.pose.position.y, 4), round(math.degrees(yaw),4))
        globalPath = self.path_client(self.curPos, self.goalPos)
        for i in globalPath.path.poses:
            (roll, pitch, yaw) = euler_from_quaternion([i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w])
            self.globalPath.append(pose(i.pose.position.x, i.pose.position.y, yaw))
        self.rviz_pathshow(self.globalPath)
        print("Path received form Astar node !")
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
        # find the localgoal in interval [a, b], then interpolate
        k = 1
        lastk = 0
        a = pose(0,0,0)
        b = pose(1000,1000,0)
        count = 0
        for i in globalPath:
            if count == 1:
                lastk = 0 
            lastk = k
            d = self.distance(i, cur_center)
            if d >= R:
                k = 1
            else:
                k = 0

            deltak = k - lastk
            if deltak == 1:
                b = i
                break
            count +=1
        
        if b not in globalPath:
            min = 1000000000000000
            for i in globalPath:
                d = self.distance(cur_center, i)
                if d < min:
                    min = d
                    b = i

        a = globalPath[globalPath.index(b)-1]
        dis = [self.distance(a, cur_center), self.distance(b, cur_center)]
        x = [a.x, b.x]
        y = [a.y, b.y]
        localgoal_x = np.interp(R, dis, x)
        localgoal_y = np.interp(R, dis, y)    
        localgoal = pose(localgoal_x, localgoal_y, a.theta)

        # min = 1000000000000000
        # if self.last_localgoal not in globalPath:
        #     self.last_localgoal = globalPath[0]

        # for i in range(globalPath.index(self.last_localgoal), len(globalPath)):
        #     dist = self.distance(globalPath[i], cur_center)
        #     # dist = (x - self.path[i, 0])**2 + (y - self.path[i, 1])
        #     if dist >= R:
        #         min_idx = i
        #         min_dist = dist
        #         break
        # localgoal = self.globalPath[min_idx]
        # self.last_localgoal = localgoal
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
        theta_err = self.theta_error(curPos.theta, goalPos.theta)
        if abs(theta_err) < self.theta_tolerance:
            return True
        else:
            return False

    def velocity_profile(self, mode, curPos, goalPos, last_vel, max_vel, accel, control_freq, brake_distance):
        if mode == "linear":
            d_vel = accel / control_freq
            output_vel = last_vel + d_vel
            
            if self.distance(curPos, goalPos) < brake_distance:
                output_vel = self.distance(curPos, goalPos) * self.k_p

            if output_vel > max_vel:
                output_vel = max_vel

            return output_vel
        
        elif mode == "angular":
            d_vel = accel / control_freq
            output_vel = last_vel + d_vel

            if self.theta_error(curPos.theta, goalPos.theta) < brake_distance:
                output_vel = self.k_theta * self.theta_convert(self.goalPos.theta - self.curPos.theta)

            if abs(output_vel) > abs(max_vel):
                output_vel = np.sign(output_vel) * max_vel

            return output_vel

    def controller(self, init_localgoal):
        curGoal = init_localgoal
        rate = rospy.Rate(self.control_freqeuncy)
        reached_target_range = 0
        xy_goalreached = 0
        linear_vel = 0
        angular_vel = 0
        rotate_direction = 1
        print("Start Tracking Path")
        # -----------xy position tracking-----------
        while not self.xy_goalReached(self.curPos, self.goalPos, self.xy_tolerance) and not rospy.is_shutdown():
            # Get current local goal from rolling window method
            if self.distance(self.goalPos, self.curPos) < self.d_lookahead + 0.1 or reached_target_range == 1:
                # Set global goal as current local goal, if robot reached in the range of global goal once
                curGoal = self.goalPos
                reached_target_range = 1
            else:
                curGoal = self.find_localgoal(self.curPos, self.d_lookahead, self.globalPath)

            # Transform current goal coordinate from "map" to "base_footprint" frame.
            curGoal_base_x = math.cos(-self.curPos.theta)*(curGoal.x-self.curPos.x) - math.sin(-self.curPos.theta)*(curGoal.y-self.curPos.y)
            curGoal_base_y = math.sin(-self.curPos.theta)*(curGoal.x-self.curPos.x) + math.cos(-self.curPos.theta)*(curGoal.y-self.curPos.y)

            # Calculate velocity vector toward current local goal
            linear_vel = self.velocity_profile("linear", self.curPos, self.goalPos, linear_vel, self.max_linear_velocity, self.linear_acceleration, self.control_freqeuncy, self.linear_brake_distance)
            direct_angle = math.atan2(curGoal_base_y, curGoal_base_x)
            vel_x = linear_vel * math.cos(direct_angle)
            vel_y = linear_vel * math.sin(direct_angle)
            self.vel_publish(vel_x, vel_y, 0)
            self.robot_locus_show(self.robot_locus)
            rate.sleep()

        xy_goalreached = 1
        self.vel_publish(0, 0, 0)
        print("xy goal reached")
        print("Start tracking orientation")
        # -----------Calculate rotate direction-----------
        fake_const = 2
        fake_x = self.goalPos.x + fake_const * math.cos(self.goalPos.theta)
        fake_y = self.goalPos.y + fake_const * math.sin(self.goalPos.theta)
        fake_base_x = math.cos(-self.curPos.theta)*(fake_x-self.curPos.x) - math.sin(-self.curPos.theta)*(fake_y-self.curPos.y)
        fake_base_y = math.sin(-self.curPos.theta)*(fake_x-self.curPos.x) + math.cos(-self.curPos.theta)*(fake_y-self.curPos.y)
        if fake_base_y < 0:
            # turn right
            rotate_direction = -1
        else:
            rotate_direction = 1
        # print("rotate direction"), rotate_direction

        # ------Orientation tracking, while xy goal reached--------
        while xy_goalreached and not self.theta_goalReached(self.curPos, self.goalPos) and not rospy.is_shutdown():
            angular_vel = self.velocity_profile("angular", self.curPos, self.goalPos, angular_vel, self.max_angular_velocity, rotate_direction * self.angular_acceleration, self.control_freqeuncy, self.angular_brake_distance)
            self.vel_publish(0,0,angular_vel)
            rate.sleep()
        
        print("theta goal reached"), math.degrees(self.curPos.theta)
        print("Goal reached !!!!")
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
        self.rviz_localgoal.publish(pos_msg)

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

    def robot_locus_show(self, astar_output):
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
        self.rviz_robotlocus.publish(pathmsg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous = True)
    rate = rospy.Rate(10)
    path_tracker = pathTracker()
    # path_tracker.start()
    rospy.spin()
