#!/usr/bin/env python
# import __future__
import rospy
import math
from astar_nav.srv import *
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
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
        self.pose_sub = rospy.Subscriber("/odom", Odometry, self.poseCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
        self.tf_listener = tf.TransformListener()

        # pos feedback from localization
        self.curPos = pose(0,0,0)
        self.goalPos = pose(0,0,0)
        self.path = Path()
        self.globalPath = []
        self.xy_tolerance = 0
        self.sub_xy_tolerance = 0.05
        self.theta_tolerance = 0.1
        self.linear_velocity = 1
        self.max_vel = 1
        self.min_vel = 0.6
        self.angular_velocity = 0

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
            if self.tf_listener.canTransform('base_footprint', 'map', rospy.Time()):
                print("tf transform recieved !")
                break

        # find the closest point on global path, set as first local goal
        min = 1000000000000000
        for i in self.globalPath:
            d = self.distance(self.curPos, i)
            if d < min:
                min = d
                self.init_goal = i
        self.localgoal = self.init_goal
        # self.controller()

    def poseCallback(self, data):
        self.curPos.x = data.pose.pose.position.x
        self.curPos.y = data.pose.pose.position.y
        quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.curPos.theta = self.theta_convert(yaw)

    def goalCallback(self, goalmsg):
        quaternion = [goalmsg.pose.orientation.x, goalmsg.pose.orientation.y,
                      goalmsg.pose.orientation.z, goalmsg.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = self.theta_convert(yaw)
        self.goalPos = pose(goalmsg.pose.position.x, goalmsg.pose.position.y, yaw)
        print("goal received ! Heading toward "), (goalmsg.pose.position.x, goalmsg.pose.position.y, yaw)
        self.globalPath = self.path_client(self.curPos, self.goalPos)
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
        # print ("dis"), dis
        x = [a.x, b.x]
        y = [a.y, b.y]
        localgoal_x = np.interp(R, dis, x)
        localgoal_y = np.interp(R, dis, y)    
        localgoal = pose(localgoal_x, localgoal_y, a.theta)
        # self.show_pose(a)
        # self.show_pose(b)
        return localgoal

    def distance(self, curPos, goalPos):
        dis = math.sqrt(pow((curPos.x - goalPos.x),2) + pow((curPos.y - goalPos.y),2))
        return dis

    def xy_goalReached(self,curPos, goalPos, tol):
        if self.distance(curPos, goalPos) < tol:
            return True
        else:
            return False

    def theta_goalReached(self,curPos, goalPos):
        if abs(self.theta_convert(goalPos.theta - curPos.theta)) < self.theta_tolerance:
            return True
        else:
            return False

    def controller(self):
        curGoal = self.localgoal
        print("current subgoal :"), [curGoal.x, curGoal.y]
        self.startTime = rospy.Time.now().to_sec()
        while not self.xy_goalReached(self.curPos, self.goalPos, self.xy_tolerance) and not rospy.is_shutdown():
            precurGoal = curGoal
            prev_vel = self.linear_velocity
            
            self.linear_velocity = 0.2 #0.5
            self.d_lookahead = 0.6# 0.6
            
            print("---------------")
            print ("curpos :"), (self.curPos.x, self.curPos.y, self.curPos.theta)
            curGoal = self.find_localgoal(self.curPos, self.d_lookahead, self.globalPath)

            # transform local goal to base frame
            curGoal_base_y = -math.sin(self.curPos.theta)*(curGoal.x-self.curPos.x) + math.cos(self.curPos.theta)*(curGoal.y-self.curPos.y)

            # self.tf_listener.waitForTransform('base_footprint', ps.header.frame_id, ps.header.stamp, rospy.Duration(3.0))
            # curGoal_base = self.tf_listener.transformPose('base_footprint', ps)
            # curGoal_base = pose(curGoal_base.pose.position.x, curGoal_base.pose.position.y, 0)
            
            L = self.distance(self.curPos, curGoal)
            # R = pow(L, 2)/2/curGoal_base.y
            R = pow(L, 2)/2/curGoal_base_y

            self.angular_velocity = self.linear_velocity/R
            self.velOutput(self.linear_velocity, self.angular_velocity)
            # print("curGoal_base :"), [curGoal_base.x, curGoal_base.y]
            print("current subgoal :"), [curGoal.x, curGoal.y]
            print("v :"), self.linear_velocity
            print("w :"), self.angular_velocity
        self.velOutput(0, 0)

    def velOutput(self, v, w):
        if w > 0.5:
            w = 0.5
        msg = Twist()
        msg.linear.x = v
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = w
        # print("current v, w ="), v, w
        self.vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous = True)
    rate = rospy.Rate(10)
    path_tracker = pathTracker()
    # path_tracker.start()
    rospy.spin()
