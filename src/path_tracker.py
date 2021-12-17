#!/usr/bin/env python
# import __future__
from operator import xor
import rospy
import math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from astar_nav.srv import *
pi = math.pi

class pose():
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta


class pathTracker():
    def __init__(self):
        self.pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.curPos = pose(0,0,0)
    
    def poseCallback(self, data):
        self.curPos.x = data.pose.pose.position.x
        self.curPos.y = data.pose.pose.position.y
        # print("aaaaaaaaaaa")
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        cur_roll, cur_pitch, cur_yaw = euler_from_quaternion(quaternion)
        self.curPos.theta = cur_yaw
        # print("aaaaaaaaaaa")
        print ("(x, y, theta) = "), (self.curPos.x, self.curPos.y, self.curPos.theta)



if __name__ == '__main__':
    rospy.init_node('path_tracker', anonymous = True)
    path_tracker = pathTracker()
    rospy.spin()