#!/usr/bin/env python
# import __future__
import rospy
import Queue
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from astar_nav.srv import *

import matplotlib.pyplot as plt
from matplotlib import colors
import math
import operator
from scipy import ndimage
import numpy as np
pi = math.pi

class pose():
    def __init__(self, x, y, theta, cost):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.succesorList = []
        
    def getSuccessors(self):
        # print self.succesorList
        del self.succesorList[:]
        processed_mapdata = getattr(astar, "processed_mapdata")
        map_height = getattr(astar,"map_height")
        map_width = getattr(astar,"map_width")
        self.x = int(self.x)
        self.y = int(self.y)
        # explore eight direction
        if processed_mapdata[self.y+1][self.x +1] ==0:
            r_cost = self.getRotateCost(pi/4)
            self.succesorList.append(pose(self.x+1, self.y+1, pi/4, self.cost + r_cost + math.sqrt(2) + self.getHeuristic(self.x+1,self.y+1)))
        if processed_mapdata[self.y][self.x +1] ==0:
            r_cost = self.getRotateCost(0)
            self.succesorList.append(pose(self.x+1, self.y, 0, self.cost+1 + r_cost + self.getHeuristic(self.x+1,self.y)))
        if processed_mapdata[self.y-1][self.x +1] ==0:
            r_cost = self.getRotateCost(-pi/4)
            self.succesorList.append(pose(self.x+1, self.y-1, -pi/4, self.cost + r_cost + math.sqrt(2) + self.getHeuristic(self.x+1,self.y-1)))
        if processed_mapdata[self.y+1][self.x-1] ==0:
            r_cost = self.getRotateCost(3*pi/4)
            self.succesorList.append(pose(self.x-1, self.y+1, 3*pi/4, self.cost + r_cost + math.sqrt(2) + self.getHeuristic(self.x-1,self.y+1)))
        if processed_mapdata[self.y][self.x-1] ==0:
            r_cost = self.getRotateCost(-pi)
            self.succesorList.append(pose(self.x-1, self.y, -pi, self.cost + r_cost +1 + self.getHeuristic(self.x-1,self.y)))
        if processed_mapdata[self.y-1][self.x-1] ==0:
            r_cost = self.getRotateCost(-3*pi/4)
            self.succesorList.append(pose(self.x-1, self.y-1, -3*pi/4, self.cost + r_cost + math.sqrt(2) + self.getHeuristic(self.x-1,self.y-1)))
            
        if processed_mapdata[self.y+1][self.x] ==0:
            r_cost = self.getRotateCost(pi/2)
            self.succesorList.append(pose(self.x, self.y+1, pi/2, self.cost + r_cost +1 + self.getHeuristic(self.x,self.y+1)))
        if processed_mapdata[self.y-1][self.x] ==0:
            r_cost = self.getRotateCost(-pi/2)
            self.succesorList.append(pose(self.x, self.y-1, -pi/2, self.cost + r_cost +1 + self.getHeuristic(self.x,self.y-1)))
        return self.succesorList

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

    def getRotateCost(self, goaltheta):
        delta_theta = self.theta_convert(goaltheta - self.theta)
        Rcost = 0.1*delta_theta/(pi/4)
        return 0

    def getHeuristic(self, nextpoint_x, nextpoint_y):
        goal = getattr(astar, "goal_pose")
        d_x = nextpoint_x-goal.x
        d_y = nextpoint_y-goal.y

        Mcost = abs(d_x) + abs(d_y)
        #Euclidean distance
        Ecost = math.sqrt(math.pow((d_x),2) + math.pow((d_y),2))
        #Chebyshev's distance
        Ccost = min(d_x, d_y) * math.sqrt(2) + (max(d_x, d_y) - min(d_x, d_y)) * 1
        return 0
    
class Astar():
    def __init__(self):
        self.mapsub = rospy.Subscriber("map", OccupancyGrid, self.mapCallback)
        self.server = rospy.Service('path', astar_controller, self.astarCallback)
        self.pathpub = rospy.Publisher('/path_rviz', Path, queue_size=10)
        # map param
        self.map_width = 0
        self.map_height = 0
        self.map_origin = []
        self.map_resolution = 0
        self.mapdata = []
        self.processed_mapdata = []
        self.inf_size = 2 #inflation_size
        # astar param
        self.init_pose = pose(500,500,0,0.0)
        self.goal_pose = pose(80,90,0,0.0)
        self.path = []
        self.current_target = pose(0,0,0,0.0)
        # Goalstate tolerance
        self.xy_tolerance = 0.1
        self.theta_tolerance = 0.15
        # output
        self.pathmsg = Path()

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

    def astarCallback(self,req):
        # goal request from client
        self.goal_pose.x, self.goal_pose.y = self.real2map([req.goal_pose.x, req.goal_pose.y])
        self.goal_pose.theta = self.theta_convert(req.goal_pose.theta)
        print("goal requested = "), [req.goal_pose.x, req.goal_pose.y, req.goal_pose.theta]
        
        # initial pose from client
        init_pose_x, init_pose_y = self.real2map([req.current_pose.x, req.current_pose.y])
        init_pose_theta = self.theta_convert(req.current_pose.theta)
        self.init_pose = pose(init_pose_x, init_pose_y, init_pose_theta, 0)
        # run astar
        self.astar(self.init_pose, [self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta])
        return astar_controllerResponse(self.pathmsg)

    def mapCallback (self, raw_map_data):
        # map parameter achieved
        self.map_width = raw_map_data.info.width
        self.map_height = raw_map_data.info.height
        self.map_origin = [raw_map_data.info.origin.position.x, raw_map_data.info.origin.position.y]
        self.map_resolution = raw_map_data.info.resolution

        self.mapdata = [[0]*self.map_width for i in range(self.map_height)]
        self.processed_mapdata = [[0]*self.map_width for i in range(self.map_height)]
        raw_map_queue = Queue.Queue()

        for i in raw_map_data.data:
            raw_map_queue.put(i)

        for i in range(self.map_height):
            for j in range(self.map_width):
                self.mapdata[i][j] = raw_map_queue.get()
        print("Map data achieved !!")
        self.mapdata = np.array(self.mapdata)    
        # map visualization
        # cmap = colors.ListedColormap(['lavender','midnightblue'])
        # plt.imshow(self.mapdata, cmap = cmap, origin = "lower")
        # plt.show()
        self.obstacle_inflation()

    def real2map(self, realcoord):
        # convert world coordinate to map coordinate
        # realcoord = [x, y]
        realx, realy = realcoord
        orgx, orgy = self.map_origin[0], self.map_origin[1]
        mapx = int((realx - orgx)/self.map_resolution)
        mapy = int((realy - orgy)/self.map_resolution)
        mapcoord = [mapx, mapy]
        return mapcoord

    def map2real(self, mapcoord):
        # convert map coordinate to world coordinate
        # mapcoord = [x, y]
        mapx, mapy = mapcoord
        orgx, orgy = self.map_origin[0], self.map_origin[1]
        realx = mapx * self.map_resolution + orgx
        realy = mapy * self.map_resolution + orgy
        realcoord = [realx, realy]
        return realcoord

    def obstacle_inflation(self):
        # occupy 100
        # empty 0
        # unknown -1        
        structure1 = ndimage.generate_binary_structure(2,2)
        self.processed_mapdata = ndimage.binary_dilation(self.mapdata, structure=structure1, iterations=20).astype(self.mapdata.dtype)
        # print self.processed_mapdata
        print("Map data inflated !!")

        # req = request()
        # req.current_pose.x = 0.175
        # req.current_pose.y = 0.175
        # req.current_pose.theta = pi/2
        # req.goal_pose.x = 1.75
        # req.goal_pose.y = 1
        # req.goal_pose.theta = pi/2
        # self.astarCallback(req)

        # map visualization
        # cmap = colors.ListedColormap(['lavender','midnightblue'])
        # plt.imshow(self.processed_mapdata, cmap = cmap, origin = "lower")
        # plt.show()

    def astar(self,init,goal):
        queue = []
        del queue[:]
        del self.path[:]
        visited = {}
        visited.clear()
        parent = {}
        parent.clear()
        astar_traversal_output = []
        self.path = []
        queue.append(init)
        parent[(init.x, init.y, init.theta)] = None
        visited[(init.x, init.y)] = True

        while queue:
            queue.sort(key=operator.attrgetter('cost'), reverse=True)
            u = queue.pop()
            if [u.x, u.y] == [goal[0],goal[1]]:
                while u is not None:
                    self.path.append([u.x, u.y, u.theta])
                    if (u.x, u.y, u.theta) == (init.x, init.y, init.theta):
                        break
                    (u.x, u.y, u.theta) = parent[(u.x, u.y, u.theta)]
                print("goal reached")
                self.path.reverse()
                break
            v = u.getSuccessors()
            for i in v:   
                if not visited.get((i.x, i.y), False):
                    visited[(i.x, i.y)] = True
                    parent[(i.x, i.y, i.theta)] = (u.x, u.y, u.theta)
                    queue.append(i)

        print "(init.x, init.y, init.theta)", (init.x, init.y, init.theta)
        print "(goal.x, goal.y, goal.theta)", (goal[0],goal[1],goal[2])
        # print self.path
        self.path[-1][2] = goal[2]
        # self.path = self.path_process(self.path)
        self.rviz_pathshow(self.path)

        ###########view path#############
        # pathMap = self.processed_mapdata
        # for point in self.path:
        #     x = point[0]
        #     y = point[1]
        #     pathMap[y][x] = 2
        # cmap = colors.ListedColormap(['lavender','midnightblue','red'])
        # plt.imshow(pathMap, cmap = cmap, origin = "lower")
        # plt.show()
        #################################

        # skipped path
        # self.path = self.path_process(self.path)
        # print ("skipped path = "), self.path

    def path_process(self, path):
        # delete the point in the straight line
        path_skipped = []
        now_theta = 100
        prev_goal = path[0]
        count = 0
        for goal in path:
            if goal[2] != now_theta:
                if count != 0:
                    path_skipped.append(prev_goal)
                    path_skipped.append(goal)
            prev_goal = goal
            now_theta = goal[2]
            count += 1
        # path_skipped.append(path[-1])
        return path_skipped
            
    def rviz_pathshow(self, astar_output):
        # visualize astar
        del self.pathmsg.poses[:]
        self.pathmsg.header.frame_id = "map"
        self.pathmsg.header.stamp = rospy.Time.now()
        for i in astar_output:
            x, y = self.map2real([i[0], i[1]])
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            quaternion = quaternion_from_euler(0, 0, i[2])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            self.pathmsg.poses.append(pose)
        self.pathpub.publish(self.pathmsg)

class request():
    def __init__(self):
        self.goal_pose = Pose2D()
        self.current_pose = Pose2D()

if __name__ == '__main__':
    rospy.init_node('astar', anonymous = True)
    astar = Astar()
    rospy.spin()