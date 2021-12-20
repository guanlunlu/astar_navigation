/**
 * @file 
 * @author Louis (lewis8908@gmail.com)
 * @brief a cpp astar for eurobot
 * @version 0.1
 * @date 2021-12-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __astar_H
#define __astar_H

#include "bits/stdc++.h"
#include "math.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "stdio.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include "time.h"
#include "tracking_pose/ask_path.h"
#include "tracking_pose/astar_controller.h"

using namespace std;

#define endl "\n"
// #define mapX 640
// #define mapY 608
#define poseMargin 0.03

/**
 * @brief check if target arrive
 *
 * @param nowX
 * @param nowY
 * @param goalX
 * @param goalY
 * @return true
 * @return false
 */
inline bool isGoal(double nowX, double nowY, double goalX, double goalY)
{
	if (fabs(goalX - nowX) < poseMargin && fabs(goalY - nowY) < poseMargin)
		return true;
	else
		return false;
}

/**
 * @brief a heuristic function for astar
 *
 * @param pose_x
 * @param pose_y
 * @param goalX
 * @param goalY
 * @return double
 */
inline double ChebyshevHeuristic(double pose_x, double pose_y, double goalX, double goalY)
{
	double dx, dy;
	dx = goalX - pose_x;
	dy = goalY - pose_y;
	dx = fabs(dx);
	dy = fabs(dy);
	//(x,y) are in Cartesian coordinate

	// return max(dx, dy);
	return 1.414 * (min(dx, dy)) + max(dx, dy) - min(dx, dy); // this is octile not Chebyshev
															  // return dx + dy; //this is Manhattan
															  // return pow(dx,2) + pow(dy,2);    //this is Euclidean
}

/**
 * @brief variable for saving map information
 *
 */
struct MapInformation
{
	double x, y;
	bool obstacleExist;
};

/**
 * @brief
 *
 */
class Node
{
	ros::Subscriber map_server;
	ros::Subscriber map_init;
	ros::ServiceServer aStarServer;

	FILE *file, *file2;

public:
	int xIndexAtMap, yIndexAtMap;
	ros::NodeHandle nh;
	bool findIndexForMap;
	double x_init, y_init, theta;
	double costEstimatedToGoal;
	double costToThisNode;
	double heuristicCost;
	vector<int> actionToThisNode;
	vector<pair<int, int>> path;

	// static members
	static bool mapBuilt;
	static bool mapInit;
	static int expandCount;
	static double mapOriginX, mapOriginY;
	static double mapRes;
	static uint32_t mapX;
	static uint32_t mapY;
	static list<pair<int, int>> direction;
	// constexpr static double distanceBetweenNode = 0.05;
	static int nodeVisitedAmount;
	static struct MapInformation **mapInformation;

	/**
	 * @brief Construct a new Node object
	 *
	 * @param heuristic
	 */
	Node(double heuristic);

	/**
	 * @brief Construct a new Node object
	 *
	 */
	Node();

	/**
	 * @brief Construct a new Node object
	 *
	 * @param x
	 * @param y
	 */
	Node(double x, double y);

	/**
	 * @brief
	 *
	 * @param nowX
	 * @param nowY
	 */
	void findIndex(double nowX, double nowY);

	/**
	 * @brief Construct a new Node object
	 *
	 * @param x
	 * @param y
	 * @param directionToHere
	 * @param pathCost   cost accumulated from start position
	 * @param heuristic  heuristic cost to goal
	 * @param fromWhere  recodring path by point passing through
	 */
	Node(double x, double y, vector<int> directionToHere, double pathCost, double heuristic, vector<pair<int, int>> fromWhere);

	void initMap(const nav_msgs::MapMetaData::ConstPtr &mapData);

	/**
	 * @brief Set the Map object
	 *
	 * @param map
	 */
	void setMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

	/**
	 * @brief
	 *
	 * @param nodeA
	 * @param nodeB
	 * @return true
	 * @return false
	 */
	static bool costComparator(const Node nodeA, const Node nodeB);

	/**
	 * @brief Get the Successor object
	 *
	 * @param nowNode
	 * @param goal_x
	 * @param goal_y
	 * @return vector<Node>
	 */
	static vector<Node> getSuccessor(Node nowNode, double goal_x, double goal_y);

	/**
	 * @brief showing the map and the result path to goal at terminal
	 *
	 * @param goal_x
	 * @param goal_y
	 */
	void showMap(double goal_x, double goal_y);

	/**
	 * @brief
	 *
	 * @param initX
	 * @param initY
	 * @param goalX
	 * @param goalY
	 * @return Node
	 */
	static Node findPath(double initX, double initY, double goalX, double goalY);

	/**
	 * @brief
	 *
	 * @param index
	 * @return vector<pair<double, double>>
	 */
	vector<pair<double, double>> indexToMap(vector<pair<int, int>> index);

	/**
	 * @brief
	 *
	 * @param req
	 * @param res
	 * @return true
	 * @return false
	 */
	bool answerPath(tracking_pose::astar_controller::Request &req, tracking_pose::astar_controller::Response &res);
};

#endif
