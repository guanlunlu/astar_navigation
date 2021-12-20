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

#include "astar.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_path");

    Node initNode;

    while (!Node::mapBuilt)
        ros::spinOnce();
    while (ros::ok())
        ros::spinOnce();
}
