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

/**
 * @brief Construct a new Node object
 *
 * @param heuristic save heuristic cost
 */
Node::Node(double heuristic)
{
    costEstimatedToGoal = heuristic;
}

/**
 * @brief Construct a new Node object
 *
 */
Node::Node()
{
    map_init = nh.subscribe("map_metadata", 1, &Node::initMap, this);
    map_server = nh.subscribe("map", 1, &Node::setMap, this);

    // aStarServer = nh.advertiseService("ask_path", &Node::answerPath, this);
    // aStarServer = nh.advertiseService("astar_controller", &Node::answerPath, this);
    aStarServer = nh.advertiseService("path", &Node::answerPath, this);
    for (int dx = 1; dx >= -1; dx--)
        for (int dy = 1; dy >= -1; dy--)
        {
            if (dx == 0 && dy == 0)
                continue;
            Node::direction.push_back(make_pair(dx, dy));
        }
}

/**
 * @brief Construct a new Node object
 *
 * @param x
 * @param y
 */
Node::Node(double x, double y)
{

    costToThisNode = 0;
    costEstimatedToGoal = 0;
    findIndexForMap = false;
    // actionToThisNode.reserve(mapX * mapY);
    // path.reserve(mapX * mapY);
    findIndex(x, y);
}

/**
 * @brief
 *
 * @param nowX
 * @param nowY
 */
void Node::findIndex(double nowX, double nowY)
{
    for (int y = 0; y < mapY; y++)
    {
        for (int x = 0; x < mapX; x++)
        {
            if (fabs(Node::mapInformation[x][y].x - nowX) < poseMargin && fabs(Node::mapInformation[x][y].y - nowY) < poseMargin)
            {
                xIndexAtMap = x;
                yIndexAtMap = y;
                findIndexForMap = true;
                break;
            }
        }
        if (findIndexForMap)
            break;
    }
}

/**
 * @brief Construct a new Node object
 *
 * @param x
 * @param y
 * @param directionToHere path length recorded from start position
 * @param pathCost   cost accumulated from start position
 * @param heuristic  heuristic cost to goal
 * @param fromWhere  recodring path by point passing through
 */
Node::Node(double x, double y, vector<int> directionToHere, double pathCost, double heuristic, vector<pair<int, int>> fromWhere)
{
    xIndexAtMap = x;
    yIndexAtMap = y;
    actionToThisNode = directionToHere;
    costToThisNode = pathCost;
    costEstimatedToGoal = costToThisNode + heuristic;
    path = fromWhere;
}

/**
 * @brief  get map metaData for preceding map processing
 *
 * @param mapData  a ros topic published by map server, initialize map size\resolution
 */
void Node::initMap(const nav_msgs::MapMetaData::ConstPtr &mapData)
{
    Node::mapOriginX = mapData->origin.position.x;
    Node::mapOriginY = mapData->origin.position.y;
    Node::mapRes = mapData->resolution;
    Node::mapX = mapData->width;
    Node::mapY = mapData->height;

    // cout << "start init map \n";
    Node::mapInformation = (struct MapInformation **)malloc(sizeof(struct MapInformation *) * Node::mapX);
    for (int i = 0; i < Node::mapX; i++)
    {
        Node::mapInformation[i] = (struct MapInformation *)malloc(sizeof(struct MapInformation) * Node::mapY);
    }

    Node::mapInit = 1;
    // cout << "end init map\n";
}

/**
 * @brief Set the Map object
 *
 * @param map a ros topic published by map server
 */
void Node::setMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    if (!Node::mapBuilt && Node::mapInit)
    {

        // //cout << "start building map\n";
        // file2 = fopen("/home/louis/Desktop/map.csv", "w+");
        // file = fopen("/home/louis/Desktop/obstacle.csv", "w+");
        for (int y = 0; y < this->mapY; y++)
            for (int x = 0; x < this->mapX; x++)
            {
                // double x_position = this->originX + x * Node::distanceBetweenNode;
                // double y_position = this->originY + y * Node::distanceBetweenNode;
                double x_position = Node::mapOriginX + x * this->mapRes;
                double y_position = Node::mapOriginY + y * this->mapRes;
                Node::mapInformation[x][y].x = x_position;
                Node::mapInformation[x][y].y = y_position;
                Node::mapInformation[x][y].obstacleExist = map->data[this->mapX * y + x];
            }
        for (int y = 0; y < this->mapY; y++)
            for (int x = 0; x < this->mapX; x++)
                if (map->data[this->mapX * y + x] == 100 || map->data[this->mapX * y + x] == -1)
                    for (int dx = 3; dx >= -3; dx--)
                        for (int dy = 3; dy >= -3; dy--)
                        // inflat 3 - (-3) times
                        {
                            int tempX = x - dx;
                            int tempY = y - dy;
                            if ((tempX >= 0 && tempX <= this->mapX - 1) && (tempY >= 0 && tempY <= this->mapY - 1))
                                Node::mapInformation[tempX][tempY].obstacleExist = 100;
                        }
        for (int y = this->mapY - 1; y >= 0; y--)
        {
            for (int x = 0; x < this->mapX; x++)
            {
                // fprintf(file, "%d,", Node::mapInformation[x][y].obstacleExist);
                // fprintf(file2, "%lf %lf,", Node::mapInformation[x][y].x, Node::mapInformation[x][y].y);
            }
            // fprintf(file, "\n");
            // fprintf(file2, "\n");
        }
        Node::mapBuilt = 1;
        // cout << "end building map! \n";
    }
}

/**
 * @brief cost function for priority queue, comparing two nodes by their costEstimatedToGoal
 *
 * @param nodeA
 * @param nodeB
 * @return true
 * @return false
 */
bool Node::costComparator(const Node nodeA, const Node nodeB)
{
    return nodeA.costEstimatedToGoal > nodeB.costEstimatedToGoal;
}

/**
 * @brief Get the possible Successor from a specific node
 *
 * @param nowNode
 * @param goal_x target tracking
 * @param goal_y target tracking
 * @return vector<Node> which is a list of possible Successor containg pose、 orientation and history path
 */
vector<Node> Node::getSuccessor(Node nowNode, double goal_x, double goal_y)
{
    int nextX;
    int nextY;
    int which_direction = 1;
    double cost;
    vector<Node> result;
    result.reserve(8);
    double heuristicCost;
    for (auto it = Node::direction.begin(); it != Node::direction.end(); it++)
    {
        nextX = nowNode.xIndexAtMap + it->first;
        nextY = nowNode.yIndexAtMap + it->second;
        if ((nextX < Node::mapX && nextX >= 0) && (nextY < Node::mapY && nextY >= 0))
        {
            if (!Node::mapInformation[nextX][nextY].obstacleExist)
            {
                Node::expandCount++;
                cost = pow(pow(it->first * Node::mapRes, 2) + pow(it->second * Node::mapRes, 2), 0.5);
                heuristicCost = ChebyshevHeuristic(Node::mapInformation[nextX][nextY].x,
                                                   Node::mapInformation[nextX][nextY].y, goal_x, goal_y);
                vector<int> act = nowNode.actionToThisNode;
                if (!act.empty())
                {
                    if (*(act.end() - 1) != which_direction)
                        //     //heuristicCost += (45 / 360) * 2 * M_PI / 10;
                        heuristicCost *= 1.5;
                }
                vector<pair<int, int>> fromWhere = nowNode.path;
                fromWhere.push_back(make_pair(nextX, nextY));
                // this will tell us the index for mapMatrix
                act.push_back(which_direction);
                Node *nodePtr = new Node(nextX, nextY, act, nowNode.costToThisNode + cost, heuristicCost, fromWhere);
                result.push_back(*nodePtr);
            }
        }
        which_direction++;
    }
    return result;
}

/**
 * @brief showing the map and the result path to goal at terminal
 *
 * @param goal_x
 * @param goal_y
 */
void Node::showMap(double goal_x, double goal_y)
{
    for (int y = mapY - 50; y >= 50; y--)
        for (int x = 50; x < mapX - 50; x++)
        {
            pair<int, int> printWhere = make_pair(x, y);
            auto temp = find(path.begin(), path.end(), printWhere);
            if (temp == path.end())
            {
                if (Node::mapInformation[x][y].obstacleExist)
                    printf("x");
                else
                    printf(".");
            }
            else
            {
                if (!isGoal(Node::mapInformation[x][y].x, Node::mapInformation[x][y].y, goal_x, goal_y))
                    printf("*");
                else
                    printf("!");
            }
        }
    printf("\n");
}

/**
 * @brief the astar itselt with a starter point and target point
 *
 * @param initX
 * @param initY
 * @param goalX
 * @param goalY
 * @return Node, the final target point with history path, which is the target path
 */
Node Node::findPath(double initX, double initY, double goalX, double goalY)
{
    double pq_time_accumulate = 0;
    double succ_time_accumulate = 0;
    vector<Node> container;
    container.reserve(mapX * mapY);
    priority_queue<Node, vector<Node>, decltype(&Node::costComparator)> pq(Node::costComparator, std::move(container));
    bool visited[mapX][mapY];
    memset(visited, 0, mapX * mapY);
    Node startNode(initX, initY);
    // clock_t t0 = clock();
    pq.push(startNode);
    // clock_t t1 = clock();
    // pq_time_accumulate += (double)(t1 - t0);
    clock_t start = clock();
    while (!pq.empty())
    {
        Node nowNode = pq.top();
        // t0 = clock();
        pq.pop();
        // t1 = clock();
        // printf("pop time = %lf\n", (t1 - t0) * 1000.0 / CLOCKS_PER_SEC);
        // pq_time_accumulate += (double)(t1 - t0);
        if (!isGoal(Node::mapInformation[nowNode.xIndexAtMap][nowNode.yIndexAtMap].x, Node::mapInformation[nowNode.xIndexAtMap][nowNode.yIndexAtMap].y, goalX, goalY))
        {
            if (!visited[nowNode.xIndexAtMap][nowNode.yIndexAtMap])
            {
                Node::nodeVisitedAmount++;
                visited[nowNode.xIndexAtMap][nowNode.yIndexAtMap] = true;
                // t0 = clock();
                auto successors = getSuccessor(nowNode, goalX, goalY);
                // t1 = clock();
                // succ_time_accumulate += (double)(t1 - t0);
                for (auto it : successors)
                    if (!visited[it.xIndexAtMap][it.yIndexAtMap])
                    {
                        // t0 = clock();
                        pq.push(it);
                        // t1 = clock();
                        // pq_time_accumulate += (double)(t1 - t0);
                    }
            }
        }
        else
        {
            nowNode.path.push_back(make_pair(nowNode.xIndexAtMap, nowNode.yIndexAtMap));
            clock_t end = clock();

            cout << "time cost = " << (double)(end - start)/ CLOCKS_PER_SEC * 1000.0  << "ms. "<< endl;
            Node::expandCount = 0;
            Node::nodeVisitedAmount = 0;
            int count = 0;

            return nowNode;
        }
    }
}

/**
 * @brief change path array from index to map in Cartesian coordinate
 *
 * @param index
 * @return vector<pair<double, double>>
 */
vector<pair<double, double>> Node::indexToMap(vector<pair<int, int>> index)
{
    vector<pair<double, double>> toMap;
    toMap.reserve(10000);
    for (auto it = index.begin(); it != index.end(); it++)
    {
        toMap.push_back(make_pair(Node::mapInformation[it->first][it->second].x, Node::mapInformation[it->first][it->second].y));
    }
    return toMap;
}

/**
 * @brief a rosservice: server to answer target path with  now position, target position given
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool Node::answerPath(astar_nav::astar_controller::Request &req, astar_nav::astar_controller::Response &res)
{
    // system("clear");
    // cout << "Using A* to find path" << endl;
    // cout << "goal(X,Y) = (" << req.goal_pose.x << ", " << req.goal_pose.y << ")" << endl;
    Node finalNode = Node::findPath(req.current_pose.x, req.current_pose.y, req.goal_pose.x, req.goal_pose.y);
    // cout << "find a path!" << endl;
    //  auto finalPath = refineMap(finalNode.path);
    double pathLength = 0;
    int previous_x = (finalNode.path.begin())->first;
    int previous_y = (finalNode.path.begin())->second;
    for (auto it = finalNode.path.begin(); it != finalNode.path.end(); it++)
    {
        int now_x = it->first;
        int now_y = it->second;

        if (abs(now_x - previous_x) == abs(now_y - previous_y))
        {
            pathLength += 1.414 * max(abs(now_x - previous_x), abs(now_y - previous_y)) * Node::mapRes;
        }
        else
        {
            pathLength += max(abs(now_x - previous_x), abs(now_y - previous_y)) * Node::mapRes;
        }
        previous_x = now_x;
        previous_y = now_y;
    }
    // printf("length = %lf\n", pathLength);
    // auto pathOnMap = indexToMap(finalPath);
    auto pathOnMap = indexToMap(finalNode.path);
    for (auto it = pathOnMap.begin(); it != pathOnMap.end(); it++)
    {
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = it->first;
        this_pose_stamped.pose.position.y = it->second;
        this_pose_stamped.header.frame_id = "map";
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 1;
        // res.nextPointX.push_back(it->first);
        // res.nextPointY.push_back(it->second);
        res.path.poses.push_back(this_pose_stamped);
        // it++;
        // //cout << "("<< it->first <<","<<it->second << ")";
        // if(it != nowNode.path.end() - 1)
        // //cout << "->";
    }
    // auto finalAct = refineDirection(finalNode.actionToThisNode);
    // for(auto it:finalAct){
    //     res.nextPointDirection.push_back(it);
    // }
    // for (auto it : finalNode.actionToThisNode)
    // {
    //     res.nextPointDirection.push_back(it);
    // }

    return true;
}

bool Node::mapBuilt = false;
bool Node::mapInit = false;
list<pair<int, int>> Node::direction;
int Node::expandCount = 0;
int Node::nodeVisitedAmount = 0;
struct MapInformation **Node::mapInformation;

double Node::mapOriginX;
double Node::mapOriginY;
double Node::mapRes;
uint32_t Node::mapX;
uint32_t Node::mapY;
