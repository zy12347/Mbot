#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <common_utils/geometric/angle_functions.hpp>

using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
     ////////////////// TODO: Implement your A* search here //////////////////////////
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    std::cout<<"meters:"<<distances.metersPerCell()<<" distances "<<distances(goal.x,goal.y)<<std::endl;
    mbot_lcm_msgs::robot_path_t path;
    path.utime  = start.utime;

    // Rule out instances where a path does not exist
    if(!distances.isCellInGrid(startCell.x, startCell.y) || !distances.isCellInGrid(goalCell.x, goalCell.y)){
        path.path.push_back(start);
        path.path_length = path.path.size();
        return path;
    }

    std::vector<Node*> nodePath;
    Node* startNode = new Node(startCell.x, startCell.y);
    startNode->g_cost = 0.;
    startNode->h_cost = 0.;

    Node* goalNode = new Node(goalCell.x, goalCell.y);
    goalNode->g_cost = 0.;
    goalNode->h_cost = 0.;

    PriorityQueue open;
    std::vector<Node*> closed;
    open.push(startNode);

    bool complete = false;
    while (!open.empty()){
        //Expand node in open list
        Node* n = open.pop();
        std::vector<Node*> kids = expand_node(n, distances, params);

        //Go through all kids
        for (int i = 0; i < kids.size(); i++){
            double g = g_cost(n, kids.at(i), distances, params);
            double h = h_cost(kids.at(i), goalNode, distances); 

            Node *check_closed = get_from_list(kids.at(i), closed);
            Node *check_open = open.get_member(kids.at(i));
            //If child is in closed, we won't revisit so we can delete
            if (check_closed){
                if (check_closed->f_cost() < kids.at(i)->f_cost()){
                    delete kids.at(i);
                    continue;
                }
            }
            //If child is in open, update the costs
            else if (check_open){
                if(check_open->f_cost() > kids.at(i)->f_cost()){
                    check_open->g_cost = g;
                    check_open->h_cost = h;
                    check_open->parent = n;
                }
            }
            //Not in closed or open, so we can add to open with updated costs
            else {
                kids.at(i)->g_cost = g;
                kids.at(i)->h_cost = h;
                kids.at(i)->parent = n;
                open.push(kids.at(i));
            }

            //Extract path if we hit the goal
            if (kids.at(i)->cell == goalCell){
                nodePath = extract_node_path(kids.at(i), startNode);
                complete = true;
                break;
            }

        }
        if (complete){
            break;
        }
        closed.push_back(n);
    }
    
    path.path = extract_pose_path(nodePath, distances);
    path.path_length = path.path.size();

    return path;
}

double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    //Manhatten cost
    int x = std::abs(goal->cell.x - from->cell.x);
    int y = std::abs(goal->cell.y - from->cell.y);

    double h_cost = (x + y);

    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost

    // cost to go from start to current
    double g_cost = from->g_cost;   
    int x = std::abs(from->cell.x - goal->cell.x);
    int y = std::abs(from->cell.y - goal->cell.y);

    //Cost to come from previous cell
    if(x==1 & y==1){
        g_cost += 1.4;//test for 1,original 1.4
    }
    else{
        g_cost += 1.;}
    
    if (distances(goal->cell.x, goal->cell.y) < params.maxDistanceWithCost && distances(goal->cell.x, goal->cell.y) > params.minDistanceToObstacle){
        //Too close to an obstacle
        g_cost += pow(params.maxDistanceWithCost - distances(from->cell.x, from->cell.y), params.distanceCostExponent);
    }
    else{
        //pass
    }

    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    const int Xdelta[8]={0, 0, 1, 1, 1, -1,-1,-1};
    const int Ydelta[8]={1,-1,-1, 0, 1, -1, 0, 1};
    std::vector<Node*> children;

    int current_x = (node->cell).x;
    int current_y = (node->cell).y;
    for(int i=0; i<8 ;i++){
        cell_t nextCell(current_x+ Xdelta[i],current_y+Ydelta[i]);

        if (distances.isCellInGrid(nextCell.x, nextCell.y) && distances(nextCell.x, nextCell.y) > params.minDistanceToObstacle){
            Node *next_node = new Node(nextCell.x, nextCell.y);
            next_node->parent = node;
            children.push_back(next_node);
        }
    }
    return children;

}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    Node* current = goal_node;
    while (current->parent != NULL)
    {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    std::vector<mbot_lcm_msgs::pose_xyt_t> path_init;

    //No need to prune if there aren't that many points (also had issues with size 0 nodes vector)
    if (nodes.size()<3){
        for (int i = 0; i <nodes.size(); i++){
            Point<double> xy = grid_position_to_global_position(nodes.at(i)->cell, distances);
            mbot_lcm_msgs::pose_xyt_t pose;
            pose.x = xy.x;
            pose.y = xy.y;
            //Not worth calculating here since we have to fill back in after pruning
            int dy = nodes.at(i)->cell.y - nodes.at(i)->parent->cell.y;
            int dx = nodes.at(i)->cell.x - nodes.at(i)->parent->cell.x;
            pose.theta = wrap_to_pi(std::atan2(dy,dx));
            path.push_back(pose);
        }
        return path;
    }
    // Prune
    else{
        //Obtain x and y values first
        for (int i = 0; i <nodes.size(); i++){
            Point<double> xy = grid_position_to_global_position(nodes.at(i)->cell, distances);
            mbot_lcm_msgs::pose_xyt_t pose;
            pose.x = xy.x;
            pose.y = xy.y;
            pose.theta = 0.;
            path_init.push_back(pose);
        }
        mbot_lcm_msgs::pose_xyt_t curr;
        mbot_lcm_msgs::pose_xyt_t prev;
        mbot_lcm_msgs::pose_xyt_t next;
        //Remove points along approximately straight line
        path.push_back(path_init.at(0));
        for(int i = 1; i <path_init.size() - 1; i++){
            curr = path_init.at(i);
            prev = path_init.at(i-1);
            next = path_init.at(i+1);

            float incAng = wrap_to_pi(std::atan2(curr.y - prev.y, curr.x - prev.x));
            float exitAng = wrap_to_pi(std::atan2(next.y - curr.y, next.x - curr.x));

            float linThresh = M_PI/360.; //If lines are within .5 deg we can call it linear and ignore these
            if (std::abs(incAng - exitAng) < linThresh){
                //pass
            }
            else{
                path.push_back(path_init.at(i));
            }
        }
        path.push_back(path_init.at(path_init.size()-1));
    }

    //Adjust angle based off new set of points
    for (int i = 1; i < path.size(); i++){
        float dy = nodes.at(i)->cell.y - nodes.at(i-1)->cell.y;
        float dx = nodes.at(i)->cell.x - nodes.at(i-1)->cell.x;
        path.at(i).theta = wrap_to_pi(std::atan2(dy,dx));
    }
    return path;
 }


bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}


