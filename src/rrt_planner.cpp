/**
* 2022.07.28 Last Edited by Evin Hsu
*/

#include<ros/ros.h>
#include<rrt_planner/rrt_planner.h>

BRRTStarPlanner::BRRTStarPlanner()
{
    initialize();
}

BRRTStarPlanner::~BRRTStarPlanner()
{}

void BRRTStarPlanner::initialize()
{
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh_;
    
    start_goal_srv_  = nh_.advertiseService("start_goal_path_inter", &BRRTStarPlanner::start_goal_path, this);
    map_info_srv_    = nh_.advertiseService("occupancy_map_info", &BRRTStarPlanner::get_map_info, this);
    random_path_srv_ = nh_.advertiseService("random_path_inter", &BRRTStarPlanner::random_get_path, this);
    occ_val_srv_     = nh_.advertiseService("occupancy_value", &BRRTStarPlanner::get_occ_value, this);
    marker_tree_pub_       = private_nh.advertise<visualization_msgs::Marker>("marker_tree", 10);
    marker_start_goal_pub_ = private_nh.advertise<visualization_msgs::Marker>("marker_start_goal", 10);
    marker_path_pub_       = private_nh.advertise<visualization_msgs::Marker>("marker_path", 10);
    marker_inter_pub_      = private_nh.advertise<visualization_msgs::Marker>("marker_inter", 10);
    
    nh_.param("/rrt_planner/mk_frame", mk_frame_, true);
    private_nh.param("/rrt_planner/BRRTStarPlanner/tolerance", tolerance_, 0.5);            // radius of goal area
    private_nh.param("/rrt_planner/BRRTStarPlanner/radius", radius_, 1.0);                  // for ChooseParent and Rewire
    private_nh.param("/rrt_planner/BRRTStarPlanner/epsilon_min", epsilon_min_, 0.01);       // min length of edges
    private_nh.param("/rrt_planner/BRRTStarPlanner/epsilon_max", epsilon_max_, 0.2);        // max length of edges
    private_nh.param("/rrt_planner/BRRTStarPlanner/max_node_num", max_node_num_, 3000);     // max nodes number
    private_nh.param("/rrt_planner/BRRTStarPlanner/robot_radius", robot_radius_, 0.4);      // for collision-checking
    private_nh.param("/rrt_planner/BRRTStarPlanner/timeout_sec", timeout_sec_, 10.0);       // timeout
    private_nh.param("/rrt_planner/BRRTStarPlanner/global_visualization", is_vis_, true);   // rviz visualization ON/OFF
    private_nh.param("/rrt_planner/BRRTStarPlanner/visualization_step", vis_step_, 5);      // rviz visualization update frequency 
    private_nh.param("/rrt_planner/BRRTStarPlanner/global_debug", debug_, false);           // rrt_planner.cpp DEBUG ON/OFF
    private_nh.param("/rrt_planner/BRRTStarPlanner/map_debug", debug_map_, false);          // grid_map.cpp DEBUG ON/OFF   
    
    printf("------------------------------------------------------------------\n");
    printf("Parameters: \n[Goal Tolerence]: %.2lf\n[Choose Parent / Rewire Radius]: %.2lf\n[Steer Length]: %.2lf ~ %.2lf\n[Robot Radius]: %.2lf\n[Max Node Number]: %d\n[Timeout Seconds]: %.2lf\n[visualization_step]: %d\n",
            tolerance_, radius_, epsilon_min_, epsilon_max_, robot_radius_, max_node_num_, timeout_sec_, vis_step_);
    
    srand((unsigned)time(NULL));

    // map init.
    this->grid_map_.initialize(robot_radius_, debug_map_);
    this->grid_map_.get_map_info_value(originX, originY, resolution, width, height);
    
    // vis. init.
    marker_reset();
    mk_tree_z = 0.0;
    mk_point_z = 0.02;
    mk_path_z = 0.01;
    mk_inter_z = 0.01;
    
    count_ = 0;
    is_arrived_ = false;
    is_first_path_ = false;
    
    printf("------------------------------------------------------------------\n");
    printf("Use B-RRT-Star Global Planner ... initialize completly !\n");
}

// ==============================
//  Marker 
// ==============================

void BRRTStarPlanner::markerTreeInitial(visualization_msgs::Marker* mk)
{
    if(mk_frame_)
      mk->header.frame_id = "odom";
    else
      mk->header.frame_id = "map";
    mk->header.stamp = ros::Time(0);
    mk->ns = "markers";
    mk->action = visualization_msgs::Marker::ADD;
    mk->scale.x = 0.1;
    mk->scale.y = 0.1;
    mk->pose.orientation.w = 1.0;
    mk->color.r = 255.0/255.0;
    mk->color.g = 0.0/255.0;
    mk->color.b = 0.0/255.0;
    mk->color.a = 1.0;
    mk->lifetime = ros::Duration();
}


void BRRTStarPlanner::marker_reset()
{
    // Clear All
    mk_start_.action = visualization_msgs::Marker::DELETEALL;
    marker_start_goal_pub_.publish(mk_start_);
    mk_start_.points.clear();
    mk_goal_.action = visualization_msgs::Marker::DELETEALL;
    marker_start_goal_pub_.publish(mk_goal_);
    mk_goal_.points.clear();
    mk_tree_1_.action = visualization_msgs::Marker::DELETEALL;
    marker_tree_pub_.publish(mk_tree_1_);
    mk_tree_1_.points.clear();
    mk_tree_2_.action = visualization_msgs::Marker::DELETEALL;
    marker_tree_pub_.publish(mk_tree_2_);
    mk_tree_2_.points.clear();
    mk_path_.action = visualization_msgs::Marker::DELETEALL;
    marker_path_pub_.publish(mk_path_);
    mk_path_.points.clear();
    mk_inter_.action = visualization_msgs::Marker::DELETEALL;
    marker_inter_pub_.publish(mk_inter_);
    mk_inter_.points.clear();
    
    // Initialize
    markerTreeInitial(&mk_start_); 
    mk_start_.type = visualization_msgs::Marker::POINTS;
    mk_start_.id = 0;
    mk_start_.color.b = 0.2f;
    mk_start_.color.g = 1.0f;
    mk_start_.color.r = 1.0f;
    mk_start_.scale.x = 0.1;
    
    markerTreeInitial(&mk_goal_); 
    mk_goal_.type = visualization_msgs::Marker::POINTS;
    mk_goal_.id = 1;
    mk_goal_.color.b = 1.0f;
    mk_goal_.color.g = 1.0f;
    mk_goal_.color.r = 0.0f;
    mk_goal_.scale.x = 0.1;
    
    markerTreeInitial(&mk_tree_1_);
    mk_tree_1_.type = visualization_msgs::Marker::LINE_LIST;
    mk_tree_1_.id = 2;
    mk_tree_1_.color.b = 0.0f;
    mk_tree_1_.color.g = 0.35f;
    mk_tree_1_.color.r = 1.0f;
    mk_tree_1_.scale.x = 0.02;
    
    markerTreeInitial(&mk_tree_2_); 
    mk_tree_2_.type = visualization_msgs::Marker::LINE_LIST;
    mk_tree_2_.id = 3;
    mk_tree_2_.color.b = 0.0f;
    mk_tree_2_.color.g = 1.0f;
    mk_tree_2_.color.r = 0.0f;
    mk_tree_2_.scale.x = 0.02;
    
    markerTreeInitial(&mk_path_); 
    mk_path_.type = visualization_msgs::Marker::LINE_LIST;
    mk_path_.id = 4;
    mk_path_.color.b = 0.0f;
    mk_path_.color.g = 0.0f;
    mk_path_.color.r = 1.0f;
    mk_path_.scale.x = 0.03;
    
    markerTreeInitial(&mk_inter_);
    mk_inter_.type = visualization_msgs::Marker::POINTS;
    mk_inter_.id = 5;
    mk_inter_.color.b = 1.0f;
    mk_inter_.color.g = 1.0f;
    mk_inter_.color.r = 0.0f;
    mk_inter_.scale.x = 0.1;
}


// ==============================
//  Service Server
// ==============================

bool BRRTStarPlanner::get_map_info(rrt_planner::OccMapInfo::Request &req, rrt_planner::OccMapInfo::Response &res)
{
    printf("Receive Request to get map information.\n");
    this->grid_map_.get_map_info_value( originX,
                                        originY,
                                        resolution,
                                        width,
                                        height);
    res.origin_x = originX;
    res.origin_y = originY;
    res.resolution = resolution;
    res.width = width;
    res.height = height;
    return true;
}

bool BRRTStarPlanner::get_occ_value( rrt_planner::MapOcc::Request &req, rrt_planner::MapOcc::Response &res)
{
    printf("Receive Request to get occupancy.\n");
    res.occ = this->grid_map_.occGridValue(req.x, req.y);
    return true;
}

// path planning with given start and goal
bool BRRTStarPlanner::start_goal_path(rrt_planner::StartGoalPath::Request  &req, rrt_planner::StartGoalPath::Response &res)
{
    count_++;
    printf("[%d]Receive Request: Start at (%.4lf, %.4lf) ; Goal at (%.4lf, %.4lf).\n",
                count_, req.start.pose.position.x, req.start.pose.position.y, req.goal.pose.position.x, req.goal.pose.position.y);
    
    geometry_msgs::PoseStamped start, goal;
    grid_map_.generate_start_goal(start, goal, count_);         
    start.pose.position.x = (double)req.start.pose.position.x; 
    start.pose.position.y = (double)req.start.pose.position.y;
    goal.pose.position.x  = (double)req.goal.pose.position.x; 
    goal.pose.position.y  = (double)req.goal.pose.position.y;
    
    if( this->grid_map_.isObstaclePoint(start.pose.position.x, start.pose.position.y) ||
        this->grid_map_.isObstaclePoint(goal.pose.position.x, goal.pose.position.y))
    {
        printf("Position of start or goal point is not available. Planning Failed.\n");
        res.success = false;
        return false;
    }

    if(is_vis_)
    {
        marker_reset();
        geometry_msgs::Point p;
        p.x = start.pose.position.x;
        p.y = start.pose.position.y;
        p.z = mk_point_z;
        mk_start_.points.push_back(p);
        p.x = goal.pose.position.x;
        p.y = goal.pose.position.y;
        mk_goal_.points.push_back(p);
        marker_start_goal_pub_.publish(mk_start_);
        marker_start_goal_pub_.publish(mk_goal_);
    }
    
    // Start Planning
    this->makePlan(start, goal, res.path, res.inters);
    
    res.success = true;
    printf(">> Planning Done.\n");
    
    return res.success;
}

// path planning with random start and goal
bool BRRTStarPlanner::random_get_path(rrt_planner::RandomGetPath::Request  &req, rrt_planner::RandomGetPath::Response &res)
{
    count_++;
    printf("[%d]Receive Request with random start and goal.\n", count_);
    
    geometry_msgs::PoseStamped start, goal;
    grid_map_.generate_start_goal(start, goal, count_);
    
    if(is_vis_)
    {
        marker_reset();
        geometry_msgs::Point p;
        p.x = start.pose.position.x;
        p.y = start.pose.position.y;
        p.z = mk_point_z;
        mk_start_.points.push_back(p);
        p.x = goal.pose.position.x;
        p.y = goal.pose.position.y;
        mk_goal_.points.push_back(p);
        marker_start_goal_pub_.publish(mk_start_);
        marker_start_goal_pub_.publish(mk_goal_);
    }
    
    res.start = start;
    res.goal = goal;
    
    this->makePlan(start, goal, res.path, res.inters);
    res.success = true;
    printf(" >> Planning Done.\n");
    
    return true;
}

// ============================
//  Algorithm
// ============================

// main
bool BRRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal, 
                                std::vector<geometry_msgs::PoseStamped>& plan, 
                                std::vector<geometry_msgs::PoseStamped>& inters)
{
    if(debug_)
    {
        printf("Start : (%.2lf, %.2lf), Goal : (%.2lf, %.2lf)\n", 
                start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    }
    
    // ------------------------
    // Initialization
    // ------------------------
    
    connect_mode_ = ConnectMode::FAIL;
    plan.clear();
    inters.clear();
    
    // Initialize tree 1
    Node node_new(start.pose.position.x, start.pose.position.y, 0, -1);
    tree_1_.clear();    
    tree_1_.reserve(max_node_num_/2);
    mk_tree_1_.points.clear();
    goal_id_1_ = -10;
    node_new.cost = 0.0;
    tree_1_.push_back(node_new);

    // Initialize tree 2
    tree_2_.clear();    
    tree_2_.reserve(max_node_num_/2);
    mk_tree_2_.points.clear();
    goal_id_2_ = -10;
    node_new.x = goal.pose.position.x;
    node_new.y = goal.pose.position.y;
    tree_2_.push_back(node_new);
    
    if(debug_)
    {
        printf("Tree_1 Add new Node %d at (%.2lf, %.2lf) follow Parent Node [%d].\n",
                tree_1_[0].node_id, tree_1_[0].x, tree_1_[0].y, tree_1_[0].parent_id);
        printf("Tree_2 Add new Node %d at (%.2lf, %.2lf) follow Parent Node [%d].\n",
                tree_2_[0].node_id, tree_2_[0].x, tree_2_[0].y, tree_2_[0].parent_id);
    }
    
    std::pair<double, double> q_rand;
    std::pair<double, double> q_new_1, q_new_2;
    Node node_near;
    
    if(debug_)
    {
        printf("Start >> ");
    }
    
    // ------------------------
    // Start Planning
    // ------------------------

    clock_t start_t;
    bool resample = true;
    
    while(tree_1_.size() + tree_2_.size() < max_node_num_)
    {
        if(debug_)  
            ("Tree 1 has %zu nodes. Tree2 has %zu nodes.\n", tree_1_.size(), tree_2_.size());
        
        // 1. Tree 1 extend
        start_t = clock();
        do{
            if(debug_)
                printf("\rExtend 1 >> ");
            // timeout
            if((clock() - start_t)/CLOCKS_PER_SEC > timeout_sec_){
                printf("Plan timeout.");
                return PlanOnlyStartGoal(plan, inters, start, goal);
            } 
            q_rand = this->SampleFree();
        }
        while(!this->ExtendTree(q_rand, tree_1_, mk_tree_1_, goal, goal_id_1_, resample));
        
        printf("\n");

        resample = false;
        
        if( is_vis_ && tree_1_.size() % vis_step_ == 0)
        {
            marker_tree_pub_.publish(mk_tree_1_);
        }

        // 2. Tree 1 arrived check
        if( isArrivedGoal(tree_1_.back(), goal) )
        {
            addNewNode(tree_1_.back().parent_id, goal.pose.position.x, goal.pose.position.y, tree_1_, mk_tree_1_, goal_id_1_);
            goal_id_1_ = tree_1_.size() - 1;
            connect_mode_ = ConnectMode::ONE_ARRIVED;
            if(debug_)
                printf("\nTree 1 Arrived Goal.\n");
            break;
        }
        // 3.  Tree 1 meets Tree 2 check
        if( isConnect(tree_1_.back(), tree_1_, tree_2_, mk_tree_1_, goal_id_1_, goal_id_2_, robot_radius_) )
        {
            connect_mode_ = ConnectMode::ONE_MEET_TWO;
            if(debug_)
                printf("\nTree 1 Meets Tree 2.\n");
            break;
        }
        // 4. Tree 2 extend
        if(debug_){
            printf("Turn Tree 2 >> ");
        }

        start_t = clock();
        while(!ExtendTree(q_rand, tree_2_, mk_tree_2_, start, goal_id_2_, resample))
        {
			// timeout
            if((clock() - start_t)/CLOCKS_PER_SEC > timeout_sec_){
                printf("Plan timeout.");
                return PlanOnlyStartGoal(plan, inters, start, goal);
            } 
            q_rand = SampleFree();
            resample = true;
        }
        resample = false;

        if(debug_)
            printf("Extend 2 >> ");
        // 5. Tree 2 arrived check
        if( isArrivedGoal(tree_2_.back(), start) )
        {
            addNewNode(tree_2_.back().parent_id, start.pose.position.x, start.pose.position.y, tree_2_, mk_tree_2_, goal_id_2_);
            goal_id_2_ = tree_2_.size() - 1;
            connect_mode_ = ConnectMode::TWO_ARRIVED;
            if(debug_)
                printf("\nTree 2 Arrived Start.\n");
            break;
        }
        // 6. Tree 2 meets Tree 1 check
        if( isConnect(tree_2_.back(), tree_2_, tree_1_, mk_tree_2_, goal_id_2_, goal_id_1_, robot_radius_) )
        {
            connect_mode_ = ConnectMode::ONE_MEET_TWO;
            if(debug_)
                printf("\nTree 2 Meets Tree 1\n");
            break;
        }

        if( is_vis_ && tree_2_.size() % vis_step_ == 0)
        {
            marker_tree_pub_.publish(mk_tree_2_);
        }
    } // end while

    if( is_vis_ )
    {
        marker_tree_pub_.publish(mk_tree_1_);
        marker_tree_pub_.publish(mk_tree_2_);
    } 

    if(connect_mode_ != ConnectMode::FAIL)
    {
        if(debug_)
        {
            printf("Start Planning Path >> ");
        }
        return getFinalPlan(tree_1_, tree_2_, goal_id_1_, goal_id_2_, connect_mode_, plan, inters, start, goal, true);
    }
    else
    {
        printf("Global Path Fail ! Unable to reach the goal.\n");
        return false;
    }
}


bool BRRTStarPlanner::ExtendTree(   std::pair<double, double>& q_rand, 
                                    std::vector<Node>& tree_node, 
                                    visualization_msgs::Marker& marker_tree, 
                                    const geometry_msgs::PoseStamped& goal, 
                                    int goal_id,
                                    bool first_tree)
{
    if(debug_)  
        printf("Sampling at (%lf, %lf).\n", q_rand.first, q_rand.second);
    Node node_near = tree_node[NearestNode(q_rand.first, q_rand.second, tree_node)];
    std::pair<double, double> q_new = SteerNode(node_near.x, node_near.y, q_rand.first, q_rand.second);
    if( !grid_map_.isObstacleBetween( node_near.x, node_near.y, q_new.first, q_new.second ) )
    {
        addNewNode(node_near.node_id, q_new.first, q_new.second, tree_node, marker_tree, goal_id);
        q_rand.first = q_new.first;
        q_rand.second = q_new.second;
        return true;
    }
    else
    {
        printf("Collision Occured. Retry.\n");
        return false;
    }
}

std::pair<double, double> BRRTStarPlanner::SampleFree()
{
    std::pair<double, double> point;
    point.first  = getRandomValue(originX, originX + width  * resolution);
    point.second = getRandomValue(originY, originY + height * resolution);
    return point;
}

int BRRTStarPlanner::NearestNode( double x, double y , const std::vector<Node>& tree_node)
{
    int nearest_id = 0;
    double min_dist = HUGE_VAL;
    for(size_t i = 0; i < tree_node.size(); i++){
        double dist = euclideanDistance2D( x, y, tree_node[i].x, tree_node[i].y );
        if(dist < min_dist)
        {
            min_dist = dist;
            nearest_id = i;
        }
    }
    if(debug_)  
        printf("Find Nearest Node %d with distance %.2lf.\n", nearest_id, min_dist);
    return nearest_id;
}

std::pair<double, double> BRRTStarPlanner::SteerNode( double nearX, double nearY, double randX, double randY )
{
    std::pair<double, double> point;
    double dist = euclideanDistance2D( nearX, nearY, randX, randY );
    if (dist <= epsilon_max_ && dist >= epsilon_min_){
        point.first  = randX;
        point.second = randY;
    }
    else{
        double theta = atan2( randY-nearY, randX-nearX );
        point.first  = nearX + epsilon_max_ * cos(theta);
        point.second = nearY + epsilon_max_ * sin(theta);
    }
    if(debug_)  
        printf("Decide New Node at (%.2lf, %.2lf).\n", point.first, point.second);
    return point;
}

void BRRTStarPlanner::addNewNode(int parent_id, double x, double y, std::vector<Node>& tree_node, visualization_msgs::Marker& marker_tree, int goal_id)
{
    Node node_new( x, y, tree_node.size(), parent_id );
    node_new = chooseParent(node_new, tree_node, radius_, goal_id);
    if(!(grid_map_.isObstacleAround(node_new.x, node_new.y, 1))){
        node_new.cost += 0.5;
    }
    if(!(grid_map_.isObstacleAround(node_new.x, node_new.y, 5))){
        node_new.cost += 0.2;
    }
    tree_node.push_back(node_new);
    rewireTree(node_new, tree_node, radius_);
    if(debug_)  
        printf("Add new Node %d at (%.2lf, %.2lf) follow Parent Node [%d].\n", node_new.node_id, node_new.x, node_new.y, node_new.parent_id);

    geometry_msgs::Point branch_new;
    branch_new.x = tree_node[node_new.parent_id].x;
    branch_new.y = tree_node[node_new.parent_id].y;
    branch_new.z = mk_tree_z;
    marker_tree.points.push_back(branch_new);
    branch_new.x = node_new.x;
    branch_new.y = node_new.y;
    marker_tree.points.push_back(branch_new);
}

Node BRRTStarPlanner::chooseParent(Node new_node, std::vector<Node>& tree_nodes, double near_radius, int goal_id)
{
    new_node.cost = tree_nodes[new_node.parent_id].cost + 
                        euclideanDistance2D(tree_nodes[new_node.parent_id].x, tree_nodes[new_node.parent_id].y,
                                            new_node.x, new_node.y );
    for(size_t i=0; i<tree_nodes.size(); i++)
    {
        if(goal_id >= 0 && i == goal_id)
            continue;
        double dist = euclideanDistance2D( tree_nodes[i].x, tree_nodes[i].y, new_node.x, new_node.y );
        if(dist < near_radius)
        {
            double cost = tree_nodes[i].cost + dist;
            if(cost < new_node.cost){
                if( !grid_map_.isObstacleBetween( tree_nodes[i].x, tree_nodes[i].y, new_node.x, new_node.y ) ){
                    new_node.parent_id = i;
                    new_node.cost = cost;
                }
            } 
        }
    }
    return new_node;
}

void BRRTStarPlanner::rewireTree(Node new_node, std::vector<Node>& tree_nodes, double near_radius)
{
    for(size_t t = 0; t < tree_nodes.size(); t++)
    {
        if(tree_nodes[t].node_id == new_node.parent_id)
            continue;
        double dist = euclideanDistance2D( tree_nodes[t].x, tree_nodes[t].y, new_node.x, new_node.y );
        if(dist < near_radius){
            double cost = new_node.cost + dist;
            if(cost < tree_nodes[t].cost){
                if( !grid_map_.isObstacleBetween( tree_nodes[t].x, tree_nodes[t].y, new_node.x, new_node.y ) ){
                    tree_nodes[t].parent_id = new_node.node_id;
                    tree_nodes[t].cost = cost;
                    if(debug_)  
                        printf(">>>> Rewire [%d] at (%.2lf, %.2lf) follow Parent Node [%d].\n", new_node.node_id, new_node.x, new_node.y, new_node.parent_id);
                }
            } 
        }
    } 
}

bool BRRTStarPlanner::isArrivedGoal(const Node& node_new, const geometry_msgs::PoseStamped& goal)
{
    if( euclideanDistance2D( node_new.x, node_new.y, goal.pose.position.x, goal.pose.position.y ) <= tolerance_ ){
        return true;
    }
    return false;
}

bool BRRTStarPlanner::isConnect(Node node_new, std::vector<Node>& this_tree,  const std::vector<Node> another_tree, visualization_msgs::Marker& this_marker_tree, int& this_goal_id, int& another_goal_id, double tolerance)
{
    int connect_id = NearestNode(node_new.x, node_new.y, another_tree);
    double dist = euclideanDistance2D( another_tree.at(connect_id).x, another_tree.at(connect_id).y, node_new.x, node_new.y);
    if(dist <= tolerance)
    {
        geometry_msgs::Point branch_new;
        branch_new.x = node_new.x;
        branch_new.y = node_new.y;
        branch_new.z = mk_tree_z;
        this_marker_tree.points.push_back(branch_new);
        branch_new.x = another_tree.at(connect_id).x;
        branch_new.y = another_tree.at(connect_id).y;
        this_marker_tree.points.push_back(branch_new);
        
        this_goal_id = node_new.node_id;        // in this tree
        another_goal_id = connect_id;           // in another tree
        return true;
    }
    return false;
}


double BRRTStarPlanner::getRandomValue(double min, double max)
{
    double f = min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (max-min)));
    return f;
}

// ============================
// Get Final Path
// ============================
// Main
bool BRRTStarPlanner::getFinalPlan( const std::vector<Node> tree_node_1,
                                    const std::vector<Node> tree_node_2,
                                    int goal_id_1,                                      // Tree 1 connecting node
                                    int goal_id_2,                                      // Tree 2 connecting node
                                    int connect_mode,
                                    std::vector<geometry_msgs::PoseStamped>& plan,      // full path
                                    std::vector<geometry_msgs::PoseStamped>& inters,    // inter points
                                    const geometry_msgs::PoseStamped& start,
                                    const geometry_msgs::PoseStamped& goal,
                                    bool final)                                         // publish or not                 
{
    std::vector<std::pair<double, double>> plan_poses;
    std::vector<std::pair<double, double>> inter_poses;
    
    // Case 1 : Tree 1 arrived Goal
    if(connect_mode == ConnectMode::ONE_ARRIVED)
    {
        Node current_node = tree_node_1.at(goal_id_1);
        std::pair<double, double> waypoint;
        while(current_node.parent_id != -1)
        {
            waypoint.first = current_node.x;
            waypoint.second = current_node.y;
            plan_poses.insert(plan_poses.begin(), waypoint);
            current_node = tree_node_1.at(current_node.parent_id);
        }
        waypoint.first = current_node.x;
        waypoint.second = current_node.y;
        plan_poses.insert(plan_poses.begin(), waypoint);
    }
    // Case 2 : Tree 2 arrived Start
    else if(connect_mode == ConnectMode::TWO_ARRIVED)
    {
        Node current_node = tree_node_2.at(goal_id_2);
        std::pair<double, double> waypoint;
        while(current_node.parent_id != -1)
        {
            waypoint.first = current_node.x;
            waypoint.second = current_node.y;
            plan_poses.push_back(waypoint);
            current_node = tree_node_2.at(current_node.parent_id);
        }
        waypoint.first = current_node.x;
        waypoint.second = current_node.y;
        plan_poses.push_back(waypoint);
    }
    // Case 3 & 4 : Tree 1 meets Tree 2 or vice versa
    else if(connect_mode == ConnectMode::ONE_MEET_TWO || connect_mode == ConnectMode::TWO_MEET_ONE)
    {
        Node current_node = tree_node_1.at(goal_id_1);
        std::pair<double, double> waypoint;
        while(current_node.parent_id != -1){
            waypoint.first = current_node.x;
            waypoint.second = current_node.y;
            plan_poses.insert(plan_poses.begin(), waypoint);
            current_node = tree_node_1.at(current_node.parent_id);
        }
        waypoint.first = current_node.x;
        waypoint.second = current_node.y;
        plan_poses.insert(plan_poses.begin(), waypoint);

        current_node = tree_node_2.at(goal_id_2);
        while(current_node.parent_id != -1){
            waypoint.first = current_node.x;
            waypoint.second = current_node.y;
            plan_poses.push_back(waypoint);
            current_node = tree_node_2.at(current_node.parent_id);
        }
        waypoint.first = current_node.x;
        waypoint.second = current_node.y;
        plan_poses.push_back(waypoint);
    }
    else
    {
        printf("Global Path Fail ! Unable to reach the goal.\n");
        return false;
    }
    
    // Get Intermidiate points
    if(!getInterPoints(plan_poses, inter_poses))
        return false;
    
    // ROS Process
    nav_plan(plan_poses, plan, start, goal, final);
    navInterPoints(inter_poses, inters, goal, final);
    
    return true;
}

// If Timeout
bool BRRTStarPlanner::PlanOnlyStartGoal(std::vector<geometry_msgs::PoseStamped>& plan,
                                        std::vector<geometry_msgs::PoseStamped>& inters, 
                                        const geometry_msgs::PoseStamped& start, 
                                        const geometry_msgs::PoseStamped& goal)
{
    std::vector<std::pair<double, double>> plan_poses;
    std::vector<std::pair<double, double>> inter_poses;

    std::pair<double, double> p;
    p.first = start.pose.position.x;
    p.second = start.pose.position.y;
    plan_poses.push_back(p);
    p.first = goal.pose.position.x;
    p.second = goal.pose.position.y;
    plan_poses.push_back(p);

    if(!getInterPoints(plan_poses, inter_poses))
        return false;
    nav_plan(plan_poses, plan, start, goal, true);
    navInterPoints(inter_poses, inters, goal, true);
    
    return true;
}


bool BRRTStarPlanner::getInterPoints(const std::vector<std::pair<double, double>>& plan, std::vector<std::pair<double, double>>& inters)
{
    if(debug_)  printf("Get inter points.\n");
    std::vector<std::pair<double, double>> new_inters;
    
    int start_id = 0;
    int current_id = start_id;
    
    new_inters.push_back(plan.at(start_id));

    while(current_id < plan.size() - 1)
    {
        current_id += 1;
        if(grid_map_.isObstacleBetween( plan.at(start_id).first, plan.at(start_id).second, 
                                        plan.at(current_id).first, plan.at(current_id).second ))
        {
            new_inters.push_back(plan.at(current_id-1));
            start_id = current_id-1;
        }
    }
    new_inters.push_back(plan.at(current_id));
    
	// Optmization
    if(plan.size() != new_inters.size()){
        return getInterPoints(new_inters, inters);
    }else{
        inters.clear();
        if(new_inters.size()>0){
            for(size_t t = 0; t < new_inters.size()-1; t++){
                if(t != 0){
                    inters.push_back(new_inters.at(t));
                    if(debug_)
                        printf("Add new inter point at (%.3f, %.3f)\n", new_inters.at(t).first, new_inters.at(t).second);
                }
            }
        }
        return true;
    }
}

// Orientation Optimization
void BRRTStarPlanner::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
{
    if(plan.size() <= 1)
        return;
    for(size_t i = 0; i < plan.size()-1; i++)
    {
        plan[i].pose.orientation = tf::createQuaternionMsgFromYaw( atan2( plan[i+1].pose.position.y - plan[i].pose.position.y,
                                                                            plan[i+1].pose.position.x - plan[i].pose.position.x ) );
    }
    if(debug_)  
        printf("Optimize Orientation Successfully.\n");
}

// pose --> posestamped
void BRRTStarPlanner::nav_plan( const std::vector<std::pair<double, double>> poses, 
                                std::vector<geometry_msgs::PoseStamped>& plan,
                                const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal,
                                bool final)
{
    plan.clear();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = grid_map_.getMapFrame();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    for (size_t i = 0; i < poses.size(); i++) 
    {
        // Marker path
        if(i > 0 && final)
        {
            geometry_msgs::Point p;
            p.x = poses[i-1].first;
            p.y = poses[i-1].second;
            p.z = mk_path_z;
            mk_path_.points.push_back(p);

            p.x = poses[i].first;
            p.y = poses[i].second;
            mk_path_.points.push_back(p);
        }
        // Posestamp plan
        pose.pose.position.x = poses[i].first;
        pose.pose.position.y = poses[i].second;
        plan.push_back(pose);
    }

    // Node 方向優化
    optimizationOrientation(plan);
    plan[0].pose.orientation = start.pose.orientation;
    plan[plan.size()-1].pose.orientation = goal.pose.orientation;

    if(final && is_vis_)  
        marker_path_pub_.publish(mk_path_);
}

// pose --> posestamped
void BRRTStarPlanner::navInterPoints(const std::vector<std::pair<double, double>> poses, 
                                            std::vector<geometry_msgs::PoseStamped>& plan,
                                            const geometry_msgs::PoseStamped& goal,
                                            bool final)
{
    plan.clear();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = grid_map_.getMapFrame();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    for (size_t i = 0; i < poses.size(); i++) 
    {
        pose.pose.position.x = poses[i].first;
        pose.pose.position.y = poses[i].second;
        plan.push_back(pose);

        if(final)
        {
            geometry_msgs::Point p;
            p.x = poses[i].first;
            p.y = poses[i].second;
            p.z = mk_inter_z;
            mk_inter_.points.push_back(p);
        }
    }

    if(final && is_vis_ && mk_inter_.points.size() > 0)  
        marker_inter_pub_.publish(mk_inter_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");
    BRRTStarPlanner global_planner_;
    ros::spin();
    return 0;
}

