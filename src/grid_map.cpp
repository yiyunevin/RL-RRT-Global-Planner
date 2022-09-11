#include<rrt_planner/grid_map.h>

GridMap::GridMap()
{
    srand((unsigned)time(NULL));
    occ_sub_ = nh_.subscribe("/map", 100, &GridMap::occMapCB, this);
    while(ros::ok() && this->occ_map_data_.data.size() < 1)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    if(debug_) get_occ_map_info();
}

GridMap::GridMap(double robot_radius, bool debug)
{
    this->robot_radius_ = robot_radius;
    this->debug_ = debug;
    srand((unsigned)time(NULL));
    occ_sub_ = nh_.subscribe("/map", 100, &GridMap::occMapCB, this);
    while(ros::ok() && this->occ_map_data_.data.size() < 1)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    if(debug_) get_occ_map_info();
}

GridMap::~GridMap()
{}

void GridMap::initialize(double robot_radius, bool debug)
{
    this->robot_radius_ = robot_radius;
    this->debug_ = debug;
}

// ============================
// ROS Subscriber Callback
// ============================

void GridMap::occMapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    this->occ_map_data_ = *msg;
    this->resolution    = occ_map_data_.info.resolution; 
    this->originX       = occ_map_data_.info.origin.position.x;
    this->originY       = occ_map_data_.info.origin.position.y;
    this->width         = occ_map_data_.info.width;
    this->height        = occ_map_data_.info.height;
}

// ============================
// Get Value
// ============================

void GridMap::get_occ_map_info()
{
    printf("Get Occupancy Map Information: \n");
    printf(" >> frame_id: %s\n" , occ_map_data_.header.frame_id.c_str());
    printf(" >> resolution: %.4f\n" , occ_map_data_.info.resolution);
    printf(" >> size: ( %d, %d )\n" , occ_map_data_.info.height, occ_map_data_.info.width);
    printf(" >> origin: ( %.2f, %.2f )\n" , occ_map_data_.info.origin.position.x, occ_map_data_.info.origin.position.y);
}

void GridMap::get_map_info_value(double& m_origin_x, double& m_origin_y, double& m_res, int& m_width, int& m_height)
{
    m_origin_x = this->originX;
    m_origin_y = this->originY;
    m_res      = this->resolution;
    m_width    = this->width;
    m_height   = this->height;
}

std::string GridMap::getMapFrame()
{
    return occ_map_data_.header.frame_id.c_str();
}

int GridMap::occGridValue(double x, double y)
{
    int idx = static_cast<int>( floor( (y-originY)/resolution ) * width + floor( (x-originX)/resolution ) );
    if(idx >= occ_map_data_.data.size())  // out of range --> unknown
        return -1;  
    return occ_map_data_.data[idx];
}


// ============================
// Collision Checking
// ============================
// > 0    ：free space
// > 100  ：occupied
// > -1   ：unknown
// ----------------------------
// > True ：Collid
// > False：Free

bool GridMap::isObstaclePoint(double x, double y)
{
    // Robot Footprint
    std::vector<geometry_msgs::Point> polygon_cells;
    if(rectangleFillCells(x+robot_radius_, y+robot_radius_, x-robot_radius_, y-robot_radius_, polygon_cells))
    {
        if(debug_)  
            printf("Out of Range.\n");
        return true;
    }
    if(debug_)  
        printf("[MAP] Rectangle Footprint : (%lf, %lf) to (%lf, %lf). Full Area = %lu\n", x+robot_radius_, y+robot_radius_, x-robot_radius_, y-robot_radius_, polygon_cells.size());
    for(unsigned int i = 0; i < polygon_cells.size(); ++i)
    {
        int idx = static_cast<int>( floor( (polygon_cells[i].y-originY)/resolution ) * width + floor( (polygon_cells[i].x-originX)/resolution ) );
        int out = (idx >= occ_map_data_.data.size()) ? -1 : occ_map_data_.data[idx];
        if(debug_)  
            printf("%u : Occupacy at (%lf, %lf) = %d", i, polygon_cells[i].x ,polygon_cells[i].y, out);
        if(out == 100 || out == -1)
        {
            if(debug_)  printf(" --> Occupied.\n");
            return true;
        }
        if(debug_)  printf(" --> Free.\n");
    }
    return false;
}

bool GridMap::isObstacleBetween(double Ax, double Ay, double Bx, double By)
{
    double distAB = euclideanDistance2D(Ax, Ay, Bx, By);
    if(distAB >= robot_radius_)
    {
        int steps_number = static_cast<int>(floor(distAB / robot_radius_));
        double theta = atan2(By-Ay, Bx-Ax);

        double wx, wy;
        for (int n = 0; n < steps_number; n++)
        {
            wx = Ax + n * robot_radius_ * cos(theta);
            wy = Ay + n * robot_radius_ * sin(theta);
            if (isObstaclePoint(wx, wy))
            {
                if(debug_)  printf("There is obstacle between the two node.\n");
                return true;
            }
        }
    }
    return (isObstaclePoint(Bx, By)) ? true : false;
}

bool GridMap::isObstacleAround(double x, double y, unsigned int step)
{
    for(int dy = -step; dy <= step; dy++)
    {
        for(int dx = -step; dx <= step; dx++)
        {
            if (isObstaclePoint(x+dx, y+dy))
            {
                if(debug_)  printf("There is obstacle around the node.\n");
                return true;
            }
        }
    }
    return false;
}


double GridMap::getRandomValue(double min, double max)
{
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}

bool GridMap::rectangleFillCells(double max_x, double max_y, double min_x, double min_y, std::vector<geometry_msgs::Point>& polygon_cells)
{
    polygon_cells.clear();

    if( min_x < originX ||
        min_y < originY ||
        max_x > originX + width*resolution ||
        max_y > originY + height*resolution)
        return true;
    
    for(double x = min_x; x < max_x; x+=resolution)
    {
        for(double y = min_y; y < max_y; y+=resolution)
        {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            polygon_cells.push_back(point);
            if(debug_)  printf(" >> add point (%lf, %lf) to fill the convex polygon.\n", point.x, point.y);
        }
    }
    return false;
}


// Randomly generate legal start and goal
void GridMap::generate_start_goal(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, int id)
{
    geometry_msgs::Pose p;

    do
    {
        p.position.x = getRandomValue(originX, originY + width*resolution);
        p.position.y = getRandomValue(originY, originY + height*resolution);
    }while(this->isObstaclePoint(p.position.x, p.position.y));
    
    double ang = getRandomValue(-M_PI, M_PI);
    p.position.z = 0.0;
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = ang;
    p.orientation.w = 1.0;
    start.pose = p;

    while(ros::ok())
    {
        do
        {
            p.position.x = getRandomValue(originX, originX + width*resolution); 
            p.position.y = getRandomValue(originY, originY + height*resolution); 
        }while(this->isObstaclePoint(p.position.x, p.position.y));
        
        if(!(p.position.x==start.pose.position.x && p.position.y==start.pose.position.y))
            break;
    }
    
    p.orientation.z = getRandomValue(-M_PI, M_PI);
    goal.pose = p;
    
    start.header.seq = id;
    start.header.frame_id = occ_map_data_.header.frame_id;
    start.header.stamp = ros::Time::now();
    
    goal.header.seq = id;
    goal.header.frame_id = start.header.frame_id;
    goal.header.stamp = start.header.stamp;
    
    if(debug_)  printf("Get Random Start and Goal Point at (%lf, %lf) and (%lf, %lf).\n", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
}


void GridMap::generate_goal(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, int id)
{
    double x, y;
    while(ros::ok())
    {
        do
        {
            x = getRandomValue(originX, originX + width*resolution);    // -----------------------
            y = getRandomValue(originY, originY + height*resolution);   // -----------------------

        }while(this->isObstaclePoint(x, y));
        
        if(!(x==start.pose.position.x && y==start.pose.position.y))
            break;
    }

    double ang = getRandomValue(-M_PI, M_PI);
    start.header.seq = id;
    start.header.frame_id = occ_map_data_.header.frame_id;
    start.header.stamp = ros::Time::now();
    start.pose.orientation.x = 0.0;
    start.pose.orientation.y = 0.0;
    start.pose.orientation.z = ang;
    start.pose.orientation.w = 1.0;

    ang = getRandomValue(-M_PI, M_PI);
    goal.header.seq = id;
    goal.header.frame_id = start.header.frame_id;
    goal.header.stamp = start.header.stamp;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = ang;
    goal.pose.orientation.w = 1.0;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    if(debug_)  printf("Get Random Start and Goal Point at (%lf, %lf) and (%lf, %lf).\n", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
}