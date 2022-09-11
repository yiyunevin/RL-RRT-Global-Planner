/**
* 2022.07.28 Last Edited by Evin Hsu
*/

#ifndef RRT_GRID_MAP_H_
#define RRT_GRID_MAP_H_

#include<math.h>
#include<stdio.h>
#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<rrt_planner/node.h>

#define TWO_PI  2*M_PI
#define PIECE   M_PI/10

class GridMap
{
public:
    
    ros::NodeHandle nh_;
    GridMap();
    GridMap(double robot_radius, bool debug);
    ~GridMap();
    
    void initialize(double robot_radius, bool debug);
    void generate_start_goal(geometry_msgs::PoseStamped& start, 
                            geometry_msgs::PoseStamped& goal,
                            int id);
    void generate_goal( geometry_msgs::PoseStamped& start, 
                        geometry_msgs::PoseStamped& goal,
                        int id);

    void get_occ_map_info();
    void get_map_info_value(double& m_origin_x, double& m_origin_y, double& m_res, int& m_width, int& m_height);
    void getOrigin(geometry_msgs::Point& origin);
    void getOrigin(double& x, double& y);
    void getResolution(double& r);
    void getMapSize(int& x, int& y);
    int occGridValue(double x, double y);
    std::string getMapFrame();
    bool isObstaclePoint(double x, double y); 
    bool isObstacleBetween(double Ax, double Ay, double Bx, double By);
    bool isObstacleAround(double x, double y, unsigned int step);

private:
    
    void occMapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    double getRandomValue(double min, double max);
    bool rectangleFillCells(double max_x, 
                            double max_y, 
                            double min_x, 
                            double min_y, 
                            std::vector<geometry_msgs::Point>& polygon_cells);

    bool debug_; 
    nav_msgs::OccupancyGrid occ_map_data_; 
    double resolution; 
    double originX;
    double originY; 
    int width; 
    int height; 
    double robot_radius_;
    ros::Subscriber occ_sub_;
};

#endif
