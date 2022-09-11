/**
* 2022.07.28 Last Edited by Evin Hsu
*/

#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include<random>
#include<utility>
#include<time.h>
#include<random>
#include<ros/ros.h>
#include<angles/angles.h>
#include<tf/tf.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Path.h>
#include<visualization_msgs/Marker.h>

#include<rrt_planner/node.h>
#include<rrt_planner/grid_map.h>
#include<rrt_planner/StartGoalPath.h>
#include<rrt_planner/RandomGetPath.h>
#include<rrt_planner/OccMapInfo.h>
#include<rrt_planner/MapOcc.h>

enum ConnectMode{
  FAIL = 0,
  ONE_ARRIVED = 1,
  TWO_ARRIVED = 2,
  ONE_MEET_TWO = 3,
  TWO_MEET_ONE = 4,
};


class BRRTStarPlanner
{
public:

    BRRTStarPlanner();
    ~BRRTStarPlanner();
    ros::NodeHandle nh_;

private:
    
    void initialize();
    void marker_reset();
    void markerTreeInitial(visualization_msgs::Marker* mk);
    
    bool get_map_info(rrt_planner::OccMapInfo::Request &req, 
                      rrt_planner::OccMapInfo::Response &res);
    bool start_goal_path(rrt_planner::StartGoalPath::Request &req, 
                         rrt_planner::StartGoalPath::Response &res); 
    bool random_get_path( rrt_planner::RandomGetPath::Request &req,   
                          rrt_planner::RandomGetPath::Response &res);
    bool get_occ_value( rrt_planner::MapOcc::Request &req,   
                          rrt_planner::MapOcc::Response &res);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan, 
                  std::vector<geometry_msgs::PoseStamped>& inters);
    bool ExtendTree( std::pair<double, double>& q_rand, 
                    std::vector<Node>& tree_node, 
                    visualization_msgs::Marker& marker_tree,
                    const geometry_msgs::PoseStamped& goal,
                    int goal_id,
                    bool first_tree);
    std::pair<double, double> SampleFree();
    int NearestNode( double x, double y , const std::vector<Node>& tree_node);
    std::pair<double, double> SteerNode( double nearX, double nearY, double randX, double randY );
    void addNewNode(int parent_id,
                    double x,
                    double y,
                    std::vector<Node>& tree_node, 
                    visualization_msgs::Marker& marker_tree,
                    int goal_id);
    Node chooseParent(Node new_node, std::vector<Node>& tree_nodes, double near_radius, int goal_id);
    void rewireTree(Node new_node, std::vector<Node>& tree_nodes, double near_radius);
    bool isArrivedGoal(const Node& node_new, const geometry_msgs::PoseStamped& goal);
    bool isConnect(Node node_new,
                    std::vector<Node>& this_tree,
                    const std::vector<Node> another_tree,
                    visualization_msgs::Marker& this_marker_tree,
                    int& this_goal_id,
                    int& another_goal_id,
                    double tolerance);
    double getRandomValue(double min, double max);
    bool getFinalPlan(const std::vector<Node> tree_node_1, 
                      const std::vector<Node> tree_node_2, 
                      int goal_id_1,
                      int goal_id_2,
                      int connect_mode,
                      std::vector<geometry_msgs::PoseStamped>& plan,
                      std::vector<geometry_msgs::PoseStamped>& inters, 
                      const geometry_msgs::PoseStamped& start, 
                      const geometry_msgs::PoseStamped& goal,
                      bool final);
    bool getInterPoints(const std::vector<std::pair<double, double>>& plan, 
                          std::vector<std::pair<double, double>>& inters);
    void nav_plan(const std::vector<std::pair<double, double>> poses,
                  std::vector<geometry_msgs::PoseStamped>& plan,
                  const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal,
                  bool final);
    void navInterPoints( const std::vector<std::pair<double, double>> poses, 
                      std::vector<geometry_msgs::PoseStamped>& plan,
                      const geometry_msgs::PoseStamped& goal,
                      bool final);
    void optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan);
    bool PlanOnlyStartGoal( std::vector<geometry_msgs::PoseStamped>& plan,
                            std::vector<geometry_msgs::PoseStamped>& inters, 
                            const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal);
    
    std::vector<Node>tree_1_, tree_2_, tree_combind_;
    ros::Publisher marker_tree_pub_;
    ros::Publisher marker_start_goal_pub_;
    ros::Publisher marker_path_pub_; 
    ros::Publisher marker_inter_pub_;
    ros::ServiceServer start_goal_srv_;
    ros::ServiceServer random_path_srv_;
    ros::ServiceServer map_info_srv_; 
    ros::ServiceServer occ_val_srv_;
    GridMap grid_map_;
    visualization_msgs::Marker mk_tree_1_;
    visualization_msgs::Marker mk_tree_2_;
    visualization_msgs::Marker mk_start_;
    visualization_msgs::Marker mk_goal_;
    visualization_msgs::Marker mk_path_;
    visualization_msgs::Marker mk_inter_;
    bool mk_frame_;
    double mk_tree_z;
    double mk_point_z;
    double mk_path_z;
    double mk_inter_z;
    double tolerance_;
    double radius_;
    double epsilon_max_;
    double epsilon_min_;
    int max_node_num_;
    double robot_radius_;
    double inter_radius_;
    double timeout_sec_;
    int vis_step_;
    bool is_vis_;
    bool debug_;
    bool debug_map_;
    int goal_id_1_, goal_id_2_; 
    int connect_mode_;
    bool is_arrived_;
    bool is_first_path_;
    int count_; 
    double resolution;
    double originX; 
    double originY; 
    int width;
    int height; 
};

#endif
