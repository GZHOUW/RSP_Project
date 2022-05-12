#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <random>
#include "turtlebot_planner/Vertex.hpp"

typedef std::pair<float, float> state;

namespace turtlebot_planner {
class Planner : public nav_core::BaseGlobalPlanner {
 public:
     Planner();

     Planner(std::string, costmap_2d::Costmap2DROS*);

     void initialize(std::string, costmap_2d::Costmap2DROS*);

     bool makePlan(const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&, std::vector<geometry_msgs::PoseStamped>&);

     state randomState();

     int nearestNeighbor(state);

     float dist(state, state);

     bool newState(int, state);

     bool isGoal(int);

     std::vector<geometry_msgs::PoseStamped> generatePath(int, const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&);

     int findPath();

     bool isFree(state, state);

 private:
     ros::NodeHandle nh;

     std::vector<bool> obstacles;

     costmap_2d::Costmap2DROS* map_ros;

     costmap_2d::Costmap2D* map;

     int max_iterations, cur_iterations;

     base_local_planner::WorldModel* model;

     bool initialized;

     float goal_radius;

     float dq, dq_step;

     float x_start, y_start, x_goal, y_goal;

     unsigned int width;

     unsigned int height;

     std::vector<turtlebot_planner::Vertex> V;
};
} 
