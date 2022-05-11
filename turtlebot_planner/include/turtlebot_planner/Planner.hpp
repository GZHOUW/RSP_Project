
/** include ROS libraries **/
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

/** for global path planner interface **/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** include standard libraries **/
#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <boost/random.hpp>

/** Local includes **/
#include "turtlebot_planner/Vertex.hpp"

typedef std::pair<float, float> state;

namespace turtlebot_planner {
class Planner : public nav_core::BaseGlobalPlanner {
 public:
     Planner();

     Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

     void initialize(std::string name,
                     costmap_2d::Costmap2DROS* costmap_ros);

     bool makePlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal,
                   std::vector<geometry_msgs::PoseStamped>& path);

    std::vector<bool> getObstacleMap() {
      return obstacles;
    }

    std::vector<turtlebot_planner::Vertex> getVertexTree() {
      return V;
    }

     state randomState();

     int nearestNeighbor(state random_point);

    void addVertex(turtlebot_planner::Vertex new_vertex) {
      V.push_back(new_vertex);
    }

     float dist(state start_point,
                       state end_point);

     bool newState(int closest_vertex,
                           state random_point);

     bool isGoal(int new_vertex);

     std::vector<geometry_msgs::PoseStamped>
       generatePath(int goal_index,
                 const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal);

     int findPath(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal);

     bool isFree(state start_point,
                 state end_point);

 private:
     ros::NodeHandle nh;

     std::vector<bool> obstacles;

     costmap_2d::Costmap2DROS* map_ros;

     costmap_2d::Costmap2D* map;

     int max_iterations;

     int cur_iterations;

     base_local_planner::WorldModel* model;

     bool initialized;

     float goal_radius;

     float dq;

     float dq_step;

     float x_start;

     float y_start;

     float x_goal;

     float y_goal;

     unsigned int width;

     unsigned int height;

     std::vector<turtlebot_planner::Vertex> V;
};
} 
