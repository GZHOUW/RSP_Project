#ifndef INCLUDE_turtlebot_planner_turtlebot_planner_H_
#define INCLUDE_turtlebot_planner_turtlebot_planner_H_

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

namespace turtlebot_planner {
class Planner : public nav_core::BaseGlobalPlanner {
 public:
     Planner();

     Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

     void initialize(std::string name,
                     costmap_2d::Costmap2DROS* costmap_ros);

     bool makePlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal,
                   std::vector<geometry_msgs::PoseStamped>& plan);

    std::vector<bool> getObstacleMap() {
      return obstacles;
    }

    std::vector<turtlebot_planner::Vertex> getVertexTree() {
      return V;
    }

     std::pair<float, float> GetRandomPoint();

     int GetClosestVertex(std::pair<float, float> random_point);

    void addVertex(turtlebot_planner::Vertex new_vertex) {
      V.push_back(new_vertex);
    }

     float GetDistance(std::pair<float, float> start_point,
                       std::pair<float, float> end_point);

     bool MoveTowardsPoint(int closest_vertex,
                           std::pair<float, float> random_point);

     bool ReachedGoal(int new_vertex);

     std::vector<geometry_msgs::PoseStamped>
       BuildPlan(int goal_index,
                 const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal);

     int FindPath(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal);

     bool IsSafe(std::pair<float, float> start_point,
                 std::pair<float, float> end_point);

 private:
     ros::NodeHandle nh;

     std::vector<bool> obstacles;

     costmap_2d::Costmap2DROS* map_ros;

     costmap_2d::Costmap2D* map;

     int max_iterations;

     int current_iterations_;

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
}  // namespace turtlebot_planner
#endif  // INCLUDE_turtlebot_planner_turtlebot_planner_H_
