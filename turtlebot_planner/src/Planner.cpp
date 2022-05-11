#include <pluginlib/class_list_macros.h>
#include "turtlebot_planner/Planner.hpp"
#include "turtlebot_planner/Vertex.hpp"

// Register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(turtlebot_planner::Planner, nav_core::BaseGlobalPlanner)

namespace turtlebot_planner {
  Planner::Planner()
    : costmap_ros_(nullptr), initialized_(false) { }

  Planner::Planner(std::string name,
                         costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(costmap_ros) {
    initialize(name, costmap_ros);
  }

  void Planner::initialize(std::string name,
                              costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();

      // Initialize node handle
      ros::NodeHandle node("~/turtlebot_planner");
      node_handle_ = node;
      world_model_ = new base_local_planner::CostmapModel(*costmap_);

      node_handle_.getParam("/move_base/step_size_", step_size_);
      node_handle_.getParam("/move_base/delta_", delta_);
      node_handle_.getParam("/move_base/goal_radius_", goal_radius_);
      node_handle_.getParam("/move_base/max_iterations_", max_iterations_);
      ROS_INFO("Step size: %.2f, goal radius: %.2f, delta: %.2f, max "
               "iterations: %d", step_size_, goal_radius_, delta_,
               max_iterations_);
      current_iterations_ = 0;

      // Get obstacles in the costmap
      map_width_cells_ = costmap_-> getSizeInCellsX();
      map_height_cells_ = costmap_-> getSizeInCellsY();

      for (unsigned int iy = 0; iy < map_height_cells_; iy++) {
        for (unsigned int ix = 0; ix < map_width_cells_; ix++) {
          unsigned char cost = static_cast<int>(costmap_->getCost(ix, iy));
          if (cost >= 115)
            obstacle_map_.push_back(false);
          else
            obstacle_map_.push_back(true);
        }
      }

      // Display info message
      ROS_INFO("RRT planner initialized successfully.");
      initialized_ = true;
    } else {
      ROS_WARN("RRT planner has already been initialized.");
    }
  }

  bool Planner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan) {
    // Check if we've initialized, if not error
    if (!initialized_) {
      ROS_ERROR("RRT planner has not been initialized, please call "
                "initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Start: %.2f, %.2f", start.pose.position.x,
              start.pose.position.y);
    ROS_DEBUG("Goal: %.2f, %.2f", goal.pose.position.x,
              goal.pose.position.y);

    // reset path, iterations, vertex tree
    plan.clear();
    current_iterations_ = 0;
    ROS_INFO("Current iterations reset to %d.", current_iterations_);
    vertex_list_.clear();

    // reset origin and goal
    x_origin_ = start.pose.position.x;
    y_origin_ = start.pose.position.y;
    x_goal_ = goal.pose.position.x;
    y_goal_ = goal.pose.position.y;

    // Initialize root node
    turtlebot_planner::Vertex root(x_origin_, y_origin_, 0, -1);
    vertex_list_.push_back(root);

    // Make sure that the goal header frame is correct
    // Goals are set within rviz
    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      ROS_ERROR("This planner will only accept goals in the %s frame,"
                "the goal was sent to the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(),
                goal.header.frame_id.c_str());
      return false;
    }

    // Have the Planner calculate the path. Returns the index of the node
    // that reaches the goal
    ROS_DEBUG("Going into FindPath");
    int goal_index = Planner::FindPath(start, goal);

    // Rebuild the plan from the goal_index to the start using the
    // desired plan message format
    ROS_DEBUG("Going into BuildPlan");
    plan = Planner::BuildPlan(goal_index, start, goal);

    if (plan.size() > 1) {
      ROS_INFO("A path was found.");
      return true;
    } else {
      ROS_WARN("No path was found.");
      return false;
    }
  }

  std::pair<float, float> Planner::GetRandomPoint() {
    // generate random x and y coords within map bounds
    std::pair<float, float> random_point;
    std::random_device rd;
    std::mt19937 gen(rd());
    float map_width = costmap_->getSizeInMetersX();
    float map_height = costmap_->getSizeInMetersY();
    std::uniform_real_distribution<> x(-map_width, map_width);
    std::uniform_real_distribution<> y(-map_height, map_height);

    random_point.first = x(gen);
    random_point.second = y(gen);

    return random_point;
  }

  int Planner::FindPath(const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal) {
    bool done = false;
    int goal_index = -1;
    current_iterations_ = 0;

    // Run until we either find the goal or reach the max iterations
    while (!done && current_iterations_ < max_iterations_) {
      ROS_DEBUG("Finding the path.");

      // get a random point on the map
      std::pair<float, float> random_point = Planner::GetRandomPoint();

      ROS_DEBUG("Random point: %.2f, %.2f", random_point.first,
                random_point.second);

      // find the closest known vertex to that point
      int closest_vertex = Planner::GetClosestVertex(random_point);
      ROS_DEBUG("Closest point %.5f, %.5f, index: %d.",
                vertex_list_.at(closest_vertex).get_location().first,
                vertex_list_.at(closest_vertex).get_location().second,
                vertex_list_.at(closest_vertex).get_index());

      // try to move from the closest known vertex towards the random point
      if (Planner::MoveTowardsPoint(closest_vertex, random_point)) {
        ROS_DEBUG("Moved, closest vertex: %d", closest_vertex);

        // If successful increase our iterations
        current_iterations_++;

        // check if we've reached our goal
        int new_vertex = vertex_list_.back().get_index();
        done = ReachedGoal(new_vertex);

        if (done) {
          goal_index = new_vertex;
        }
      }

      if (current_iterations_ == max_iterations_)
        ROS_INFO("Max iterations reached, no plan found.");
    }
    return goal_index;
  }

  int Planner::GetClosestVertex(std::pair<float, float> random_point) {
    int closest = -1;

    // closest_distance will keep track of the closest distance we find
    float closest_distance = std::numeric_limits<float>::infinity();

    // current_distance will keep track of the distance of the current
    float current_distance = std::numeric_limits<float>::infinity();

    // iterate through the vertex list to find the closest
    for (turtlebot_planner::Vertex v : vertex_list_) {
      current_distance = GetDistance(v.get_location(), random_point);

      // If the current distance is closer than what was previously
      // saved, update
      if (current_distance < closest_distance) {
        ROS_DEBUG("Closest distance: %.5f, vertex: %d.",
                  current_distance, v.get_index());
        closest = v.get_index();
        closest_distance = current_distance;
      }
    }
    return closest;
  }

  float Planner::GetDistance(std::pair<float, float> start_point,
                                 std::pair<float, float> end_point) {
    // coords for our first point
    float x1 = start_point.first;
    float y1 = start_point.second;

    // coords for our second point
    float x2 = end_point.first;
    float y2 = end_point.second;

    // euclidean distance
    float distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));

    ROS_DEBUG("Distance: %.5f", distance);
    return distance;
  }

  bool Planner::MoveTowardsPoint(int closest_vertex,
                                    std::pair<float, float> random_point) {
    ROS_DEBUG("In MoveTowardsPoint");
    float x_closest = vertex_list_.at(closest_vertex).get_location().first;
    float y_closest = vertex_list_.at(closest_vertex).get_location().second;
    float x_random = random_point.first;
    float y_random = random_point.second;

    // get the angle between the random point and our closest point (in rads)
    float theta = atan2(y_random - y_closest, x_random - x_closest);

    // proposed new point step_size_ from our closest vertex towards
    // the random point
    float new_x = x_closest + step_size_ * cos(theta);
    float new_y = y_closest + step_size_ * sin(theta);

    std::pair<float, float> proposed_point(new_x, new_y);
    std::pair<float, float> closest_point(x_closest, y_closest);

    // Check if the path between closest_vertex and the new point
    // is safe
    if (IsSafe(closest_point, proposed_point)) {
      // If safe, add new Vertex to the back of vertex_list_
      turtlebot_planner::Vertex new_vertex(new_x, new_y, vertex_list_.size(),
                                       closest_vertex);
      ROS_DEBUG("Added new vertex at: %.5f, %.5f, index: %d",
               new_x, new_y, new_vertex.get_index());
      addVertex(new_vertex);

      // Return true, that we moved towards the proposed point
      return true;
    }
    // Return false, move not made
    return false;
  }

  bool Planner::IsSafe(std::pair<float, float> start_point,
                           std::pair<float, float> end_point) {
    unsigned int map_x, map_y;

    // first check to make sure the end point is safe. Saves us processing
    // time if somebody wants to jump into the middle of an obstacle
    costmap_->worldToMap(end_point.first, end_point.second, map_x, map_y);
    int idx = map_y * map_height_cells_ + map_x;
    int map_size = obstacle_map_.size();
    if (idx <0 || idx >= map_size){
      return false;
    }
    if (!obstacle_map_.at(map_y * map_height_cells_ + map_x))
        return false;

    // check the path at intervals of delta for collision
    float theta = atan2(end_point.second - start_point.second,
                        end_point.first - start_point.first);
    float current_x = start_point.first;
    float current_y = start_point.second;

    ROS_DEBUG("Testing proposed point %.5f, %.5f.", end_point.first,
                                                    end_point.second);

    while (GetDistance(std::pair<float, float>(current_x, current_y),
                       end_point) > delta_) {
      // increment towards end point
      current_x += delta_ * cos(theta);
      current_y += delta_ * sin(theta);

      // convert world coords to map coords
      costmap_->worldToMap(current_x, current_y, map_x, map_y);

      // check for collision
      int idx = map_y * map_height_cells_ + map_x;
      if (idx <0 || idx >= map_size){
        return false;
      }
      if (!obstacle_map_.at(idx))
        return false;
    }
    return true;
  }

  bool Planner::ReachedGoal(int new_vertex) {
    ROS_DEBUG("In ReachedGoal, vertex index: %d.", new_vertex);

    // save our goal and current location as pairs
    std::pair<float, float> goal(x_goal_, y_goal_);
    std::pair<float, float> current_location;
    current_location.first =
      vertex_list_.at(new_vertex).get_location().first;
    current_location.second =
      vertex_list_.at(new_vertex).get_location().second;

    ROS_DEBUG("cx: %.5f, cy: %.5f, gx: %.5f, gy: %.5f",
              current_location.first,
              current_location.second,
              goal.first,
              goal.second);

    // Check distance between current point and goal, if distance is less
    // than goal_radius_ return true, otherwise return false
    float distance = GetDistance(current_location, goal);
    ROS_DEBUG("Distance to goal: %.5f", distance);

    if (distance <= goal_radius_)
      return true;
    else
      return false;
  }

  std::vector<geometry_msgs::PoseStamped>
    Planner::BuildPlan(int goal_index,
                           const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal) {
      ROS_INFO("Building the plan.");

      // reset our current iterations
      current_iterations_ = 0;

      // The plan we'll be adding to and returning
      std::vector<geometry_msgs::PoseStamped> plan;

      // no plan found
      if (goal_index == -1)
        return plan;

      // The list of vertex indices we pass through to get to the goal
      std::deque<int> index_path;
      int current_index = goal_index;
      while (current_index > 0) {
        index_path.push_front(current_index);
        current_index = vertex_list_.at(current_index).get_parent();
      }
      index_path.push_front(0);

      // build the plan back up in PoseStamped messages
      for (int i : index_path) {
        if (i == 0) {
          plan.push_back(start);
        } else {
          geometry_msgs::PoseStamped pos;

          pos.pose.position.x = vertex_list_.at(i).get_location().first;
          pos.pose.position.y = vertex_list_.at(i).get_location().second;
          pos.pose.position.z = 0.0;

          pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
          plan.push_back(pos);
        }
      }
      plan.push_back(goal);
      unsigned int map_x, map_y;
      for (geometry_msgs::PoseStamped p : plan) {
        costmap_->worldToMap(p.pose.position.x, p.pose.position.y,
                             map_x, map_y);
        ROS_INFO("x: %.2f (%d), y: %.2f (%d)", p.pose.position.x,
                                               map_x,
                                               p.pose.position.y,
                                               map_y);
      }
      return plan;
  }
};  // namespace turtlebot_planner
