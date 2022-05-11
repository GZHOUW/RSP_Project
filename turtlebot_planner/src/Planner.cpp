#include <pluginlib/class_list_macros.h>
#include "turtlebot_planner/Planner.hpp"
#include "turtlebot_planner/Vertex.hpp"

// Register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(turtlebot_planner::Planner, nav_core::BaseGlobalPlanner)

namespace turtlebot_planner {
  Planner::Planner(){
    map_ros = nullptr;
    initialized = false;
  }

  Planner::Planner(std::string name, costmap_2d::Costmap2DROS* m){
    map_ros = m;
    initialize(name, m);
  }

  void Planner::initialize(std::string name, costmap_2d::Costmap2DROS* m) {
    /*
    if (!initialized) {
      // Initialize map
      map_ros = m;
      map = m->getCostmap();

      // Initialize node handle
      ros::NodeHandle node("~/turtlebot_planner");
      nh = node;
      model = new base_local_planner::CostmapModel(*map);

      nh.getParam("/move_base/dq", dq);
      nh.getParam("/move_base/dq_step", dq_step);
      nh.getParam("/move_base/goal_radius", goal_radius);
      nh.getParam("/move_base/max_iterations", max_iterations);
      ROS_INFO("Step size: %.2f, goal radius: %.2f, dq_step: %.2f, max "
               "iterations: %d", dq, goal_radius, dq_step,
               max_iterations);
      current_iterations_ = 0;

      // Get obstacles in the costmap
      width = map-> getSizeInCellsX();
      height = map-> getSizeInCellsY();

      for (unsigned int iy = 0; iy < height; iy++) {
        for (unsigned int ix = 0; ix < width; ix++) {
          unsigned char cost = static_cast<int>(map->getCost(ix, iy));
          if (cost >= 115)
            obstacles.push_back(false);
          else
            obstacles.push_back(true);
        }
      }

      // Display info message
      ROS_INFO("RRT planner initialized successfully.");
      initialized = true;
    } else {
      ROS_WARN("RRT planner has already been initialized.");
    }
    */
    if (!initialized){
      map_ros = m;
      map = map_ros->getCostmap();
      nh = ros::NodeHandle("~/turtlebot_planner");
      model = new base_local_planner::CostmapModel(*map);

      nh.getParam("/move_base/dq", dq);
      nh.getParam("/move_base/dq_step", dq_step);
      nh.getParam("/move_base/goal_radius", goal_radius);
      nh.getParam("/move_base/max_iterations", max_iterations);

      width = map->getSizeInCellsX();
      height = map->getSizeInCellsY();

      // fill the obstacle vector
      for (unsigned int x = 0; x < width; x++){
        for (unsigned int y = 0; y < height; y++){
          int cost = map->getCost(x, y);
          if (cost >= 115){
            // true means there is an obstacle
            obstacles.push_back(false);
          }
          else{
            obstacles.push_back(true);
          }
        }
      }
      initialized = true;
    }
  }

  bool Planner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized) {
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
    V.clear();

    // reset origin and goal
    x_start = start.pose.position.x;
    y_start = start.pose.position.y;
    x_goal = goal.pose.position.x;
    y_goal = goal.pose.position.y;
    // Initialize root node
    turtlebot_planner::Vertex root(x_start, y_start, 0, -1);
    V.push_back(root);

    // Make sure that the goal header frame is correct
    // Goals are set within rviz
    if (goal.header.frame_id != map_ros->getGlobalFrameID()) {
      ROS_ERROR("This planner will only accept goals in the %s frame,"
                "the goal was sent to the %s frame.",
                map_ros->getGlobalFrameID().c_str(),
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
    float width_m = map->getSizeInMetersX();
    float height_m = map->getSizeInMetersY();
    std::uniform_real_distribution<> x(-width_m, width_m);
    std::uniform_real_distribution<> y(-height_m, height_m);

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
    while (!done && current_iterations_ < max_iterations) {
      ROS_DEBUG("Finding the path.");

      // get a random point on the map
      std::pair<float, float> random_point = Planner::GetRandomPoint();

      ROS_DEBUG("Random point: %.2f, %.2f", random_point.first,
                random_point.second);

      // find the closest known vertex to that point
      int closest_vertex = Planner::GetClosestVertex(random_point);
      ROS_DEBUG("Closest point %.5f, %.5f, index: %d.",
                V.at(closest_vertex).get_location().first,
                V.at(closest_vertex).get_location().second,
                V.at(closest_vertex).get_index());

      // try to move from the closest known vertex towards the random point
      if (Planner::MoveTowardsPoint(closest_vertex, random_point)) {
        ROS_DEBUG("Moved, closest vertex: %d", closest_vertex);

        // If successful increase our iterations
        current_iterations_++;

        // check if we've reached our goal
        int new_vertex = V.back().get_index();
        done = ReachedGoal(new_vertex);

        if (done) {
          goal_index = new_vertex;
        }
      }

      if (current_iterations_ == max_iterations)
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
    for (turtlebot_planner::Vertex v : V) {
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
    float x_closest = V.at(closest_vertex).get_location().first;
    float y_closest = V.at(closest_vertex).get_location().second;
    float x_random = random_point.first;
    float y_random = random_point.second;

    // get the angle between the random point and our closest point (in rads)
    float theta = atan2(y_random - y_closest, x_random - x_closest);

    // proposed new point dq from our closest vertex towards
    // the random point
    float new_x = x_closest + dq * cos(theta);
    float new_y = y_closest + dq * sin(theta);

    std::pair<float, float> proposed_point(new_x, new_y);
    std::pair<float, float> closest_point(x_closest, y_closest);

    // Check if the path between closest_vertex and the new point
    // is safe
    if (IsSafe(closest_point, proposed_point)) {
      // If safe, add new Vertex to the back of V
      turtlebot_planner::Vertex new_vertex(new_x, new_y, V.size(),
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
    map->worldToMap(end_point.first, end_point.second, map_x, map_y);
    int idx = map_y * height + map_x;
    int map_size = obstacles.size();
    if (idx <0 || idx >= map_size){
      return false;
    }
    if (!obstacles.at(map_y * height + map_x))
        return false;

    // check the path at intervals of dq_step for collision
    float theta = atan2(end_point.second - start_point.second,
                        end_point.first - start_point.first);
    float current_x = start_point.first;
    float current_y = start_point.second;

    ROS_DEBUG("Testing proposed point %.5f, %.5f.", end_point.first,
                                                    end_point.second);

    while (GetDistance(std::pair<float, float>(current_x, current_y), end_point) > dq_step) {
      // increment towards end point
      current_x += dq_step * cos(theta);
      current_y += dq_step * sin(theta);

      // convert world coords to map coords
      map->worldToMap(current_x, current_y, map_x, map_y);

      // check for collision
      int idx = map_y * height + map_x;
      if (idx <0 || idx >= map_size){
        return false;
      }
      if (!obstacles.at(idx))
        return false;
    }
    return true;
  }

  bool Planner::ReachedGoal(int new_vertex) {
    ROS_DEBUG("In ReachedGoal, vertex index: %d.", new_vertex);

    // save our goal and current location as pairs
    std::pair<float, float> goal(x_goal, y_goal);
    std::pair<float, float> current_location;
    current_location.first =
      V.at(new_vertex).get_location().first;
    current_location.second =
      V.at(new_vertex).get_location().second;

    ROS_DEBUG("cx: %.5f, cy: %.5f, gx: %.5f, gy: %.5f",
              current_location.first,
              current_location.second,
              goal.first,
              goal.second);

    // Check distance between current point and goal, if distance is less
    // than goal_radius return true, otherwise return false
    float distance = GetDistance(current_location, goal);
    ROS_DEBUG("Distance to goal: %.5f", distance);

    if (distance <= goal_radius)
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
        current_index = V.at(current_index).get_parent();
      }
      index_path.push_front(0);

      // build the plan back up in PoseStamped messages
      for (int i : index_path) {
        if (i == 0) {
          plan.push_back(start);
        } else {
          geometry_msgs::PoseStamped pos;

          pos.pose.position.x = V.at(i).get_location().first;
          pos.pose.position.y = V.at(i).get_location().second;
          pos.pose.position.z = 0.0;

          pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
          plan.push_back(pos);
        }
      }
      plan.push_back(goal);
      unsigned int map_x, map_y;
      for (geometry_msgs::PoseStamped p : plan) {
        map->worldToMap(p.pose.position.x, p.pose.position.y,
                             map_x, map_y);
        ROS_INFO("x: %.2f (%d), y: %.2f (%d)", p.pose.position.x,
                                               map_x,
                                               p.pose.position.y,
                                               map_y);
      }
      return plan;
  }
};  // namespace turtlebot_planner
