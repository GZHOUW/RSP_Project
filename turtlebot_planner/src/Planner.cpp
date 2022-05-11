#include <pluginlib/class_list_macros.h>
#include "turtlebot_planner/Planner.hpp"

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
                            std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized) {
      return false;
    }
    path.clear();
    V.clear();
    cur_iterations = 0;
    x_start = start.pose.position.x;
    y_start = start.pose.position.y;

    x_goal = goal.pose.position.x;
    y_goal = goal.pose.position.y;
    std::cout << "searching for path, start pose = " <<x_start<<", "<<y_start <<
    " goal pose = "<< x_goal <<", "<<y_goal <<std::endl;
    // create start Vertex
    turtlebot_planner::Vertex v_start(x_start, y_start, 0, -1);
    // push into V list
    V.push_back(v_start);

    // check frame id
    if (goal.header.frame_id != map_ros->getGlobalFrameID()){
      std::cout << "frame id incorrect" << std::endl;
      return false;
    }

    // Run RTT algorithm and find the path
    int goal_idx = findPath(start, goal);
    path = generatePath(goal_idx, start, goal);
    if (path.size()> 1){
      std::cout << "path found, printing the path" << std::endl;
      for (auto p: path){
        std::cout << p.pose.position.x <<' ' <<p.pose.position.y <<std::endl;
      }
    }
    else{
      std::cout << "path NOT found" << std::endl;
    }
    return path.size() > 1;
  }

  state Planner::randomState() {
    // generate random x and y coords within map bounds
    
    std::random_device rd;
    std::mt19937 gen(rd());

    float width_m = map->getSizeInMetersX();
    float height_m = map->getSizeInMetersY();

    std::uniform_real_distribution<> x(-width_m, width_m);
    std::uniform_real_distribution<> y(-height_m, height_m);

    state q_rand = {x(gen), y(gen)};

    return q_rand;
  }

  int Planner::findPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) {
    cur_iterations = 0;

    while (cur_iterations < max_iterations){
      
      // generate a random state
      state q_rand = randomState();
      
      // Find the nearest node in V to q_rand
      int q_near_idx = nearestNeighbor(q_rand);
      //state q_near = {V[q_near_idx].x, V[q_near_idx].y};
      if (newState(q_near_idx, q_rand)){
        cur_iterations++;
        // Create a Vertex for q_new
        int new_idx = V.back().idx;
        //turtlebot_planner::Vertex v_new(q_new.first, q_new.second, new_idx, q_near_idx);
        //V.push_back(v_new);

        if (isGoal(new_idx)){
          return new_idx; // goal_idx found
        }
      }
    }
    std::cout << "max iterations reached" << std::endl;
    return -1; // max iteration reached, no path found
  }

  int Planner::nearestNeighbor(state q_rand) {
    int q_near_idx = -1;
    float d_min = FLT_MAX;
    
    for (auto v : V){
      state q_v = {v.x, v.y};
      float d = dist(q_rand, q_v);
      if (d < d_min){
        d_min = d;
        q_near_idx = v.idx;
      }
    }
    return q_near_idx;
  }

  float Planner::dist(state q1, state q2) {
    float x1 = q1.first;
    float y1 = q1.second;
    float x2 = q2.first;
    float y2 = q2.second;
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
  }

  bool Planner::newState(int q_near_idx, state q_rand) {
    float x_near = V[q_near_idx].x;
    float y_near = V[q_near_idx].y;
    state q_near = {x_near, y_near};
    float x_rand = q_rand.first;
    float y_rand = q_rand.second;

    float x_diff = x_rand - x_near;
    float y_diff = y_rand - y_near;

    float d = dist(q_near, q_rand);

    float x_new = (x_diff/d)*dq;
    float y_new = (y_diff/d)*dq;

    state q_new = {x_new, y_new};
   

    if (isFree(q_near, q_new)) {
      turtlebot_planner::Vertex new_vertex(x_new, y_new, V.size(), q_near_idx);
      addVertex(new_vertex);
      return true;
    }

    return false;
  }

  bool Planner::isFree(state start_point,
                           state end_point) {
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

    while (dist(state(current_x, current_y), end_point) > dq_step) {
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

  bool Planner::isGoal(int q_new_idx) {
    state q_goal(x_goal, y_goal);
    state q_new;
    q_new.first = V[q_new_idx].x;
    q_new.second = V[q_new_idx].y;

    float d = dist(q_new, q_goal);
    return d < goal_radius;
  }

  std::vector<geometry_msgs::PoseStamped> Planner::generatePath(int goal_index,
                           const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal) {
      ROS_INFO("Building the path.");

      // reset our current iterations
      cur_iterations = 0;
      std::vector<geometry_msgs::PoseStamped> path;

      // no path found
      if (goal_index == -1)
        return path;

      // The list of vertex indices we pass through to get to the goal
      std::deque<int> index_path;
      int current_index = goal_index;
      while (current_index > 0) {
        index_path.push_front(current_index);
        current_index = V[current_index].parent;
      }
      index_path.push_front(0);

      // build the path back up in PoseStamped messages
      for (int i : index_path) {
        if (i == 0) {
          path.push_back(start);
        } else {
          geometry_msgs::PoseStamped pos;

          pos.pose.position.x = V[i].x;
          pos.pose.position.y = V[i].y;
          pos.pose.position.z = 0.0;

          pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
          path.push_back(pos);
        }
      }
      path.push_back(goal);
      unsigned int map_x, map_y;
      for (geometry_msgs::PoseStamped p : path) {
        map->worldToMap(p.pose.position.x, p.pose.position.y, map_x, map_y);
      }
      return path;
  }
};
