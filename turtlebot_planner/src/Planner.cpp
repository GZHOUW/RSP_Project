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

  bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path) {
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
    int goal_idx = findPath();
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

  int Planner::findPath() {
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
        if (isGoal(new_idx)){
          return new_idx; // goal_idx found
        }
      }
    }
    std::cout << "max iterations reached" << std::endl;
    return -1; // max iteration reached, no path found
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
      V.push_back(new_vertex);
      return true;
    }

    return false;
  }

  bool Planner::isFree(state q1, state q2) {
    float x1 = q1.first;
    float y1 = q1.second;
    float x2 = q2.first;
    float y2 = q2.second;

    // check if q2 is free
    unsigned int x_map, y_map;
    int map_size = obstacles.size();
    map->worldToMap(x2, y2, x_map, y_map);
    int idx = y_map * height + x_map;
    if (!obstacles[idx]){
      return false;
    }

    // check if the edge is free
    float x_diff = x2 - x1;
    float y_diff = y2 - y2;
    float d = dist(q1, q2);
    float x_dir = x_diff/d;
    float y_dir = y_diff/d;

    float x_cur = x1;
    float y_cur = y1;

    state cur = {x_cur, y_cur};

    while (dist(cur, q2) > dq_step){
      x_cur += x_dir * dq_step;
      y_cur += y_dir * dq_step;

      // check for collision 
      map->worldToMap(x_cur, y_cur, x_map, y_map);
      int idx = y_map * height + x_map;
      if (idx <0 || idx >= map_size){
        return false;
      }
      if (!obstacles[idx]){
        return false;
      }

      cur = {x_cur, y_cur};
    }
    // no collision found, return true
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

  std::vector<geometry_msgs::PoseStamped> Planner::generatePath(int goal_idx, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) {
      cur_iterations = 0;
      std::vector<geometry_msgs::PoseStamped> path;
      path.push_back(goal);

      int i = goal_idx;
      while (i != -1){
        // get the ith vertex from V
        turtlebot_planner::Vertex v = V[i];

        // convert v to a poseStamped
        geometry_msgs::PoseStamped p;
        p.pose.position.x = v.x;
        p.pose.position.y = v.y;
        p.pose.position.z = 0.0;
        p.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        path.push_back(p);
        // go to parent idx
        i = v.parent;
      }
      path.push_back(start);
      // reverse path
      std::reverse(path.begin(), path.end());
      return path;
  }
};
