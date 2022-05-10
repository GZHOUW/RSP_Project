#include <turtlebot_planner/Planner.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(Planner, nav_core::BaseGlobalPlanner)

Planner::Planner(){
    cost_map_ros = NULL;
    initialized = false;
}

Planner::Planner(std::string name, costmap_2d::Costmap2DROS* m){
    initialize(name, m);
}

void Planner::initialize(std::string name, costmap_2d::Costmap2DROS* m){
    if (!initialized){
        cost_map_ros = m;
        cost_map = cost_map_ros->getCostmap();
        nh = ros::NodeHandle("~/turtlebot_planner");
        world_model = new base_local_planner::CostmapModel(*cost_map);

        nh.getParam("/move_base/dq", dq);
        nh.getParam("/move_base/dq_step", dq_step);
        nh.getParam("/move_base/goal_radius", goal_radius);
        nh.getParam("/move_base/max_iterations", max_iterations);

        map_width = cost_map->getSizeInCellsX();
        map_height = cost_map->getSizeInCellsY();

        // fill the obstacle vector
        for (unsigned int x = 0; x < map_width; x++){
            for (unsigned int y = 0; y < map_height; y++){
                int cost = cost_map->getCost(x, y);
                if (cost > 100){
                    // true means there is an obstacle
                    obstacles.push_back(true);
                }
                else{
                    obstacles.push_back(false);
                }
            }
        }
        initialized = true;
    }
}

bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path){
    path.clear();
    V.clear();

    x_start = start.pose.position.x;
    y_start = start.pose.position.y;

    x_goal = goal.pose.position.x;
    y_goal = goal.pose.position.y;
    
    // create start Vertex
    Vertex v_start = Vertex(x_start, y_start, 0, -1);
    // push into V list
    V.push_back(v_start);

    // check frame id
    if (goal.header.frame_id != cost_map_ros->getGlobalFrameID()){
        return false;
    }

    // Run RTT algorithm and find the path
    int goal_idx = findPath();
    path = generatePath(start, goal, goal_idx);
    return path.size() > 1;
}

int Planner::findPath(){
    for (int i = 0; i < max_iterations; i++){
        // generate a random state
        state q_rand = randomState();
        
        // Find the nearest node in V to q_rand
        int q_near_idx = nearestNeighbor(q_rand);
        state q_near; 
        q_near.first = V[q_near_idx].x;
        q_near.second = V[q_near_idx].y;

        state q_new = newState(q_near_idx, q_rand);

        // check if the edge {q_near, q_new} is in collision
        if (!isCollision(q_near, q_new)){
            // Create a Vertex for q_new
            int new_idx = V.size();
            Vertex v_new(q_new.first, q_new.second, new_idx, q_near_idx);
            V.push_back(v_new);

            if (isGoal(q_new)){
                return new_idx; // goal_idx found
            }
        }
    }
    return -1; // max iteration reached, no path found
}

state Planner::randomState(){
    // generate random floats within the map's boundary
    std::random_device rd;
    std::mt19937 gen(rd());

    float width_m = cost_map -> getSizeInMetersX();
    float height_m = cost_map -> getSizeInMetersY();

    std::uniform_real_distribution<> x_rand(-width_m, width_m);
    std::uniform_real_distribution<> y_rand(-height_m, height_m);

    state q_rand = {x_rand(gen), y_rand(gen)};

    return q_rand;
}

int Planner::nearestNeighbor(state q_rand){
    int q_near_idx;
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

float Planner::dist(state q1, state q2){
    float x1 = q1.first;
    float y1 = q1.second;
    float x2 = q2.first;
    float y2 = q2.second;
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

state Planner::newState(int q_near_idx, state q_rand){
    float x_near = V[q_near_idx].x;
    float y_near = V[q_near_idx].y;
    float x_rand = q_rand.first;
    float y_rand = q_rand.second;

    float x_diff = x_rand - x_near;
    float y_diff = y_rand - y_near;

    state q_near;
    q_near.first = x_near;
    q_near.second = y_near;
    float d = dist(q_near, q_rand);

    float x_dir = x_diff/d;
    float y_dir = y_diff/d;

    state q_new;
    q_new.first = x_dir*dq;
    q_new.second = y_dir*dq;

    return q_new;
}

bool Planner::isCollision(state q1, state q2){
    float x1 = q1.first;
    float y1 = q1.second;
    float x2 = q2.first;
    float y2 = q2.second;

    // check if q2 is free
    unsigned int x_map, y_map;
    cost_map->worldToMap(x2, y2, x_map, y_map);
    if (obstacles[y_map*map_height+x_map]){
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
        cost_map->worldToMap(x_cur, y_cur, x_map, y_map);
        if (obstacles[y_map*map_height+x_map]){
            return false;
        }
    }
    // no collision found, return true
    return true;
}

bool Planner::isGoal(state q){
    state q_goal;
    q_goal.first = x_goal;
    q_goal.second = y_goal;
    float d = dist(q, q_goal);
    return d < goal_radius;
}


std::vector<geometry_msgs::PoseStamped> Planner::generatePath(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal, int goal_idx){
    std::vector<geometry_msgs::PoseStamped> path;

    int i = goal_idx;
    while (i != -1){
        // get the ith vertex from V
        Vertex v = V[i];

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
    // reverse path
    std::reverse(path.begin(), path.end());
    return path;
}
