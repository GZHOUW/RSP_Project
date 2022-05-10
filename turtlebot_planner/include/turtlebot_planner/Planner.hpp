#include <ros/ros.h>
#include <turtlebot_planner/Vertex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <utility>
typedef std::pair<float, float> state;

class Planner : public nav_core::BaseGlobalPlanner{
public:

    Planner();

    Planner(std::string name, costmap_2d::Costmap2DROS* m);

    // override base class
    void initialize(std::string name, costmap_2d::Costmap2DROS* m);

    // override base class
    bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& path);
    
    int findPath();

    // false means obstacle, true means free
    std::vector<bool> obstacles;
    state randomState();

    int nearestNeighbor(state q);

    // move a small step delta from q_near to q_rand
    state newState(int q_near_idx, state q_rand);

    float dist(state q1, state q2);

    bool isCollision(state q1, state q2);

    bool isGoal(state q);

    std::vector<geometry_msgs::PoseStamped> generatePath(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,int goal_idx);


private:
    

    ros::NodeHandle nh;

    costmap_2d::Costmap2DROS* cost_map_ros;

    costmap_2d::Costmap2D* cost_map;

    int max_iterations;

    base_local_planner::WorldModel* world_model;

    float dq_step;

    float dq;

    float goal_radius; // distance close to the goal

    float x_start;

    float y_start;

    std::vector<Vertex> V;

    float x_goal;

    float y_goal;

    bool initialized;

    unsigned int map_width;

    unsigned int map_height;
};