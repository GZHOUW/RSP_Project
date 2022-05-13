#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

class random_move{
private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;

    nav_msgs::OccupancyGrid my_local_map;
    nav_msgs::Odometry my_odom;
    move_base_msgs::MoveBaseGoal goal_msg;

    float initial_x, initial_y, initial_theta;
    float dx,dy,theta;
    float step_length;

public:

    random_move(ros::NodeHandle& nh);
    void odom_callback(const nav_msgs::OdometryPtr odom);
    void map_callback(const nav_msgs::OccupancyGridConstPtr local_map);
    void get_next_pos(float& dx,float& dy,float& theta);
    move_base_msgs::MoveBaseGoal run();
    int step_size=10;
};
