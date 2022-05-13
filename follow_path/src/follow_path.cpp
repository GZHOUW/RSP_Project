#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <iostream>

int main(int argc, char** argv){
	
    ros::init(argc, argv, "follow_path_node");
    ros::NodeHandle nh;
    std::vector<std::vector<float>> goals;
    ros::Subscriber map_sub;
    std::string path;

    ros::NodeHandle nhp("~");
    nhp.getParam("path",path);
    //std::cout<<path;
    freopen(path.c_str(), "r",stdin);
    int num_goals=0;
    scanf("%d",&num_goals);
    //std::cout<<"goals: "<<num_goals<<std::endl;
    for(int i=0;i<num_goals;i++)
    {
      float x,y,theta;
      scanf("%f%f%f",&x,&y,&theta);
      std::vector<float> goal{x,y,theta};
      std::cout<<"goal: "<<x<<" "<<y<<" "<<theta<<std::endl;
      goals.push_back(goal);
    }

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);    
    while(!ac.waitForServer (ros::Duration(4.0))){ ROS_INFO("Waiting for the move_base action server !!!"); }
	
    move_base_msgs::MoveBaseGoal goal_msg;
    goal_msg.target_pose.header.frame_id = "map";
    goal_msg.target_pose.header.stamp = ros::Time::now();

    
   
    for (int i =0; i< num_goals; i++)
    {
	// Define a position and orientation for the robot to reach
	goal_msg.target_pose.pose.position.x = goals[i][0];
	goal_msg.target_pose.pose.position.y = goals[i][1];
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, goals[i][2]);
	q.normalize();
	quaternionTFToMsg(q, goal_msg.target_pose.pose.orientation);
	
	// Send the goal
	ROS_INFO ("Sending goal");
	
	try{
	 ac.sendGoal (goal_msg);
	 ac.waitForResult();
        }
        catch(...)
	{
		std::cout<<"Cannot make a plan";
		break;
	}
    }


    return 0;
}
