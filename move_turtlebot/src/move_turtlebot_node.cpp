#include <move_turtlebot/random_move.hpp>

int main(int argc , char** argv){

    ros::init(argc,argv,"move_turtlebot_node");
    ros::NodeHandle nh;


    random_move node(nh);
   
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); 
    while( !ac.waitForServer(ros::Duration(4.0)) ) { ROS_INFO("Waiting for the move_base action server !!!"); }

    move_base_msgs::MoveBaseGoal goal_msg;
    for (int i=0; i< node.step_size ; i++)
    {
	goal_msg = node.run();
	ac.sendGoal (goal_msg);
	ac.waitForResult();
    }
    std::cout<<"move_node finished";
    return 0;
}
