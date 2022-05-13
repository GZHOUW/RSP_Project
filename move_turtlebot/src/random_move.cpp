#include <move_turtlebot/random_move.hpp>

void random_move::odom_callback(const nav_msgs::OdometryPtr odom)
{
	//std::cout<<"in odom\n";
	my_odom = *odom;
}
void random_move::map_callback(const nav_msgs::OccupancyGridConstPtr local_map)
{
      //std::cout<<"in map\n";
      my_local_map = *local_map;
}
void random_move::get_next_pos(float& dx,float& dy,float& theta)
{
    float resolution = my_local_map.info.resolution;
    int height = (int)(my_local_map.info.height);
    int width = (int)(my_local_map.info.width);
    bool flag = 0;
    int dx8[]={1,1,0,-1,-1,-1,0,1}; //8 directions
    int dy8[]={0,1,1,1,0,-1,-1,-1};
/*
    int dx4[]={1,0,-1,0}; //4 directions N E S W
    int dy4[]={0,1,0,-1};
*/
    double theta8[]={1.5708, 0.7854, 0.0, -0.7854, -1.5708, -2.3562, -3.1416, 2.3562};

    int grids = int( step_length / resolution);
    //std::cout<<grids<<"\n";
    while(1)
    {
	int i=rand()%8;
        
	if( (int)(my_local_map.data[ (width/2 + dx8[i]*grids)*height + width/2 + dy8[i]*grids ]) < 10 )
	{
	    dx = float(dy8[i])*step_length;
	    dy = float(dx8[i])*step_length;
	    theta = theta8[i];
            std::cout<<"next direction: ";
            if(i==0) std::cout<<"North"<<"\n";
            if(i==1) std::cout<<"NorthEast"<<"\n";
            if(i==2) std::cout<<"East"<<"\n";
            if(i==3) std::cout<<"EastSouth"<<"\n";
            if(i==4) std::cout<<"South"<<"\n";
            if(i==5) std::cout<<"SouthWest"<<"\n";
            if(i==6) std::cout<<"West"<<"\n";
            if(i==7) std::cout<<"WestNorth"<<"\n";

	    break;
	}
    }
}

random_move::random_move(ros::NodeHandle& nh):nh(nh)
{
    ros::NodeHandle nhp("~");
    nhp.getParam("x",initial_x);
    nhp.getParam("y",initial_y);
    nhp.getParam("theta",initial_theta);
    nhp.getParam("step_length",step_length);
    nhp.getParam("step_size",step_size);

    map_sub = nh.subscribe("/move_base/local_costmap/costmap", 1, &random_move::map_callback,this);
    odom_sub = nh.subscribe("/odom", 1, &random_move::odom_callback,this);
    goal_msg.target_pose.header.frame_id = "map";

}
move_base_msgs::MoveBaseGoal random_move::run()
{
	ros::spinOnce();
	goal_msg.target_pose.header.stamp = ros::Time::now();
	get_next_pos(dx,dy,theta);
        goal_msg.target_pose.pose.position.x = my_odom.pose.pose.position.x + dx + initial_x;
	goal_msg.target_pose.pose.position.y = my_odom.pose.pose.position.y + dy + initial_y;
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, theta);
	q.normalize();
	quaternionTFToMsg(q, goal_msg.target_pose.pose.orientation);
	ROS_INFO ("Sending goal");
	std::cout<<"goal: "<<goal_msg.target_pose.pose.position.x<<" "<<goal_msg.target_pose.pose.position.y<<"\n";
	return goal_msg;
}
