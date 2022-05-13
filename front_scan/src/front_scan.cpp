#include <front_scan/front_scan.hpp>

front_scan::front_scan(ros::NodeHandle& nh){
    std::cout << "front_scan constructor called" << std::endl;
    ROS_INFO("front_scan constructor called");
    node_handle = nh;
    ros::NodeHandle nhp("~");
    nhp.getParam("width",width);
    nhp.getParam("distance",distance);
    nhp.getParam("ratio_threshold",ratio_threshold);

    sub_pointCloud = nh.subscribe("/point_cloud", 10, &front_scan::callback, this);

    pub_blocked = nh.advertise<std_msgs::Bool>("/front_blocked", 1);
}

void front_scan::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){

    //std::cout << "Point cloud has: " << cloud->size () << " data points." << std::endl;
    int height = cloud->height;
    int width = cloud->width;
    //std::cout<<height<<" "<<width<<std::endl;

    int close_points=0;
    int front_points=0;

    if( cloud->isOrganized() )
    {
        for(int i=0;i<height;i++)
	{
  	  for(int j=0; j<width; j++)
	  {
            if( fabs( cloud->at(i,j).y )< width ) 
            {
		front_points++;
		if( (cloud->at(i,j).x) < distance )
		{
		  close_points++;
		} 
            }
	  }
        }
    }
    else
    {    
        for(int j=0; j<width; j++){
            if( fabs( cloud->at(j).y )< width  ) 
            {
		front_points++;
		if( (cloud->at(j).x) < distance )
		{
		  close_points++;
		} 
            }
	  }
    }
    bool blocked = (float(close_points) / front_points ) > ratio_threshold ;// too many close points ahead 
    std_msgs::Bool msg;
    msg.data = blocked; 
    pub_blocked.publish(msg);
    return;
}
