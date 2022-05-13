#include <front_scan/front_scan.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "front_scan_node");
    ros::NodeHandle nh;
    front_scan node(nh);
    ros::spin();
    return 0;
}
