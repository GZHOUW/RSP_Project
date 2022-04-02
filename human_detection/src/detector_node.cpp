#include <human_detection/detector.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "detector_node");
    ros::NodeHandle nh;
    Detector pub(nh);
    ros::spin();
    return 0;
}