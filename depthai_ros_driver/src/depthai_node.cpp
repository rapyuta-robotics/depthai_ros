#include <ros/ros.h>

#include <depthai_ros_driver/depthai_base.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "depthai_node");

    rr::DepthAINode depthai;
    depthai.init();

    ros::spin();

    return 0;
}
