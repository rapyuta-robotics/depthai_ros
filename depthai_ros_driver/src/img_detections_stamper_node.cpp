#include <pluginlib/class_list_macros.h>

#include <depthai_ros_driver/img_detections_stamper.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "img_detections_stamper");

    rr::ImgDetectionsStamperNode node;
    node.init();

    ros::spin();

    return 0;
}
