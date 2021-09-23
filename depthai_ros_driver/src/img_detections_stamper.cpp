#include <depthai_ros_driver/img_detections_stamper.hpp>

#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>

namespace rr {
template class ConverterAndStamper<depthai_ros_msgs::ImgDetectionsStamped,
        ROS1Node<>>;  // ImgDetectionsStamper<ROS1Node<>>;
template class ConverterAndStamper<depthai_ros_msgs::ImgDetectionsStamped,
        nodelet::Nodelet>;  // ImgDetectionsStamper<nodelet::Nodelet>;
}  // namespace rr
