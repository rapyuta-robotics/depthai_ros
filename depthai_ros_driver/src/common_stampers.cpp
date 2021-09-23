#include <depthai_ros_driver/common_stampers.hpp>

#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>

namespace rr {
template class ConverterAndStamper<depthai_ros_msgs::ImgDetectionsStamped,
        ROS1Node<>>;  // ImgDetectionsStamper<ROS1Node<>>;
template class ConverterAndStamper<depthai_ros_msgs::ImgDetectionsStamped,
        nodelet::Nodelet>;  // ImgDetectionsStamper<nodelet::Nodelet>;

template class ConverterAndStamper<depthai_ros_msgs::NNStamped,
        ROS1Node<>>;  // NNStamper<ROS1Node<>>;
template class ConverterAndStamper<depthai_ros_msgs::NNStamped,
        nodelet::Nodelet>;  // NNStamper<nodelet::Nodelet>;
}  // namespace rr
