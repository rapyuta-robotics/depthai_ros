#pragma once
/**
 * @file img_detection_stamper.hpp
 * @brief Publish an image detection (NN) with the timestamp from the latest Image from sensor
 */

#include <depthai_ros_driver/converter_and_stamper.hpp>

#include <depthai_datatype_msgs/ImgDetectionsStamped.h>

namespace rr {
template <class Node>
using ImgDetectionsStamper = ConverterAndStamper<depthai_datatype_msgs::ImgDetectionsStamped, Node>;
}  // namespace rr

#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>
namespace rr {
using ImgDetectionsStamperNode = ImgDetectionsStamper<ROS1Node<>>;
using ImgDetectionsStamperNodelet = ImgDetectionsStamper<nodelet::Nodelet>;
}  // namespace rr
