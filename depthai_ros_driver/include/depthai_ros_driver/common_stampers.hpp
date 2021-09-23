#pragma once
/**
 * @file img_detection_stamper.hpp
 * @brief Publish an image detection (NN) with the timestamp from the latest Image from sensor
 */

#include <depthai_ros_driver/converter_and_stamper.hpp>

#include <depthai_ros_msgs/ImgDetectionsStamped.h>
#include <depthai_ros_msgs/NNStamped.h>

namespace rr {
template <class Node>
using ImgDetectionsStamper = ConverterAndStamper<depthai_ros_msgs::ImgDetectionsStamped, Node>;

template <class Node>
using NNStamper = ConverterAndStamper<depthai_ros_msgs::NNStamped, Node>;
}  // namespace rr

#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>
namespace rr {
using ImgDetectionsStamperNode = ImgDetectionsStamper<ROS1Node<>>;
using ImgDetectionsStamperNodelet = ImgDetectionsStamper<nodelet::Nodelet>;

using NNStamperNode = NNStamper<ROS1Node<>>;
using NNStamperNodelet = NNStamper<nodelet::Nodelet>;
}  // namespace rr