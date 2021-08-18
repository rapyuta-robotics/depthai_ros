#pragma once
/**
 * @file img_detection_stamper.hpp
 * @brief Publish an image detection (NN) with the timestamp from the latest Image from sensor
 */
#include <ros/ros.h>
#include <depthai_datatype_msgs/ImgDetectionsStamped.h>
#include <depthai_datatype_msgs/RawImgDetections.h>
#include <sensor_msgs/Image.h>

namespace rr {
template <class Node>
struct ImgDetectionsStamper : Node {
public:
    ImgDetectionsStamper() = default;
    ~ImgDetectionsStamper() = default;

private:
    void onInit() override;
    void img_callback(const sensor_msgs::ImageConstPtr& img_msg);
    void nn_callback(const depthai_datatype_msgs::RawImgDetectionsConstPtr& nn_msg_ptr);
    sensor_msgs::ImageConstPtr last_recv_msg_;
    depthai_datatype_msgs::ImgDetectionsStamped msg_stamped_; // have it as property so we don't need to allocate

    ros::Publisher pub_stamped_;
    ros::Subscriber sub_img_;
    ros::Subscriber sub_nn_;
};
}  // namespace rr


#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>
namespace rr {
    using ImgDetectionsStamperNode = ImgDetectionsStamper<ROS1Node<>>;
    using ImgDetectionsStamperNodelet = ImgDetectionsStamper<nodelet::Nodelet>;
}
