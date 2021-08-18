#pragma once

#include <depthai_ros_driver/depthai_common.hpp>

// core ROS dependency includes
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <ros/ros.h>

namespace rr {

//==============================================================================
template<class Node>
class DepthAIBase : public Node
{
public:
  DepthAIBase() = default;
  ~DepthAIBase() = default;

private:
  std::array<std::string, Stream::END> _topic_names;

  std::array<std::unique_ptr<ros::Publisher>, Stream::END> _stream_publishers;
  std::array<std::unique_ptr<ros::Publisher>,
    Stream::IMAGE_END> _camera_info_publishers;

  std::array<std::unique_ptr<camera_info_manager::CameraInfoManager>,
    Stream::IMAGE_END> _camera_info_managers;
  ros::ServiceServer _camera_info_default;
  ros::Subscriber _af_ctrl_sub;
  ros::Subscriber _disparity_conf_sub;

  ros::Timer _cameraReadTimer;

  // Parameters
  int _queue_size = 10;
  std::string _camera_name = "";
  std::string _camera_param_uri = "package://depthai_ros_driver/params/camera/";

  std::unique_ptr<DepthAICommon> _depthai_common;

  ros::Time _stamp;
  double _depthai_ts_offset = -1;  // sadly, we don't have a way of measuring drift
  const ros::Time get_rostime(const double camera_ts);

  void prepareStreamConfig();

  void publishImageMsg(const HostDataPacket& packet, Stream type,
    const ros::Time& stamp);
  void publishObjectInfoMsg(const dai::Detections& detections,
    const ros::Time& stamp);

  bool defaultCameraInfo(depthai_ros_msgs::TriggerNamed::Request& req,
    depthai_ros_msgs::TriggerNamed::Response& res);

  void onInit() override;
};
}  // namespace rr

//==============================================================================
#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>
namespace rr {
using DepthAINode = DepthAIBase<ROS1Node<>>;
using DepthAINodelet = DepthAIBase<nodelet::Nodelet>;
}  // namespace rr
