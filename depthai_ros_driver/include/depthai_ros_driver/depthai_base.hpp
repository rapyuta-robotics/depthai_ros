#pragma once

#include <ros/ros.h>
#include <depthai_ros_driver/depthai_common.hpp>

namespace rr {

//==============================================================================
template<class Node>
class DepthAIBase : public Node
{
public:
  DepthAIBase() = default;
  ~DepthAIBase() = default;

private:
  ros::ServiceServer _camera_info_default;
  ros::Subscriber _af_ctrl_sub;
  ros::Subscriber _disparity_conf_sub;
  ros::Timer _cameraReadTimer;

  std::unique_ptr<DepthAICommon> _depthai_common;

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
