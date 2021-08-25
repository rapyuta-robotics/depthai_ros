#include <ros/ros.h>
#include <depthai_ros_driver/depthai_base.hpp>

namespace rr {

//==============================================================================
template<class Node>
void DepthAIBase<Node>::onInit()
{
  auto& p_nh = this->getPrivateNodeHandle();
  auto& nh = this->getNodeHandle();

  _depthai_common = std::make_unique<DepthAICommon>(nh, p_nh);

  _cameraReadTimer = p_nh.createTimer(ros::Duration(1. / 500),
      [&](const ros::TimerEvent&)
      {
        _depthai_common->process_and_publish_packets();
      });

  const auto queue_size = _depthai_common->get_config().queue_size;

  // create Trigger default camera param service
  using TriggerSrv = depthai_ros_msgs::TriggerNamed;
  _camera_info_default = nh.template
    advertiseService<TriggerSrv::Request, TriggerSrv::Response>(
    ResetCameraServiceName,
    [&](TriggerSrv::Request& req, TriggerSrv::Response& res)
    {
      const auto& name = req.name;
      res.success = _depthai_common->set_camera_info_manager(name, "default/");
      return true;
    });

  // autofocus 'service' subscriber
  _af_ctrl_sub = nh.template subscribe<depthai_ros_msgs::AutoFocusCtrl>(
    SetAutoFocusTopicName, queue_size,
    [&](const depthai_ros_msgs::AutoFocusCtrlConstPtr& msg)
    {
      _depthai_common->set_autofocus(
        msg->trigger_auto_focus, msg->auto_focus_mode);
    });

  // disparity_confidence 'service' subscriber
  _disparity_conf_sub = nh.template subscribe<std_msgs::Float32>(
    SetDisparityTopicName, queue_size,
    [&](const std_msgs::Float32::ConstPtr& msg)
    {
      _depthai_common->set_disparity(msg->data);
    });
}
}  // namespace rr
