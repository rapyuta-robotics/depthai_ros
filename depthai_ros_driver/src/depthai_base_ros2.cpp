/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <depthai_ros_driver/depthai_base_ros2.hpp>

namespace rr {

//==============================================================================
DepthAIBaseRos2::DepthAIBaseRos2(const rclcpp::NodeOptions& options)
: Node("depthai_node", options)
{
  // TODO use shared pointer
  const auto node = this->create_sub_node("").get();
  _depthai_common = std::make_unique<DepthAICommon>(node, node);

  _cameraReadTimer = this->create_wall_timer(0.002s,
      [&]()
      {
        _depthai_common->process_and_publish_packets();
      });

  /// ros2 dds qos profile
  const auto qos = rclcpp::ServicesQoS().best_effort();

  // create Trigger default camera param service
  _camera_info_default = this->create_service<TriggerSrv>(
    ResetCameraServiceName,
    [this](
      const std::shared_ptr<TriggerSrv::Request> req,
      std::shared_ptr<TriggerSrv::Response> res)
    {
      const auto& name = req->name;
      res->success = _depthai_common->set_camera_info_manager(name, "default/");
    }
  );

  // disparity_confidence 'service' subscriber
  _disparity_conf_sub = this->create_subscription<Float32Msg>(
    SetDisparityTopicName, qos,
    [&](const Float32Msg::UniquePtr msg)
    {
      if (!_depthai_common->set_disparity(msg->data))
        RCLCPP_WARN(this->get_logger(),
        "Disparity confidence value:%f, is invalid", msg->data);
    });

  // autofocus 'service' subscriber
  _af_ctrl_sub = this->create_subscription<AutoFocusCtrlMsg>(
    SetAutoFocusTopicName, qos,
    [&](const AutoFocusCtrlMsg::UniquePtr msg)
    {
      if (!_depthai_common->set_autofocus(
        msg->trigger_auto_focus, msg->auto_focus_mode))
        RCLCPP_WARN(this->get_logger(), "Invalid Auto Focus mode requested");
    });
}
}  // namespace rr

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(rr::DepthAIBaseRos2)
