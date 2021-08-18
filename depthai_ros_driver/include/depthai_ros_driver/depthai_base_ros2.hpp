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

#ifndef DEPTHAI_ROS_DRIVER__DEPTHAI_BASE_ROS2_HPP
#define DEPTHAI_ROS_DRIVER__DEPTHAI_BASE_ROS2_HPP

#include <chrono>
#include <variant>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <depthai_ros_driver/depthai_common.hpp>

#include <image_transport/camera_publisher.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <depthai_ros_driver/visibility_control.h>

using namespace std::chrono_literals;

using TriggerSrv = depthai_ros_msgs::srv::TriggerNamed;
using AutoFocusCtrlMsg = depthai_ros_msgs::msg::AutoFocusCtrl;
using Float32Msg = std_msgs::msg::Float32;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

namespace rr {

//==============================================================================
class DepthAIBaseRos2 : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit DepthAIBaseRos2(const rclcpp::NodeOptions& options);

private:

  void publishObjectInfoMsg(
    const dai::Detections& detections, const rclcpp::Time& stamp);

  void publishImageMsg(
    const HostDataPacket& packet, Stream type, const rclcpp::Time& stamp);

  const rclcpp::Time get_rostime(const double camera_ts);

  rclcpp::TimerBase::SharedPtr _cameraReadTimer;
  rclcpp::Subscription<Float32Msg>::SharedPtr _disparity_conf_sub;
  rclcpp::Subscription<AutoFocusCtrlMsg>::SharedPtr _af_ctrl_sub;
  rclcpp::Service<TriggerSrv>::SharedPtr _camera_info_default;

  using ObjectPubPtr = rclcpp::Publisher<ObjectMsg>::SharedPtr;
  using ObjectsPubPtr = rclcpp::Publisher<ObjectsMsg>::SharedPtr;
  using ImagePubPtr = rclcpp::Publisher<ImageMsg>::SharedPtr;
  using ComImagePubPtr = rclcpp::Publisher<CompressedImageMsg>::SharedPtr;
  using CameraInfoPubPtr =
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr;

  using StreamPubVariant = std::variant<ObjectsPubPtr, ObjectPubPtr,
      ImagePubPtr, ComImagePubPtr>;
  using CameraInfoManagerPtr =
    std::unique_ptr<camera_info_manager::CameraInfoManager>;

  std::array<StreamPubVariant, Stream::END> _stream_publishers;
  std::array<CameraInfoPubPtr, Stream::IMAGE_END> _camera_info_publishers;
  std::array<CameraInfoManagerPtr, Stream::IMAGE_END> _camera_info_managers;

  std::unique_ptr<DepthAICommon> _depthai_common;
  std::array<std::string, Stream::END> _topic_names;

  // params
  std::string _camera_name;
  std::string _camera_param_uri;
  int _queue_size = 10;

  rclcpp::Time _stamp;
  double _depthai_init_ts = -1;  // sadly, we don't have a way of measuring drift

};

}  // namespace rr

#endif
