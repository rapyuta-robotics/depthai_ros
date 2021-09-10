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

#include <rclcpp/rclcpp.hpp>
#include <depthai_ros_driver/depthai_common.hpp>
#include <depthai_ros_driver/visibility_control.h>

namespace rr {

//==============================================================================
class DepthAIBaseRos2 : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit DepthAIBaseRos2(const rclcpp::NodeOptions& options);

private:
  std::unique_ptr<DepthAICommon> _depthai_common;
};

}  // namespace rr

#endif
