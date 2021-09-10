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

#include <chrono>
#include <depthai_ros_driver/depthai_base_ros2.hpp>

namespace rr {

//==============================================================================
DepthAIBaseRos2::DepthAIBaseRos2(const rclcpp::NodeOptions& options)
: Node("depthai_node", options)
{
  const auto node = this->create_sub_node(""); // private?
  _depthai_common = std::make_unique<DepthAICommon>(node, node);
}
}  // namespace rr

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(rr::DepthAIBaseRos2)
