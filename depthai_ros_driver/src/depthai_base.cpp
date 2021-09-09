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

#include <ros/ros.h>
#include <depthai_ros_driver/depthai_base.hpp>

namespace rr {

//==============================================================================
template<class Node>
void DepthAIBase<Node>::onInit()
{
  auto p_nh = std::make_shared<ros::NodeHandle>(this->getPrivateNodeHandle());
  auto nh = std::make_shared<ros::NodeHandle>(this->getNodeHandle());

  _depthai_common = std::make_unique<DepthAICommon>(nh, p_nh);

  _cameraReadTimer = p_nh->createTimer(ros::Duration(1. / 500),
      [&](const ros::TimerEvent&)
      {
        _depthai_common->process_and_publish_packets();
      });
}

//==============================================================================
template class DepthAIBase<ROS1Node<>>;
template class DepthAIBase<nodelet::Nodelet>;
}  // namespace rr
