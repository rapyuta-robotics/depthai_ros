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

#ifndef DEPTHAI_ROS_DRIVER__ROS_AGNOSTIC_HPP
#define DEPTHAI_ROS_DRIVER__ROS_AGNOSTIC_HPP

#if defined(USE_ROS2)
  #include <rclcpp/node.hpp>
  #define ROS_LOGGER(...) RCUTILS_LOG_WARN_NAMED("depthai_ros", __VA_ARGS__)
using ROSNodeHandle = std::shared_ptr<rclcpp::Node>;
using RosTime = rclcpp::Time;
using RosDuration = rclcpp::Duration;
#else
  #include <ros/ros.h>
  #define ROS_LOGGER(...) ROS_WARN_NAMED("depthai_ros", __VA_ARGS__)
using ROSNodeHandle = std::shared_ptr<ros::NodeHandle>;
using RosTime = ros::Time;
using RosDuration = ros::Duration;
#endif

namespace rr {

/*
 * Ros agnostic is a group of ROS wrapper classes, which enables agnostic ROS
 * implementation of both ros1 and ros2. This can be switched during compile
 * by defining 'USE_ROS2' in the Cmake file
 */
namespace ros_agnostic {

//==============================================================================
class Publisher;
class Subscription;
class Service;
class Timer;

//==============================================================================
class NodeInterface
{
public:
  NodeInterface(ROSNodeHandle nh): _nh(nh){};

  /// Create publisher
  template<class Msg>
  Publisher create_publisher(
    const std::string& topic_name,
    const uint32_t queue_size);

  /// Create Subsriber
  template<class Msg, typename Callback>
  Subscription create_subscription(
    const std::string& topic_name,
    const uint32_t queue_size,
    const Callback& callback);

  /// Create Service
  template<class Msg, typename Callback>
  Service create_service(
    const std::string& srv_name,
    const Callback& callback);

  /// Create Timer
  template<typename Callback>
  Timer create_timer(
    const double period_sec,
    const Callback& callback);

  /// get ROSNodeHandle
  ROSNodeHandle get_node_handle(){ return _nh; };

  /// Get current ros time
  const RosTime current_time()
  {
    #if defined(USE_ROS2)
    return _nh->now();
    #else
    return RosTime::now();
    #endif
  };

private:
  ROSNodeHandle _nh;
};

//==============================================================================

/// @brief Get ROS param
template<typename Param>
void get_param(
  ROSNodeHandle node_handle, 
  const std::string& name,
  Param& variable);

}  // namespace ros_agnostic
}  // namespace rr

#endif // DEPTHAI_ROS_DRIVER__ROS_AGNOSTIC_HPP
