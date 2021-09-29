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
  #define ROS_MSG_TYPE(PACKAGE, NAME) PACKAGE::msg::NAME
  #define ROS_SRV_TYPE(PACKAGE, NAME) PACKAGE::srv::NAME
using RosNodeHandle = std::shared_ptr<rclcpp::Node>;
using RosTime = rclcpp::Time;
using RosDuration = rclcpp::Duration;
using RosPub = std::shared_ptr<void>;
using RosSub = std::shared_ptr<void>;
using RosSrv = std::shared_ptr<void>;
using RosTimer = rclcpp::TimerBase::SharedPtr;
#else
  #include <ros/ros.h>
  #define ROS_LOGGER(...) ROS_WARN_NAMED("depthai_ros", __VA_ARGS__)
  #define ROS_MSG_TYPE(PACKAGE, NAME) PACKAGE::NAME
  #define ROS_SRV_TYPE(PACKAGE, NAME) PACKAGE::NAME
using RosNodeHandle = std::shared_ptr<ros::NodeHandle>;
using RosTime = ros::Time;
using RosDuration = ros::Duration;
using RosPub = ros::Publisher;
using RosSub = ros::Subscriber;
using RosSrv = ros::ServiceServer;
using RosTimer = ros::Timer;
#endif

namespace rr {

/*
 * Ros agnostic is a group of ROS wrapper classes, which enables agnostic ROS
 * implementation of both ros1 and ros2. This can be switched during compile
 * by defining 'USE_ROS2' in the Cmake file
 */
namespace ros_agnostic {

//==============================================================================
class Publisher
{
public:
  /// @brief Publish ros msg
  template<typename Msg>
  void publish(Msg& msg);

private:
  /// @brief Create ros publisher
  template<class Msg>
  inline void create_publisher(
    RosNodeHandle node_handle,
    const std::string& topic_name,
    const uint32_t queue_size);

  RosPub _pub;
  friend class NodeInterface;
};

//==============================================================================
class Subscription
{
private:
  /// @brief Create ros subscription
  template<class Msg, typename Callback>
  inline void create_subscription(
    RosNodeHandle node_handle,
    const std::string& topic_name,
    const uint32_t queue_size,
    const Callback& callback);

  RosSub _sub;
  friend class NodeInterface;
};

//==============================================================================
class Service
{
private:
  /// @brief Create ros service
  template<class Msg, typename Callback>
  void create_service(
    RosNodeHandle node_handle,
    const std::string& srv_name,
    const Callback& callback);

  RosSrv _srv;
  friend class NodeInterface;
};

//==============================================================================
class Timer
{
private:
  /// @brief Create ros timer
  template<typename Callback>
  void create_timer(
    RosNodeHandle node_handle,
    const double period_sec,
    const Callback& callback);

  RosTimer _timer;
  friend class NodeInterface;
};

//==============================================================================
class NodeInterface
{
public:
  NodeInterface(RosNodeHandle nh)
  : _nh(nh) {}

  /// @brief Create publisher
  template<class Msg>
  Publisher create_publisher(
    const std::string& topic_name,
    const uint32_t queue_size);

  /// @brief Create Subsriber
  template<class Msg, typename Callback>
  Subscription create_subscription(
    const std::string& topic_name,
    const uint32_t queue_size,
    const Callback& callback);

  /// @brief Create Service
  template<class Msg, typename Callback>
  Service create_service(
    const std::string& srv_name,
    const Callback& callback);

  /// @brief Create Timer
  template<typename Callback>
  Timer create_timer(
    const double period_sec,
    const Callback& callback);

  /// @brief get node
  RosNodeHandle get_node() { return _nh; }

  /// @brief Get sub node
  inline RosNodeHandle get_sub_node(const std::string& ns);

  /// @brief Get current ros time
  inline const RosTime current_time();

private:
  RosNodeHandle _nh;
};

//==============================================================================

/// @brief Get ROS param
template<typename Param>
void get_param(
  RosNodeHandle node_handle,
  const std::string& name,
  Param& variable);

}  // namespace ros_agnostic
}  // namespace rr

#include <depthai_ros_driver/impl/ros_agnostic.hpp>

#endif // DEPTHAI_ROS_DRIVER__ROS_AGNOSTIC_HPP
