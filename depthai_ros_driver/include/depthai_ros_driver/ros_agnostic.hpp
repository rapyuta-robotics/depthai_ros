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

#else
  #include <ros/ros.h>
  #define ROS_LOGGER(...) ROS_WARN_NAMED("depthai_ros", __VA_ARGS__)
using ROSNodeHandle = std::shared_ptr<ros::NodeHandle>;
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
  /// @brief Create ros publisher
  template<class Msg>
  void create_publisher(
    ROSNodeHandle node_handle,
    const std::string& topic_name,
    const uint32_t queue_size)
  {
    #if defined(USE_ROS2)
    // using qos: KeepLast(QueueSize)
    _ptr = node_handle->create_publisher<Msg>(topic_name, queue_size);
    #else
    _pub = node_handle->advertise<Msg>(topic_name, queue_size);
    #endif
  }

  /// @brief Publish ros msg
  template<typename Msg>
  void publish(Msg& msg)
  {
    #if defined(USE_ROS2)
    // move the ptr and subscriber check here
    auto pub = std::static_pointer_cast<rclcpp::Publisher<Msg>>(_ptr);
    if (pub && pub->get_subscription_count() > 0)
      pub->publish(msg);
    #else
    if (_pub.getNumSubscribers() > 0);
    _pub.publish(msg);
    #endif
  }

private:
  #if defined(USE_ROS2)
  std::shared_ptr<void> _ptr;
  #else
  ros::Publisher _pub;
  #endif
};

//==============================================================================
class Subscription
{
public:
  /// @brief Create ros subscription
  template<class Msg, typename Callback>
  void create_subscription(
    ROSNodeHandle node_handle,
    const std::string& topic_name,
    const uint32_t queue_size,
    const Callback& callback)
  {
    #if defined(USE_ROS2)
    // using qos: KeepLast(QueueSize)
    _ptr = node_handle->create_subscription<Msg>(topic_name, queue_size,
        callback);
    #else
    _sub = node_handle->subscribe<Msg>(topic_name, queue_size, callback);
    #endif
  }
private:
  #if defined(USE_ROS2)
  std::shared_ptr<void> _ptr;
  #else
  ros::Subscriber _sub;
  #endif
};

//==============================================================================
class Service
{
public:
  /// @brief Create ros service
  template<class Msg, typename Callback>
  void create_service(
    ROSNodeHandle node_handle,
    const std::string& srv_name,
    const Callback& callback)
  {
    #if defined(USE_ROS2)
    _ptr = node_handle->create_service<Msg>(srv_name, callback);
    #else
    using RequestMsg = typename Msg::Request;
    using ResponseMsg = typename Msg::Response;
    _srv = node_handle->advertiseService<RequestMsg, ResponseMsg>(
      srv_name,
      [callback = std::move(callback)](const RequestMsg& req, ResponseMsg& res)
      {
        // internally convert ref to ptr
        auto res_ptr = std::make_shared<ResponseMsg>();
        callback(std::make_shared<RequestMsg>(req), res_ptr);
        res = *res_ptr;
        return true;
      });
    #endif
  }
private:
  #if defined(USE_ROS2)
  std::shared_ptr<void> _ptr;
  #else
  ros::ServiceServer _srv;
  #endif
};

//==============================================================================
class Timer
{
public:
  /// @brief Create ros timer
  template<typename Callback>
  void create_timer(
    ROSNodeHandle node_handle,
    const double period_sec,
    const Callback& callback)
  {
    #if defined(USE_ROS2)
    const auto period = std::chrono::duration<double>(period_sec);
    _timer = node_handle->create_wall_timer(period, callback);
    #else
    _timer = node_handle->createTimer(ros::Duration(period_sec),
        [callback = std::move(callback)](const ros::TimerEvent&)
        {
          callback();
        });
    #endif
  }
private:
  #if defined(USE_ROS2)
  rclcpp::TimerBase::SharedPtr _timer;
  #else
  ros::Timer _timer;
  #endif
};

//==============================================================================

/// @brief Get ROS param
template<typename Param>
void get_param(
  ROSNodeHandle node_handle, const std::string& name, Param& variable)
{
#if defined(USE_ROS2)
  using var_type = std::remove_reference_t<decltype(variable)>;
  node_handle->declare_parameter<var_type>(name, variable);
  node_handle->get_parameter(name, variable);
#else
  if (node_handle->hasParam(name))
    node_handle->getParam(name, variable);
  else
    node_handle->setParam(name, variable);
#endif
}

}  // namespace ros_agnostic
}  // namespace rr

#endif // DEPTHAI_ROS_DRIVER__ROS_AGNOSTIC_HPP
