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

#ifndef DEPTHAI_ROS_DRIVER__DEPTHAI_COMMON_HPP
#define DEPTHAI_ROS_DRIVER__DEPTHAI_COMMON_HPP


// Common ros and ros2 api
#include <cv_bridge/cv_bridge.h>

// relevant 3rd party includes
#include <depthai/device.hpp>
#include <depthai/host_data_packet.hpp>
#include <depthai/nnet/nnet_packet.hpp>
#include <depthai/pipeline/cnn_host_pipeline.hpp>

#include <depthai_ros_driver/ros_agnostic.hpp>

#if defined(USE_ROS2)
  #include <std_msgs/msg/float32.hpp>
  #include <sensor_msgs/msg/image.hpp>
  #include <depthai_ros_msgs/srv/trigger_named.hpp>
  #include <depthai_ros_msgs/msg/objects.hpp>
  #include <depthai_ros_msgs/msg/auto_focus_ctrl.hpp>
  #include <camera_info_manager/camera_info_manager.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using ObjectsMsg = depthai_ros_msgs::msg::Objects;
using ObjectMsg = depthai_ros_msgs::msg::Object;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using HeaderMsg = std_msgs::msg::Header;
using AutoFocusCtrlMsg = depthai_ros_msgs::msg::AutoFocusCtrl;
using Float32Msg = std_msgs::msg::Float32;
using TriggerSrv = depthai_ros_msgs::srv::TriggerNamed;

#else
  #include <std_msgs/Float32.h>
  #include <sensor_msgs/Image.h>
  #include <depthai_ros_msgs/AutoFocusCtrl.h>
  #include <depthai_ros_msgs/Objects.h>
  #include <depthai_ros_msgs/TriggerNamed.h>
  #include <camera_info_manager/camera_info_manager.h>

using ImageMsg = sensor_msgs::Image;
using CompressedImageMsg = sensor_msgs::CompressedImage;
using ObjectsMsg = depthai_ros_msgs::Objects;
using ObjectMsg = depthai_ros_msgs::Object;
using CameraInfoMsg = sensor_msgs::CameraInfo;
using HeaderMsg = std_msgs::Header;
using AutoFocusCtrlMsg = depthai_ros_msgs::AutoFocusCtrl;
using Float32Msg = std_msgs::Float32;
using TriggerSrv = depthai_ros_msgs::TriggerNamed;
#endif

using CameraInfoPub = std::shared_ptr<rr::ros_agnostic::Publisher>;
using StreamPub = std::shared_ptr<rr::ros_agnostic::Publisher>;
using CameraInfoManagerPtr =
  std::shared_ptr<camera_info_manager::CameraInfoManager>;

namespace rr {

//==============================================================================
const std::string ResetCameraServiceName = "reset_camera_info";
const std::string SetAutoFocusTopicName = "auto_focus_ctrl";
const std::string SetDisparityTopicName = "disparity_confidence";

//==============================================================================
enum Stream : std::size_t
{
  // all images at the beginning
  LEFT,
  RIGHT,
  RECTIFIED_LEFT,
  RECTIFIED_RIGHT,
  DISPARITY,
  DISPARITY_COLOR,
  DEPTH,
  PREVIEW_OUT,
  // compressed image streams
  JPEG_OUT,
  VIDEO,
  // non-image streams
  META_D2H, // TODO: Clarify! this should be img stream?
  META_OUT,
  OBJECT_TRACKER,  // TODO: Clarify! this is not in used?
  // utility enums
  END,
  IMAGE_END = META_D2H,
  UNCOMPRESSED_IMG_END = JPEG_OUT
};

//==============================================================================
struct DepthAICommonConfig
{
  // General configs config
  bool force_usb2 = false;
  bool depthai_block_read = false;
  bool request_jpegout = false;

  // Pipeline configs
  std::string calib_file = "";
  std::string blob_file = "";
  std::string blob_file_config = "";
  bool compute_bbox_depth = false;
  bool full_fov_nn = false;
  bool sync_video_meta = false;
  int shaves = 14;
  int cmx_slices = 14;
  int nn_engines = 2;
  int rgb_height = 1080;
  int rgb_fps = 30;
  int depth_height = 720;
  int depth_fps = 30;
  int queue_size = 10;
  std::string camera_name = "";
  std::string camera_param_uri =
    "package://depthai_ros_driver/params/camera/";
  std::vector<std::string> stream_list = {"video", "left", "depth"};
};

//==============================================================================
/* DepthAICommon class handles the interface with depthai hardware api and
 * manager all ROS stream publishers of the DepthAI driver. User-specific
 * configurations are specified via rosparam. User is required to init their own
 * ROS node and provide it to this class.
 */
class DepthAICommon
{
public:
  /// @brief Initiaize DepthAICommon and create camera stream publisher
  ///
  /// \param nh:
  ///    main nodehandle for stream publishers
  ///
  /// \param private_nh
  ///    private nodehandle for setting param, use when private ns is needed
  ///    else, provide the same nh to here.
  DepthAICommon(const ROSNodeHandle nh, const ROSNodeHandle private_nh);

  /// @brief get and process packets
  void process_and_publish_packets();

  /// @brief set camera disparity threshold
  ///
  /// \param val
  ///    disparity_confidence value
  bool set_disparity(const float val);

  /// set camera autofocus
  bool set_autofocus(const bool trigger, const uint8_t mode);

  /// @brief set camera info manager
  ///
  /// \param name
  ///    this is generally the topic name
  ///
  /// \param prefix
  ///    optional prefix indicate dir path, e.g: default/
  ///
  /// \return
  ///    false if failed, true if success
  const bool set_camera_info_manager(
    const std::string& name, const std::string& prefix);

  /// @brief Get DepthAICommon Config
  ///
  /// \return
  ///    return DepthAICommon Config
  const DepthAICommonConfig& get_config();

private:
  /// Create camera stream publisher
  void create_stream_publishers();

  /// publish object info msg
  void publishObjectInfoMsg(
    const dai::Detections& detections, const RosTime& stamp);

  /// publish Image Msg
  void publishImageMsg(
    const HostDataPacket& packet, Stream type, const RosTime& stamp);

  /// get ros time
  const RosTime get_rostime(const double camera_ts);

  /// create detection to object msg
  ObjectsMsg convert(const dai::Detections& detections);

  /// Create pipeline config string stream
  const std::string create_pipeline_config();

  /// get camera info msg
  CameraInfoMsg get_camera_info_msg(const Stream& id);

  DepthAICommonConfig _cfg;
  std::unique_ptr<Device> _depthai;
  std::shared_ptr<CNNHostPipeline> _pipeline;

  std::array<std::string, Stream::END> _stream_name{
    "left", "right", "rectified_left", "rectified_right", "disparity",
    "disparity_color", "depth", "previewout", "jpegout", "video",
    "meta_d2h", "metaout", "object_tracker"};

  std::array<std::string, Stream::END> _topic_names{"left", "right",
    "rectified_left", "rectified_right", "disparity",
    "disparity_color", "depth", "previewout", "jpeg", "mjpeg", "meta_d2h",
    "object_info", "object_tracker"};

  RosTime _stamp;
  double _depthai_init_ts = -1;  // sadly, we don't have a way of measuring drift

  ROSNodeHandle _node_handle;

  std::array<StreamPub, Stream::END> _stream_publishers;
  std::array<CameraInfoPub, Stream::IMAGE_END> _camera_info_publishers;
  std::array<CameraInfoManagerPtr, Stream::IMAGE_END> _camera_info_managers;

  // service, subscriptions and timer
  ros_agnostic::Service _camera_info_default;
  ros_agnostic::Subscription _af_ctrl_sub;
  ros_agnostic::Subscription _disparity_conf_sub;
  ros_agnostic::Timer _camera_read_timer;
};

//==============================================================================
/// Get image data from packet
///
/// \return encoding and img data pair
const std::pair<std::string, cv_bridge::CvImage> get_image_data(
  const HostDataPacket& packet,
  const Stream& type);

}  // namespace rr

#endif
