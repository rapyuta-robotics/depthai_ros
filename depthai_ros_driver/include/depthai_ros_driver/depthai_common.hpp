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

// std includes
#include <algorithm>
#include <array>
#include <memory>
#include <string>

#if defined(USE_ROS2)
  #include <std_msgs/msg/float32.hpp>
  #include <sensor_msgs/msg/image.hpp>
  #include <depthai_ros_msgs/srv/trigger_named.hpp>
  #include <depthai_ros_msgs/msg/objects.hpp>
  #include <depthai_ros_msgs/msg/auto_focus_ctrl.hpp>
  #include <camera_info_manager/camera_info_manager.hpp>
  #include <rclcpp/node.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using ObjectsMsg = depthai_ros_msgs::msg::Objects;
using ObjectMsg = depthai_ros_msgs::msg::Object;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using ROSNodeHandle = rclcpp::Node*;
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
using ROSNodeHandle = ros::NodeHandle;
#endif

using CameraInfoManagerPtr =
  std::shared_ptr<camera_info_manager::CameraInfoManager>;

namespace rr {

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
/// Get image data from packet
///
/// \return encoding and img data pair
const std::pair<std::string, cv_bridge::CvImage> get_image_data(
  const HostDataPacket& packet,
  const Stream& type);

//==============================================================================
class DepthAICommon
{
public:
  /// Initiaize DepthAICommon
  ///
  /// \param get_param_method:
  ///    lambda std::function<void(string, auto)>, in which the auto should
  ///    support static types of: bool, string, int, vector<string>
  template<typename T>
  DepthAICommon(T get_param_method);

  /// Create camera stream publisher
  ///
  /// \param set_stream_pub_method
  ///    lambda to set a single stream publisher
  template<typename T>
  void create_stream_publishers(T set_stream_pub_method);

  using PublishImageFn =
    std::function<void(const HostDataPacket&, Stream type, double ts)>;
  using PublishObjectsFn =
    std::function<void(const dai::Detections& detections, double ts)>;

  /// Callback funcs which will be called when process_and_publish_packets()
  /// received the relevant packets
  ///
  /// \param img_fn
  ///   impementation function to publish the image
  ///
  /// \param objs_fn
  ///   impementation function to publish detection objs
  void register_callbacks(PublishImageFn img_fn, PublishObjectsFn objs_fn);

  /// get and process packets
  void process_and_publish_packets();

  /// set camera disparity threshold
  bool set_disparity(const float val);

  /// set camera autofocus
  bool set_autofocus(const bool trigger, const uint8_t mode);

  /// create detection to object msg
  ObjectsMsg convert(const dai::Detections& detections);

  /// Create pipeline config string stream
  const std::string create_pipeline_config();

  /// Get topic names
  const std::array<std::string, Stream::END>& get_topic_names();

  /// set camera info manager
  const bool set_camera_info_manager(
    const Stream& id,
    const std::string& name,
    const std::string& prefix,
    const ROSNodeHandle node_handle);

  /// get camera info msg
  CameraInfoMsg get_camera_info_msg(const Stream& id);

private:

  // internal
  struct DepthAIBaseConfig
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
    std::string camera_param_uri =
      "package://depthai_ros_driver/params/camera/";
    std::vector<std::string> stream_list = {"video", "left", "depth"};
  };

  DepthAIBaseConfig _cfg;
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

  PublishImageFn _publish_img_fn;
  PublishObjectsFn _publish_objs_fn;
  std::array<CameraInfoManagerPtr, Stream::IMAGE_END> _camera_info_managers;
};

//==============================================================================
//==============================================================================
// TODO Move these to impl hpp
template<typename T>
void DepthAICommon::create_stream_publishers(T set_stream_pub_method)
{
  for (const auto& stream : _cfg.stream_list)
  {
    const auto it =
      std::find(_stream_name.cbegin(), _stream_name.cend(), stream);
    const auto index =
      static_cast<Stream>(std::distance(_stream_name.cbegin(), it));
    const auto& topic_name = _topic_names[index];

    if (index < Stream::IMAGE_END)
    {
      if (index < Stream::UNCOMPRESSED_IMG_END)
        set_stream_pub_method(index, topic_name, ImageMsg{});
      else
        set_stream_pub_method(index, topic_name, CompressedImageMsg{});
    }
    else
    {
      switch (index)
      {
        case Stream::META_OUT:
          set_stream_pub_method(index, topic_name, ObjectsMsg{});
          break;
        case Stream::OBJECT_TRACKER:
          set_stream_pub_method(index, topic_name, ObjectMsg{});
          break;
        case Stream::META_D2H:
          set_stream_pub_method(index, topic_name, ImageMsg{});
          break;
        default:
          // TODO: roslog?
          std::cout << "Unknown stream requested: "
                    << stream << std::endl;
      }
    }
  }
}

//==============================================================================
template<typename T>
DepthAICommon::DepthAICommon(T get_param_method)
{
  get_param_method("calibration_file", _cfg.calib_file);
  get_param_method("blob_file", _cfg.blob_file);
  get_param_method("blob_file_config", _cfg.blob_file_config);
  get_param_method("stream_list", _cfg.stream_list);
  get_param_method("depthai_block_read", _cfg.depthai_block_read);
  get_param_method("enable_sync", _cfg.sync_video_meta);
  get_param_method("full_fov_nn", _cfg.full_fov_nn);
  get_param_method("disable_depth", _cfg.compute_bbox_depth);
  get_param_method("force_usb2", _cfg.force_usb2);
  get_param_method("rgb_height", _cfg.rgb_height);
  get_param_method("rgb_fps", _cfg.rgb_fps);
  get_param_method("depth_height", _cfg.depth_height);
  get_param_method("depth_fps", _cfg.depth_fps);
  get_param_method("shaves", _cfg.shaves);
  get_param_method("cmx_slices", _cfg.cmx_slices);
  get_param_method("nn_engines", _cfg.nn_engines);
  get_param_method("camera_param_uri", _cfg.camera_param_uri);

  if (_cfg.camera_param_uri.back() != '/')
    _cfg.camera_param_uri += "/";

  _depthai = std::make_unique<Device>("", _cfg.force_usb2);
  _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(4));

  const auto _pipeline_config_json = create_pipeline_config();
  _pipeline = _depthai->create_pipeline(_pipeline_config_json);
}

}  // namespace rr

#endif
