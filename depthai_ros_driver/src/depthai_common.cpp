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

#include <regex>

// general 3rd party includes
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/opencv.hpp>

#include <depthai_ros_driver/depthai_common.hpp>

namespace rr {

//==============================================================================
const std::pair<std::string, cv_bridge::CvImage> get_image_data(
  const HostDataPacket& packet,
  const Stream& type)
{
  std::pair<std::string, cv_bridge::CvImage> img_data;
  cv_bridge::CvImage cvImg;
  std::string encoding = "";

  const int rows = packet.dimensions[0];
  const int cols = packet.dimensions[1];

  // cv::Mat needs `void *` not `const void *`
  auto* data = const_cast<uint8_t*>(packet.getData());

  switch (type)
  {
    case Stream::LEFT:
    case Stream::RIGHT:
    case Stream::RECTIFIED_LEFT:
    case Stream::RECTIFIED_RIGHT:
    case Stream::DISPARITY:
      cvImg.image = cv::Mat(rows, cols, CV_8UC1, data);
      encoding = "mono8";
      break;
    case Stream::PREVIEW_OUT: {
      std::vector<cv::Mat> toMerge(3);
      const auto offset = cols * cols;
      for (int i = 0; i < 3; ++i)
      {
        toMerge[i] = cv::Mat(cols, cols, CV_8UC1, data + i * offset);
      }
      cv::merge(toMerge, cvImg.image);
      encoding = "bgr8";
      break;
    }
    case Stream::JPEG_OUT:
    case Stream::VIDEO: {
      // TODO<YL> deal with this
      // const auto img = boost::make_shared<sensor_msgs::CompressedImage>();
      // img->header = std::move(cvImg.header);
      // img->format = "jpeg";
      // img->data.assign(packet.data->cbegin(), packet.data->cend());
      // pubPtr->publish(img);
      // return;
      break;
    }
    case Stream::DEPTH:
    case Stream::DISPARITY_COLOR: {
      const int ndim = packet.dimensions.size();
      const int elemSize = packet.elem_size;

      if (ndim == 2)
      {
        if (elemSize == 1)
        {
          cvImg.image = cv::Mat(rows, cols, CV_8UC1, data);
          encoding = "mono8";
        }
        else    // depth
        {
          cv::Mat bgrImg;
          cv::Mat monoImg(rows, cols, CV_8UC1, data);
          cv::applyColorMap(monoImg, bgrImg, cv::COLORMAP_HOT);
          // cv::applyColorMap(monoImg, bgrImg, cv::COLORMAP_JET);
          cvImg.image = bgrImg;
          encoding = "bgr8";
        }
      }
      else    // disparity_color
      {
        cvImg.image = cv::Mat(rows, cols, CV_8UC3, data);
        encoding = "bgr8";
      }
      break;
    }
  }
  img_data.first = encoding;
  img_data.second = cvImg;
  return img_data;
}

//==============================================================================
const std::string DepthAICommon::create_pipeline_config()
{
  boost::property_tree::ptree root, streams, depth, ai, board_config,
    camera, camera_rgb, camera_mono, video_config, app, ot;
  std::regex reg("\"(null|true|false|[0-9]+(\\.[0-9]+)?)\"");
  std::ostringstream oss;
  depth.put<std::string>("calibration_file", _cfg.calib_file);
  depth.put("padding_factor", 0.3f);

  ai.put("blob_file", _cfg.blob_file);
  ai.put("blob_file_config", _cfg.blob_file_config);
  //    ai.put("blob_file2", _blob_file2);
  //    ai.put("blob_file_config2", _blob_file_config2);
  ai.put("calc_dist_to_bb", _cfg.compute_bbox_depth);
  ai.put("keep_aspect_ratio", !_cfg.full_fov_nn);
  //    ai.put("camera_input", "left_right");
  ai.put("camera_input", "rgb");
  ai.put("shaves", _cfg.shaves);
  ai.put("cmx_slices", _cfg.cmx_slices);
  ai.put("NN_engines", _cfg.nn_engines);

  // maximum 20 is supported
  ot.put("max_tracklets", 20);
  // object is tracked only for detections over this threshold
  ot.put("confidence_threshold", 0.5);

  board_config.put("swap_left_and_right_cameras", false);
  board_config.put("left_fov_deg", 69.0f);
  board_config.put("left_to_right_distance_cm", 10.93f);
  board_config.put("left_to_rgb_distance_cm", 22.75f);

  camera_rgb.put("resolution_h", _cfg.rgb_height);
  camera_rgb.put("fps", _cfg.rgb_fps);
  camera_mono.put("resolution_h", _cfg.depth_height);
  camera_mono.put("fps", _cfg.depth_fps);

  camera.add_child("rgb", camera_rgb);
  camera.add_child("mono", camera_mono);

  video_config.put("profile", "mjpeg");
  video_config.put("quality", 95);

  app.put("sync_video_meta_streams", _cfg.sync_video_meta);

  for (const auto& stream : _cfg.stream_list)
  {
    // std::cout << "Requested Streams: " << _stream_list[i] << std::endl;
    boost::property_tree::ptree stream_config;
    stream_config.put("", stream);
    streams.push_back(std::make_pair("", stream_config));
  }

  root.add_child("streams", streams);
  root.add_child("depth", depth);
  root.add_child("ai", ai);
  root.add_child("ot", ot);
  root.add_child("board_config", board_config);
  root.add_child("camera", camera);
  root.add_child("video_config", video_config);
  root.add_child("app", app);

  boost::property_tree::write_json(oss, root);
  return std::regex_replace(oss.str(), reg, "$1");
}

//==============================================================================
DepthAICommon::DepthAICommon(
  const ROSNodeHandle nh, const ROSNodeHandle p_nh):
  _node_interface(nh)
{
  using namespace ros_agnostic;
  get_param(p_nh, "calibration_file", _cfg.calib_file);
  get_param(p_nh, "blob_file", _cfg.blob_file);
  get_param(p_nh, "blob_file_config", _cfg.blob_file_config);
  get_param(p_nh, "stream_list", _cfg.stream_list);
  get_param(p_nh, "depthai_block_read", _cfg.depthai_block_read);
  get_param(p_nh, "enable_sync", _cfg.sync_video_meta);
  get_param(p_nh, "full_fov_nn", _cfg.full_fov_nn);
  get_param(p_nh, "disable_depth", _cfg.compute_bbox_depth);
  get_param(p_nh, "force_usb2", _cfg.force_usb2);
  get_param(p_nh, "rgb_height", _cfg.rgb_height);
  get_param(p_nh, "rgb_fps", _cfg.rgb_fps);
  get_param(p_nh, "depth_height", _cfg.depth_height);
  get_param(p_nh, "depth_fps", _cfg.depth_fps);
  get_param(p_nh, "shaves", _cfg.shaves);
  get_param(p_nh, "cmx_slices", _cfg.cmx_slices);
  get_param(p_nh, "nn_engines", _cfg.nn_engines);
  get_param(p_nh, "camera_param_uri", _cfg.camera_param_uri);
  get_param(p_nh, "camera_name", _cfg.camera_name);
  get_param(p_nh, "queue_size", _cfg.queue_size);

  if (_cfg.camera_param_uri.back() != '/')
    _cfg.camera_param_uri += "/";

  _depthai = std::make_unique<Device>("", _cfg.force_usb2);
  _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(4));

  const auto _pipeline_config_json = create_pipeline_config();
  _pipeline = _depthai->create_pipeline(_pipeline_config_json);
  this->create_stream_publishers();

  // autofocus 'service' subscriber
  _af_ctrl_sub = _node_interface.create_subscription<AutoFocusCtrlMsg>(
    SetAutoFocusTopicName, _cfg.queue_size,
    [&](const AutoFocusCtrlMsg::ConstPtr msg)
    {
      this->set_autofocus(msg->trigger_auto_focus, msg->auto_focus_mode);
    });

  // disparity_confidence 'service' subscriber
  _disparity_conf_sub = _node_interface.create_subscription<Float32Msg>(
    SetDisparityTopicName, _cfg.queue_size,
    [&](const Float32Msg::ConstPtr msg)
    {
      this->set_disparity(msg->data);
    });

  // Service to Trigger default camera param
  _camera_info_default = _node_interface.create_service<TriggerSrv>(
    ResetCameraServiceName,
    [&](
      const std::shared_ptr<TriggerSrv::Request> req,
      std::shared_ptr<TriggerSrv::Response> res)
    {
      const auto& name = req->name;
      res->success = this->set_camera_info_manager(name, "default/");
    });

  // periodic callback timer to publish stream packets
  _camera_read_timer = _node_interface.create_timer(1. / 500,
    [&]()
    {
      this->process_and_publish_packets();
    });
}

//==============================================================================
void DepthAICommon::create_stream_publishers()
{
  // Lambda function to create stream publishers
  auto set_stream_pub_method = [&](
    const Stream& id, const std::string& topic_name, auto msg_type)
    {
      std::string suffix;
      if (id < Stream::IMAGE_END)
      {
        // set camera info publisher
        auto pub = _node_interface.create_publisher<CameraInfoMsg>(
            topic_name + "/camera_info", _cfg.queue_size);
        _camera_info_publishers[id] =
          std::make_shared<ros_agnostic::Publisher>(pub);
        set_camera_info_manager(topic_name, _cfg.camera_name + "/");
        // set stream topic name
        suffix =
          (id < Stream::UNCOMPRESSED_IMG_END) ? "/image_raw" : "/compressed";
      }

      using type = decltype(msg_type);
            auto pub = _node_interface.create_publisher<type>(
        topic_name + suffix, _cfg.queue_size);
      _stream_publishers[id] = std::make_shared<ros_agnostic::Publisher>(pub);
    };

  // loop through selected stream list and create stream
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
          ROS_LOGGER("Unknown stream requested: %s", stream.c_str());
      }
    }
  }
}

//==============================================================================
void DepthAICommon::process_and_publish_packets()
{
  std::list<std::shared_ptr<NNetPacket>> nnet_packet;
  std::list<std::shared_ptr<HostDataPacket>> data_packet;

  if (_cfg.request_jpegout)
    _depthai->request_jpeg();

  if (_pipeline != NULL)
    tie(nnet_packet, data_packet) =
      _pipeline->getAvailableNNetAndDataPackets(_cfg.depthai_block_read);

  if (nnet_packet.size() != 0)
  {
    for (const std::shared_ptr<NNetPacket>& packet : nnet_packet)
    {
      auto detections = packet->getDetectedObjects();
      if (detections == nullptr)
      {
        continue;
      }
      auto meta_data = packet->getMetadata();
      const auto seq_num = meta_data->getSequenceNum();
      const auto ts = meta_data->getTimestamp();
      publishObjectInfoMsg(*detections.get(), get_rostime(ts));
    }
  }
  if (data_packet.size() != 0)
  {
    for (const std::shared_ptr<HostDataPacket>& packet : data_packet)
    {
      if (packet == nullptr)
      {
        // just a sanity check to prevent null dereference
        continue;
      }
      const auto& name = packet->stream_name;
      const auto stream = std::find(_stream_name.cbegin(),
          _stream_name.cend(), name);
      if (stream == _stream_name.end())
      {
        continue;
      }

      const auto index = std::distance(_stream_name.cbegin(), stream);
      if (index == Stream::VIDEO)
      {
        // workaround for the bug in DepthAI-Core library
        // use -1 to represent use rostime
        publishImageMsg(
          *(packet.get()), static_cast<Stream>(index), get_rostime(-1));
        continue;
      }

      auto meta_data = packet->getMetadata();
      const auto seq_num = meta_data->getSequenceNum();
      const auto ts = meta_data->getTimestamp();
      publishImageMsg(
        *(packet.get()), static_cast<Stream>(index), get_rostime(ts));
    }
  }
}

//==============================================================================
void DepthAICommon::publishObjectInfoMsg(
  const dai::Detections& detections, const RosTime& stamp)
{
  const auto pubPtr =
    _stream_publishers[Stream::META_OUT]; // TODO .get()?

  if (!pubPtr)
    return;// Not avail

  auto msg = convert(detections);
  msg.header.stamp = stamp;
  pubPtr->publish(msg);
}

//==============================================================================
void DepthAICommon::publishImageMsg(
  const HostDataPacket& packet, Stream type, const RosTime& stamp)
{
  HeaderMsg header;
  header.stamp = stamp;
  header.frame_id = packet.stream_name;

  const auto camInfoPubPtr = _camera_info_publishers[type];
  if (camInfoPubPtr)
  {
    auto msg = get_camera_info_msg(type);
    msg.header = header;
    camInfoPubPtr->publish(msg);
  }

  // check if to publish it out as Compressed or ImageMsg
  if (type == Stream::JPEG_OUT || type == Stream::VIDEO)
  {
    const auto pubPtr = _stream_publishers[type];

    if (!pubPtr)
      return;// Not avail

    const auto img = std::make_shared<CompressedImageMsg>();
    img->header = std::move(header);
    img->format = "jpeg";
    img->data.assign(packet.data->cbegin(), packet.data->cend());
    pubPtr->publish(*img);
    return;
  }
  else
  {
    const auto pubPtr = _stream_publishers[type];

    if (!pubPtr)
      return;// Not avail

    const auto img_data = get_image_data(packet, type);
    if (img_data.first.empty())
      return;

    const auto msg = img_data.second.toImageMsg();
    msg->encoding = img_data.first;
    msg->header = std::move(header);
    pubPtr->publish(*msg);
  }
}

//==============================================================================
bool DepthAICommon::set_disparity(const float val)
{
  if ((val >= 0.0) && (val <= 255.0))
  {
    _depthai->send_disparity_confidence_threshold(val);
    return true;
  }
  ROS_LOGGER("Disparity confidence value:%f, is invalid", val);
  return false;
}

//==============================================================================
bool DepthAICommon::set_autofocus(const bool trigger, const uint8_t mode)
{
  if (trigger)
    _depthai->request_af_trigger();

  if ((mode >= 0) && (mode <= 4))
  {
    _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(mode));
    return true;
  }
  ROS_LOGGER("Invalid Auto Focus mode requested");
  return false;
}

//==============================================================================
ObjectsMsg DepthAICommon::convert(const dai::Detections& detections)
{
  ObjectsMsg msg;
  ObjectMsg object;
  msg.objects.reserve(detections.detection_count);

  for (int i = 0; i < detections.detection_count; ++i)
  {
    object.label_id = detections.detections[i].label;
    object.confidence = detections.detections[i].confidence;
    object.bb.x_min = detections.detections[i].x_min;
    object.bb.y_min = detections.detections[i].y_min;
    object.bb.x_max = detections.detections[i].x_max;
    object.bb.y_max = detections.detections[i].y_max;

    if (_cfg.compute_bbox_depth)
    {
      object.bb.depth_x = detections.detections[i].depth_x;
      object.bb.depth_y = detections.detections[i].depth_y;
      object.bb.depth_z = detections.detections[i].depth_z;
    }
    msg.objects.push_back(object);
  }
  return msg;
}

//==============================================================================
const bool DepthAICommon::set_camera_info_manager(
  const std::string& name,
  const std::string& prefix)
{
  ROS_LOGGER(" setting camera info manager: %s", name.c_str());
  const auto uri = _cfg.camera_param_uri + prefix + name + ".yaml";

  const auto it = std::find(_topic_names.cbegin(), _topic_names.cend(), name);
  if (it == _topic_names.cend())
    return false;

  const auto idx = std::distance(_topic_names.cbegin(), it);

  /// Check if camera info manager is initialized
  if (_camera_info_managers[idx])
  {
    return _camera_info_managers[idx]->setCameraName(name) &&
      _camera_info_managers[idx]->loadCameraInfo(uri);
  }
  else
  {
    auto node_handle = _node_interface.get_node_handle();
    #if defined(USE_ROS2)   
    const ROSNodeHandle& nh_ptr = node_handle->create_sub_node(name);
    auto lnh = nh_ptr.get();
    #else
    auto lnh = ros::NodeHandle{*node_handle, name};
    #endif
    
    /// input arg: lnh is nodehandle for ros1, and raw node ptr in ros2
    _camera_info_managers[idx] =
      std::make_shared<camera_info_manager::CameraInfoManager>(lnh, name, uri);
    return true;
  }
}

//==============================================================================
CameraInfoMsg DepthAICommon::get_camera_info_msg(const Stream& id)
{
  return _camera_info_managers[id]->getCameraInfo();
}

//==============================================================================
const DepthAICommonConfig& DepthAICommon::get_config()
{
  return _cfg;
}

//==============================================================================
const RosTime DepthAICommon::get_rostime(const double camera_ts)
{
  // only during init
  if (_depthai_init_ts == -1)
  {
    _depthai_init_ts = camera_ts;
    _stamp = _node_interface.current_time();
  }
  return _stamp + RosDuration(camera_ts - _depthai_init_ts);
}

}  // namespace rr
