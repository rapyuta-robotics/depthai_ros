#include <ros/ros.h>
#include <depthai_ros_driver/depthai_base.hpp>

namespace rr {

//==============================================================================
template<class Node>
void DepthAIBase<Node>::publishObjectInfoMsg(
  const dai::Detections& detections, const ros::Time& stamp)
{
  const auto* pubPtr = _stream_publishers[Stream::META_OUT].get();
  if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0)
  {
    return;  // No subscribers
  }

  auto msg = _depthai_common->convert(detections);
  msg.header.stamp = stamp;
  pubPtr->publish(msg);
}

//==============================================================================
template<class Node>
void DepthAIBase<Node>::publishImageMsg(
  const HostDataPacket& packet, Stream type, const ros::Time& stamp)
{
  const auto* camInfoPubPtr = _camera_info_publishers[type].get();
  const auto* pubPtr = _stream_publishers[type].get();

  std_msgs::Header header;
  header.stamp = stamp;
  header.frame_id = packet.stream_name;

  if (camInfoPubPtr->getNumSubscribers() > 0)
  {
    const auto cameraInfo =
      boost::make_shared<sensor_msgs::CameraInfo>(
      _camera_info_managers[type]->getCameraInfo());
    cameraInfo->header = header;
    camInfoPubPtr->publish(cameraInfo);
  }

  if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0)
  {
    return;  // No subscribers
  }

  if (type == Stream::JPEG_OUT || type == Stream::VIDEO)
  {
    const auto img = std::make_shared<sensor_msgs::CompressedImage>();
    img->header = std::move(header);
    img->format = "jpeg";
    img->data.assign(packet.data->cbegin(), packet.data->cend());
    pubPtr->publish(*img);
    return;
  }

  const auto img_data = get_image_data(packet, type);
  if (img_data.first.empty())
    return;

  const sensor_msgs::ImagePtr msg = img_data.second.toImageMsg();
  msg->encoding = img_data.first;
  msg->header = std::move(header);
  pubPtr->publish(msg);
}

//==============================================================================
// TODO: check if refactoring makes sense
template<class Node>
bool DepthAIBase<Node>::defaultCameraInfo(
  depthai_ros_msgs::TriggerNamed::Request& req,
  depthai_ros_msgs::TriggerNamed::Response& res)
{
  const auto& name = req.name;
  const auto it =
    std::find(_topic_names.cbegin(), _topic_names.cend(), req.name);
  const auto index = std::distance(_topic_names.cbegin(), it);

  if (it == _topic_names.cend())
  {
    res.success = false;
    res.message = "No such camera known";
    return true;
  }

  if (!_camera_info_managers[index])   // check if nullopt
  {
    res.success = false;
    res.message = "stream index is not valid";
    return true;
  }

  const auto uri = _camera_param_uri + "default/" + name + ".yaml";
  res.success = (
    _camera_info_managers[index]->setCameraName(name) &&
    _camera_info_managers[index]->loadCameraInfo(uri));
  return true;
}

//==============================================================================
template<class Node>
void DepthAIBase<Node>::onInit()
{
  auto& nh = this->getPrivateNodeHandle();

  // with templated lambda, replace type with a template-typename
  auto get_param = [&nh](const std::string& name, auto& variable)
    {
      if (nh.hasParam(name))
      {
        nh.getParam(name, variable);
      }
      else
      {
        nh.setParam(name, variable);
      }
    };

  get_param("queue_size", _queue_size);
  get_param("camera_name", _camera_name);
  get_param("camera_param_uri", _camera_param_uri);

  if (_camera_param_uri.back() != '/')
  {
    _camera_param_uri += "/";
  }

  _depthai_common = std::make_unique<DepthAICommon>(get_param);
  _depthai_common->register_callbacks(
    [this](const HostDataPacket& packet, Stream type, double ts)
    {
      this->publishImageMsg(packet, type, this->get_rostime(ts));
    },
    [this](const dai::Detections& detections, double ts)
    {
      this->publishObjectInfoMsg(detections, this->get_rostime(ts));
    }
  );

  _topic_names = _depthai_common->get_topic_names();

  prepareStreamConfig();

  _cameraReadTimer = nh.createTimer(ros::Duration(1. / 500),
      [&](const ros::TimerEvent&)
      {
        _depthai_common->process_and_publish_packets();
      });
}

//==============================================================================
template<class Node>
void DepthAIBase<Node>::prepareStreamConfig()
{
  auto& nh = this->getNodeHandle();
  auto set_stream_pub = [&](const Stream& id, auto message_type)
    {
      const auto& name = _topic_names[id];
      std::string suffix;

      // if is image stream
      if (id < Stream::IMAGE_END)
      {
        // set camera info publisher
        const auto uri = _camera_param_uri + _camera_name + "/" + name +
          ".yaml";
        _camera_info_managers[id] =
          std::make_unique<camera_info_manager::CameraInfoManager>(
          ros::NodeHandle{nh, name}, name, uri);
        _camera_info_publishers[id] = std::make_unique<ros::Publisher>(
          nh.template advertise<sensor_msgs::CameraInfo>(name + "/camera_info",
          _queue_size));

        // set stream topic name
        suffix =
          (id < Stream::UNCOMPRESSED_IMG_END) ? "/image_raw" : "/compressed";
      }

      using type = decltype(message_type);
      _stream_publishers[id] =
        std::make_unique<ros::Publisher>(nh.template advertise<type>(name +
          suffix, _queue_size));
    };

  _depthai_common->create_stream_publishers(set_stream_pub);

  _camera_info_default = nh.advertiseService("reset_camera_info",
      &DepthAIBase::defaultCameraInfo,
      this);

  _af_ctrl_sub = nh.template subscribe<depthai_ros_msgs::AutoFocusCtrl>(
    "auto_focus_ctrl", _queue_size,
    [&](const depthai_ros_msgs::AutoFocusCtrlConstPtr& msg)
    {
      if (!_depthai_common->set_autofocus(
        msg->trigger_auto_focus, msg->auto_focus_mode))
      {
        ROS_ERROR_NAMED(this->getName(), "Invalid Auto Focus mode requested");
      }
    });

  _disparity_conf_sub = nh.template subscribe<std_msgs::Float32>(
    "disparity_confidence", _queue_size,
    [&](const std_msgs::Float32::ConstPtr& msg)
    {
      if (!_depthai_common->set_disparity(msg->data))
      {
        ROS_ERROR_NAMED(this->getName(),
        "Disparity confidence value:%f, is invalid", msg->data);
      }
    });
}

//==============================================================================
template<class Node>
const ros::Time DepthAIBase<Node>::get_rostime(const double camera_ts)
{
  // only during init
  if (_depthai_ts_offset == -1)
  {
    ros::Time stamp = ros::Time::now();
    _depthai_ts_offset = camera_ts;
    _stamp = stamp;
  }

  if (camera_ts < 0)
    return ros::Time::now();

  return _stamp + ros::Duration(camera_ts - _depthai_ts_offset);
}

}  // namespace rr
