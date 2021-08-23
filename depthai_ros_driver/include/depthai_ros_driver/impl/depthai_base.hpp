#include <ros/ros.h>
#include <depthai_ros_driver/depthai_base.hpp>

namespace rr {

//==============================================================================
template<class Node>
void DepthAIBase<Node>::onInit()
{
  auto& p_nh = this->getPrivateNodeHandle();
  auto& nh = this->getNodeHandle();

  // with templated lambda, replace type with a template-typename
  auto get_param = [&p_nh](const std::string& name, auto& variable)
    {
      if (p_nh.hasParam(name))
        p_nh.getParam(name, variable);
      else
        p_nh.setParam(name, variable);
    };

  get_param("queue_size", _queue_size);
  get_param("camera_name", _camera_name);

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

  _cameraReadTimer = p_nh.createTimer(ros::Duration(1. / 500),
      [&](const ros::TimerEvent&)
      {
        _depthai_common->process_and_publish_packets();
      });

  // provide lambda function to construct stream msg publishers
  _depthai_common->create_stream_publishers(
    [&](const Stream& id, const std::string& topic_name, auto message_type)
    {
      std::string suffix;
      // if is image stream
      if (id < Stream::IMAGE_END)
      {
        // set camera info publisher
        _depthai_common->set_camera_info_manager(
          id, topic_name, _camera_name + "/", ros::NodeHandle{nh, topic_name});
        _camera_info_publishers[id] = std::make_unique<ros::Publisher>(
          nh.template advertise<sensor_msgs::CameraInfo>(
            topic_name + "/camera_info", _queue_size));

        // set stream topic name
        suffix =
        (id < Stream::UNCOMPRESSED_IMG_END) ? "/image_raw" : "/compressed";
      }

      using type = decltype(message_type);
      _stream_publishers[id] =
      std::make_unique<ros::Publisher>(nh.template advertise<type>(
        topic_name + suffix, _queue_size));
    });

  // create Trigger default camera param service
  _camera_info_default = nh.template
    advertiseService<TriggerSrv::Request, TriggerSrv::Response>(
    "reset_camera_info",
    [&](TriggerSrv::Request& req, TriggerSrv::Response& res)
    {
      const auto& name = req.name;
      const auto& topic_names = _depthai_common->get_topic_names();
      const auto it =
      std::find(topic_names.cbegin(), topic_names.cend(), req.name);
      const auto index = std::distance(topic_names.cbegin(), it);

      if (it == topic_names.cend())
      {
        res.success = false;
        res.message = "No such camera known";
        return true;
      }

      auto& nh = this->getNodeHandle();
      res.success = _depthai_common->set_camera_info_manager(
        static_cast<Stream>(index), name, "default/", ros::NodeHandle{nh,
          name});
      return true;
    });

  // autofocus 'service' subscriber
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

  // disparity_confidence 'service' subscriber
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
void DepthAIBase<Node>::publishObjectInfoMsg(
  const dai::Detections& detections, const ros::Time& stamp)
{
  const auto* pubPtr = _stream_publishers[Stream::META_OUT].get();
  if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0)
    return;// No subscribers

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
    auto msg = _depthai_common->get_camera_info_msg(type);
    msg.header = header;
    camInfoPubPtr->publish(msg);
  }

  if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0)
    return;// No subscribers

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
template<class Node>
const ros::Time DepthAIBase<Node>::get_rostime(const double camera_ts)
{
  // only during init
  if (_depthai_init_ts == -1)
  {
    _depthai_init_ts = camera_ts;
    _stamp = ros::Time::now();
  }
  return _stamp + ros::Duration(camera_ts - _depthai_init_ts);
}

}  // namespace rr
