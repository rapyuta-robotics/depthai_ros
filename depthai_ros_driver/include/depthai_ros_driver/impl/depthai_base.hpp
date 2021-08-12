#include <regex>

#include <ros/ros.h>
#include <depthai_ros_driver/depthai_base.hpp>

namespace rr {

template <class Node>
void DepthAIBase<Node>::publishObjectInfoMsg(const dai::Detections& detections, const ros::Time& stamp) {
    const auto* pubPtr = _stream_publishers[Stream::META_OUT].get();
    if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0) {
        return;  // No subscribers
    }

    auto msg = _depthai_common->convert(detections);
    msg.header.stamp = stamp;
    pubPtr->publish(msg);
}

template <class Node>
void DepthAIBase<Node>::publishImageMsg(const HostDataPacket& packet, Stream type, const ros::Time& stamp) {
    const auto* camInfoPubPtr = _camera_info_publishers[type].get();
    const auto* pubPtr = _stream_publishers[type].get();

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = packet.stream_name;

    if (camInfoPubPtr->getNumSubscribers() > 0) {
        const auto cameraInfo =
                boost::make_shared<sensor_msgs::CameraInfo>(_camera_info_manager[type]->getCameraInfo());
        cameraInfo->header = header;
        camInfoPubPtr->publish(cameraInfo);
    }

    if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0) {
        return;  // No subscribers
    }

    if (type == Stream::JPEG_OUT || type == Stream::JPEG_OUT){
        const auto img = boost::make_shared<sensor_msgs::CompressedImage>();
        img->header = std::move(header);
        img->format = "jpeg";
        img->data.assign(packet.data->cbegin(), packet.data->cend());
        pubPtr->publish(img);
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

template <class Node>
bool DepthAIBase<Node>::defaultCameraInfo(
        depthai_ros_msgs::TriggerNamed::Request& req, depthai_ros_msgs::TriggerNamed::Response& res) {
    const auto it = std::find(_topic_name.cbegin(), _topic_name.cend(), req.name);
    if (it == _topic_name.cend()) {
        res.success = false;
        res.message = "No such camera known";
        return true;
    }

    const auto& name = req.name;
    auto& nh = this->getPrivateNodeHandle();
    const auto uri = _camera_param_uri + "default/" + name + ".yaml";
    if (_defaultManager == nullptr) {
        _defaultManager =
                std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, "_default"}, name, uri);
    } else {
        _defaultManager->setCameraName(name);
        _defaultManager->loadCameraInfo(uri);
    }
    const auto index = std::distance(_topic_name.cbegin(), it);
    const auto cameraInfo = _defaultManager->getCameraInfo();
    res.success = _camera_info_manager[index]->setCameraInfo(cameraInfo);

    _defaultManager->setCameraName("_default");
    _defaultManager->loadCameraInfo("");
    return true;
}

template <class Node>
void DepthAIBase<Node>::onInit() {
    auto& nh = this->getPrivateNodeHandle();

    // with templated lambda, replace type with a template-typename
    auto get_param = [&nh](const std::string& name, auto& variable) {
        if (nh.hasParam(name)) {
            nh.getParam(name, variable);
        } else {
            nh.setParam(name, variable);
        }
    };

    get_param("queue_size", _queue_size);
    get_param("camera_name", _camera_name);
    get_param("camera_param_uri", _camera_param_uri);

    if (_camera_param_uri.back() != '/') {
        _camera_param_uri += "/";
    }

    _depthai_common = std::make_unique<DepthAICommon>(get_param);
    _depthai_common->register_callbacks(
        [this](const HostDataPacket& packet, Stream type, double ts){
            this->publishImageMsg(packet, type, this->get_rostime(ts));
        },
        [this](const dai::Detections& detections, double ts){
            this->publishObjectInfoMsg(detections, this->get_rostime(ts));
        }
    );

    prepareStreamConfig();

    _cameraReadTimer = nh.createTimer(ros::Duration(1. / 500),
        [&](const ros::TimerEvent&){
            _depthai_common->process_and_publish_packets();
        });
}

template <class Node>
void DepthAIBase<Node>::prepareStreamConfig() {
    auto& nh = this->getNodeHandle();

    auto set_camera_info_pub = [&](const Stream& id) {
        const auto& name = _topic_name[id];
        const auto uri = _camera_param_uri + _camera_name + "/" + name + ".yaml";
        _camera_info_manager[id] =
                std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, name}, name, uri);
        _camera_info_publishers[id] = std::make_unique<ros::Publisher>(
                nh.template advertise<sensor_msgs::CameraInfo>(name + "/camera_info", _queue_size));
    };
    auto set_stream_pub = [&](const Stream& id, auto message_type) {
        using type = decltype(message_type);
        std::string suffix;
        if (id < Stream::IMAGE_END) {
            suffix = (id < Stream::UNCOMPRESSED_IMG_END) ? "/image_raw" : "/compressed";
        }
        const auto& name = _topic_name[id];
        _stream_publishers[id] =
                std::make_unique<ros::Publisher>(nh.template advertise<type>(name + suffix, _queue_size));
    };

    _depthai_common->create_stream(set_camera_info_pub, set_stream_pub);

    _camera_info_default = nh.advertiseService("reset_camera_info", &DepthAIBase::defaultCameraInfo, this);

    _af_ctrl_sub = nh.template subscribe<depthai_ros_msgs::AutoFocusCtrl>(
        "auto_focus_ctrl", _queue_size,
        [&](const depthai_ros_msgs::AutoFocusCtrlConstPtr& msg) {
            if (!_depthai_common->set_autofocus(
                    msg->trigger_auto_focus, msg->auto_focus_mode)){
                ROS_ERROR_NAMED(this->getName(), "Invalid Auto Focus mode requested");
            }
        });

    _disparity_conf_sub = nh.template subscribe<std_msgs::Float32>(
        "disparity_confidence", _queue_size,
        [&](const std_msgs::Float32::ConstPtr& msg) {
            if (!_depthai_common->set_disparity(msg->data)){
                ROS_ERROR_NAMED(this->getName(),
                    "Disparity confidence value:%f, is invalid", msg->data);
            }
        });
}

template <class Node>
const ros::Time DepthAIBase<Node>::get_rostime(const double camera_ts)
{
    // only during init
    if (_depthai_ts_offset == -1) {
        ros::Time stamp = ros::Time::now();
        _depthai_ts_offset = camera_ts;
        _stamp = stamp;
    }

    if (camera_ts < 0)
        return ros::Time::now();

    return _stamp + ros::Duration(camera_ts - _depthai_ts_offset);
};

}  // namespace rr
