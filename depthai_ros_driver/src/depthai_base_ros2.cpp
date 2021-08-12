#include <depthai_ros_driver/depthai_base_ros2.hpp>

namespace rr {

//==============================================================================
DepthAIBaseRos2::DepthAIBaseRos2(const rclcpp::NodeOptions & options) 
    : Node("depthai_node", options)
{
    auto get_param = [this](const std::string& name, auto& variable) {
        using var_type = std::remove_reference_t<decltype(variable)>;
        if (this->has_parameter(name))
            this->get_parameter(name, variable);
        else
            this->declare_parameter<var_type>(name, variable);
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

    _cameraReadTimer = this->create_wall_timer(2ms,
        [this](){
            this->_depthai_common->process_and_publish_packets();
        });
}

//==============================================================================
void DepthAIBaseRos2::prepareStreamConfig()
{
    const auto driver_qos = rclcpp::ServicesQoS().best_effort();

    // create service
    _camera_info_default = this->create_service<TriggerSrv>(
        "reset_camera_info", 
        [this](
            const std::shared_ptr<TriggerSrv::Request> req,
            std::shared_ptr<TriggerSrv::Response> res)
        {
            const auto it = std::find(
                _topic_name.cbegin(), _topic_name.cend(), req->name);
            if (it == _topic_name.cend()) {
                res->success = false;
                res->message = "No such camera known";
                return;
            }

            const auto& name = req->name;
            const auto uri = _camera_param_uri + "default/" + name + ".yaml";

            // set camera info
            if (!_defaultManager) {
                _defaultManager = 
                    std::make_unique<camera_info_manager::CameraInfoManager>(
                        this, name, uri);
            }
            else
            {
                _defaultManager->setCameraName(name);
                _defaultManager->loadCameraInfo(uri);
            }

            const auto index = std::distance(_topic_name.cbegin(), it);
            const auto cameraInfo = _defaultManager->getCameraInfo();
            res->success = _camera_info_manager[index]->setCameraInfo(cameraInfo);

            _defaultManager->setCameraName("_default");
            _defaultManager->loadCameraInfo("");
        }
    );
    
    _disparity_conf_sub = this->create_subscription<Float32Msg>(
        "auto_focus_ctrl", driver_qos,
        [&](const Float32Msg::UniquePtr msg)
        {
        if (!_depthai_common->set_disparity(msg->data))
            RCLCPP_WARN(this->get_logger(),
                "Disparity confidence value:%f, is invalid", msg->data);
        });

    _af_ctrl_sub = this->create_subscription<AutoFocusCtrlMsg>(
        "disparity_confidence", driver_qos,
        [&](const AutoFocusCtrlMsg::UniquePtr msg)
        {
            if (!_depthai_common->set_autofocus(
                msg->trigger_auto_focus, msg->auto_focus_mode))
            RCLCPP_WARN(this->get_logger(), "Invalid Auto Focus mode requested");
        });

    // lambda function to construct camera info publishers
    auto set_camera_info_pub = [&](const Stream& id) {
        const auto& name = _topic_name[id];
        const auto uri = _camera_param_uri + _camera_name + "/" + name + ".yaml";
        _camera_info_manager[id] =
            std::make_unique<camera_info_manager::CameraInfoManager>(
                this, name, uri);
        _camera_info_publishers[id] = this->create_publisher<CameraInfoMsg>(
            name + "/camera_info", _queue_size);
    };

    // lambda function to construct stream msg publishers
    auto set_stream_pub = [&](const Stream& id, auto message_type) {
        const auto& name = _topic_name[id];
        using type = decltype(message_type);
        std::string suffix;
        if (id < Stream::IMAGE_END) {
            suffix = (id < Stream::UNCOMPRESSED_IMG_END) ? "/image_raw" : "/compressed";
        }

        _stream_publishers[id] = 
            this->create_publisher<type>(name + suffix, _queue_size);
    };

    _depthai_common->create_stream(set_camera_info_pub, set_stream_pub);
}

//==============================================================================
void DepthAIBaseRos2::publishObjectInfoMsg(
    const dai::Detections& detections, const rclcpp::Time& stamp)
{
    const auto pubPtr = std::get<ObjectsPubPtr>(_stream_publishers[Stream::META_OUT]);

    // check subsribers num, TODO check performance
    if (!pubPtr || pubPtr->get_subscription_count() == 0)
        return;  // No subscribers

    // Note: Duplicatied Method as ROS1
    auto msg = _depthai_common->convert(detections);
    msg.header.stamp = stamp;
    pubPtr->publish(msg);
}

//==============================================================================
void DepthAIBaseRos2::publishImageMsg(
    const HostDataPacket& packet, Stream type, const rclcpp::Time& stamp)
{
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = packet.stream_name;

    const auto camInfoPubPtr = _camera_info_publishers[type];

    // publish camera info
    if(camInfoPubPtr->get_subscription_count() > 0)
    {
        auto msg = _camera_info_manager[type]->getCameraInfo();
        msg.header = header;
        camInfoPubPtr->publish(msg);
    }

    // check if to publish it out as Compressed or ImageMsg
    if (type == Stream::JPEG_OUT || type == Stream::JPEG_OUT){
        const auto pubPtr = std::get<ComImagePubPtr>(_stream_publishers[type]);
        
        if (!pubPtr || pubPtr->get_subscription_count() == 0)
            return; // No subscribers;
    
        const auto img = boost::make_shared<CompressedImageMsg>();
        img->header = std::move(header);
        img->format = "jpeg";
        img->data.assign(packet.data->cbegin(), packet.data->cend());
        pubPtr->publish(*img);
        return;
    }
    else
    {
        const auto pubPtr = std::get<ImagePubPtr>(_stream_publishers[type]);

        if (!pubPtr || pubPtr->get_subscription_count() == 0)
            return; // No subscribers;

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
const rclcpp::Time DepthAIBaseRos2::get_rostime(const double camera_ts)
{
    // only during init
    if (_depthai_ts_offset == -1) {
        rclcpp::Time stamp = this->now();
        _depthai_ts_offset = camera_ts;
        _stamp = stamp;
    }

    if (camera_ts < 0)
        return this->now();

    return _stamp + rclcpp::Duration(camera_ts - _depthai_ts_offset);
};

}  // namespace rr

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(rr::DepthAIBaseRos2)
