#include <regex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <depthai_ros_driver/depthai_base.hpp>

namespace rr {
template <class Node>
void DepthAIBase<Node>::disparityConfCb(const std_msgs::Float32::ConstPtr& msg) {
    if ((msg->data >= 0.0) && (msg->data <= 255.0)) {
        _depthai->send_disparity_confidence_threshold(msg->data);
    } else {
        ROS_ERROR_NAMED(this->getName(), "Disparity confidence value:%f, is invalid", msg->data);
    }
}

template <class Node>
void DepthAIBase<Node>::afCtrlCb(const depthai_ros_msgs::AutoFocusCtrl msg) {
    if (msg.trigger_auto_focus) {
        _depthai->request_af_trigger();
    }
    if ((msg.auto_focus_mode >= 0) && (msg.auto_focus_mode <= 4)) {
        _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(msg.auto_focus_mode));
    } else {
        ROS_ERROR_NAMED(this->getName(), "Invalid Auto Focus mode requested");
    }
}

template <class Node>
void DepthAIBase<Node>::publishObjectInfoMsg(const dai::Detections& detections, const ros::Time& stamp) {
    const auto* pubPtr = _stream_publishers[Stream::META_OUT].get();
    if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0) {
        return;  // No subscribers
    }

    depthai_ros_msgs::Object object;
    depthai_ros_msgs::Objects msg;
    msg.objects.reserve(detections.detection_count);

    for (int i = 0; i < detections.detection_count; ++i) {
        object.label_id = detections.detections[i].label;
        object.confidence = detections.detections[i].confidence;
        object.bb.x_min = detections.detections[i].x_min;
        object.bb.y_min = detections.detections[i].y_min;
        object.bb.x_max = detections.detections[i].x_max;
        object.bb.y_max = detections.detections[i].y_max;

        if (_compute_bbox_depth) {
            object.bb.depth_x = detections.detections[i].depth_x;
            object.bb.depth_y = detections.detections[i].depth_y;
            object.bb.depth_z = detections.detections[i].depth_z;
        }
        msg.objects.push_back(object);
    }

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

    cv_bridge::CvImage cvImg;
    cvImg.header = std::move(header);
    std::string encoding = "";

    const int rows = packet.dimensions[0];
    const int cols = packet.dimensions[1];

    // cv::Mat needs `void *` not `const void *`
    auto* data = const_cast<uint8_t*>(packet.getData());

    switch (type) {
        case Stream::LEFT:
        case Stream::RIGHT:
        case Stream::DISPARITY:
            cvImg.image = cv::Mat(rows, cols, CV_8UC1, data);
            encoding = "mono8";
            break;
        case Stream::PREVIEW_OUT: {
            std::vector<cv::Mat> toMerge(3);
            const auto offset = cols * cols;
            for (int i = 0; i < 3; ++i) {
                toMerge[i] = cv::Mat(cols, cols, CV_8UC1, data + i * offset);
            }
            cv::merge(toMerge, cvImg.image);
            encoding = "bgr8";
            break;
        }
        case Stream::JPEG_OUT:
        case Stream::VIDEO: {
            const auto img = boost::make_shared<sensor_msgs::CompressedImage>();
            img->header = std::move(cvImg.header);
            img->format = "jpeg";
            img->data.assign(packet.data->cbegin(), packet.data->cend());
            pubPtr->publish(img);
            return;
        }
        case Stream::DEPTH:
        case Stream::DISPARITY_COLOR: {
            const int ndim = packet.dimensions.size();
            const int elemSize = packet.elem_size;

            if (ndim == 2) {
                if (elemSize == 1) {
                    cvImg.image = cv::Mat(rows, cols, CV_8UC1, data);
                    encoding = "mono8";
                } else {  // depth
                    cv::Mat bgrImg;
                    cv::Mat monoImg(rows, cols, CV_8UC1, data);
                    cv::applyColorMap(monoImg, bgrImg, cv::COLORMAP_HOT);
                    // cv::applyColorMap(monoImg, bgrImg, cv::COLORMAP_JET);
                    cvImg.image = bgrImg;
                    encoding = "bgr8";
                }
            } else {  // disparity_color
                cvImg.image = cv::Mat(rows, cols, CV_8UC3, data);
                encoding = "bgr8";
            }
            break;
        }
        default:  // should be unreachable
            return;
    }

    const sensor_msgs::ImagePtr msg = cvImg.toImageMsg();
    msg->encoding = encoding;
    pubPtr->publish(msg);
}

template <class Node>
void DepthAIBase<Node>::getPackets() {
    if (_request_jpegout) {
        _depthai->request_jpeg();
    }

    if (_pipeline != NULL) {
        tie(_nnet_packet, _data_packet) = _pipeline->getAvailableNNetAndDataPackets(_depthai_block_read);
    }
}

template <class Node>
void DepthAIBase<Node>::processPacketsAndPub() {
    ros::Time stamp = ros::Time::now();
    auto get_ts = [&](double camera_ts) {
        if (_depthai_ts_offset == -1) {
            _depthai_ts_offset = camera_ts;
            _stamp = stamp;
        }
        return _stamp + ros::Duration(camera_ts - _depthai_ts_offset);
    };

    if (_nnet_packet.size() != 0) {
        for (const std::shared_ptr<NNetPacket>& packet : _nnet_packet) {
            auto detections = packet->getDetectedObjects();
            if (detections == nullptr) {
                continue;
            }
            auto meta_data = packet->getMetadata();
            const auto seq_num = meta_data->getSequenceNum();
            const auto ts = meta_data->getTimestamp();
            const auto sync_ts = get_ts(ts);
            ROS_DEBUG_NAMED(this->getName(), "Stream: metaout, Original TS: %f, SeqNum: %d, Synced TS: %f", ts, seq_num,
                    sync_ts.toSec());
            publishObjectInfoMsg(*detections.get(), sync_ts);
        }
    }

    if (_data_packet.size() != 0) {
        for (const std::shared_ptr<HostDataPacket>& packet : _data_packet) {
            if (packet == nullptr) {
                // just a sanity check to prevent null dereference
                continue;
            }
            const auto& name = packet->stream_name;
            const auto stream = std::find(_stream_name.cbegin(), _stream_name.cend(), name);
            if (stream == _stream_name.end()) {
                ROS_WARN_THROTTLE_NAMED(10, this->getName(), "Stream: %s is not implemented", name.c_str());
                continue;
            }

            const auto index = std::distance(_stream_name.cbegin(), stream);
            if (index == Stream::VIDEO) {
                // workaround for the bug in DepthAI-Core library
                publishImageMsg(*(packet.get()), static_cast<Stream>(index), stamp);
                continue;
            }

            auto meta_data = packet->getMetadata();
            const auto seq_num = meta_data->getSequenceNum();
            const auto ts = meta_data->getTimestamp();
            const auto sync_ts = get_ts(ts);
            ROS_DEBUG_NAMED(this->getName(), "Stream: %s, Original TS: %f, SeqNum: %d, Synced TS: %f",
                    packet->stream_name.c_str(), ts, seq_num, sync_ts.toSec());
            publishImageMsg(*(packet.get()), static_cast<Stream>(index), sync_ts);
        }
    }
}

template <class Node>
void DepthAIBase<Node>::cameraReadCb(const ros::TimerEvent&) {
    getPackets();
    processPacketsAndPub();
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
    auto get_param = [](auto& nh, auto type, const auto& name, auto& variable) {
        if (nh.hasParam(name)) {
            nh.getParam(name, variable);
        } else {
            nh.setParam(name, variable);
        }
    };

    get_param(nh, std::string{}, "camera_name", _camera_name);
    get_param(nh, std::string{}, "camera_param_uri", _camera_param_uri);
    get_param(nh, std::string{}, "calibration_file", _calib_file);
    get_param(nh, std::string{}, "cmd_file", _cmd_file);
    get_param(nh, std::string{}, "blob_file", _blob_file);
    get_param(nh, std::string{}, "blob_file_config", _blob_file_config);

    get_param(nh, std::vector<std::string>{}, "stream_list", _stream_list);

    get_param(nh, bool{}, "depthai_block_read", _depthai_block_read);
    get_param(nh, bool{}, "enable_sync", _sync_video_meta);
    get_param(nh, bool{}, "full_fov_nn", _full_fov_nn);
    get_param(nh, bool{}, "disable_depth", _compute_bbox_depth);
    get_param(nh, bool{}, "force_usb2", _force_usb2);

    get_param(nh, int{}, "rgb_height", _rgb_height);
    get_param(nh, int{}, "rgb_fps", _rgb_fps);
    get_param(nh, int{}, "depth_height", _depth_height);
    get_param(nh, int{}, "depth_fps", _depth_fps);
    get_param(nh, int{}, "shaves", _shaves);
    get_param(nh, int{}, "cmx_slices", _cmx_slices);
    get_param(nh, int{}, "nn_engines", _nn_engines);

    get_param(nh, int{}, "queue_size", _queue_size);

    if (_camera_param_uri.back() != '/') {
        _camera_param_uri += "/";
    }

    prepareStreamConfig();

    // device init
    _depthai = std::make_unique<Device>("", _force_usb2);

    _available_streams = _depthai->get_available_streams();
    _nn2depth_map = _depthai->get_nn_to_depth_bbox_mapping();

    for (const auto& stream : _available_streams) {
        std::cout << "Available Streams: " << stream << std::endl;
    }

    _pipeline = _depthai->create_pipeline(_pipeline_config_json);

    _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(4));

    _cameraReadTimer = nh.createTimer(ros::Duration(1. / 500), &DepthAIBase::cameraReadCb, this);
}

template <class Node>
void DepthAIBase<Node>::prepareStreamConfig() {
    boost::property_tree::ptree root, streams, depth, ai, board_config, camera, camera_rgb, camera_mono, video_config,
            app, ot;
    std::regex reg("\"(null|true|false|[0-9]+(\\.[0-9]+)?)\"");
    std::ostringstream oss;
    _request_jpegout = false;

    auto& nh = this->getNodeHandle();
    image_transport::ImageTransport it{nh};

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

    _camera_info_default = nh.advertiseService("reset_camera_info", &DepthAIBase::defaultCameraInfo, this);

    for (const auto& stream : _stream_list) {
        // std::cout << "Requested Streams: " << _stream_list[i] << std::endl;
        boost::property_tree::ptree stream_config;
        stream_config.put("", stream);
        streams.push_back(std::make_pair("", stream_config));

        const auto it = std::find(_stream_name.cbegin(), _stream_name.cend(), stream);
        const auto index = static_cast<Stream>(std::distance(_stream_name.cbegin(), it));

        if (index < Stream::IMAGE_END) {
            set_camera_info_pub(index);
            if (index < Stream::UNCOMPRESSED_IMG_END) {
                set_stream_pub(index, sensor_msgs::Image{});
            } else {
                set_stream_pub(index, sensor_msgs::CompressedImage{});
            }
            continue;
        }
        switch (index) {
            case Stream::META_OUT:
                set_stream_pub(index, depthai_ros_msgs::Objects{});
                break;
            case Stream::OBJECT_TRACKER:
                set_stream_pub(index, depthai_ros_msgs::Object{});
                break;
            case Stream::META_D2H:
                set_stream_pub(index, sensor_msgs::Image{});
                break;
            default:
                ROS_ERROR_STREAM_NAMED(this->getName(), "Uknown stream requested: " << stream);
        }
    }

    depth.put<std::string>("calibration_file", _calib_file);
    depth.put("padding_factor", 0.3f);

    ai.put("blob_file", _blob_file);
    ai.put("blob_file_config", _blob_file_config);
    //    ai.put("blob_file2", _blob_file2);
    //    ai.put("blob_file_config2", _blob_file_config2);
    ai.put("calc_dist_to_bb", _compute_bbox_depth);
    ai.put("keep_aspect_ratio", !_full_fov_nn);
    //    ai.put("camera_input", "left_right");
    ai.put("camera_input", "rgb");
    ai.put("shaves", _shaves);
    ai.put("cmx_slices", _cmx_slices);
    ai.put("NN_engines", _nn_engines);

    // maximum 20 is supported
    ot.put("max_tracklets", 20);
    // object is tracked only for detections over this threshold
    ot.put("confidence_threshold", 0.5);

    board_config.put("swap_left_and_right_cameras", false);
    board_config.put("left_fov_deg", 69.0f);
    board_config.put("left_to_right_distance_cm", 10.93f);
    board_config.put("left_to_rgb_distance_cm", 22.75f);

    camera_rgb.put("resolution_h", _rgb_height);
    camera_rgb.put("fps", _rgb_fps);
    camera_mono.put("resolution_h", _depth_height);
    camera_mono.put("fps", _depth_fps);

    camera.add_child("rgb", camera_rgb);
    camera.add_child("mono", camera_mono);

    video_config.put("profile", "mjpeg");
    video_config.put("quality", 95);

    app.put("sync_video_meta_streams", _sync_video_meta);

    root.add_child("streams", streams);
    root.add_child("depth", depth);
    root.add_child("ai", ai);
    root.add_child("ot", ot);
    root.add_child("board_config", board_config);
    root.add_child("camera", camera);
    root.add_child("video_config", video_config);
    root.add_child("app", app);

    boost::property_tree::write_json(oss, root);
    _pipeline_config_json = std::regex_replace(oss.str(), reg, "$1");

    _af_ctrl_sub = nh.subscribe("auto_focus_ctrl", _queue_size, &DepthAIBase::afCtrlCb, this);
    _disparity_conf_sub = nh.subscribe("disparity_confidence", _queue_size, &DepthAIBase::disparityConfCb, this);
}
}  // namespace rr
