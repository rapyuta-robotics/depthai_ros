#include <regex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <depthai_ros_driver/pipeline.hpp>

#include <depthai_ros_driver/depthai_base.hpp>


namespace rr {


template <class Node>
void DepthAIBase<Node>::disparityConfCb(const std_msgs::Float32::ConstPtr& msg) {
    if ((msg->data >= 0.0) && (msg->data <= 255.0)) {
        //_depthai->send_disparity_confidence_threshold(msg->data);
    } else {
        ROS_ERROR_NAMED(this->getName(), "Disparity confidence value:%f, is invalid", msg->data);
    }
}

template <class Node>
void DepthAIBase<Node>::afCtrlCb(const depthai_ros_msgs::AutoFocusCtrl msg) {
    if (msg.trigger_auto_focus) {
        //_depthai->request_af_trigger();
    }
    if ((msg.auto_focus_mode >= 0) && (msg.auto_focus_mode <= 4)) {
        //_depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(msg.auto_focus_mode));
    } else {
        ROS_ERROR_NAMED(this->getName(), "Invalid Auto Focus mode requested");
    }
}

template <class Node>
void DepthAIBase<Node>::publishObjectInfoMsg(NNDataConstPtr detections, const ros::Time& stamp) {
    const auto* pubPtr = _stream_publishers[Stream::META_OUT].get();
    if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0) {
        return;  // No subscribers
    }

    depthai_ros_msgs::Objects msg;
    msg.header.stamp = stamp;

    std::vector<float> detData = detections->getFirstLayerFp16();
    if(detData.size() > 0){
        int i = 0;
        while (detData[i * 7] != -1.0f) {
            depthai_ros_msgs::Object object;
            object.label_id = detData[i * 7 + 1];
            object.confidence = detData[i * 7 + 2];
            object.bb.x_min = detData[i * 7 + 3];
            object.bb.y_min = detData[i * 7 + 4];
            object.bb.x_max = detData[i * 7 + 5];
            object.bb.y_max = detData[i * 7 + 6];
            // if (_compute_bbox_depth) {
            //     object.bb.depth_x = detections.detections[i].depth_x;
            //     object.bb.depth_y = detections.detections[i].depth_y;
            //     object.bb.depth_z = detections.detections[i].depth_z;
            // }
            i++;
            msg.objects.emplace_back(object);
        }
    }

    pubPtr->publish(msg);
}

template <class Node>
void DepthAIBase<Node>::publishImageMsg(ImageFramePtr frame, Stream type, ros::Time& stamp) {

    // https://answers.ros.org/question/336045/how-to-convert-a-stdchronohigh_resolution_clock-to-rostime/
    // const auto& tstamp = frame->getTimestamp();
    // int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp - start).count();
    // int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp - start).count() % 1000000000UL;
    // ros::Time stamp(sec, nsec);

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = _topic_name[type];

    const auto* camInfoPubPtr = _camera_info_publishers[type].get();
    if (camInfoPubPtr->getNumSubscribers() > 0) {
        const auto cameraInfo =
                boost::make_shared<sensor_msgs::CameraInfo>(_camera_info_manager[type]->getCameraInfo());
        cameraInfo->header = header;
        camInfoPubPtr->publish(cameraInfo);
    }

    const auto* pubPtr = _stream_publishers[type].get();
    if (pubPtr == nullptr || pubPtr->getNumSubscribers() <= 0) {
        return;  // No subscribers
    }

    cv_bridge::CvImage cvImg;
    cvImg.header = std::move(header);
    std::string encoding = "";

    const int cols = frame->getWidth();
    const int rows = frame->getHeight();

    // cv::Mat needs `void *` not `const void *`
    auto* data = const_cast<uint8_t*>(frame->getData().data());

    switch (type) {
        case Stream::LEFT:
        case Stream::RIGHT:
            cvImg.image = cv::Mat(rows, cols, CV_8UC1, data);
            encoding = "mono8";
            break;
        case Stream::PREVIEW_OUT:
            cvImg.image = cv::Mat(rows, cols, CV_8UC3, data);
            encoding = "bgr8";
            break;
        case Stream::DISPARITY: {
            cvImg.image = cv::Mat(rows, cols, _subpixel ? CV_16UC1 : CV_8UC1, data);
            cvImg.image.convertTo(cvImg.image, CV_8UC1, 255.0 / _maxDisp); // Extend disparity range
            encoding = "mono8";
            break;
        }
        case Stream::DEPTH:
        case Stream::DISPARITY_COLOR: {
            const int ndim = 2; // packet.dimensions.size();
            const int elemSize = 1; // packet.elem_size;  <- subpixel

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
            } else {  // disparity_color  <- ??
                cvImg.image = cv::Mat(rows, cols, CV_8UC3, data);
                encoding = "bgr8";
            }
            break;
        }
        case Stream::JPEG_OUT:
        case Stream::VIDEO: {
            const auto img = boost::make_shared<sensor_msgs::CompressedImage>();
            img->header = std::move(cvImg.header);
            img->format = "jpeg";
            img->data = frame->getData();
            pubPtr->publish(img);
            return;  // already published CompressedImage
        }
        default:  // should be unreachable
            return;
    }

    const sensor_msgs::ImagePtr msg = cvImg.toImageMsg();
    msg->encoding = encoding;
    pubPtr->publish(msg);
}

template <class Node>
void DepthAIBase<Node>::cameraReadCb(const ros::TimerEvent&) {
    // if (_request_jpegout) {
    //     _depthai->request_jpeg();
    // }

    ros::Time stamp = ros::Time::now();
    // auto get_ts = [&](double camera_ts) {
    //     if (_depthai_ts_offset == -1) {
    //         _depthai_ts_offset = camera_ts;
    //         _stamp = stamp;
    //     }
    //     return _stamp + ros::Duration(camera_ts - _depthai_ts_offset);
    // };


    auto has_data_queue = [&] (const std::string& stream_name) {
        return (_data_output_queue.find(stream_name) != _data_output_queue.end());
    };


    if (has_data_queue("preview")) {
        const auto& imgFrame = _data_output_queue["preview"]->get<dai::ImgFrame>();
        if(imgFrame){
            publishImageMsg(imgFrame, Stream::PREVIEW_OUT, stamp);
        }
    }
    if (has_data_queue("disparity")) {
        const auto& dispFrame = _data_output_queue["disparity"]->get<dai::ImgFrame>();
        if(dispFrame){
            publishImageMsg(dispFrame, Stream::DISPARITY, stamp);
        }
    }
    if (has_data_queue("detections")) {
        const auto& detections = _data_output_queue["detections"]->get<dai::NNData>();
        if(detections) {
            publishObjectInfoMsg(detections, stamp);
        }
    }

    // if (_data_packet.size() != 0) {
    //     for (const std::shared_ptr<HostDataPacket>& packet : _data_packet) {
    //         if (packet == nullptr) {
    //             // just a sanity check to prevent null dereference
    //             continue;
    //         }
    //         const auto& name = packet->stream_name;
    //         const auto stream = std::find(_stream_name.cbegin(), _stream_name.cend(), name);
    //         if (stream == _stream_name.end()) {
    //             ROS_WARN_THROTTLE_NAMED(10, this->getName(), "Stream: %s is not implemented", name.c_str());
    //             continue;
    //         }

    //         const auto index = std::distance(_stream_name.cbegin(), stream);
    //         if (index == Stream::VIDEO) {
    //             // workaround for the bug in DepthAI-Core library
    //             publishImageMsg(*(packet.get()), static_cast<Stream>(index), stamp);
    //             continue;
    //         }

    //         auto meta_data = packet->getMetadata();
    //         const auto seq_num = meta_data->getSequenceNum();
    //         const auto ts = meta_data->getTimestamp();
    //         const auto sync_ts = get_ts(ts);
    //         ROS_DEBUG_NAMED(this->getName(), "Stream: %s, Original TS: %f, SeqNum: %d, Synced TS: %f",
    //                 packet->stream_name.c_str(), ts, seq_num, sync_ts.toSec());
    //         publishImageMsg(*(packet.get()), static_cast<Stream>(index), sync_ts);
    //     }
    // }
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
    get_param(nh, bool{}, "force_usb2", _force_usb2);

    get_param(nh, int{}, "rgb_height", _rgb_height);
    get_param(nh, int{}, "rgb_fps", _rgb_fps);
    get_param(nh, int{}, "depth_height", _depth_height);
    get_param(nh, int{}, "depth_fps", _depth_fps);

    get_param(nh, bool{}, "extended_disparity", _extended_disparity);
    get_param(nh, bool{}, "subpixel", _subpixel);

    get_param(nh, int{}, "queue_size", _queue_size);

    if (_camera_param_uri.back() != '/') {
        _camera_param_uri += "/";
    }

    // set unsupported flags
    _request_jpegout = false;

    // disparity mode
    _maxDisp = 96;
    if (_extended_disparity) _maxDisp *= 2;
    if (_subpixel) _maxDisp *= 32; // 5 bits fractional disparity

    // generate config json
    _pipeline_config_json = generatePipelineConfigJson();

    // Prepare streams (pub, sub, etc)
    prepareStreamConfig();

    auto has_stream = [&] (const std::string& stream_name) {
        const auto itr = std::find(_stream_list.begin(), _stream_list.end(), stream_name);
        return (itr != _stream_list.end());
    };

    // Start pipeline
    _pipeline_loader = std::make_unique<pluginlib::ClassLoader<rr::Pipeline>>("depthai_ros_driver", "rr::Pipeline");
    try {
        std::string plugin_name;
        if (has_stream("previewout") && !has_stream("metaout")) {
            plugin_name = "depthai_ros_driver/PreviewPipeline";
            ROS_INFO_STREAM("Stream: previewout");
        } else if (has_stream("disparity")) {
            plugin_name = "depthai_ros_driver/StereoPipeline";
            ROS_INFO_STREAM("Stream: disparity");
        } else if (has_stream("metaout")) {
            plugin_name = "depthai_ros_driver/MobilenetSSDPipeline";
            ROS_INFO_STREAM("Stream: mataout");
        } else {
            ROS_ERROR("Unknown stream. Will not load pipeline plugin.");
        }
        _pipeline_plugin = _pipeline_loader->createInstance(plugin_name);
        _pipeline_plugin->configure(_pipeline_config_json);
        _pipeline = _pipeline_plugin->getPipeline();
    }
    catch(pluginlib::PluginlibException &ex) {
        ROS_ERROR("failed to load the plugin. Error: %s", ex.what());
    }

    // const auto cameras = _pipeline_plugin->getCameras();
    // for (const auto cam: cameras) {
    //     ROS_INFO("camera");
    //     std::cout << "    " << cam->getName() << ": " << cam->id << std::endl;
    //     // const auto outputs = cam->getOutputs();
    //     // for (const auto& out: outputs) {
    //     //     std::cout << "    out" << std::endl;
    //     //     for (const auto& con: out.getConnections()) {
    //     //         std::cout << "        " << con.inputName << ", " << con.outputName << std::endl;
    //     //     }
    //     // }
    // }

    // device init
    _depthai = std::make_unique<dai::Device>(_pipeline);
    _depthai->startPipeline();


    if (has_stream("previewout")) {
        _data_output_queue["preview"] = _depthai->getOutputQueue("preview");
    }
    if  (has_stream("disparity")) {
        _data_output_queue["disparity"] = _depthai->getOutputQueue("disparity", 8, false);
    }
    if (has_stream("metaout")) {
        _data_output_queue["detections"] = _depthai->getOutputQueue("detections");
    }

    // _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(4));

    // Control
    _af_ctrl_sub = nh.subscribe("auto_focus_ctrl", _queue_size, &DepthAIBase::afCtrlCb, this);
    _disparity_conf_sub = nh.subscribe("disparity_confidence", _queue_size, &DepthAIBase::disparityConfCb, this);

    // camera frame loop
    _cameraReadTimer = nh.createTimer(ros::Duration(1. / 500), &DepthAIBase::cameraReadCb, this);
}

template <class Node>
void DepthAIBase<Node>::prepareStreamConfig() {

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
}

template <class Node>
std::string DepthAIBase<Node>::generatePipelineConfigJson() const {
    boost::property_tree::ptree root, streams, depth, ai, board_config, camera, camera_rgb, camera_mono, video_config,
            app, ot;

    for (const auto& stream : _stream_list) {
        // std::cout << "Requested Streams: " << _stream_list[i] << std::endl;
        boost::property_tree::ptree stream_config;
        stream_config.put("", stream);
        streams.push_back(std::make_pair("", stream_config));
    }

    depth.put<std::string>("calibration_file", _calib_file);
    depth.put("padding_factor", 0.3f);
    depth.put("extended", _extended_disparity);
    depth.put("subpixel", _subpixel);

    ai.put("blob_file", _blob_file);
    ai.put("blob_file_config", _blob_file_config);

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

    std::regex reg("\"(null|true|false|[0-9]+(\\.[0-9]+)?)\"");
    std::ostringstream oss;
    boost::property_tree::write_json(oss, root);

    return std::regex_replace(oss.str(), reg, "$1");
}

}  // namespace rr
