
#include <regex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <depthai/pipeline/datatype/CameraControl.hpp>

#include <depthai_ros_driver/pipeline.hpp>
#include <depthai_ros_driver/depthai_base.hpp>


namespace rr {

template <class Node>
bool DepthAIBase<Node>::has_stream(const std::string& stream) const {
    return (_enabled_streams.find(stream) != _enabled_streams.cend());
};

// template <class Node>
// void DepthAIBase<Node>::disparityConfCb(const std_msgs::Float32::ConstPtr& msg) {
//     if ((msg->data >= 0.0) && (msg->data <= 255.0)) {
//         //_depthai->send_disparity_confidence_threshold(msg->data);
//     } else {
//         ROS_ERROR_NAMED(this->getName(), "Disparity confidence value:%f, is invalid", msg->data);
//     }
// }

// template <class Node>
// void DepthAIBase<Node>::focusControlCallback(const depthai_ros_msgs::FocusControlCommand& msg) {
//     auto focus_control = FocusControl::createControl(msg);
//     if (focus_control) {
//         _color_control_queue->send(*focus_control);
//     } else {
//         ROS_ERROR_STREAM_NAMED(this->getName(), "Invalid Focus mode is requested: " << msg);
//     }
// }

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
void DepthAIBase<Node>::publishImageMsg(ImageFramePtr frame, const std::string& stream, ros::Time& stamp) {

    // https://answers.ros.org/question/336045/how-to-convert-a-stdchronohigh_resolution_clock-to-rostime/
    // const auto& tstamp = frame->getTimestamp();
    // int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp - start).count();
    // int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp - start).count() % 1000000000UL;
    // ros::Time stamp(sec, nsec);

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = stream_info[stream].topic;

    Stream type = stream_info[stream].id;
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
        case Stream::RECTIFIED_LEFT:
        case Stream::RECTIFIED_RIGHT:
            cvImg.image = cv::Mat(rows, cols, CV_8UC1, data);
            encoding = "mono8";
            break;
        case Stream::PREVIEW_OUT:
            cvImg.image = cv::Mat(rows, cols, CV_8UC3, data);
            encoding = "bgr8";
            break;
        case Stream::DISPARITY: {
            cvImg.image = cv::Mat(rows, cols, _subpixel ? CV_16UC1 : CV_8UC1, data);
            encoding = _subpixel ? "mono16" : "mono8";
            break;
        }
        case Stream::DISPARITY_COLOR: {
            cv::Mat disp(rows, cols, _subpixel ? CV_16UC1 : CV_8UC1, data);
            disp.convertTo(disp, CV_8UC1, 255.0 / _maxDisp); // Extend disparity range
            cv::applyColorMap(disp, cvImg.image, cv::COLORMAP_JET);
            encoding = "bgr8";
            break;
        }
        case Stream::DEPTH:
            cvImg.image = cv::Mat(rows, cols, CV_16UC1, data);
            encoding = "mono16";
            break;
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

    for(const auto& dat: _data_output_queue) {
        const auto& stream = dat.first;
        const auto& queue = dat.second;
        Stream stream_id = stream_info[stream].id;
        if (stream_id < Stream::IMAGE_END) {
            const auto& frame = queue->get<dai::ImgFrame>();
            if(frame){
                publishImageMsg(frame, stream, stamp);
            }
        }
        else if (stream == "metaout") {
            const auto& detections = queue->get<dai::NNData>();
            if(detections) {
                publishObjectInfoMsg(detections, stamp);
            }
        }
    }
}

// template <class Node>
// void DepthAIBase<Node>::publishImageMsg(const HostDataPacket& packet, Stream type, const ros::Time& stamp) {
//     const auto* camInfoPubPtr = _camera_info_publishers[type].get();
//     const auto* pubPtr = _stream_publishers[type].get();

//     std_msgs::Header header;
//     header.stamp = stamp;
//     header.frame_id = packet.stream_name;

//     for(const auto& dat: _data_output_queue) {
//         const auto& stream = dat.first;
//         const auto& queue = dat.second;
//         Stream stream_id = stream_info[stream].id;
//         if (stream_id < Stream::IMAGE_END) {
//             const auto& frame = queue->get<dai::ImgFrame>();
//             if(frame){
//                 publishImageMsg(frame, stream, stamp);
//             }
//             cv::merge(toMerge, cvImg.image);
//             encoding = "bgr8";
//             break;
//         }
//         else if (stream == "metaout") {
//             const auto& detections = queue->get<dai::NNData>();
//             if(detections) {
//                 publishObjectInfoMsg(detections, stamp);
//             }
//         }
//     } else {
//         std::cout << "Not ImgFrame" << std::endl;
//     }
//     if (has_data_queue("detections")) {
//         const auto& detections = _data_output_queue["detections"]->get<dai::NNData>();
//         if(detections) {
//             publishObjectInfoMsg(detections, stamp);
//         }
//     }

//     if(dispFrame){
//         static int count = 0;
//         constexpr bool subpixel = false;
//         constexpr int maxDisp = 96;

//         cv::Mat disp(dispFrame->getHeight(), dispFrame->getWidth(),
//                 subpixel ? CV_16UC1 : CV_8UC1, dispFrame->getData().data());
//         disp.convertTo(disp, CV_8UC1, 255.0 / maxDisp); // Extend disparity range
//         cv::imshow("disparity", disp);
//         cv::waitKey(1);
//         std::cout << "disparity " << count << std::endl;
//         count++;

//         // if (key == 'q'){
//         //     return;
//         // }
//     } else {
//         std::cout << "Not DepthFrame" << std::endl;
//     }

//     // if (_data_packet.size() != 0) {
//     //     for (const std::shared_ptr<HostDataPacket>& packet : _data_packet) {
//     //         if (packet == nullptr) {
//     //             // just a sanity check to prevent null dereference
//     //             continue;
//     //         }
//     //         const auto& name = packet->stream_name;
//     //         const auto stream = std::find(_stream_name.cbegin(), _stream_name.cend(), name);
//     //         if (stream == _stream_name.end()) {
//     //             ROS_WARN_THROTTLE_NAMED(10, this->getName(), "Stream: %s is not implemented", name.c_str());
//     //             continue;
//     //         }

//     //         const auto index = std::distance(_stream_name.cbegin(), stream);
//     //         if (index == Stream::VIDEO) {
//     //             // workaround for the bug in DepthAI-Core library
//     //             publishImageMsg(*(packet.get()), static_cast<Stream>(index), stamp);
//     //             continue;
//     //         }

//     //         auto meta_data = packet->getMetadata();
//     //         const auto seq_num = meta_data->getSequenceNum();
//     //         const auto ts = meta_data->getTimestamp();
//     //         const auto sync_ts = get_ts(ts);modified:
//     //         ROS_DEBUG_NAMED(this->getName(), "Stream: %s, Original TS: %f, SeqNum: %d, Synced TS: %f",
//     //                 packet->stream_name.c_str(), ts, seq_num, sync_ts.toSec());
//     //         publishImageMsg(*(packet.get()), static_cast<Stream>(index), sync_ts);
//     //     }
//     // }
// }

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

    get_param(nh, std::string{}, "pipeline_name", _pipeline_name);

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
    get_param(nh, bool{}, "lrcheck", _lrcheck);

    get_param(nh, int{}, "queue_size", _queue_size);

    if (_camera_param_uri.back() != '/') {
        _camera_param_uri += "/";
    }

    // set unsupported flags
    _request_jpegout = false;

    // disparity mode. necessary only for visualization
    _maxDisp = 96;
    if (_extended_disparity) _maxDisp *= 2;
    if (_subpixel) _maxDisp *= 32; // 5 bits fractional disparity

    // generate config json
    _pipeline_config_json = generatePipelineConfigJson();

    // Prepare streams (pub, sub, etc)
    prepareStreamConfig();

    // Start pipeline
    _pipeline_loader = std::make_unique<pluginlib::ClassLoader<rr::Pipeline>>("depthai_ros_driver", "rr::Pipeline");
    try {
        _pipeline_plugin = _pipeline_loader->createUniqueInstance(_pipeline_name);
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
    for (const auto& stream: _enabled_streams) {
        _data_output_queue[stream] = _depthai->getOutputQueue(stream_info[stream].output_queue, 4, false);
    }

    // Control
    const auto& input_queues = _depthai->getInputQueueNames();
    for (const auto& input_queue: input_queues) {
        if (input_queue == "control") {
            _color_control_queue = _depthai->getInputQueue("control");
            // _focus_ctrl_sub = nh.subscribe("focus_ctrl", _queue_size, &DepthAIBase::focusControlCallback, this);
            // _disparity_conf_sub = nh.subscribe("disparity_confidence", _queue_size, &DepthAIBase::disparityConfCb, this);
        } else if (input_queue == "config") {
            _color_config_queue = _depthai->getInputQueue("config");
        }
    }

    // camera frame loop
    _cameraReadTimer = nh.createTimer(ros::Duration(1. / 500), &DepthAIBase::cameraReadCb, this);
}

template <class Node>
void DepthAIBase<Node>::prepareStreamConfig() {
    auto& nh = this->getNodeHandle();
    image_transport::ImageTransport it{nh};

    auto set_camera_info_pub = [&](const std::string& stream) {
        const auto& name = stream_info[stream].name;
        const auto& topic = stream_info[stream].topic;

        const auto uri = _camera_param_uri + _camera_name + "/" + name + ".yaml";
        _camera_info_manager[stream_info[stream].id] =
                std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, name}, name, uri);
        _camera_info_publishers[stream_info[stream].id] = std::make_unique<ros::Publisher>(
                nh.template advertise<sensor_msgs::CameraInfo>(topic + "/camera_info", _queue_size));
    };

    auto set_stream_pub = [&](const std::string& stream, auto message_type) {
        using type = decltype(message_type);
        std::string suffix;
        Stream id = stream_info[stream].id;
        if (id < Stream::IMAGE_END) {
            suffix = (id < Stream::UNCOMPRESSED_IMG_END) ? "/image_raw" : "/compressed";
        }
        const auto& name = stream_info[stream].topic;
        _stream_publishers[id] =
                std::make_unique<ros::Publisher>(nh.template advertise<type>(name + suffix, _queue_size));
    };

    for (const auto& stream : _stream_list) {
        // std::cout << "Requested Streams: " << _stream_list[i] << std::endl;
        if (stream_info.find(stream) == stream_info.end()) {
            ROS_WARN_STREAM("Unknown stream: " << stream);
            continue;
        }

        const auto index = stream_info[stream].id;

        if (index < Stream::IMAGE_END) {
            set_camera_info_pub(stream);
            if (index < Stream::UNCOMPRESSED_IMG_END) {
                set_stream_pub(stream, sensor_msgs::Image{});
            } else {
                set_stream_pub(stream, sensor_msgs::CompressedImage{});
            }
        }
        else {
            switch (index) {
                case Stream::META_OUT:
                    set_stream_pub(stream, depthai_ros_msgs::Objects{});
                    break;
                case Stream::OBJECT_TRACKER:
                    set_stream_pub(stream, depthai_ros_msgs::Object{});
                    break;
                case Stream::META_D2H:
                    set_stream_pub(stream, sensor_msgs::Image{});
                    break;
                default:
                    ROS_ERROR_STREAM_NAMED(this->getName(), "Uknown stream requested: " << stream);
            }
        }

        _enabled_streams.emplace(stream);
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
    depth.put("extended", _extended_disparity);
    depth.put("subpixel", _subpixel);
    depth.put("lrcheck", _lrcheck);

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
