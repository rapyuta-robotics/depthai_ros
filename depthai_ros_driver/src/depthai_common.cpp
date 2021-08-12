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

    switch (type) {
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
            for (int i = 0; i < 3; ++i) {
                toMerge[i] = cv::Mat(cols, cols, CV_8UC1, data + i * offset);
            }
            cv::merge(toMerge, cvImg.image);
            encoding = "bgr8";
            break;
        }
        case Stream::JPEG_OUT:
        case Stream::VIDEO: {
            // const auto img = boost::make_shared<sensor_msgs::CompressedImage>();
            // TODO<YL> deal with this
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
    }
    img_data.first = encoding;
    img_data.second = cvImg;
    return img_data;
}

//==============================================================================
const std::string create_pipeline_config(const DepthAIBaseConfig& cfg)
{
    boost::property_tree::ptree root, streams, depth, ai, board_config,
        camera, camera_rgb, camera_mono, video_config, app, ot;
    std::regex reg("\"(null|true|false|[0-9]+(\\.[0-9]+)?)\"");
    std::ostringstream oss;
    depth.put<std::string>("calibration_file", cfg.calib_file);
    depth.put("padding_factor", 0.3f);

    ai.put("blob_file", cfg.blob_file);
    ai.put("blob_file_config", cfg.blob_file_config);
    //    ai.put("blob_file2", _blob_file2);
    //    ai.put("blob_file_config2", _blob_file_config2);
    ai.put("calc_dist_to_bb", cfg.compute_bbox_depth);
    ai.put("keep_aspect_ratio", !cfg.full_fov_nn);
    //    ai.put("camera_input", "left_right");
    ai.put("camera_input", "rgb");
    ai.put("shaves", cfg.shaves);
    ai.put("cmx_slices", cfg.cmx_slices);
    ai.put("NN_engines", cfg.nn_engines);

    // maximum 20 is supported
    ot.put("max_tracklets", 20);
    // object is tracked only for detections over this threshold
    ot.put("confidence_threshold", 0.5);

    board_config.put("swap_left_and_right_cameras", false);
    board_config.put("left_fov_deg", 69.0f);
    board_config.put("left_to_right_distance_cm", 10.93f);
    board_config.put("left_to_rgb_distance_cm", 22.75f);

    camera_rgb.put("resolution_h", cfg.rgb_height);
    camera_rgb.put("fps", cfg.rgb_fps);
    camera_mono.put("resolution_h", cfg.depth_height);
    camera_mono.put("fps", cfg.depth_fps);

    camera.add_child("rgb", camera_rgb);
    camera.add_child("mono", camera_mono);

    video_config.put("profile", "mjpeg");
    video_config.put("quality", 95);

    app.put("sync_video_meta_streams", cfg.sync_video_meta);

    for (const auto& stream : cfg.stream_list) {
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
void DepthAICommon::process_and_publish_packets()
{
    std::list<std::shared_ptr<NNetPacket>> nnet_packet;
    std::list<std::shared_ptr<HostDataPacket>> data_packet;

    if (_cfg.request_jpegout)
        _depthai->request_jpeg();

    if (_pipeline != NULL)
        tie(nnet_packet, data_packet) =
            _pipeline->getAvailableNNetAndDataPackets(_cfg.depthai_block_read);

    if (nnet_packet.size() != 0) {
        for (const std::shared_ptr<NNetPacket>& packet : nnet_packet) {
            auto detections = packet->getDetectedObjects();
            if (detections == nullptr) {
                continue;
            }
            auto meta_data = packet->getMetadata();
            const auto seq_num = meta_data->getSequenceNum();
            const auto ts = meta_data->getTimestamp();
            if (_publish_objs_fn)
                _publish_objs_fn(*detections.get(), ts);
        }
    }
    if (data_packet.size() != 0) {
        for (const std::shared_ptr<HostDataPacket>& packet : data_packet) {
            if (packet == nullptr) {
                // just a sanity check to prevent null dereference
                continue;
            }
            const auto& name = packet->stream_name;
            const auto stream = std::find(_stream_name.cbegin(), _stream_name.cend(), name);
            if (stream == _stream_name.end()) {
                continue;
            }

            const auto index = std::distance(_stream_name.cbegin(), stream);
            if (index == Stream::VIDEO) {
                // workaround for the bug in DepthAI-Core library
                // use -1 to represent use rostime
                if (_publish_img_fn)
                    _publish_img_fn(*(packet.get()), static_cast<Stream>(index), -1);
                continue;
            }

            auto meta_data = packet->getMetadata();
            const auto seq_num = meta_data->getSequenceNum();
            const auto ts = meta_data->getTimestamp();
            if (_publish_img_fn)
                _publish_img_fn(*(packet.get()), static_cast<Stream>(index), ts);
        }
    }
};

//==============================================================================
bool DepthAICommon::set_disparity(const float val){
    if ((val >= 0.0) && (val <= 255.0)) {
        _depthai->send_disparity_confidence_threshold(val);
        return true; 
    }
    return false;
};

//==============================================================================
bool DepthAICommon::set_autofocus(const bool trigger, const uint8_t mode){
    if (trigger)
        _depthai->request_af_trigger();

    if ((mode >= 0) && (mode <= 4)) {
        _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(mode));
        return true;
    }
    return false;
};

//==============================================================================
ObjectsMsg DepthAICommon::convert(const dai::Detections& detections){
    ObjectsMsg msg;
    ObjectMsg object;
    msg.objects.reserve(detections.detection_count);

    for (int i = 0; i < detections.detection_count; ++i) {
        object.label_id = detections.detections[i].label;
        object.confidence = detections.detections[i].confidence;
        object.bb.x_min = detections.detections[i].x_min;
        object.bb.y_min = detections.detections[i].y_min;
        object.bb.x_max = detections.detections[i].x_max;
        object.bb.y_max = detections.detections[i].y_max;

        if (_cfg.compute_bbox_depth) {
            object.bb.depth_x = detections.detections[i].depth_x;
            object.bb.depth_y = detections.detections[i].depth_y;
            object.bb.depth_z = detections.detections[i].depth_z;
        }
        msg.objects.push_back(object);
    }
    return msg;
}

}  // namespace rr
