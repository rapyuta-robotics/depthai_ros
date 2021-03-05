#include <ros/console.h>
#include <pluginlib/class_list_macros.h>

#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <depthai/pipeline/node/VideoEncoder.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>

#include <depthai_ros_driver/multicam_pipeline.hpp>

namespace depthai_ros_driver
{
void MulticamPipeline::onConfigure(const std::string& config_json) {
    ROS_INFO_STREAM("Multicam pipline config json:\n" << config_json);

    // convert json string to nlohmann::json
    const nlohmann::json json = nlohmann::json::parse(config_json);
    if (!json.contains("streams")) {
        ROS_ERROR("MulticamPipline needs \"streams\" tag for an array of streams in config_json.");
        return;
    }
    _streams = json["streams"];

    if (has_any(stereo_stream_list)) {
        configure_stereo_pipeline(config_json);
    }
    if (has_any(color_stream_list)) {
        configure_color_pipeline(config_json);
    }
}

bool MulticamPipeline::has_stream(const std::string& name) const {
    return (std::find(_streams.cbegin(), _streams.cend(), name) != _streams.cend());
}

bool MulticamPipeline::has_any(const std::vector<std::string>& list) const {
    for (const std::string& item: list) {
        if (has_stream(item)) {
            return true; // found
        }
    }

    return false;  // didn't find any
}

bool MulticamPipeline::link_to_xout(auto& source , const std::string& stream) {
    if (has_stream(stream)) {
        auto xout  = _pipeline.create<dai::node::XLinkOut>();
        xout->setStreamName(stream);
        source.link(xout->input);
    }
}

void MulticamPipeline::configure_color_pipeline(const std::string& config_json) {
    // convert json string to nlohmann::json
    const nlohmann::json json = nlohmann::json::parse(config_json);
    if (!json.contains("camera")) {
        ROS_ERROR("color pipline needs \"camera\" tag for config_json.");
        return;
    }

    float fps = 15.0;
    auto sensorResolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    try {
        fps = json["camera"].at("rgb").at("fps");
        int resolution_h = static_cast<int>(json["camera"].at("rgb").at("resolution_h"));
        if (json.contains("camera")) {
            switch(resolution_h) {
            case 1080:
                sensorResolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
                break;
            case 2160:
                sensorResolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
                break;
            case 3040:
                sensorResolution = dai::ColorCameraProperties::SensorResolution::THE_12_MP;
                break;
            default:
                ROS_WARN_STREAM("Unknown rgb camera resolution " << resolution_h
                                << ". Default resolution 1920 will be used.");
                break;
            }
        }
    } catch(const std::exception& ex) { // const nlohmann::basic_json::out_of_range& ex
        ROS_ERROR(ex.what());
        return;
    }

    std::string nnPath;
    bool withNN = false;
    if (has_any({"meta_d2h", "metaout", "object_tracker"})) {  // TODO: nn_stream_list
        if (!json.contains("ai")) {
            ROS_ERROR("mobilenet_ssd pipline needs \"ai\" tag for config_json.");
            return;
        }

        const auto& config = json["ai"];
        try {
            nnPath = config.at("blob_file");

        } catch(const std::exception& ex) { // const nlohmann::basic_json::out_of_range& ex
            ROS_ERROR(ex.what());
            return;
        }

        withNN = true;
    }

    // Color camera
    auto colorCam = _pipeline.create<dai::node::ColorCamera>();
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setFps(fps);
    if (withNN) {
        colorCam->setInterleaved(false); // 3 x height x width
        colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    } else {
        colorCam->setInterleaved(true); // height x width x 3
    }

    // Streams
    if (has_any({"previewout", "metaout"})) {
        auto xoutPreviewout = _pipeline.create<dai::node::XLinkOut>();
        xoutPreviewout->setStreamName("preview");
        colorCam->preview.link(xoutPreviewout->input);

        if (has_stream("metaout")) {
            auto nn1 = _pipeline.create<dai::node::NeuralNetwork>();
            auto nnOut = _pipeline.create<dai::node::XLinkOut>();
            nn1->setBlobPath(nnPath);
            nnOut->setStreamName("detections");
            colorCam->preview.link(nn1->input);
            nn1->out.link(nnOut->input);
        }
    }
    if (has_stream("video")) {
        auto xoutVideo = _pipeline.create<dai::node::XLinkOut>();
        auto videnc = _pipeline.create<dai::node::VideoEncoder>();

        const auto resolution_wh = colorCam->getResolutionSize();
        xoutVideo->setStreamName("mjpeg");
        videnc->setDefaultProfilePreset(std::get<0>(resolution_wh), std::get<1>(resolution_wh),
                                        fps, dai::VideoEncoderProperties::Profile::MJPEG);
        colorCam->video.link(videnc->input);
        videnc->bitstream.link(xoutVideo->input);

        // xoutVideo->setStreamName("video");
        // colorCam->video.link(xoutVideo->input); // <- uncompressed case. streamed in YUV
    }
    // if (has_stream(streams, "jpegout")) {
    //     auto xoutJpeg = _pipeline.create<dai::node::XLinkOut>();
    //     xoutJpeg->setStreamName("jpegout");
    //     colorCam->still.link(xoutJpeg->input);
    // }

    ROS_INFO("Initialized color pipeline.");
}

void MulticamPipeline::configure_stereo_pipeline(const std::string& config_json) {
    // convert json string to nlohmann::json
    const nlohmann::json json = nlohmann::json::parse(config_json);
    if (!json.contains("depth")) {
        ROS_ERROR("stereo pipline needs \"depth\" tag for config_json.");
        return;
    }

    // set configuration based on streams
    const bool withDepth = has_any({"disparity", "disparity_color", "depth", "rectified_left", "rectified_right"});

    // parse json parameters
    std::string calibrationFile;
    bool extended, subpixel, lrcheck;
    float fps = 10.0;
    auto sensorResolution = dai::MonoCameraProperties::SensorResolution::THE_720_P;
    try {
        if (json.contains("camera")) {
            fps = json["camera"].at("mono").at("fps");
            int resolution_h = static_cast<int>(json["camera"].at("mono").at("resolution_h"));
            switch(resolution_h) {
            case 400:
                sensorResolution = dai::MonoCameraProperties::SensorResolution::THE_400_P;
                break;
            case 720:
                sensorResolution = dai::MonoCameraProperties::SensorResolution::THE_720_P;
                break;
            case 800:
                sensorResolution = dai::MonoCameraProperties::SensorResolution::THE_800_P;
                break;
            default:
                ROS_WARN_STREAM("Unknown mono camera resolution " << resolution_h
                                << ". Default resolution 720 will be used.");
                break;
            }
        }

        if (withDepth) {
            const auto& depthConfig = json["depth"];
            calibrationFile = depthConfig.at("calibration_file");
            extended = depthConfig.at("extended");
            subpixel = depthConfig.at("subpixel");
            lrcheck = depthConfig.at("lrcheck");
        }
    } catch(const std::exception& ex) { // const nlohmann::basic_json::out_of_range& ex
        ROS_ERROR(ex.what());
        return;
    }

    std::shared_ptr<dai::node::MonoCamera> monoLeft, monoRight;
    if (withDepth || has_stream("left")) {
        monoLeft  = _pipeline.create<dai::node::MonoCamera>();
        monoLeft->setResolution(sensorResolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoLeft->setFps(fps);
    }

    if (withDepth || has_stream("right")) {
        monoRight = _pipeline.create<dai::node::MonoCamera>();
        monoRight->setResolution(sensorResolution);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        monoRight->setFps(fps);
    }

    if (withDepth) {
        auto stereo = _pipeline.create<dai::node::StereoDepth>();

        const bool outputDepth = has_stream("depth"); // direct depth computation
        const bool outputRectified = has_any({"rectified_left", "rectified_right"});
        const auto resolution_wh = monoLeft->getResolutionSize();

        // StereoDepth
        stereo->setOutputDepth(outputDepth);
        stereo->setOutputRectified(outputRectified);
        stereo->setConfidenceThreshold(200);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
        stereo->loadCalibrationFile(calibrationFile);
        stereo->setInputResolution(std::get<0>(resolution_wh), std::get<1>(resolution_wh));
        // TODO: median filtering is disabled on device with (lrcheck || extended || subpixel)
        //stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Link plugins CAM -> STEREO
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        link_to_xout(stereo->syncedLeft, "left");
        link_to_xout(stereo->syncedRight, "right");
        link_to_xout(stereo->disparity, "disparity");
        link_to_xout(stereo->disparity, "disparity_color");
        link_to_xout(stereo->depth, "depth");

        // setting streams for recified images are not necessary for computing depth
        link_to_xout(stereo->rectifiedLeft, "rectified_left");
        link_to_xout(stereo->rectifiedRight, "rectified_right");
    } else {
        link_to_xout(monoLeft->out, "left");
        link_to_xout(monoRight->out, "right");
    }

    ROS_INFO("Initialized stereo pipeline.");
}

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::MulticamPipeline, rr::Pipeline)

} // namespace depthai_ros_driver