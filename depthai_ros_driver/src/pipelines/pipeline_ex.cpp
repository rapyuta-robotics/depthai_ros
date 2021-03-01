#include <ros/console.h>

#include <depthai_ros_driver/pipeline_ex.hpp>

#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <depthai/pipeline/node/VideoEncoder.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>


namespace rr
{
void PipelineEx::configure_color_pipeline(const std::string& config_json) {
    // convert json string to nlohmann::json
    const nlohmann::json json = nlohmann::json::parse(config_json);

    if (!json.contains("streams")) {
        ROS_ERROR("color pipline needs \"streams\" tag and \"depth\" tag for config_json.");
        return;
    }
    const auto& streams = json["streams"];

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

    // Color camera
    auto colorCam = _pipeline.create<dai::node::ColorCamera>();
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setFps(fps);
    colorCam->setInterleaved(true);

    // Streams
    if (has_any(streams, {"previewout"})) {
        auto xoutPreviewout = _pipeline.create<dai::node::XLinkOut>();
        xoutPreviewout->setStreamName("preview");
        colorCam->preview.link(xoutPreviewout->input);
    }
    if (has_any(streams, {"video"})) {
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
    // if (has_any(streams, {"jpegout"})) {
    //     auto xoutJpeg = _pipeline.create<dai::node::XLinkOut>();
    //     xoutJpeg->setStreamName("jpegout");
    //     colorCam->still.link(xoutJpeg->input);
    // }

    ROS_INFO("Initialized color pipeline.");
}

void PipelineEx::configure_stereo_pipeline(const std::string& config_json) {
    // convert json string to nlohmann::json
    const nlohmann::json json = nlohmann::json::parse(config_json);
    if (!json.contains("streams") || !json.contains("depth")) {
        ROS_ERROR("stereo pipline needs \"streams\" tag and \"depth\" tag for config_json.");
        return;
    }

    // set configuration based on streams
    const auto& streams = json["streams"];
    const bool withDepth = has_any(streams, {"disparity", "disparity_color", "depth"});  // with disparity
    const bool outputDisparity = withDepth && has_any(streams, {"disparity", "disparity_color"});
    const bool outputDepth = withDepth && has_any(streams, {"depth"});  // direct depth computation
    const bool outputRectified = withDepth && has_any(streams, {"rectified_left", "rectified_right"});

    // parse json parameters
    std::string calibrationFile;
    bool maxDisp = 96;
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
            lrcheck = depthConfig.at("subpixel");

            if (extended) maxDisp *= 2;
            if (subpixel) maxDisp *= 32; // 5 bits fractional disparity
        }
    } catch(const std::exception& ex) { // const nlohmann::basic_json::out_of_range& ex
        ROS_ERROR(ex.what());
        return;
    }

    auto monoLeft  = _pipeline.create<dai::node::MonoCamera>();
    auto monoRight = _pipeline.create<dai::node::MonoCamera>();
    monoLeft->setResolution(sensorResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(fps);
    monoRight->setResolution(sensorResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(fps);

    auto xoutLeft  = _pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = _pipeline.create<dai::node::XLinkOut>();
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    if (withDepth) {
        auto stereo    = withDepth ? _pipeline.create<dai::node::StereoDepth>() : nullptr;
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

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);
        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);

        if (outputDisparity) {
            auto xoutDisp  = _pipeline.create<dai::node::XLinkOut>();
            xoutDisp->setStreamName("disparity");
            stereo->disparity.link(xoutDisp->input);
        }

        if (outputDepth) {
            auto xoutDepth = _pipeline.create<dai::node::XLinkOut>();
            xoutDepth->setStreamName("depth");
            stereo->depth.link(xoutDepth->input);
        }

        if (outputRectified) {
            // setting streams for recified images are not necessary for computing depth
            auto xoutRectifL = _pipeline.create<dai::node::XLinkOut>();
            auto xoutRectifR = _pipeline.create<dai::node::XLinkOut>();
            xoutRectifL->setStreamName("rectified_left");
            xoutRectifR->setStreamName("rectified_right");
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }
    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    ROS_INFO("Initialized stereo pipeline.");
}

void PipelineEx::configure_mobilenet_ssd_pipeline(const std::string& config_json) {
    // convert json string to nlohmann::json
    const nlohmann::json json = nlohmann::json::parse(config_json);
    if (!json.contains("ai")) {
        ROS_ERROR("mobilenet_ssd pipline needs \"ai\" tag for config_json.");
        return;
    }

    std::string nnPath;
    const auto& config = json["ai"];
    try {
        nnPath = config.at("blob_file");

    } catch(const std::exception& ex) { // const nlohmann::basic_json::out_of_range& ex
        ROS_ERROR(ex.what());
        return;
    }

    auto colorCam = _pipeline.create<dai::node::ColorCamera>();
    auto xoutColor = _pipeline.create<dai::node::XLinkOut>();
    auto nn1 = _pipeline.create<dai::node::NeuralNetwork>();
    auto nnOut = _pipeline.create<dai::node::XLinkOut>();

    nn1->setBlobPath(nnPath);

    // XLinkOut
    xoutColor->setStreamName("preview");
    nnOut->setStreamName("detections");

    // Color camera
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(nn1->input);
    colorCam->preview.link(xoutColor->input);
    nn1->out.link(nnOut->input);

    ROS_INFO("Mobilenet SSD pipeline initialized.");
}



} // namespace depthai_ros_driver