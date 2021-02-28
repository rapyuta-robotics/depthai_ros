#include <ros/console.h>

#include <depthai_ros_driver/pipeline_ex.hpp>

#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>


namespace rr
{
void PipelineEx::configure_preview_pipeline(const std::string& config_json) {
    auto colorCam = _pipeline.create<dai::node::ColorCamera>();
    auto xoutColor = _pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutColor->setStreamName("preview");

    // Color camera
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    colorCam->preview.link(xoutColor->input);

    ROS_INFO("Initialized preview pipeline.");
}

void PipelineEx::configure_stereo_pipeline(const std::string& config_json) {
    // convert json string to nlohmann::json
    const nlohmann::json json = nlohmann::json::parse(config_json);
    if (!json.contains("streams") || !json.contains("depth")) {
        ROS_ERROR("mobilenet_ssd pipline needs \"streams\" tag and \"depth\" tag for config_json.");
        return;
    }

    // set configuration based on streams
    const auto& streams = json["streams"];
    const bool withDepth = has_any(streams, {"disparity", "depth", "disparity_color"});  // with disparity
    const bool outputDepth = withDepth & has_any(streams, {"depth"});  // direct depth computation
    const bool outputRectified = withDepth & has_any(streams, {"rectified_left", "rectified_right"});

    // parse json parameters
    std::string calibrationFile;
    bool extended, subpixel, lrcheck;
    auto sensorResolution = dai::MonoCameraProperties::SensorResolution::THE_720_P;
    const auto& depthConfig = json["depth"];
    try {
        calibrationFile = depthConfig.at("calibration_file");
        extended = depthConfig.at("extended");
        subpixel = depthConfig.at("subpixel");
        lrcheck = false;

        if (json.contains("mono")) {
            int resolution_h = json["mono"].at("resolution_h");
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
    } catch(const std::exception& ex) { // const nlohmann::basic_json::out_of_range& ex
        ROS_ERROR(ex.what());
        return;
    }


    int maxDisp = 96;
    if (extended) maxDisp *= 2;
    if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    auto monoLeft  = _pipeline.create<dai::node::MonoCamera>();
    auto monoRight = _pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft  = _pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = _pipeline.create<dai::node::XLinkOut>();
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    //monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    //monoRight->setFps(5.0);

    if (withDepth) {
        auto stereo    = withDepth ? _pipeline.create<dai::node::StereoDepth>() : nullptr;
        auto xoutDisp  = _pipeline.create<dai::node::XLinkOut>();

        // StereoDepth
        stereo->setOutputDepth(outputDepth);
        stereo->setOutputRectified(outputRectified);
        stereo->setConfidenceThreshold(200);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
        stereo->loadCalibrationFile(calibrationFile);
        //stereo->setInputResolution(1280, 720);
        // TODO: median filtering is disabled on device with (lrcheck || extended || subpixel)
        //stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // XLinkOut
        xoutDisp->setStreamName("disparity");

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);
        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        stereo->disparity.link(xoutDisp->input);

        if (outputDepth) {
            auto xoutDepth = _pipeline.create<dai::node::XLinkOut>();
            xoutDepth->setStreamName("depth");  // XLinkOut
            stereo->depth.link(xoutDepth->input);  // Link: depth -> XLINK
        }

        if(outputRectified)
        {
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