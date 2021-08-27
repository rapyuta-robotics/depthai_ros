#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline.hpp>
#include <depthai_ros_driver/pipeline_common.hpp>

#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/DetectionNetwork.hpp>
#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>

namespace depthai_ros_driver {
class StereoWithDetectionPipeline : public rr::Pipeline {
    using OpenvVINOVersion = dai::OpenVINO::Version;

public:
protected:
    /**
     * @brief Configuring stereo with detection pipeline
     */
    void onConfigure(ros::NodeHandle& nh) {
        std::string blob_file = "./mobilenet-ssd.blob";
        int openvino_version = static_cast<int>(_pipeline.getOpenVINOVersion());
        bool with_depth = true;
        bool stereo_lrcheck = true;
        bool stereo_extended = false;
        bool stereo_subpixel = true;
        int stereo_confidence_threshold = 200;
        rr::sync_ros_param<bool>(nh, "with_depth", with_depth);
        rr::sync_ros_param<bool>(nh, "stereo_lrcheck", stereo_lrcheck);
        rr::sync_ros_param<bool>(nh, "stereo_extended", stereo_extended);
        rr::sync_ros_param<bool>(nh, "stereo_subpixel", stereo_subpixel);
        rr::sync_ros_param<int>(nh, "stereo_confidence_threshold", stereo_confidence_threshold);
        rr::sync_ros_param<std::string>(nh, "blob_file", blob_file);
        rr::sync_ros_param<int>(nh, "mobilenetssd_openvino_version", openvino_version);

        auto camRgb = _pipeline.create<dai::node::ColorCamera>();
        auto nn = _pipeline.create<dai::node::MobileNetDetectionNetwork>();
        auto monoLeft = _pipeline.create<dai::node::MonoCamera>();
        auto monoRight = _pipeline.create<dai::node::MonoCamera>();
        auto xoutRgb = _pipeline.create<dai::node::XLinkOut>();
        auto xoutPreview = _pipeline.create<dai::node::XLinkOut>();
        auto nnOut = _pipeline.create<dai::node::XLinkOut>();
        auto xoutLeft = _pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = _pipeline.create<dai::node::XLinkOut>();
        auto stereo = with_depth ? _pipeline.create<dai::node::StereoDepth>() : nullptr;
        auto xoutDepth = _pipeline.create<dai::node::XLinkOut>();
        auto xoutRectifL = _pipeline.create<dai::node::XLinkOut>();
        auto xoutRectifR = _pipeline.create<dai::node::XLinkOut>();

        _pipeline.setOpenVINOVersion(static_cast<dai::OpenVINO::Version>(openvino_version));

        // XLinkOut
        xoutRgb->setStreamName("rgb");
        xoutPreview->setStreamName("preview");
        nnOut->setStreamName("nn");
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if (with_depth) {
            xoutDepth->setStreamName("depth");
            xoutRectifL->setStreamName("rectified_left");
            xoutRectifR->setStreamName("rectified_right");
        }

        // RGB camera
        camRgb->setPreviewSize(300, 300);  // NN input
        camRgb->setPreviewKeepAspectRatio(false);
        camRgb->setInterleaved(false);
        camRgb->setFps(30);

        // Define a neural network that will make predictions based on the source frames
        nn->setConfidenceThreshold(0.01);
        ROS_INFO_STREAM("BLOB FILE: " << blob_file);
        nn->setBlobPath(blob_file);
        nn->setNumInferenceThreads(2);
        nn->input.setBlocking(false);

        // MonoCamera
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        // monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        // monoLeft->setFps(5.0);
        // monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        // monoRight->setFps(5.0);

        camRgb->preview.link(xoutPreview->input);
        camRgb->preview.link(nn->input);
        camRgb->video.link(xoutRgb->input);
        nn->out.link(nnOut->input);
        int maxDisp = 96;
        if (stereo_extended)
            maxDisp *= 2;
        if (stereo_subpixel)
            maxDisp *= 32;  // 5 bits fractional disparity
        if (with_depth) {
            // StereoDepth
            stereo->initialConfig.setConfidenceThreshold(stereo_confidence_threshold);
            stereo->setRectifyEdgeFillColor(0);
            stereo->setInputResolution(1280, 720);
            // stereo->setInputResolution(640, 400);
            stereo->setLeftRightCheck(stereo_lrcheck);
            stereo->setExtendedDisparity(stereo_extended);
            stereo->setSubpixel(stereo_subpixel);

            // Link plugins CAM -> STEREO -> XLINK
            monoLeft->out.link(stereo->left);
            monoRight->out.link(stereo->right);

            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
            stereo->depth.link(xoutDepth->input);

        } else {
            // Link plugins CAM -> XLINK
            monoLeft->out.link(xoutLeft->input);
            monoRight->out.link(xoutRight->input);
        }

        ROS_INFO("StereoWithDetectionPipeline is initialized.");
    }

    /**
     * @brief Preprocessor for the pipeline getter preprocessor
     */
    void onGetPipeline() const {};
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::StereoWithDetectionPipeline, rr::Pipeline)

}  // namespace depthai_ros_driver
