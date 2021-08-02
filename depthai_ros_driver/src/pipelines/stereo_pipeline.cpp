#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>

namespace depthai_ros_driver
{
class StereoPipeline : public rr::Pipeline {
public:

protected:
    /**
     * @brief Default implementation for configure step, does nothing
     */
    void onConfigure(const std::string& config_json) {
        ROS_WARN("For running this stereo example pipeline, "
                 "Please specify \"stream_list\" as "
                 "\"[left, right, disparity, depth, rectified_left, rectified_right]\". "
                 "Missing streams linked in this example will fail streaming the data.");
        // TODO - split this example into two separate examples
        bool withDepth = true;

        auto monoLeft  = _pipeline.create<dai::node::MonoCamera>();
        auto monoRight = _pipeline.create<dai::node::MonoCamera>();
        auto xoutLeft  = _pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = _pipeline.create<dai::node::XLinkOut>();
        auto stereo    = withDepth ? _pipeline.create<dai::node::StereoDepth>() : nullptr;
        auto xoutDisp  = _pipeline.create<dai::node::XLinkOut>();
        auto xoutDepth = _pipeline.create<dai::node::XLinkOut>();
        auto xoutRectifL = _pipeline.create<dai::node::XLinkOut>();
        auto xoutRectifR = _pipeline.create<dai::node::XLinkOut>();

        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if (withDepth) {
            xoutDisp->setStreamName("disparity");
            xoutDepth->setStreamName("depth");
            xoutRectifL->setStreamName("rectified_left");
            xoutRectifR->setStreamName("rectified_right");
        }

        // MonoCamera
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        //monoLeft->setFps(5.0);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        //monoRight->setFps(5.0);

        bool lrcheck  = true;
        bool extended = false;
        bool subpixel = true;

        int maxDisp = 96;
        if (extended) maxDisp *= 2;
        if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

        if (withDepth) {
            // StereoDepth
            stereo->setConfidenceThreshold(200);
            stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
            //stereo->loadCalibrationFile("../../../../depthai/resources/depthai.calib");
            //stereo->setInputResolution(1280, 720);
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
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
            stereo->disparity.link(xoutDisp->input);
            stereo->depth.link(xoutDepth->input);

        } else {
            // Link plugins CAM -> XLINK
            monoLeft->out.link(xoutLeft->input);
            monoRight->out.link(xoutRight->input);
        }

        ROS_INFO("StereoPipeline is initialized.");
    }

    /**
     * @brief Returns a pipeline in constant time
     */
    void onGetPipeline() const {};
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::StereoPipeline, rr::Pipeline)

} // namespace depthai_ros_driver
