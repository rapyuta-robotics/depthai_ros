#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>

namespace depthai_ros_driver
{
class PreviewPipeline : public rr::Pipeline {
public:

protected:
    /**
     * @brief Configuring preview pipeline
     */
    void onConfigure(ros::NodeHandle& nh) {
        auto colorCam = _pipeline.create<dai::node::ColorCamera>();
        auto xlinkOut = _pipeline.create<dai::node::XLinkOut>();
        xlinkOut->setStreamName("preview");

        colorCam->setPreviewSize(300, 300);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setInterleaved(true);

        // Link plugins CAM -> XLINK
        colorCam->preview.link(xlinkOut->input);
    }

    /**
     * @brief Preprocessor for the pipeline getter preprocessor
     */
    void onGetPipeline() const {};
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::PreviewPipeline, rr::Pipeline)

} // namespace depthai_ros_driver
