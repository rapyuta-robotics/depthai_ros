#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/ImageManip.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>

namespace depthai_ros_driver
{
class MonoCameraControlPipeline : public rr::Pipeline {
public:

protected:
    /**
     * @brief Configuring mono camera control pipeline
     */
    void onConfigure(ros::NodeHandle& nh) {
        auto monoRight = _pipeline.create<dai::node::MonoCamera>();
        auto monoLeft = _pipeline.create<dai::node::MonoCamera>();
        auto manipRight = _pipeline.create<dai::node::ImageManip>();
        auto manipLeft = _pipeline.create<dai::node::ImageManip>();

        auto controlIn = _pipeline.create<dai::node::XLinkIn>();
        auto configIn = _pipeline.create<dai::node::XLinkIn>();
        auto manipOutRight = _pipeline.create<dai::node::XLinkOut>();
        auto manipOutLeft = _pipeline.create<dai::node::XLinkOut>();

        controlIn->setStreamName("control");
        configIn->setStreamName("config");
        manipOutRight->setStreamName("right");
        manipOutLeft->setStreamName("left");

        // Crop range
        dai::Point2f topLeft(0.2, 0.2);
        dai::Point2f bottomRight(0.8, 0.8);

        // Properties
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        manipRight->initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
        manipLeft->initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);

        // Linking
        monoRight->out.link(manipRight->inputImage);
        monoLeft->out.link(manipLeft->inputImage);
        controlIn->out.link(monoRight->inputControl);
        controlIn->out.link(monoLeft->inputControl);
        configIn->out.link(manipRight->inputConfig);
        configIn->out.link(manipLeft->inputConfig);
        manipRight->out.link(manipOutRight->input);
        manipLeft->out.link(manipOutLeft->input);
    }

    /**
     * @brief Preprocessor for the pipeline getter preprocessor
     */
    void onGetPipeline() const {};
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::MonoCameraControlPipeline, rr::Pipeline)

} // namespace depthai_ros_driver
