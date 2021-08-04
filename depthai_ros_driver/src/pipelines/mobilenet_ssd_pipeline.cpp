#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/DetectionNetwork.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>


namespace depthai_ros_driver
{
class MobilenetSSDPipeline : public rr::Pipeline {
    using OpenvVINOVersion = dai::OpenVINO::Version;
public:

protected:
    /**
     * @brief Configuring mobilenet-ssd pipeline
     */
    void onConfigure(ros::NodeHandle& nh) {
        std::string blob_file = "./mobilenet-ssd.blob";
        int openvino_version = static_cast<int>(_pipeline.getOpenVINOVersion());

        if (!nh.getParam("blob_file", blob_file)) {
            nh.setParam("blob_file", blob_file);
        }
        if (!nh.getParam("openvino_version", openvino_version)) {
            nh.setParam("openvino_version", openvino_version);
        }

        // Define sources and outputs
        auto camRgb = _pipeline.create<dai::node::ColorCamera>();
        auto nn = _pipeline.create<dai::node::MobileNetDetectionNetwork>();
        auto xoutRgb = _pipeline.create<dai::node::XLinkOut>();
        auto nnOut = _pipeline.create<dai::node::XLinkOut>();

        // Set openvino version
        _pipeline.setOpenVINOVersion(static_cast<OpenvVINOVersion>(openvino_version));

        xoutRgb->setStreamName("rgb");
        nnOut->setStreamName("nn");

        // Properties
        camRgb->setPreviewSize(300, 300);  // NN input
        camRgb->setInterleaved(false);
        camRgb->setFps(30);

        // Define a neural network that will make predictions based on the source frames
        nn->setConfidenceThreshold(0.5);
        nn->setBlobPath(blob_file);
        nn->setNumInferenceThreads(2);
        nn->input.setBlocking(false);

        // Link pipelines CAM -> NN -> XLINK
        camRgb->preview.link(xoutRgb->input);
        camRgb->preview.link(nn->input);
        nn->out.link(nnOut->input);

        ROS_INFO("Mobilenet SSD pipeline initialized.");
    }

    /**
     * @brief Preprocessor for the pipeline getter preprocessor
     */
    void onGetPipeline() const {};
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::MobilenetSSDPipeline, rr::Pipeline)

} // namespace depthai_ros_driver
