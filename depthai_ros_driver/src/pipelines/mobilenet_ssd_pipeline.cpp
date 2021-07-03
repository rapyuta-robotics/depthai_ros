#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>


namespace depthai_ros_driver
{
class MobilenetSSDPipeline : public rr::Pipeline {
public:

protected:
    /**
     * @brief Configuring mobilenet-ssd configuration
     */
    void onConfigure(const std::string& config_json) {
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

    /**
     * @brief Returns a pipeline in constant time
     */
    void onGetPipeline() const {};

    /**
     * @brief returns the compressed streams exposed by the ROS interface
     */
    std::vector<NodeConstPtr> getCompressedVideoStreams(){};

    /**
     * @brief returns the camera control streams exposed by the ROS interface
     */
    std::vector<NodeConstPtr> getControlStreams(){};

    /**
     * @brief returns the streams required to pass information into the device, via ROS interface
     */
    std::vector<NodeConstPtr> getInputStreams(){};

    /**
     * @brief returns the streams that return a tensor, for interfacing with ROS
     */
    std::vector<NodeConstPtr> getTensorOutStreams(){};
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::MobilenetSSDPipeline, rr::Pipeline)

} // namespace depthai_ros_driver