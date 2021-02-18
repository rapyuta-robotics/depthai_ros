#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>


namespace depthai_ros_driver
{
class StereoPipeline : public rr::Pipeline {
public:

protected:
    /**
     * @brief Default implementation for configure step, does nothing
     */
    void onConfigure() {
        // _pipeline = _depthai->create_pipeline(_pipeline_config_json);

        auto colorCam = _pipeline.create<dai::node::ColorCamera>();
        auto xlinkOut = _pipeline.create<dai::node::XLinkOut>();
        xlinkOut->setStreamName("preview");

        colorCam->setPreviewSize(300, 300);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setInterleaved(true);

        // Link plugins CAM -> XLINK
        colorCam->preview.link(xlinkOut->input);

        // device init
        _depthai = std::make_unique<dai::Device>(_pipeline);

        _depthai->startPipeline();
        _data_output_queue = _depthai->getOutputQueue("preview");

        // _available_streams = _depthai->get_available_streams();
        // _nn2depth_map = _depthai->get_nn_to_depth_bbox_mapping();

        // for (const auto& stream : _available_streams) {
        //     std::cout << "Available Streams: " << stream << std::endl;
        // }

        // _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(4));

        ROS_INFO("Stereo pipeline initialized.");
    }

    /**
     * @brief Returns a pipeline in constant time
     */
    dai::Pipeline onGetPipeline() const {};

    /**
     * @brief Get the list of names used for different camera node types
     * @note Please note that the list is of node types, not the names provided to the individual nodes
     * @details By default, it provides the 2 cameras in depthai-core: "{Mono,Color}Camera"
     * The names should be the same as the return value of `dai::Node::getName()`
     * @return std::vector<std::string> vector of node types
     */
    std::vector<std::string> getCameraNodeNames() const { return {"MonoCamera", "ColorCamera"}; }

    /////////////////////
    std::vector<NodeConstPtr> getRawOutputs() const { return filterNodesByName({"XLinkOut"}); }
    std::vector<NodeConstPtr> getRawInputs() const { return filterNodesByName({"XLinkIn"}); }

    // /**
    //  * @brief returns the raw streams exposed by the ROS interface
    //  */
    // std::vector<NodeConstPtr> getRawVideoStreams() {
    //     // camera.{video,preview} is linked with XLinkOut
    //     const auto& cameras = getCameras();
    //     const auto& outputNodes = getRawOutputs();
    //     for (const auto& kv : getPipeline().getConnectionMap()) {
    //         // filter connections with:
    //         // input == {video, preview}
    //         // no link with VideoEncoder
    //         // link with XLinkOut
    //     }
    //     return {};
    // }

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
    std::vector<NodeConstPtr> getTensorOutStreams(){

    };

    /**
     * @brief filters the nodes in the pipeline based on requested node types
     *
     * @tparam Range an iterable of objects capable of holding a name,
     * eg: std::vector<std::string> or std::array<std::string>
     * @param node_names list of names to select
     * @return std::vector<Pipeline::NodeConstPtr> newly created vector with the requested nodes
     */
    template <class Range = std::vector<std::string>>
    std::vector<NodeConstPtr> filterNodesByName(Range&& node_names) const {};

    /**
     * @brief filters out the nodes in the pipeline based on requested node types
     *
     * @tparam Range an iterable of objects capable of holding a name,
     * eg: std::vector<std::string> or std::array<std::string>
     * @param node_names list of names to not select
     * @return std::vector<Pipeline::NodeConstPtr> newly created vector with the requested nodes
     */
    template <class Range = std::vector<std::string>>
    std::vector<NodeConstPtr> filterOutNodesByName(Range&& node_names) const {};
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::StereoPipeline, rr::Pipeline)

} // namespace depthai_ros_driver