#pragma once

#include <memory>
#include <vector>

#include <depthai/device/Device.hpp>
#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/Pipeline.hpp>


namespace rr {
class Pipeline {
protected: // delete
    std::shared_ptr<dai::DataOutputQueue> _data_output_queue; // delete
public: // delete
    const std::shared_ptr<dai::DataOutputQueue>& getDataOutputQueue() const { return _data_output_queue; };



protected:
    dai::Pipeline _pipeline;

    std::unique_ptr<dai::Device> _depthai;

public:
    using NodeConstPtr = std::shared_ptr<const dai::Node>;

    [[nodiscard]] std::vector<Pipeline::NodeConstPtr> getNodesByName(const std::vector<std::string>& node_names);

    /**
     * @brief The first function to get called, should have the setup ready for any future calls
     *
     */
    void configure() { onConfigure(); }

    /**
     * @brief interface called by the driver to get a pipeline ready-to-go
     *
     * @return dai::Pipeline
     */
    dai::Pipeline getPipeline() const { return _pipeline; }

    /**
     * @brief returns the nodes that access the raw camera data. Not used for ROS interface
     * @details uses names provided by `CameraNodeNames` to filter for the camera nodes
     * @return std::vector<Pipeline::NodeConstPtr> list of camera nodes
     */
    std::vector<NodeConstPtr> getCameras() const { return filterNodesByName(getCameraNodeNames()); }

protected:
    /**
     * @brief Default implementation for configure step, does nothing
     */
    virtual void onConfigure() {}

    /**
     * @brief Returns a pipeline in constant time
     */
    virtual dai::Pipeline onGetPipeline() const = 0;

    /**
     * @brief Get the list of names used for different camera node types
     * @note Please note that the list is of node types, not the names provided to the individual nodes
     * @details By default, it provides the 2 cameras in depthai-core: "{Mono,Color}Camera"
     * The names should be the same as the return value of `dai::Node::getName()`
     * @return std::vector<std::string> vector of node types
     */
    virtual std::vector<std::string> getCameraNodeNames() const { return {"MonoCamera", "ColorCamera"}; }

    /////////////////////
    std::vector<NodeConstPtr> getRawOutputs() const { return filterNodesByName({"XLinkOut"}); }
    std::vector<NodeConstPtr> getRawInputs() const { return filterNodesByName({"XLinkIn"}); }

    /**
     * @brief returns the raw streams exposed by the ROS interface
     */
    std::vector<NodeConstPtr> getRawVideoStreams() {
        // camera.{video,preview} is linked with XLinkOut
        const auto& cameras = getCameras();
        const auto& outputNodes = getRawOutputs();
        for (const auto& kv : getPipeline().getConnectionMap()) {
            // filter connections with:
            // input == {video, preview}
            // no link with VideoEncoder
            // link with XLinkOut
        }
        return {};
    }

    /**
     * @brief returns the compressed streams exposed by the ROS interface
     */
    virtual std::vector<NodeConstPtr> getCompressedVideoStreams() = 0;

    /**
     * @brief returns the camera control streams exposed by the ROS interface
     */
    virtual std::vector<NodeConstPtr> getControlStreams() = 0;

    /**
     * @brief returns the streams required to pass information into the device, via ROS interface
     */
    virtual std::vector<NodeConstPtr> getInputStreams() = 0;

    /**
     * @brief returns the streams that return a tensor, for interfacing with ROS
     */
    virtual std::vector<NodeConstPtr> getTensorOutStreams() = 0;

    /**
     * @brief filters the nodes in the pipeline based on requested node types
     *
     * @tparam Range an iterable of objects capable of holding a name,
     * eg: std::vector<std::string> or std::array<std::string>
     * @param node_names list of names to select
     * @return std::vector<Pipeline::NodeConstPtr> newly created vector with the requested nodes
     */
    template <class Range = std::vector<std::string>>
    std::vector<NodeConstPtr> filterNodesByName(Range&& node_names) const;

    /**
     * @brief filters out the nodes in the pipeline based on requested node types
     *
     * @tparam Range an iterable of objects capable of holding a name,
     * eg: std::vector<std::string> or std::array<std::string>
     * @param node_names list of names to not select
     * @return std::vector<Pipeline::NodeConstPtr> newly created vector with the requested nodes
     */
    template <class Range = std::vector<std::string>>
    std::vector<NodeConstPtr> filterOutNodesByName(Range&& node_names) const;
};
}  // namespace rr

#include <range/v3/view.hpp>
#include <range/v3/algorithm/find.hpp>

namespace rr {
namespace rv = ranges::view;

template <class Range>
std::vector<Pipeline::NodeConstPtr> Pipeline::filterNodesByName(Range&& node_names) const {
    const auto pipeline = getPipeline();
    const auto& nodeMap = pipeline.getNodeMap();

    std::vector<NodeConstPtr> filtered_nodes;
    filtered_nodes.reserve(nodeMap.size() / 2);  // guess of size could be much better

    filtered_nodes =
            nodeMap | rv::transform([](const auto& kv) { return kv.second; }) | rv::filter([&](const auto& nodePtr) {
                // return true if name is in th list
                return ranges::find(node_names, nodePtr->getName()) != ranges::end(node_names);
            });
    return filtered_nodes;
}

template <class Range>
std::vector<Pipeline::NodeConstPtr> Pipeline::filterOutNodesByName(Range&& node_names) const {
    const auto pipeline = getPipeline();
    const auto& nodeMap = pipeline.getNodeMap();

    std::vector<NodeConstPtr> filtered_nodes;
    filtered_nodes.reserve(nodeMap.size());  // guess of size could be much better

    filtered_nodes =
            nodeMap | rv::transform([](const auto& kv) { return kv.second; }) | rv::filter([&](const auto& nodePtr) {
                // return true if name is not in the list
                return ranges::find(node_names, nodePtr->getName()) == ranges::end(node_names);
            });
    return filtered_nodes;
}
}  // namespace rr
