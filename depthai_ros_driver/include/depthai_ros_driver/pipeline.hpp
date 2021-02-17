#pragma once

#include <memory>
#include <vector>

#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/Pipeline.hpp>

namespace rr {
class Pipeline {
protected:
    dai::Pipeline _pipeline;

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

    /**
     * @brief Get the nodes with raw video streams output exposed by the ROS interface
     *
     * @return std::vector<NodeConstPtr>
     */
    virtual std::vector<NodeConstPtr> getRawVideoStreams();

    /**
     * @brief Get the nodes with compressed video streams exposed by the ROS interface
     *
     * @return std::vector<NodeConstPtr>
     */
    virtual std::vector<NodeConstPtr> getCompressedVideoStreams() = 0;

    /**
     * @brief Get the nodes with camera control streams exposed by the ROS interface
     *
     * @return std::vector<NodeConstPtr>
     */
    virtual std::vector<NodeConstPtr> getControlStreams() = 0;

    /**
     * @brief Get the nodes with streams required to pass information into the device, via ROS interface
     *
     * @return std::vector<NodeConstPtr>
     */
    virtual std::vector<NodeConstPtr> getInputStreams() = 0;

    /**
     * @brief Get the nodes with streams that return a tensor, for interfacing with ROS
     *
     * @return std::vector<NodeConstPtr>
     */
    virtual std::vector<NodeConstPtr> getTensorOutStreams() = 0;

    /////////////////////
    std::vector<NodeConstPtr> getRawOutputs() const { return filterNodesByName({"XLinkOut"}); }
    std::vector<NodeConstPtr> getRawInputs() const { return filterNodesByName({"XLinkIn"}); }

    enum class Recurse { CONTINUE, FAIL, SUCCESS };

    template <class Functor>
    const NodeConstPtr& recurseUntil(const NodeConstPtr& nodePtr, Functor&& f) {
        const auto& connections = getPipeline().getConnectionMap();
        const auto& nodes = getPipeline().getNodeMap();

        bool select = false;
        for (const auto& connection : connections[nodePtr->id]) {
            const auto& prev_node = nodes[connection.outputId];
            const auto result = f(prev_node);
            switch (result) {
                case Recurse::FAIL:
                    continue;
                case Recurse::SUCCESS:
                    return nodePtr;
                case Recurse::CONTINUE:
                    // @TODO: This
                    if (recurseUntil(prev_node, f) == nullptr) {
                        continue;
                    }
                    return nodePtr;
            }
        }
        return nullptr;
    }
    Recurse rawStreamCondition(const NodeConstPtr& node) {
        {
            // auto validConnection = [](Node::{In,Out}put put) {
            //     const auto& inp_type = put.possibleDatatypes;
            //     const auto& valid_input = std::find_if(inp_type.cbegin(), inp_type.cend(), [](const auto& dataType) {
            //         // the connection should be able to handle images
            //         return dai::isDatatypeSubclassOf(dataType, dai::DatatypeEnum::ImgFrame);
            //     });
            //     return valid_input != inp_type.cend();
            // }
            if (valid_input == inp_type.cend()) {
                continue;
            }
            if (node.getName() == "VideoEncoder") {
                // This path is not good
                return Recurse::FAILED;
            }
            if (node.getName() == "ColorCamera" || node.getName() == "MonoCamera") {
                // select node
                return Recurse::SUCCESS;
            }
            recurseNodes(f);
        }
    }

    virtual std::vector<NodeConstPtr> getRawVideoStreams() {
        // camera.{video,preview} is linked with XLinkOut
        const auto& cameras = getCameras();
        const auto& outputNodes = getRawOutputs();

        for (input : node.getInputs()) {
            // This input can take
            // store the node connected to somehow...
        }
        return {};
    }

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
