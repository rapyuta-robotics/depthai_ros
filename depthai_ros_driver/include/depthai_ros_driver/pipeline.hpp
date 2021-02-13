#pragma once

#include <memory>
#include <vector>

#include <depthai/device/Device.hpp>
#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/Pipeline.hpp>

namespace rr {
class Pipeline {
protected:
    dai::Pipeline _pipeline;

public:
    using NodeConstPtr = std::shared_ptr<const dai::Node>;

    /**
     * @brief The first function to get called, should have the setup ready for any future calls
     *
     */
    void configure(const std::string& config_json = "") { onConfigure(config_json); }

    /**
     * @brief interface called by the driver to get a pipeline ready-to-go
     *
     * @return dai::Pipeline
     */
    [[nodiscard]] dai::Pipeline getPipeline() const {
        onGetPipeline();
        return _pipeline;
    }

protected:
    /**
     * @brief Default implementation for configure step, does nothing
     */
    virtual void onConfigure(const std::string& config_json) {}

    /**
     * @brief Returns a pipeline in constant time
     */
    virtual void onGetPipeline() const {}
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
