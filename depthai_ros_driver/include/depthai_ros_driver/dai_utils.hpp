#pragma once

#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/Node.hpp>
#include <depthai/xlink/XLinkStream.hpp>
#include <depthai/pipeline/datatype/StreamPacketParser.hpp>

#include <boost/lambda/detail/is_instance_of.hpp>

#include <range/v3/view.hpp>
#include <range/v3/algorithm/find.hpp>
#include <range/v3/range.hpp>
#include <memory>

namespace rr {
// convenieve type
using NodeConstPtr = std::shared_ptr<const dai::Node>;

// short hand for views
namespace rv = ranges::views;

/**
 * @brief Search for nodes with certain names in a pipeline
 * @tparam Range a range of string
 * @param[in] pipeline Pipeline to search for nodes in
 * @param[in] node_names Names against which exact match is checked
 * @return vector of node shared pointers
 */
template <class Range>
std::vector<NodeConstPtr> filterNodesByNames(const dai::Pipeline& pipeline, Range&& node_names) {
    const auto& nodeMap = pipeline.getNodeMap();

    return nodeMap | rv::values | rv::filter([&](const auto& nodePtr) {
        // return true if name is in th list
        return ranges::find(node_names, nodePtr->getName()) != ranges::end(node_names);
    }) | rv::transform([](const auto& nodePtr) -> NodeConstPtr { return nodePtr; }) |
           ranges::to<std::vector>();
}

/**
 * @brief Search for nodes with a certain name in a pipeline
 * @param[in] pipeline Pipeline to search for nodes in
 * @param[in] name name against which exact match is checked
 * @return vector of node shared pointers
 * @sa filterNodesByNames
 */
std::vector<NodeConstPtr> filterNodesByName(const dai::Pipeline& pipeline, std::string name);

namespace detail {
// @TODO: FIX, doesn't work
template <class T>
constexpr auto is_shared_ptr_v = boost::lambda::is_instance_of_1<T, std::shared_ptr>::value;

template <class NodeT, std::enable_if_t<is_shared_ptr_v<NodeT>> = true>
auto constTransformNode(const NodeConstPtr& nodePtr) {
    return std::dynamic_pointer_cast<const typename NodeT::element_type>(nodePtr);
}
template <class NodeT, std::enable_if_t<!is_shared_ptr_v<NodeT>> = true>
auto constTransformNode(const NodeConstPtr& nodePtr) {
    return std::dynamic_pointer_cast<const NodeT>(nodePtr);
}
}  // namespace detail

/**
 * @brief Search for nodes with a certain type in a pipelne
 * @param[in] pipeline Pipeline to search for nodes in
 * @tparam NodeT type of node to dynamic cast into
 * @note Ensure that `NodeT` is `NodeType` and not `shared_ptr<NodeType>`
 * @return vector of node shared pointers
 * @sa filterNodesByNames
 */
template <class NodeT>
std::vector<std::shared_ptr<const NodeT>> filterNodesByType(const dai::Pipeline& pipeline) {
    const auto& nodeMap = pipeline.getNodeMap();

    return nodeMap | rv::values |
           rv::transform([](const auto& nodePtr) { return std::dynamic_pointer_cast<const NodeT>(nodePtr); }) |
           rv::filter([&](const auto& nodePtr) { return nodePtr != nullptr; }) | ranges::to<std::vector>;
}

/**
 * @brief Get the connections ending at a particular node
 * @param[in] node The node of concern
 * @return Unspecified collection of dai::Node::Connection containing connections that end at the specified node. The
 * collection can be copied/moved and iterated upon using a bi-directional iterator
 * @sa getConnectionsFrom
 */
inline auto getConnectionsTo(const NodeConstPtr& node);

/**
 * @brief Get the connections originating from a particular node
 * @param[in] node The node of concern
 * @return vector of connections that start from the specified node
 * @sa getConnectionsTo
 */
std::vector<dai::Node::Connection> getConnectionsFrom(const NodeConstPtr& node);

/**
 * @brief Get the Outputs that are connected to the specified input of the specified node
 * @param[in] node The node of concern
 * @param[in] in The name of the input
 * @return Unspecified collection of dai::Node::Output containing outputs that are connected to the specified node
 * at the specified input. The collection can be copied/moved and iterated upon using a bi-directional iterator
 */
inline auto getInputOf(const NodeConstPtr& node, const std::string& in);

/**
 * @brief Get the Inputs that are connected to the specified output of the specified node
 * @param[in] node The node of concern
 * @param[in] in The name of the output
 * @return Unspecified collection of dai::Node::Input containing inputs that are connected to the specified node
 * at the specified output. The collection can be copied/moved and iterated upon using a bi-directional iterator
 */
inline auto getOutputOf(const NodeConstPtr& node, const std::string& out);

/**
 * @brief Reads a packet from an XLinkStream and frees it on destruction
 * @note Make sure to call `getData` or else the packet will go un-processed
 */
class PacketReader {
    PacketReader(dai::XLinkStream& stream)
            : _stream(stream) {
        _packet = _stream.readRaw();  // blocking read
    }
    ~PacketReader() { _stream.readRawRelease(); }
    auto getData() { return dai::StreamPacketParser::parsePacketToADatatype(_packet); }
    dai::XLinkStream& _stream;
    streamPacketDesc_t* _packet;
};

// @TODO: explicit return type or move them into an impl header
auto getConnectionsTo(const NodeConstPtr& node) {
    const auto& connectionMap = node->getParentPipeline().getConnectionMap();
    return connectionMap.at(node->id);
}

auto getInputOf(const NodeConstPtr& node, const std::string& in) {
    const auto& nodeMap = node->getParentPipeline().getNodeMap();
    const auto& connections = getConnectionsTo(node);

    using OutVec = decltype(nodeMap.at(0)->getOutputs());
    using Output = typename OutVec::value_type;
    std::vector<Output> outs;
    for (const auto& conn : connections) {
        if (conn.inputName != in) {
            continue;
        }
        auto outputs = nodeMap.at(conn.outputId)->getOutputs();
        for (auto& out : outputs) {
            if (out.name != conn.outputName) {
                continue;
            }
            outs.emplace_back(std::move(out));
        }
    }
    return outs;
}

auto getOutputOf(const NodeConstPtr& node, const std::string& out) {
    const auto& nodeMap = node->getParentPipeline().getNodeMap();
    const auto& connections = getConnectionsFrom(node);

    using InVec = decltype(nodeMap.at(0)->getInputs());
    using Input = typename InVec::value_type;
    std::vector<Input> ins;
    for (const auto& conn : connections) {
        if (conn.outputName != out) {
            continue;
        }
        auto inputs = nodeMap.at(conn.inputId)->getInputs();
        for (auto& in : inputs) {
            if (in.name != conn.inputName) {
                continue;
            }
            ins.emplace_back(std::move(in));
        }
    }
    return ins;
}
}  // namespace rr
