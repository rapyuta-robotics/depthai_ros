#pragma once

#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/StreamPacketParser.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/xlink/XLinkStream.hpp>

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
std::vector<NodeConstPtr> filterNodesByNames(const dai::Pipeline& pipeline, const Range& node_names) {
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
template <class T>
struct remove_smart_ptr {
    using type = T;
};
template <class T>
using remove_smart_ptr_t = typename remove_smart_ptr<T>::type;

template <class T>
struct remove_smart_ptr<std::shared_ptr<T>> {
    using type = T;
};
template <class T>
struct remove_smart_ptr<std::unique_ptr<T>> {
    using type = T;
};
template <class T>
struct remove_smart_ptr<std::weak_ptr<T>> {
    using type = T;
};
}  // namespace detail

/**
 * @brief Search for nodes with a certain type in a pipeline
 * @param[in] pipeline Pipeline to search for nodes in
 * @tparam NodeT type of node to dynamic cast into. If it's of form shared_ptr<T>, then the smart pointer is stripped
 * away
 * @return vector of node shared pointers
 * @sa filterNodesByNames
 */
template <class NodeT>
std::vector<std::shared_ptr<const NodeT>> filterNodesByType(const dai::Pipeline& pipeline) {
    const auto& nodeMap = pipeline.getNodeMap();

    return nodeMap | rv::values | rv::transform([](const auto& nodePtr) {
        return std::dynamic_pointer_cast<const detail::remove_smart_ptr_t<NodeT>>(nodePtr);
    }) | rv::filter([&](const auto& nodePtr) { return nodePtr != nullptr; }) |
           ranges::to<std::vector>;
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
 * @brief get a type shared by the links (which may belong to different nodes/pipelines)
 * @param[in] links should be iterable and the value_type should have a possibleDatatypes data member
 * @tparam Range Example: std::vector<dai::{In,Out}put> or a variant of the same
 * @return A type which encompasses the types of all other links
 */
template <class Range>
dai::DatatypeEnum getCommonType(const Range& links) {
    std::optional<dai::DatatypeEnum> common_type;
    for (const auto& link : links) {
        // get the link type
        const auto& allowed_types = link.possibleDatatypes;
        const dai::DatatypeEnum type = [&] {
            if (allowed_types.size() > 1) {
                return dai::DatatypeEnum::Buffer;
            } else {
                return allowed_types[0].datatype;
            }
        }();
        if (!common_type) {
            common_type = type;
        } else if (common_type.value() != type) {
            // @TODO: find the common type, not necessarily the buffer
            common_type = dai::DatatypeEnum::Buffer;
        }
    }
    if (!common_type) {
        return dai::DatatypeEnum::Buffer;
    }
    return common_type.value();
}

template <class Range>
auto getAllInputs(const Range& nodes, const dai::Pipeline& pipeline) {
    // hack to get the Output type
    using OutVec = decltype(nodes[0]->getOutputRefs());
    using Output = std::remove_const_t<std::remove_pointer_t<typename OutVec::value_type>>;

    struct ret {
        std::shared_ptr<const dai::node::XLinkOut> node_to;
        std::vector<Output> out_from;
    };

    std::vector<ret> out;
    for (const auto& node : nodes) {
        // for all input slots of the node
        ret data;
        data.node_to = node;
        const auto& inputs = node->getInputRefs();
        for (const auto& inp : inputs) {
            const auto& links = getInputOf(node, inp->name);
            // for all links to the slot
            for (const auto& link : links) {
                data.out_from.push_back(link);  // @TODO: check if this is working
            }
        }
        out.emplace_back(std::move(data));
    }
    return out;
}

template <class NodeT = dai::node::XLinkOut>
auto getAllInputs(const dai::Pipeline& pipeline) {
    const auto& nodes = filterNodesByType<detail::remove_smart_ptr_t<NodeT>>(pipeline);
    return getAllInputs(nodes, pipeline);
}

template <class Range>
auto getAllOutputs(const Range& nodes, const dai::Pipeline& pipeline) {
    // hack to get the Input type
    using InVec = decltype(nodes[0]->getInputRefs());
    using Input = std::remove_const_t<std::remove_pointer_t<typename InVec::value_type>>;

    struct ret {
        std::shared_ptr<const dai::node::XLinkIn> node_from;
        std::vector<Input> in_to;
    };

    std::vector<ret> in;
    for (const auto& node : nodes) {
        // for all output slots of the node
        ret data;
        data.node_from = node;
        const auto& outputs = node->getOutputRefs();
        for (const auto& outp : outputs) {
            const auto& links = getOutputOf(node, outp->name);
            // for all links to the slot
            for (const auto& link : links) {
                data.in_to.push_back(link);  // @TODO: check if this is working
            }
        }
        in.emplace_back(std::move(data));
    }
    return in;
}

template <class NodeT = dai::node::XLinkIn>
auto getAllOutputs(const dai::Pipeline& pipeline) {
    const auto& nodes = filterNodesByType<detail::remove_smart_ptr_t<NodeT>>(pipeline);
    return getAllOutputs(nodes, pipeline);
}

///////////////// IMPL ////////////////////
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

/**
 * @brief inverts the 4 bytes as follows:
 * Original: | A | B | C | D |  <-- data
 * Returned: | D | C | B | A |  <-- out
 *           |MSB|...|...|LSB| (Most and Least Significant bits)
 *
 * @param data pointer to data, at least 4 byte wide
 * @param out pointer, at least 4 byte wide
 */
void switchEndianness(const std::uint8_t* const data, std::uint8_t* const out);

/**
 * @brief Switches the endianness of the input
 *
 * @param data number to convert
 * @return std::uint32_t data, but in a different endian format
 * @sa switchEndianness
 */
std::uint32_t switchEndianness(std::uint32_t data);
}  // namespace rr
