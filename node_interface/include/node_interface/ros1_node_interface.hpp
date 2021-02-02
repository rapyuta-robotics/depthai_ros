#pragma once

/**
 * SPDX-License-Identifier: MPLv2
 *
 * Copyright (c) 2020- Rapyuta Robotics Inc.
 * Copyright (c) 2019-2020 Rakuten Inc. (MPL v2)
 */

#include <node_interface/introspection.hpp>

#include <ros/ros.h>

namespace rr {
namespace detail {
SETUP_HAS_PUBLIC_TYPE_ALIAS(nodehandle_t);

template <class Node, class = void>
struct node_traits {
    using nodehandle_t = ros::NodeHandle;
};

template <class Node>
struct node_traits<Node, detail::HasPublicTypeAlias_nodehandle_t<Node>> {
    using nodehandle_t = typename Node::nodehandle_t;
};
}  // namespace detail

/**
 * @brief indirection to get the type of nodehandle without using decltype everytime
 * @tparam Node type to introspect
 * @details If `Node::nodehandle_t` exists, use that, else use `ros::NodeHandle`
 */
template <class Node>
using node_traits = detail::node_traits<Node>;

/**
 * @brief Interface node to make transitions easier to handle:
 * * node -> nodelet
 * * using custom Node handles
 */
template <class NodeHandle = ros::NodeHandle>
class ROS1Node {
protected:
    using Base = NodeHandle;

private:
    using Self = ROS1Node<NodeHandle>;

    NodeHandle m_nh, m_privNh = {"~"};

public:
    using nodehandle_t = NodeHandle;

    NodeHandle& getNodeHandle() { return m_nh; }
    NodeHandle& getPrivateNodeHandle() { return m_privNh; }

    const std::string& getName() const { return ros::this_node::getName(); }

    std::string getSuffixedName(const std::string& suffix_) const { return getName() + "." + suffix_; }

    ROS1Node() = default;
    virtual ~ROS1Node() = default;

    /**
     * @brief provided to make "node" independent from ros::init
     * @details brings the ROS1 node closer to the independent nodes in ROS2
     */
    void init() { onInit(); }

    /**
     * @brief ALMOST a drop-in replacement for ros.nodelet::Nodelet
     * @details argv_, qSingleThread_ and qMultiThread_ are ignored
     */
    void init(const std::string& name_, const ros::M_string& remappingArgs_, const ros::V_string& /*argv_*/ = {},
            ros::CallbackQueueInterface* /*qSingleThread_*/ = nullptr,
            ros::CallbackQueueInterface* /*qMultiThread_*/ = nullptr) {
        m_privNh = NodeHandle(name_, remappingArgs_);
        m_nh = NodeHandle(ros::names::parentNamespace(name_), remappingArgs_);
        onInit();
    }

protected:
    /**
     * @brief Interface required to be implemented by derived class
     */
    virtual void onInit() = 0;
};

/**
 * @brief Default type as a drop-in replacement for ROS1 nodelets
 */
using DefaultROS1Node = ROS1Node<>;

static_assert(detail::has_public_type_alias_nodehandle_t_v<DefaultROS1Node>);
}  // namespace rr
