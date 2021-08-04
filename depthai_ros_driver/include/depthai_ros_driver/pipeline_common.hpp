#pragma once

#include <ros/ros.h>

namespace rr {

/**
 * @brief get ros param if the prameter exists. set ros param if the parameter doesn't exist
 */
template <typename T>
void sync_ros_param(const ros::NodeHandle& nh, const std::string& key, T& val) {
    if (!nh.getParam(key, val)) {
        nh.setParam(key, val);
    }
}

}  // namespace rr
