#pragma once

#include <string>

#include <pluginlib/class_loader.hpp>

#include <depthai_ros_driver/pipeline.hpp>

namespace rr {
[[nodiscard]] pluginlib::UniquePtr<Pipeline> load_pipeline(std::string plugin_name);
}  // namespace rr
