#include <depthai_ros_driver/pipeline_loader.hpp>

namespace rr {
pluginlib::UniquePtr<Pipeline> load_pipeline(std::string plugin_name) {
    static pluginlib::ClassLoader<Pipeline> pipeline_loader("depthai_ros_driver", "rr::Pipeline");

    try {
        // create a unique instance to pass on the ownership
        return pipeline_loader.createUniqueInstance(plugin_name);
    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR("DepthAI Driver failed to load plugin due to some reason. Error: %s", ex.what());
        return nullptr;
    }
}
}  // namespace rr