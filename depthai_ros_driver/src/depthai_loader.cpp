#include <depthai_ros_driver/device_ros.hpp>
#include <depthai_ros_driver/pipeline.hpp>
#include <depthai_ros_driver/pipeline_loader.hpp>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "depthai");

    ros::NodeHandle private_nh("~");

    std::string pipeline_name = "depthai_ros_driver/StereoWithMobilenetSSDPipeline";
    if (!private_nh.getParam("pipeline", pipeline_name)) {
        private_nh.setParam("pipeline", pipeline_name);
    }
    std::string mx_id;
    dai::DeviceInfo device_info;
    if (!private_nh.getParam("mx_id", mx_id)) {
        ROS_INFO("Trying to find a device");
        auto info = rr::DeviceROS::getFirstAvailableDevice();
        device_info = std::get<1>(info);
        mx_id = device_info.getMxId();
        if (!std::get<0>(info)) {
            ROS_WARN("Device not available (mx_id = %s)", mx_id.c_str());
        }
        private_nh.setParam("mx_id", mx_id);
        ROS_INFO("Found device: %s", mx_id.c_str());
    } else {
        auto info = rr::DeviceROS::getDeviceByMxId(mx_id);
        device_info = std::get<1>(info);
        assert(mx_id == device_info.getMxId());
        if (!std::get<0>(info)) {
            ROS_WARN("Device not available (mx_id = %s)", mx_id.c_str());
        }
        ROS_INFO("Using device: %s", mx_id.c_str());
    }

    // load pipeline using pluginlib
    auto plugin = rr::load_pipeline(pipeline_name);
    plugin->configure(private_nh);
    dai::Pipeline p = plugin->getPipeline();

    rr::DeviceROS driver(private_nh, p.getOpenVINOVersion(), device_info);
    driver.startPipeline(p);

    ros::spin();
    return 0;
}
