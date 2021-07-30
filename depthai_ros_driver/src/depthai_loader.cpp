#include <depthai_ros_driver/device_ros.hpp>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "temp");

    ros::NodeHandle private_nh("~");

    std::string pipeline_name = "depthai_ros_driver/PreviewPipeline";
    if (!private_nh.getParam("pipeline", pipeline_name)) {
        private_nh.setParam("pipeline", pipeline_name);
    }

    std::string config;
    if (!private_nh.getParam("config", config)) {
        private_nh.setParam("config", config);
    }

    std::string blob_file = "./mobilenet-ssd.blob";
    private_nh.getParam("blob_file", blob_file);

    // load pipeline using pluginlib
    dai::Pipeline p;
    pluginlib::ClassLoader<rr::Pipeline> pipeline_loader("depthai_ros_driver", "rr::Pipeline");
    try {
        auto plugin = pipeline_loader.createUniqueInstance(pipeline_name);
        plugin->configure("");
        p = plugin->getPipeline();
    } catch (pluginlib::LibraryLoadException& ex) {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    // auto openvino_version = dai::OpenVINO::Version::VERSION_2020_3;
    // rr::DeviceROS driver(openvino_version);
    rr::DeviceROS driver;
    driver.startPipeline(p);

    ros::spin();
    return 0;
}
