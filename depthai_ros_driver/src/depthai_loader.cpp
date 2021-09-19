#include <depthai_ros_driver/device_ros.hpp>
#include <depthai_ros_driver/pipeline.hpp>
#include <depthai_ros_driver/pipeline_loader.hpp>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "temp");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string pipeline_name = "depthai_ros_driver/StereoWithMobilenetSSDPipeline";
    if (!private_nh.getParam("pipeline", pipeline_name)) {
        private_nh.setParam("pipeline", pipeline_name);
    }

    // load pipeline using pluginlib
    auto plugin = rr::load_pipeline(pipeline_name);
    plugin->configure(private_nh);
    dai::Pipeline p = plugin->getPipeline();

    rr::DeviceROS driver(nh, p.getOpenVINOVersion());
    driver.startPipeline(p);

    ros::spin();
    return 0;
}
