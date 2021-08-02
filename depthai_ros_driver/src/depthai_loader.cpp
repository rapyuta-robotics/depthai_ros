#include <depthai_ros_driver/device_ros.hpp>
#include <depthai_ros_driver/pipeline.hpp>
#include <depthai_ros_driver/pipeline_loader.hpp>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// For generating pipeline config config
#include <regex>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "temp");

    ros::NodeHandle private_nh("~");

    std::string pipeline_name = "depthai_ros_driver/MobilenetSSDPipeline";
    if (!private_nh.getParam("pipeline", pipeline_name)) {
        private_nh.setParam("pipeline", pipeline_name);
    }

    std::string blob_file = "./mobilenet-ssd.blob";
    if (!private_nh.getParam("blob_file", blob_file)) {
        private_nh.setParam("blob_file", blob_file);
    }

    // Generating config string
    std::string config;
    {
        boost::property_tree::ptree root, ai;
        ai.put("blob_file", blob_file);
        root.add_child("ai", ai);

        std::ostringstream oss;
        std::regex reg("\"(null|true|false|[0-9]+(\\.[0-9]+)?)\"");
        boost::property_tree::write_json(oss, root);
        config = std::regex_replace(oss.str(), reg, "$1");
    }

    // load pipeline using pluginlib
    auto plugin = rr::load_pipeline(pipeline_name);
    plugin->configure(config);
    dai::Pipeline p = plugin->getPipeline();

    auto openvino_version = dai::OpenVINO::Version::VERSION_2020_3;
    rr::DeviceROS driver(openvino_version);
    driver.startPipeline(p);

    ros::spin();
    return 0;
}
