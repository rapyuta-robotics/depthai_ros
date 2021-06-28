#include <depthai_ros_driver/device_ros.hpp>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/nodes.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "temp");
    ros::NodeHandle nh;
    std::string pipeline_name = "rr::DefaultPipeline";
    if (!nh.getParam("pipeline", pipeline_name)) {
        nh.setParam("pipeline", pipeline_name);
    }

    // load pipeline using pluginlib
    pluginlib::ClassLoader<rr::Pipeline> pipeline_loader("depthai_ros_driver", "rr::Pipeline");
    try {
    const auto loader = pipeline_loader.createUniqueInstance(pipeline_name);
    loader->configure("");
    } catch (pluginlib::LibraryLoadException& ex) {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    dai::Pipeline p;  // = loader->getPipeline();
    {
        using namespace dai::node;
        auto cam = p.create<ColorCamera>();
        auto out = p.create<XLinkOut>();
        auto inp = p.create<XLinkIn>();

        inp->out.link(cam->inputConfig);
        cam->preview.link(out->input);
    }

    rr::DeviceROS driver;
    driver.startPipeline(p);

    ros::spin();
    return 0;
}
