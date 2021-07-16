#include <depthai_ros_driver/device_ros.hpp>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "temp");
    ros::NodeHandle nh;
    std::string pipeline_name = "rr::DefaultPipeline";
    if (!nh.getParam("pipeline", pipeline_name)) {
        nh.setParam("pipeline", pipeline_name);
    }

    ros::NodeHandle private_nh("~");
    std::string blob_file = "./mobilenet-ssd.blob";
    private_nh.getParam("blob_file", blob_file);
    auto openvino_version = dai::OpenVINO::Version::VERSION_2020_3;

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
        auto nn1 = p.create<dai::node::NeuralNetwork>();
        auto nno = p.create<dai::node::XLinkOut>();
        auto inp = p.create<XLinkIn>();
        auto ctl = p.create<XLinkIn>();

        out->setStreamName("preview");
        nno->setStreamName("detections");
        inp->setStreamName("input_config");
        ctl->setStreamName("control");

        // Color camera
        cam->setPreviewSize(300, 300);
        cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        cam->setInterleaved(false);
        cam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

        // object detection
        nn1->setBlobPath(blob_file);

        inp->out.link(cam->inputConfig);
        ctl->out.link(cam->inputControl);
        cam->preview.link(out->input);
        cam->preview.link(nn1->input);
        nn1->out.link(nno->input);
    }

    rr::DeviceROS driver(openvino_version);
    driver.startPipeline(p);

    ros::spin();
    return 0;
}
