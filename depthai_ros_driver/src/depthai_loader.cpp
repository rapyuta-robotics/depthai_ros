#include <depthai_ros_driver/device_ros.hpp>
#include <depthai_ros_driver/pipeline.hpp>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "temp");

    ros::NodeHandle private_nh("~");

    std::string pipeline_name = "rr::DefaultPipeline";
    if (!private_nh.getParam("pipeline", pipeline_name)) {
        private_nh.setParam("pipeline", pipeline_name);
    }

    std::string config;
    if (!private_nh.getParam("config", config)) {
        private_nh.setParam("config", config);
    }

    std::string blob_file = "./mobilenet-ssd.blob";
    private_nh.getParam("blob_file", blob_file);
    auto openvino_version = dai::OpenVINO::Version::VERSION_2020_3;

    // load pipeline using pluginlib
    pluginlib::ClassLoader<rr::Pipeline> pipeline_loader("depthai_ros_driver", "rr::Pipeline");
    try {
    const auto loader = pipeline_loader.createUniqueInstance(pipeline_name);
    loader->configure(config);
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

        // video output
        auto xout_video = p.create<XLinkOut>();
        xout_video->setStreamName("video");
        cam->video.link(xout_video->input);

        // mono left
        auto mono_left = p.create<MonoCamera>();
        auto xout_left = p.create<XLinkOut>();
        xout_left->setStreamName("left");
        mono_left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        mono_left->setBoardSocket(dai::CameraBoardSocket::LEFT);
        mono_left->out.link(xout_left->input);

        // mono right
        auto mono_right = p.create<MonoCamera>();
        auto xout_right = p.create<XLinkOut>();
        xout_right->setStreamName("right");
        mono_right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        mono_right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        mono_right->out.link(xout_right->input);

        // stereo
        auto stereo = p.create<dai::node::StereoDepth>();
        stereo->setConfidenceThreshold(200);
        stereo->setRectifyEdgeFillColor(0);
        stereo->setLeftRightCheck(false);
        stereo->setExtendedDisparity(false);
        stereo->setSubpixel(false);
        mono_left->out.link(stereo->left);
        mono_right->out.link(stereo->right);
        stereo->syncedLeft.link(xout_left->input);
        stereo->syncedRight.link(xout_right->input);

        // depth
        auto xout_depth = p.create<XLinkOut>();
        xout_depth->setStreamName("depth");
        stereo->depth.link(xout_depth->input);

        // disparity
        auto xout_disparity = p.create<XLinkOut>();
        xout_disparity->setStreamName("disparity");
        stereo->disparity.link(xout_disparity->input);
    }

    rr::DeviceROS driver(openvino_version);
    driver.startPipeline(p);

    ros::spin();
    return 0;
}
