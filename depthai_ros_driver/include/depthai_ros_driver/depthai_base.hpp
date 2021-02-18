#pragma once

// self-message includes
#include <depthai_ros_msgs/AutoFocusCtrl.h>
#include <depthai_ros_msgs/Object.h>
#include <depthai_ros_msgs/Objects.h>
#include <depthai_ros_msgs/TriggerNamed.h>

// core ROS dependency includes
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <ros/ros.h>

// ROS1 messages
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

// relevant 3rd party includes
#include <depthai/depthai.hpp> // to be more modular
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/Pipeline.hpp>

// #include <depthai/device.hpp>
//#include <depthai/host_data_packet.hpp>
//#include <depthai/nnet/nnet_packet.hpp>
//#include <depthai/pipeline/cnn_host_pipeline.hpp>

// general 3rd party includes
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/opencv.hpp>

// std includes
#include <algorithm>
#include <array>
#include <memory>
#include <string>

#include <depthai_ros_driver/pipeline.hpp>

namespace rr {

template <class Node>
class DepthAIBase : public Node {
public:
    DepthAIBase() = default;
    ~DepthAIBase() = default;

private:
    enum Stream : std::size_t {
        // all images at the beginning
        LEFT,
        RIGHT,
        RECTIFIED_LEFT,
        RECTIFIED_RIGHT,
        DISPARITY,
        DISPARITY_COLOR,
        DEPTH,
        PREVIEW_OUT,
        // compressed image streams
        JPEG_OUT,
        VIDEO,
        // non-image streams
        META_D2H,
        META_OUT,
        OBJECT_TRACKER,
        // utility enums
        END,
        IMAGE_END = META_D2H,
        UNCOMPRESSED_IMG_END = JPEG_OUT
    };
    std::array<std::string, Stream::END> _stream_name{"left", "right", "rectified_left", "rectified_right", "disparity",
            "disparity_color", "depth", "previewout", "jpegout", "video", "meta_d2h", "metaout", "object_tracker"};
    std::array<std::string, Stream::END> _topic_name{"left", "right", "rectified_left", "rectified_right", "disparity",
            "disparity_color", "depth", "previewout", "jpeg", "mjpeg", "meta_d2h", "object_info", "object_tracker"};

    std::array<std::unique_ptr<ros::Publisher>, Stream::END> _stream_publishers;
    std::array<std::unique_ptr<ros::Publisher>, Stream::IMAGE_END> _camera_info_publishers;

    std::array<std::unique_ptr<camera_info_manager::CameraInfoManager>, Stream::IMAGE_END> _camera_info_manager;
    std::unique_ptr<camera_info_manager::CameraInfoManager> _defaultManager;
    ros::ServiceServer _camera_info_default;

    boost::shared_ptr<rr::Pipeline> _stereo_pipeline;

    ros::Publisher _camera_info_pub;

    ros::Subscriber _af_ctrl_sub;
    ros::Subscriber _disparity_conf_sub;

    ros::Timer _cameraReadTimer;

    std::string _pipeline_config_json;

    // Parameters
    std::string _camera_name = "";
    std::string _camera_param_uri = "package://depthai_ros_driver/params/camera/";
    std::string _cmd_file = "";
    std::string _calib_file = "";
    std::string _blob_file = "";
    std::string _blob_file_config = "";

    bool _depthai_block_read = false;
    bool _request_jpegout = false;
    bool _sync_video_meta = false;
    bool _full_fov_nn = false;
    bool _force_usb2 = false;
    bool _compute_bbox_depth = false;

    int _rgb_height = 1080;
    int _rgb_fps = 30;
    int _depth_height = 720;
    int _depth_fps = 30;

    int _shaves = 14;
    int _cmx_slices = 14;
    int _nn_engines = 2;

    int _queue_size = 10;

    std::vector<std::string> _stream_list = {"video", "left", "depth"};

    ros::Time _stamp;
    double _depthai_ts_offset = -1;  // sadly, we don't have a way of measuring drift

    std::map<std::string, int> _nn2depth_map;
    // std::list<std::shared_ptr<NNetPacket>> _nnet_packet;
    // std::list<std::shared_ptr<HostDataPacket>> _data_packet;
    dai::Pipeline _pipeline;
    std::vector<std::string> _available_streams;

    void prepareStreamConfig();

    void afCtrlCb(const depthai_ros_msgs::AutoFocusCtrl msg);
    void disparityConfCb(const std_msgs::Float32::ConstPtr& msg);

    // void publishImageMsg(const HostDataPacket& packet, Stream type, const ros::Time& stamp);
    // void publishObjectInfoMsg(const dai::Detections& detections, const ros::Time& stamp);
    void publishCameraInfo(ros::Time stamp);

    void cameraReadCb(const ros::TimerEvent&);
    bool defaultCameraInfo(depthai_ros_msgs::TriggerNamed::Request& req, depthai_ros_msgs::TriggerNamed::Response& res);

    void createPipeline();
    void getAvailableStreams();

    void onInit() override;
};
}  // namespace rr

#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>
namespace rr {
using DepthAINode = DepthAIBase<ROS1Node<>>;
using DepthAINodelet = DepthAIBase<nodelet::Nodelet>;
}  // namespace rr
