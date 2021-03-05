#pragma once

// self-message includes
#include <depthai_ros_msgs/FocusControlCommand.h>
#include <depthai_ros_msgs/Object.h>
#include <depthai_ros_msgs/Objects.h>
#include <depthai_ros_msgs/TriggerNamed.h>

// core ROS dependency includes
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <pluginlib/class_loader.h>

// ROS1 messages
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

// relevant 3rd party includes
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>
#include <depthai/device/Device.hpp>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/Pipeline.hpp>

// general 3rd party includes
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/opencv.hpp>

// std includes
#include <array>
#include <string>

#include <depthai_ros_driver/pipeline.hpp>

namespace rr {
using ImageFramePtr = std::shared_ptr<dai::ImgFrame>;
using ImageFrameConstPtr = std::shared_ptr<dai::ImgFrame const>;

using NNDataPtr = std::shared_ptr<dai::NNData>;
using NNDataConstPtr = std::shared_ptr<dai::NNData const>;

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

// NOTE: output_queue can be arbitrary names set by setStreamName. to be deleted.
struct StreamInfo {
    Stream id;
    std::string name;
    std::string topic;
    std::string output_queue;
};

std::unordered_map<std::string, StreamInfo> stream_info {
    {"left",            {LEFT,            "left",            "left",            "left",            }},
    {"right",           {RIGHT,           "right",           "right",           "right",           }},
    {"rectified_left",  {RECTIFIED_LEFT,  "rectified_left",  "rectified_left",  "rectified_left",  }},
    {"rectified_right", {RECTIFIED_RIGHT, "rectified_right", "rectified_right", "rectified_right", }},
    {"disparity",       {DISPARITY,       "disparity",       "disparity",       "disparity",       }},
    {"disparity_color", {DISPARITY_COLOR, "disparity_color", "disparity_color", "disparity_color", }},
    {"depth",           {DEPTH,           "depth",           "depth",           "depth",           }},
    {"previewout",      {PREVIEW_OUT,     "previewout",      "previewout",      "preview",         }},
    {"jpegout",         {JPEG_OUT,        "jpegout",         "jpeg",            "jpeg",            }},
    {"video",           {VIDEO,           "video",           "mjpeg",           "mjpeg",           }},
    {"meta_d2h",        {META_D2H,        "meta_d2h",        "meta_d2h",        "meta_d2h",        }},
    {"metaout",         {META_OUT,        "metaout",         "object_info",     "detections",      }},
    {"object_tracker",  {OBJECT_TRACKER,  "object_tracker",  "object_tracker",  "object_tracker",  }},
};


template <class Node>
class DepthAIBase : public Node {
public:
    DepthAIBase() = default;
    ~DepthAIBase() = default;

private:

    std::set<std::string> _enabled_streams;
    std::array<std::unique_ptr<ros::Publisher>, Stream::END> _stream_publishers;
    std::array<std::unique_ptr<ros::Publisher>, Stream::IMAGE_END> _camera_info_publishers;

    std::array<std::unique_ptr<camera_info_manager::CameraInfoManager>, Stream::IMAGE_END> _camera_info_manager;

    std::unique_ptr<camera_info_manager::CameraInfoManager> _defaultManager;

    std::unique_ptr<pluginlib::ClassLoader<rr::Pipeline> > _pipeline_loader;

    class_loader::ClassLoader::UniquePtr<rr::Pipeline> _pipeline_plugin;

    dai::Pipeline _pipeline;
    std::unique_ptr<dai::Device> _depthai;
    std::unordered_map<std::string, std::shared_ptr<dai::DataOutputQueue>> _data_output_queue;

    std::shared_ptr<dai::DataInputQueue> _color_control_queue = nullptr;
    std::shared_ptr<dai::DataInputQueue> _color_config_queue = nullptr;
    ros::Publisher _camera_info_pub;

    ros::Subscriber _af_ctrl_sub;
    ros::Subscriber _disparity_conf_sub;

    ros::Timer _cameraReadTimer;

    std::string _pipeline_config_json;

    // Parameters
    std::string _camera_name = "";
    std::string _camera_param_uri = "package://depthai_ros_driver/params/camera/";

    std::string _pipeline_name = "";

    std::string _cmd_file = "";
    std::string _calib_file = "";
    std::string _blob_file = "";
    std::string _blob_file_config = "";

    bool _depthai_block_read = false;
    bool _request_jpegout = false;
    bool _sync_video_meta = false;
    bool _force_usb2 = false;
    // bool _compute_bbox_depth = false;

    int _rgb_height = 1080;
    int _rgb_fps = 15;
    int _depth_height = 720;
    int _depth_fps = 10;

    int _maxDisp;
    bool _extended_disparity = false;
    bool _subpixel = false;
    bool _lrcheck = false;

    int _queue_size = 10;

    std::vector<std::string> _stream_list = {"video", "left", "depth"};

    ros::Time _stamp;
    // double _depthai_ts_offset = -1;  // sadly, we don't have a way of measuring drift

    bool has_stream(const std::string& stream) const;

    void prepareStreamConfig();

    std::string generatePipelineConfigJson() const;

    void focusControlCallback(const depthai_ros_msgs::FocusControlCommand& msg);
    void disparityConfCb(const std_msgs::Float32::ConstPtr& msg);

    void publishImageMsg(ImageFramePtr frame, const std::string& stream, ros::Time& stamp);
    void publishObjectInfoMsg(const NNDataConstPtr detections, const ros::Time& stamp);
    void publishCameraInfo(ros::Time stamp);

    void cameraReadCb(const ros::TimerEvent&);

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
