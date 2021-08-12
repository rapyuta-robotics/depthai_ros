#ifndef DEPTHAI_ROS_DRIVER__DEPTHAI_COMMON_HPP
#define DEPTHAI_ROS_DRIVER__DEPTHAI_COMMON_HPP

#include <regex>

// Common ros and ros2 api
#include <cv_bridge/cv_bridge.h>

// relevant 3rd party includes
#include <depthai/device.hpp>
#include <depthai/host_data_packet.hpp>
#include <depthai/nnet/nnet_packet.hpp>
#include <depthai/pipeline/cnn_host_pipeline.hpp>

// general 3rd party includes
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/opencv.hpp>

// std includes
#include <algorithm>
#include <array>
#include <memory>
#include <string>

#if USE_ROS2
#include <depthai_ros_msgs/srv/trigger_named.hpp>
#include <depthai_ros_msgs/msg/objects.hpp>
#include <depthai_ros_msgs/msg/auto_focus_ctrl.hpp>
#include <sensor_msgs/msg/image.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using ObjectsMsg = depthai_ros_msgs::msg::Objects;
using ObjectMsg = depthai_ros_msgs::msg::Object;

#else
#include <depthai_ros_msgs/AutoFocusCtrl.h>
#include <depthai_ros_msgs/Object.h>
#include <depthai_ros_msgs/Objects.h>
#include <depthai_ros_msgs/TriggerNamed.h>

using ImageMsg = sensor_msgs::Image;
using CompressedImageMsg = sensor_msgs::CompressedImage;
using ObjectsMsg = depthai_ros_msgs::Objects;
using ObjectMsg = depthai_ros_msgs::Object;
#endif

namespace rr {

//==============================================================================
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

//==============================================================================
struct DepthAIBaseConfig{
    // General configs config
    bool force_usb2 = false;
    bool depthai_block_read = false;
    bool request_jpegout = false;

    // Pipeline configs
    std::string calib_file = "";
    std::string blob_file = "";
    std::string blob_file_config = "";
    bool compute_bbox_depth = false;
    bool full_fov_nn = false;
    bool sync_video_meta = false;
    int shaves = 14;
    int cmx_slices = 14;
    int nn_engines = 2;
    int rgb_height = 1080;
    int rgb_fps = 30;
    int depth_height = 720;
    int depth_fps = 30;
    std::vector<std::string> stream_list = {"video", "left", "depth"};
};

//==============================================================================
/// Get image data from packet
const std::pair<std::string, cv_bridge::CvImage> get_image_data(
    const HostDataPacket& packet,
    const Stream& type);

//==============================================================================
/// Create pipeline config string stream
const std::string create_pipeline_config(const DepthAIBaseConfig& cfg);

//==============================================================================
class DepthAICommon
{
public:
    template <typename T>
    DepthAICommon(T get_param_method);

    /// create camera stream publisher
    template <typename T>
    void create_stream(
        std::function<void(const Stream&)> set_camera_info_pub_method,
        T set_stream_pub_method);

    using PublishImageFn = 
        std::function<void(const HostDataPacket&, Stream type, double ts)>;
    using PublishObjectsFn = 
        std::function<void(const dai::Detections& detections, double ts)>;

    /// Callback funcs which will be called when process_and_publish_packets()
    /// received the relevant packets
    void register_callbacks(PublishImageFn img_fn, PublishObjectsFn objs_fn)
    {
        _publish_img_fn = std::move(img_fn);
        _publish_objs_fn = std::move(objs_fn);
    };

    /// get and process packets
    void process_and_publish_packets();

    /// set camera disparity threshold
    bool set_disparity(const float val);

    /// set camera autofocus
    bool set_autofocus(const bool trigger, const uint8_t mode);

    /// create detection to object msg
    ObjectsMsg convert(const dai::Detections& detections);

private:
    DepthAIBaseConfig _cfg;
    std::unique_ptr<Device> _depthai;
    std::shared_ptr<CNNHostPipeline> _pipeline;

    // TODO, bettwer way?
    std::array<std::string, Stream::END> _stream_name{
        "left", "right", "rectified_left", "rectified_right", "disparity",
        "disparity_color", "depth", "previewout", "jpegout", "video",
        "meta_d2h", "metaout", "object_tracker"};

    PublishImageFn _publish_img_fn;
    PublishObjectsFn _publish_objs_fn;
};

//==============================================================================
//==============================================================================
// TODO Move these to impl hpp
template <typename T>
void DepthAICommon::create_stream(
    std::function<void(const Stream&)> set_camera_info_pub_method,
    T set_stream_pub_method)
{
    for (const auto& stream : _cfg.stream_list) {
        const auto it = std::find(_stream_name.cbegin(), _stream_name.cend(), stream);
        const auto index = static_cast<Stream>(std::distance(_stream_name.cbegin(), it));

        if (index < Stream::IMAGE_END) {
            set_camera_info_pub_method(index);
            if (index < Stream::UNCOMPRESSED_IMG_END) {
                set_stream_pub_method(index, ImageMsg{});
            } else {
                set_stream_pub_method(index, CompressedImageMsg{});
            }
            continue;
        }
        switch (index) {
            case Stream::META_OUT:
                set_stream_pub_method(index, ObjectsMsg{});
                break;
            case Stream::OBJECT_TRACKER:
                set_stream_pub_method(index, ObjectMsg{});
                break;
            case Stream::META_D2H:
                set_stream_pub_method(index, ImageMsg{});
                break;
            default:
                // TODO: roslog?
                std::cout << "Unknown stream requested: " << stream << std::endl;
        }
    }
}

//==============================================================================
template <typename T>
DepthAICommon::DepthAICommon(T get_param_method)
{
    get_param_method("calibration_file", _cfg.calib_file);
    get_param_method("blob_file", _cfg.blob_file);
    get_param_method("blob_file_config", _cfg.blob_file_config);
    get_param_method("stream_list", _cfg.stream_list);
    get_param_method("depthai_block_read", _cfg.depthai_block_read);
    get_param_method("enable_sync", _cfg.sync_video_meta);
    get_param_method("full_fov_nn", _cfg.full_fov_nn);
    get_param_method("disable_depth", _cfg.compute_bbox_depth);
    get_param_method("force_usb2", _cfg.force_usb2);
    get_param_method("rgb_height", _cfg.rgb_height);
    get_param_method("rgb_fps", _cfg.rgb_fps);
    get_param_method("depth_height", _cfg.depth_height);
    get_param_method("depth_fps", _cfg.depth_fps);
    get_param_method("shaves", _cfg.shaves);
    get_param_method("cmx_slices", _cfg.cmx_slices);
    get_param_method("nn_engines", _cfg.nn_engines);

    _depthai = std::make_unique<Device>("", _cfg.force_usb2);
    _depthai->request_af_mode(static_cast<CaptureMetadata::AutofocusMode>(4));

    const auto _pipeline_config_json = create_pipeline_config(_cfg);
    _pipeline = _depthai->create_pipeline(_pipeline_config_json);
}

}  // namespace rr

#endif
