#pragma once

// core ros headers
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/console.h>
#include <ros/publisher.h>
#include <sensor_msgs/image_encodings.h>

// lib headers
#include <opencv2/core.hpp>

// message headers
#include <depthai_datatype_msgs/RawCameraControl.h>
#include <depthai_datatype_msgs/RawImgFrame.h>
#include <sensor_msgs/Image.h>

// std headers
#include <memory>
#include <string>
#include <type_traits>

namespace rr {
template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

const static std::string camera_param_uri{"package://depthai_ros_driver/params/camera"};

struct ImagePublishers {
    ros::Publisher raw_image_pub, compressed_image_pub, camera_info_pub;
    // needs to be shared_ptr since:
    // * can't be normal variable -> no copy/move function
    // * can't be unique_ptr -> lambda needs to be copied to create the pub thread
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_ptr;
};

std::uint32_t getNumSubscribers(const ros::Publisher& pub);

std::uint32_t getNumSubscribers(const ImagePublishers& pubs);

template <class T>
struct adapt_dai2ros {
    using InputType = remove_cvref_t<T>;
    using OutputType = remove_cvref_t<T>;

    // by default, return stuff as it is
    static void publish(const ros::Publisher& pub, InputType& input, const std::string& frame_id) {
        pub.publish(std::move(input));
    }

    // by default, use ros::Publisher
    static ros::Publisher create_publisher(ros::NodeHandle& nh, const std::string& name, std::size_t q_size) {
        auto result = nh.advertise<OutputType>(name, q_size);
        return result;
    }
};

template <class T>
auto create_publisher(ros::NodeHandle& nh, const std::string& name, std::size_t q_size) {
    return adapt_dai2ros<T>::create_publisher(nh, name, q_size);
}

template <class Pub, class T>
void publish(const Pub& pub, T& input, const std::string& frame_id) {
    adapt_dai2ros<T>::publish(pub, input, frame_id);
}

// retrieve input_t of the adapt_dai2ros
template <typename T>
using adapt_dai2ros_input_t = typename adapt_dai2ros<T>::InputType;

// retrieve output_t of the adapt_dai2ros
template <typename T>
using adapt_dai2ros_output_t = typename adapt_dai2ros<T>::OutputType;

// retrieve publisher_t of the adapt_dai2ros
template <typename T>
using adapt_dai2ros_publisher_t = typename adapt_dai2ros<T>::PublisherType;

// conversion from depthai_datatype_msgs::RawImgFrame to cv::Mat
cv::Mat convert_img(const depthai_datatype_msgs::RawImgFrame& input);

cv::Mat planar2interleaved(const cv::Mat& mat);

cv::Mat interleaved2planar(const cv::Mat& mat);

// template specialization for converting depthai_datatype_msgs::RawImgFrame to sensor_msgs::Image
template <>
struct adapt_dai2ros<depthai_datatype_msgs::RawImgFrame> {
    using InputType = depthai_datatype_msgs::RawImgFrame;
    using OutputType = sensor_msgs::Image;

    static void publish(const ImagePublishers& pub, InputType& input, const std::string& frame_id) {
        cv_bridge::CvImage bridge;
        bridge.header.frame_id = frame_id + "_optical_frame";
        bridge.header.stamp.sec = input.ts.sec;
        bridge.header.stamp.nsec = input.ts.nsec;

        if (static_cast<dai::RawImgFrame::Type>(input.fb.type) == dai::RawImgFrame::Type::BITSTREAM) {
            auto msg = boost::make_shared<sensor_msgs::CompressedImage>();
            msg->header = bridge.header;
            msg->format = "jpeg";  // @TODO: read from encoder
            msg->data = std::move(input.data);
            pub.compressed_image_pub.publish(msg);
        } else {
            bridge.image = convert_img(input);

            switch (bridge.image.type()) {
                case CV_8UC3:
                    bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
                    break;
                case CV_8UC1:
                    bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
                    break;
                case CV_16UC1:
                    bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                    break;
                default:
                    ROS_WARN_STREAM_ONCE("Unknown type: " << bridge.image.type() << " in adapt_dai2ros::convert");
                    break;
            }
            pub.raw_image_pub.publish(bridge.toImageMsg());
        }

        const auto camera_info = boost::make_shared<sensor_msgs::CameraInfo>(pub.info_manager_ptr->getCameraInfo());
        camera_info->header = bridge.header;
        pub.camera_info_pub.publish(camera_info);
    }

    static ImagePublishers create_publisher(ros::NodeHandle& nh, const std::string& name, std::size_t q_size) {
        ImagePublishers pubs;
        pubs.raw_image_pub = nh.advertise<OutputType>(name + "/image_raw", q_size);
        pubs.compressed_image_pub = nh.advertise<OutputType>(name + "/image_raw/compressed", q_size);
        pubs.camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(name + "/camera_info", q_size);

        std::string camera_name = "default";
        if (!nh.getParam("camera_name", camera_name)) {
            nh.setParam("camera_name", camera_name);
        }
        const auto uri = camera_param_uri + "/" + camera_name + "/" + name + ".yaml";
        pubs.info_manager_ptr =
                std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, name}, name, uri);
        return pubs;
    }
};

template <class T>
struct adapt_ros2dai {
    using InputType = remove_cvref_t<T>;
    using OutputType = remove_cvref_t<T>;

    // by default, return stuff as it is
    static const OutputType& convert(const InputType& input) { return input; }
};

#define RR_MOVE(r, d, name) output.name = std::move(reinterpret_cast<decltype(output.name)&>(input.name));
#define RR_MOVE_ALL(...) BOOST_PP_SEQ_FOR_EACH(RR_MOVE, _, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))

template <>
struct adapt_ros2dai<depthai_datatype_msgs::RawCameraControl> {
    using InputType = depthai_datatype_msgs::RawCameraControl;
    using OutputType = dai::RawCameraControl;

    static OutputType convert(InputType input) {
        OutputType output;
        RR_MOVE_ALL(cmdMask, autoFocusMode, lensPosition, expManual, aeRegion, afRegion, awbMode, sceneMode,
                antiBandingMode, effectMode, aeLockMode, awbLockMode, expCompensation, brightness, contrast, saturation,
                sharpness, lumaDenoise, chromaDenoise);
        return output;
    }
};
}  // namespace rr
