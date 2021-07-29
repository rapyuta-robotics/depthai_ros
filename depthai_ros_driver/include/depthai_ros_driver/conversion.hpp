#pragma once

#include <depthai_datatype_msgs/RawImgFrame.h>
#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <ros/console.h>

namespace rr {
template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

struct ImagePublishers {
    ros::Publisher image_pub, camera_info_pub;
};

auto getNumSubscribers(const ros::Publisher& pub) {
    return pub.getNumSubscribers();
}

auto getNumSubscribers(const ImagePublishers& pubs) {
    return pubs.image_pub.getNumSubscribers() + pubs.camera_info_pub.getNumSubscribers();
}

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

// convert opencv order from (Channel, Height, Width) to (Height, Width, Channel)
cv::Mat chw2hwc(const cv::Mat& mat);

// template specialization for converting depthai_datatype_msgs::RawImgFrame to sensor_msgs::Image
template <>
struct adapt_dai2ros<depthai_datatype_msgs::RawImgFrame> {
    using InputType = depthai_datatype_msgs::RawImgFrame;
    using OutputType = sensor_msgs::Image;

    static void publish(const ImagePublishers& pub, InputType& input, const std::string& frame_id) {
        cv_bridge::CvImage bridge;
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
        bridge.header.frame_id = frame_id + "_optical_frame";
        bridge.header.stamp.sec = input.ts.sec;
        bridge.header.stamp.nsec = input.ts.nsec;

        pub.image_pub.publish(bridge.toImageMsg());
        sensor_msgs::CameraInfo info;  // @TODO(kunaltyagi): get camera info from somewhere
        pub.camera_info_pub.publish(info);
    }

    static ImagePublishers create_publisher(ros::NodeHandle& nh, const std::string& name, std::size_t q_size) {
        ImagePublishers pubs;
        pubs.image_pub =
                nh.advertise<OutputType>(name + "/image_raw", q_size);  // @TODO(kunaltyagi): compressed if needed
        pubs.camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(name + "/camera_info", q_size);
        return pubs;
    }
};
}  // namespace rr
