#pragma once

#include <depthai_datatype_msgs/RawImgFrame.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <ros/console.h>

namespace rr {

template <typename T>
struct adapt_dai2ros {
    using InputType = std::remove_cv_t<std::remove_reference_t<T>>;
    using OutputType = std::remove_cv_t<std::remove_reference_t<T>>;

    // by default, return stuff as it is
    static inline OutputType convert(const InputType& input) { return input; }
};

// retrieve input_t of the adapt_dai2ros
template <typename T>
using adapt_dai2ros_input_t = typename adapt_dai2ros<T>::InputType;

// retrieve output_t of the adapt_dai2ros
template <typename T>
using adapt_dai2ros_output_t = typename adapt_dai2ros<T>::OutputType;

// conversion from depthai_datatype_msgs::RawImgFrame to cv::Mat
cv::Mat convert_img(const depthai_datatype_msgs::RawImgFrame& input);

// convert opencv order from (Channel, Height, Width) to (Height, Width, Channel)
cv::Mat chw2hwc(const cv::Mat& mat);

// template specialization for converting depthai_datatype_msgs::RawImgFrame to sensor_msgs::Image
template <>
struct adapt_dai2ros<depthai_datatype_msgs::RawImgFrame> {
    using InputType = depthai_datatype_msgs::RawImgFrame;
    using OutputType = sensor_msgs::Image;

    static OutputType convert(const InputType& input) {
        cv_bridge::CvImage bridge;
        bridge.image = convert_img(input);

        // handle weird case
        if (static_cast<dai::RawImgFrame::Type>(input.fb.type) == dai::RawImgFrame::Type::NV12) {
            // weird case when 'video' image is type dai::RawImgFrame::Type::NV12 instead of BGR
            cv::cvtColor(bridge.image, bridge.image, cv::COLOR_YUV2BGR_NV12);
        } else if (static_cast<dai::RawImgFrame::Type>(input.fb.type) == dai::RawImgFrame::Type::BGR888p) {
            // weird case of 'preview' image when setInterleaved is set to False
            bridge.image = chw2hwc(bridge.image);
        }

        switch (bridge.image.type()) {
            case CV_8UC3:
                bridge.encoding = sensor_msgs::image_encodings::BGR8;
                break;
            case CV_8UC1:
                bridge.encoding = sensor_msgs::image_encodings::MONO8;
                break;
            case CV_16UC1:
                bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                break;
            default:
                ROS_WARN_STREAM_ONCE("Unknown type: " << bridge.image.type() << " in adapt_dai2ros::convert");
                break;
        }
        // bridge.header.frame_id = ? TODO:
        bridge.header.stamp.sec = input.ts.sec;
        bridge.header.stamp.nsec = input.ts.nsec;

        return *bridge.toImageMsg();
    }
};

}  // namespace rr
