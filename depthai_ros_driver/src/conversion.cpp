#include "depthai_ros_driver/conversion.hpp"

namespace rr {
cv::Mat chw2hwc(const cv::Mat& mat) {
    cv::Mat new_mat = cv::Mat(mat.size(), mat.type());
    const auto WH = mat.cols * mat.rows;

    const uchar* src = mat.data;
    uchar* dst = new_mat.data;
    for (int i = 0; i < WH; ++i) {
        dst[3 * i + 0] = src[0 * WH + i];
        dst[3 * i + 1] = src[1 * WH + i];
        dst[3 * i + 2] = src[2 * WH + i];
    }
    return new_mat;
}

// prepare the cv size and cv type given dai::RawImgFrame::Type
cv::Mat prepare_cv_mat_from_dai_type(dai::RawImgFrame::Type dai_type, int rows, int cols, int size) {
    cv::Size cv_size = {0, 0};
    int cv_type = 0;

    switch (dai_type) {
        case dai::RawImgFrame::Type::RGB888i:
        case dai::RawImgFrame::Type::BGR888i:
        case dai::RawImgFrame::Type::BGR888p:
        case dai::RawImgFrame::Type::RGB888p:
            cv_size = cv::Size(cols, rows);
            cv_type = CV_8UC3;
            break;

        case dai::RawImgFrame::Type::YUV420p:
        case dai::RawImgFrame::Type::NV12:
        case dai::RawImgFrame::Type::NV21:
            cv_size = cv::Size(cols, rows * 3 / 2);
            cv_type = CV_8UC1;
            break;

        case dai::RawImgFrame::Type::RAW8:
        case dai::RawImgFrame::Type::GRAY8:
            cv_size = cv::Size(cols, rows);
            cv_type = CV_8UC1;
            break;

        case dai::RawImgFrame::Type::GRAYF16:
            cv_size = cv::Size(cols, rows);
            cv_type = CV_16FC1;
            break;

        case dai::RawImgFrame::Type::RAW16:
            cv_size = cv::Size(cols, rows);
            cv_type = CV_16UC1;
            break;

        case dai::RawImgFrame::Type::RGBF16F16F16i:
        case dai::RawImgFrame::Type::BGRF16F16F16i:
        case dai::RawImgFrame::Type::RGBF16F16F16p:
        case dai::RawImgFrame::Type::BGRF16F16F16p:
            cv_size = cv::Size(cols, rows);
            cv_type = CV_16FC3;
            break;

        case dai::RawImgFrame::Type::BITSTREAM:
        default:
            cv_size = cv::Size(static_cast<int>(size), 1);
            cv_type = CV_8UC1;
            break;
    }
    return cv::Mat(cv_size, cv_type);
}

cv::Mat convert_img(const depthai_datatype_msgs::RawImgFrame& input) {
    // prepare cv_mat
    cv::Mat mat = prepare_cv_mat_from_dai_type(
            static_cast<dai::RawImgFrame::Type>(input.fb.type), input.fb.height, input.fb.width, input.data.size());

    // Check if enough data
    long requiredSize = CV_ELEM_SIZE(mat.type()) * mat.size().area();
    long actualSize = static_cast<long>(input.data.size());
    if (actualSize < requiredSize) {
        ROS_WARN_STREAM("ImgFrame doesn't have enough data to encode specified frame, required "
                        << requiredSize << ", actual " << actualSize << ". Maybe metadataOnly transfer was made?");
    }
    if (input.fb.width <= 0 || input.fb.height <= 0) {
        ROS_WARN("ImgFrame metadata not valid (width or height = 0)");
    }

    // actual copy to cv_mat
    assert(input.data.size() <= static_cast<long>(mat.dataend - mat.datastart));
    std::memcpy(mat.data, input.data.data(), input.data.size());

    // handle weird case
    if (static_cast<dai::RawImgFrame::Type>(input.fb.type) == dai::RawImgFrame::Type::NV12) {
        // weird case when 'video' image is type dai::RawImgFrame::Type::NV12 instead of BGR
        cv::cvtColor(mat, mat, cv::COLOR_YUV2BGR_NV12);
    } else if (static_cast<dai::RawImgFrame::Type>(input.fb.type) == dai::RawImgFrame::Type::BGR888p) {
        // weird case of 'preview' image when setInterleaved is set to False
        mat = chw2hwc(mat);
    }

    return mat;
}

}  // namespace rr
