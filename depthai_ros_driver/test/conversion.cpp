#include <depthai_ros_driver/conversion.hpp>

#include <gtest/gtest.h>

namespace rr {
bool cvMatAreEqual(const cv::Mat& a, const cv::Mat& b) {
    if (a.size() != b.size()) {
        return false;
    }

    cv::Mat temp;
    cv::bitwise_xor(a, b, temp);
    return cv::countNonZero(temp.reshape(1)) == 0;
}

void iota_fill(uchar* data, int n) {
    for (int i = 0; i < n; ++i) {
        data[i] = i;
    }
}

TEST(conversion, base_comparison) {
    // test that cvMatAreEqual is working as expected
    uchar* a = new uchar[3]{1, 2, 3};
    uchar* b = new uchar[3]{1, 2, 3};
    uchar* c = new uchar[3]{4, 5, 6};

    cv::Mat A(3, 1, CV_8UC1, a);
    cv::Mat B(3, 1, CV_8UC1, b);
    cv::Mat C(3, 1, CV_8UC1, c);

    EXPECT_TRUE(cvMatAreEqual(A, B));
    EXPECT_FALSE(cvMatAreEqual(B, C));
    EXPECT_FALSE(cvMatAreEqual(A, C));

    delete[] a;
    delete[] b;
    delete[] c;
}

TEST(conversion, base_fill) {
    // test that iota_fill is working as expected
    cv::Mat A(1, 2, CV_8UC3);

    iota_fill(A.data, A.cols * A.rows * A.channels());

    auto pt1 = A.at<cv::Vec3b>(0);
    auto pt2 = A.at<cv::Vec3b>(1);

    EXPECT_EQ(pt1[0], 0);
    EXPECT_EQ(pt1[1], 1);
    EXPECT_EQ(pt1[2], 2);

    EXPECT_EQ(pt2[0], 3);
    EXPECT_EQ(pt2[1], 4);
    EXPECT_EQ(pt2[2], 5);
}

TEST(conversion, planar2interleaved2planar_3x3) {
    cv::Mat planar_source(3, 3, CV_8UC3);
    uchar* src = planar_source.data;
    iota_fill(src, planar_source.rows * planar_source.cols * planar_source.channels());

    cv::Mat interleaved = rr::planar2interleaved(planar_source);
    cv::Mat planar = rr::interleaved2planar(interleaved);

    EXPECT_FALSE(cvMatAreEqual(planar_source, interleaved));
    EXPECT_TRUE(cvMatAreEqual(planar_source, planar));
}

TEST(conversion, interleaved2planar2interleaved_3x3) {
    cv::Mat interleaved_source(3, 3, CV_8UC3);
    uchar* src = interleaved_source.data;
    iota_fill(src, interleaved_source.rows * interleaved_source.cols * interleaved_source.channels());


    cv::Mat planar = rr::interleaved2planar(interleaved_source);
    cv::Mat interleaved = rr::planar2interleaved(planar);

    EXPECT_FALSE(cvMatAreEqual(interleaved_source, planar));
    EXPECT_TRUE(cvMatAreEqual(interleaved_source, interleaved));
}

TEST(conversion, planar2interleaved2planar_5x5) {
    cv::Mat planar_source(5, 5, CV_8UC3);
    uchar* src = planar_source.data;
    iota_fill(src, planar_source.rows * planar_source.cols * planar_source.channels());

    cv::Mat interleaved = rr::planar2interleaved(planar_source);
    cv::Mat planar = rr::interleaved2planar(interleaved);

    EXPECT_FALSE(cvMatAreEqual(planar_source, interleaved));
    EXPECT_TRUE(cvMatAreEqual(planar_source, planar));
}

TEST(conversion, interleaved2planar2interleaved_5x5) {
    cv::Mat interleaved_source(5, 5, CV_8UC3);
    uchar* src = interleaved_source.data;
    iota_fill(src, interleaved_source.rows * interleaved_source.cols * interleaved_source.channels());


    cv::Mat planar = rr::interleaved2planar(interleaved_source);
    cv::Mat interleaved = rr::planar2interleaved(planar);

    EXPECT_FALSE(cvMatAreEqual(interleaved_source, planar));
    EXPECT_TRUE(cvMatAreEqual(interleaved_source, interleaved));
}

}  // namespace rr
