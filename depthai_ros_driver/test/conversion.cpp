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

TEST(chw2hwc, base_comparison) {
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

TEST(chw2hwc, tripleTrip) {
    cv::Mat mat(3, 3, CV_8UC3);
    uchar* mat_ptr = mat.data;
    for (int i = 1; i <= 27; ++i) {
        mat_ptr[i] = i;
    }

    cv::Mat once = chw2hwc(mat);
    cv::Mat twice = chw2hwc(chw2hwc(mat));
    cv::Mat thrice = chw2hwc(chw2hwc(chw2hwc(mat)));

    // expect doing three times resulting the original, rest gonna be different
    // chw -> hwc -> wch -> chw
    EXPECT_FALSE(cvMatAreEqual(mat, once));
    EXPECT_FALSE(cvMatAreEqual(mat, twice));
    EXPECT_TRUE(cvMatAreEqual(mat, thrice));
}

}  // namespace rr
