#include <depthai_ros_driver/dai_utils.hpp>

#include <gtest/gtest.h>

namespace rr {
TEST(Endianness, roundTrip) {
    std::uint32_t num;

    num = 87;
    EXPECT_NE(num, switchEndianness(num));
    EXPECT_EQ(num, switchEndianness(switchEndianness(num)));

    num = 76 << 8;
    EXPECT_NE(num, switchEndianness(num));
    EXPECT_EQ(num, switchEndianness(switchEndianness(num)));

    num = 65 << 16;
    EXPECT_NE(num, switchEndianness(num));
    EXPECT_EQ(num, switchEndianness(switchEndianness(num)));

    num = 54 << 24;
    EXPECT_NE(num, switchEndianness(num));
    EXPECT_EQ(num, switchEndianness(switchEndianness(num)));
}

TEST(Endianness, swapPosition) {
    std::array<std::uint8_t, 4> num = {1, 2, 3, 4}, expected = {4, 3, 2, 1}, out;

    switchEndianness(num.data(), out.data());
    EXPECT_EQ(out, expected);
}
}  // namespace rr