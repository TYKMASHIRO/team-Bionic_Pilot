#include <gtest/gtest.h>

#include "drivers/realman/include/RmErrorMap.hpp"

using namespace robotics;

// RM 错误码 → 统一 Error 转换测试（无硬件）
TEST(RmErrorMapTest, ZeroIsOk) {
    auto e = realman::rm_to_error(0, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_TRUE(e.is_ok());
}

TEST(RmErrorMapTest, ParamError) {
    auto e = realman::rm_to_error(1, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_EQ(e.category, domain::ErrorCategory::Validation);
    EXPECT_TRUE(e.retryable);
    EXPECT_EQ(e.raw_vendor_code.value(), 1);
}

TEST(RmErrorMapTest, SendFail) {
    auto e = realman::rm_to_error(-1, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_EQ(e.category, domain::ErrorCategory::Communication);
    EXPECT_TRUE(e.retryable);
}

TEST(RmErrorMapTest, Timeout) {
    auto e = realman::rm_to_error(-2, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_EQ(e.category, domain::ErrorCategory::Timeout);
}

TEST(RmErrorMapTest, ParseFail) {
    auto e = realman::rm_to_error(-3, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_EQ(e.category, domain::ErrorCategory::Protocol);
}

TEST(RmErrorMapTest, MotionStop) {
    auto e = realman::rm_to_error(-6, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_EQ(e.category, domain::ErrorCategory::Cancelled);
}

TEST(RmErrorMapTest, UnsupportedFourGen) {
    auto e = realman::rm_to_error(-4, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_EQ(e.category, domain::ErrorCategory::Motion);
}

TEST(RmErrorMapTest, MessageNonEmpty) {
    auto e = realman::rm_to_error(-2, domain::DeviceType::RobotArm,
                                  "RealManAdapter", "rm_movej");
    EXPECT_FALSE(e.message.empty());
    EXPECT_NE(e.message.find("rm_movej"), std::string::npos);
}
