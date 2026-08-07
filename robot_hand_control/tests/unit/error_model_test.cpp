#include <gtest/gtest.h>

#include "robotics/domain/commands/Command.hpp"
#include "robotics/domain/errors/Error.hpp"

using namespace robotics::domain;

// 验收：错误码转换/模型测试
TEST(ErrorModelTest, ErrorOk) {
    Error e;
    EXPECT_TRUE(e.is_ok());
    EXPECT_FALSE(e.is_error());
    EXPECT_EQ(e.category, ErrorCategory::None);
}

TEST(ErrorModelTest, ErrorMakeCarriesFields) {
    auto e = Error::make(ErrorCategory::Communication, DeviceType::RobotArm,
                         "RealManAdapter", 42, "连接失败", Severity::Critical,
                         true);
    e.raw_vendor_code = -2;

    EXPECT_EQ(e.category, ErrorCategory::Communication);
    EXPECT_EQ(e.device, DeviceType::RobotArm);
    EXPECT_EQ(e.module, "RealManAdapter");
    EXPECT_EQ(e.code, 42);
    EXPECT_EQ(e.severity, Severity::Critical);
    EXPECT_TRUE(e.retryable);
    ASSERT_TRUE(e.raw_vendor_code.has_value());
    EXPECT_EQ(e.raw_vendor_code.value(), -2);
    EXPECT_FALSE(e.to_string().empty());
}

TEST(ErrorModelTest, ResultOkFail) {
    auto ok = Result::ok();
    EXPECT_TRUE(ok.success);
    EXPECT_TRUE(ok.error.is_ok());

    auto fail = Result::fail(Error::make(ErrorCategory::Timeout, DeviceType::Unknown,
                                         "Test", 1, "超时"));
    EXPECT_FALSE(fail.success);
    EXPECT_EQ(fail.error.category, ErrorCategory::Timeout);
}

TEST(CommandTest, CommandIdUnique) {
    const auto id1 = make_command_id();
    const auto id2 = make_command_id();
    EXPECT_NE(id1, id2);
    EXPECT_FALSE(id1.empty());
}

TEST(CommandTest, CommandStateToString) {
    EXPECT_EQ(to_string(CommandState::Created), "created");
    EXPECT_EQ(to_string(CommandState::SafetyStopped), "safety_stopped");
    EXPECT_EQ(to_string(CommandState::Cancelling), "cancelling");
}
