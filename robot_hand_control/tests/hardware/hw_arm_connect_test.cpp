// 硬件测试：RM75 无运动连接测试。
// 仅当 ENABLE_HARDWARE_TESTS=ON 时构建；默认 DISABLED。
// 运行需真实 RM75 机械臂（默认 192.168.1.18:8080）。
#include <gtest/gtest.h>

#include "drivers/realman/include/RealManAdapter.hpp"

using namespace robotics;

namespace {
const char* kArmIp = "192.168.1.18";
const int kArmPort = 8080;
}  // namespace

// 连接 + 版本查询 + 状态读取（不运动）
TEST(HardwareArmConnect, ConnectAndReadState) {
    realman::RealManAdapter arm(kArmIp, kArmPort);

    auto r = arm.connect();
    ASSERT_TRUE(r.success) << r.error.to_string();
    ASSERT_TRUE(arm.is_connected());

    auto st = arm.get_state();
    EXPECT_TRUE(st.valid) << "机械臂状态无效";
    EXPECT_TRUE(st.fresh);

    auto h = arm.health_check();
    EXPECT_TRUE(h.online);
    EXPECT_EQ(h.level, domain::HealthLevel::Ok);

    arm.disconnect();
    EXPECT_FALSE(arm.is_connected());
}

// SDK 版本读取
TEST(HardwareArmConnect, SdkVersion) {
    realman::RealManAdapter arm(kArmIp, kArmPort);
    auto r = arm.connect();
    ASSERT_TRUE(r.success) << r.error.to_string();
    auto h = arm.health_check();
    EXPECT_EQ(h.sdk_version, "1.1.6");
    arm.disconnect();
}

// 未连接时运动应被拒绝（不运动）
TEST(HardwareArmConnect, MotionRejectedWhenDisconnected) {
    realman::RealManAdapter arm(kArmIp, kArmPort);
    domain::ArmJointVector j{};
    auto r = arm.move_joint(j, 0.1, true);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error.category, domain::ErrorCategory::NotConnected);
}
