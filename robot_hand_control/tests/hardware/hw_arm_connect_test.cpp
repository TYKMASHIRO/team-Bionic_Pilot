// 硬件测试：RM75 无运动连接测试（阶段2 后实现）。
// 默认不加入 ctest；通过 -DENABLE_HARDWARE_TESTS=ON 构建后显式运行。
// 本文件当前为占位，保证硬件测试目录可编译。
#include <gtest/gtest.h>

TEST(HardwareArmConnect, Placeholder) {
    // 阶段2: 使用 RealManAdapter 连接 192.168.1.18:8080，只读版本，不运动
    GTEST_SKIP() << "阶段2 实现：RM75 无运动硬件连接测试";
}
