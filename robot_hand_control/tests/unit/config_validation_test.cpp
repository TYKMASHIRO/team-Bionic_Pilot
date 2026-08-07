#include <cstdio>

#include <gtest/gtest.h>

#include "robotics/infrastructure/config/Configuration.hpp"

using namespace robotics;

// 验收：非法配置测试
TEST(ConfigValidationTest, DefaultConfigValid) {
    infra::Configuration config;
    auto r = config.load("", "", "");  // 默认值
    ASSERT_TRUE(r.success) << r.error.to_string();

    auto v = config.validate();
    EXPECT_TRUE(v.success) << v.error.to_string();
}

TEST(ConfigValidationTest, InvalidPortRejected) {
    infra::Configuration config;
    config.robot_mut().port = 0;
    auto v = config.validate();
    EXPECT_FALSE(v.success);
    EXPECT_EQ(v.error.category, domain::ErrorCategory::Configuration);
}

TEST(ConfigValidationTest, InvalidBaudRateRejected) {
    infra::Configuration config;
    config.hand_mut().baudrate = 9600;  // O6 固定 115200
    auto v = config.validate();
    EXPECT_FALSE(v.success);
    EXPECT_EQ(v.error.category, domain::ErrorCategory::Configuration);
}

TEST(ConfigValidationTest, InvalidSampleRateRejected) {
    infra::Configuration config;
    config.hand_mut().sample_rate_hz = 0;
    auto v = config.validate();
    EXPECT_FALSE(v.success);
}

TEST(ConfigValidationTest, ConfigHashStable) {
    infra::Configuration config;
    config.load("", "", "");
    const auto h1 = config.config_hash();
    const auto h2 = config.config_hash();
    EXPECT_EQ(h1, h2);
    EXPECT_FALSE(h1.empty());
}

TEST(ConfigValidationTest, InvalidYamlRejected) {
    // 写入临时非法 YAML
    const char* bad = "/tmp/bad_config_robot.yaml";
    FILE* f = fopen(bad, "w");
    ASSERT_TRUE(f);
    fputs("arm:\n  ip: [broken", f);  // 非法 YAML
    fclose(f);

    infra::Configuration config;
    auto r = config.load(bad, "", "");
    EXPECT_FALSE(r.success);

    remove(bad);
}
