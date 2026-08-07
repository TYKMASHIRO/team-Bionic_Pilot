#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "robotics/skills/SkillManifest.hpp"

using namespace robotics::domain;
using robotics::skills::load_skill_manifest;

namespace {

// 把内容写入临时 YAML 文件，返回路径。
std::string write_yaml(const std::string& name, const std::string& body) {
    const std::string path = ::testing::TempDir() + "/" + name;
    std::ofstream ofs(path, std::ios::trunc);
    ofs << body;
    return path;
}

const std::string kGood = R"(
id: hand.open
name: "张开手掌"
version: "1.0.0"
description: "测试用"
required_resources: ["hand"]
parameters:
  - name: "preset"
    type: "preset"
    required: false
    default: "open"
preconditions:
  - "手已连接"
execution_stages:
  - "apply_preset"
success_conditions:
  - "张开成功"
failure_conditions:
  - "未连接"
timeout_ms: 5000
cancellation_policy: "controlled_stop"
recovery_policy: "stop_and_report"
safety_profile:
  real_motion: true
trajectory_references: ["none"]
calibration_references: ["c0"]
safe_pose: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
safe_pose_speed: 0.35
preset: "open"
)";

}  // namespace

TEST(SkillManifest, LoadsAllFields) {
    SkillDescriptor d;
    std::string err;
    ASSERT_TRUE(load_skill_manifest(write_yaml("good.yaml", kGood), d, err))
        << err;

    EXPECT_EQ(d.id, "hand.open");
    EXPECT_EQ(d.name, "张开手掌");
    EXPECT_EQ(d.version, "1.0.0");
    EXPECT_EQ(d.description, "测试用");
    ASSERT_EQ(d.required_resources.size(), 1u);
    EXPECT_EQ(d.required_resources[0], "hand");

    // 参数表
    ASSERT_EQ(d.parameters.size(), 1u);
    EXPECT_EQ(d.parameters[0].name, "preset");
    EXPECT_EQ(d.parameters[0].type, "preset");
    EXPECT_FALSE(d.parameters[0].required);
    EXPECT_EQ(d.parameters[0].default_value, "open");

    // 条件/阶段
    ASSERT_EQ(d.preconditions.size(), 1u);
    EXPECT_EQ(d.preconditions[0], "手已连接");
    ASSERT_EQ(d.execution_stages.size(), 1u);
    EXPECT_EQ(d.execution_stages[0], "apply_preset");
    ASSERT_EQ(d.success_conditions.size(), 1u);
    ASSERT_EQ(d.failure_conditions.size(), 1u);

    // 超时/取消/恢复/安全
    EXPECT_EQ(d.timeout, std::chrono::milliseconds(5000));
    EXPECT_EQ(d.cancellation_policy, "controlled_stop");
    EXPECT_EQ(d.recovery_policy, "stop_and_report");
    EXPECT_TRUE(d.real_motion);
    ASSERT_EQ(d.trajectory_references.size(), 1u);
    ASSERT_EQ(d.calibration_references.size(), 1u);
    EXPECT_EQ(d.calibration_references[0], "c0");

    // 技能专属数据
    ASSERT_EQ(d.safe_pose.size(), 7u);
    EXPECT_DOUBLE_EQ(d.safe_pose[0], 0.1);
    EXPECT_DOUBLE_EQ(d.safe_pose_speed, 0.35);
    EXPECT_EQ(d.preset, "open");

    EXPECT_EQ(d.manifest_path, ::testing::TempDir() + "/good.yaml");
}

TEST(SkillManifest, MissingIdFails) {
    const std::string yaml = R"(
version: "1.0.0"
required_resources: ["hand"]
)";
    SkillDescriptor d;
    std::string err;
    EXPECT_FALSE(load_skill_manifest(write_yaml("no_id.yaml", yaml), d, err));
    EXPECT_NE(err.find("id"), std::string::npos) << err;
}

TEST(SkillManifest, MissingVersionFails) {
    const std::string yaml = R"(
id: hand.open
required_resources: ["hand"]
)";
    SkillDescriptor d;
    std::string err;
    EXPECT_FALSE(
        load_skill_manifest(write_yaml("no_ver.yaml", yaml), d, err));
    EXPECT_NE(err.find("version"), std::string::npos) << err;
}

TEST(SkillManifest, EmptyRequiredResourcesFails) {
    const std::string yaml = R"(
id: hand.open
version: "1.0.0"
required_resources: []
)";
    SkillDescriptor d;
    std::string err;
    EXPECT_FALSE(
        load_skill_manifest(write_yaml("no_res.yaml", yaml), d, err));
    EXPECT_NE(err.find("required_resources"), std::string::npos) << err;
}

TEST(SkillManifest, MalformedYamlFails) {
    const std::string yaml = "id: [unclosed\n  : : :";
    SkillDescriptor d;
    std::string err;
    EXPECT_FALSE(
        load_skill_manifest(write_yaml("bad.yaml", yaml), d, err));
}

TEST(SkillManifest, MissingOptionalFieldsUseDefaults) {
    const std::string yaml = R"(
id: arm.move_to_safe_pose
version: "1.0.0"
required_resources: ["arm"]
)";
    SkillDescriptor d;
    std::string err;
    ASSERT_TRUE(load_skill_manifest(write_yaml("min.yaml", yaml), d, err))
        << err;

    EXPECT_EQ(d.name, d.id);  // name 缺省 = id
    EXPECT_FALSE(d.real_motion);
    EXPECT_EQ(d.timeout, std::chrono::milliseconds(0));
    EXPECT_TRUE(d.safe_pose.empty());
    EXPECT_DOUBLE_EQ(d.safe_pose_speed, 0.2);
    EXPECT_TRUE(d.preset.empty());
}

TEST(SkillManifest, NonexistentFileFails) {
    SkillDescriptor d;
    std::string err;
    EXPECT_FALSE(load_skill_manifest("/nonexistent/xyz.yaml", d, err));
}
