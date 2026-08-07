#include <gtest/gtest.h>

#include <functional>
#include <memory>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/safety/SafetySupervisor.hpp"
#include "robotics/skills/SkillContext.hpp"
#include "robotics/skills/SkillFactory.hpp"
#include "src/infrastructure/time/SystemClock.hpp"

using namespace robotics;
using namespace robotics::domain;
using robotics::skills::SkillContext;
using robotics::skills::make_skill;

namespace {

SkillContext make_ctx(
    const std::shared_ptr<mock::MockRobotArm>& arm,
    const std::shared_ptr<mock::MockDexterousHand>& hand,
    const std::shared_ptr<domain::SafetySupervisor>& safety) {
    SkillContext ctx;
    ctx.arm = arm;
    ctx.hand = hand;
    ctx.safety = safety;
    ctx.clock = std::make_shared<infra::SystemClock>();
    return ctx;
}

infra::SafetyConfig safety_cfg() {
    infra::SafetyConfig cfg;
    cfg.max_state_delay_ms = 5000.0;  // 宽松：测试环境
    return cfg;
}

std::shared_ptr<ISkill> build(const std::string& id, SkillContext ctx) {
    SkillDescriptor desc;
    desc.id = id;
    desc.version = "1.0.0";
    desc.real_motion = true;  // 真实运动技能
    if (id == "arm.move_to_safe_pose") {
        desc.required_resources = {"arm"};
        desc.safe_pose = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
        desc.safe_pose_speed = 0.3;
    } else if (id == "arm.drag_teach_record") {
        desc.required_resources = {"combined"};
        desc.timeout = std::chrono::milliseconds(2000);
    } else if (id == "hand.open" || id == "hand.close") {
        desc.required_resources = {"hand"};
        desc.preset = (id == "hand.open") ? "open" : "close";
    } else if (id == "hand.apply_preset") {
        desc.required_resources = {"hand"};
    } else if (id == "combined.safe_release") {
        desc.required_resources = {"arm", "hand"};
        desc.safe_pose = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
        desc.safe_pose_speed = 0.3;
    } else if (id == "combined.synchronized_replay") {
        desc.required_resources = {"arm", "hand"};
    }
    std::string err;
    auto s = make_skill(desc, std::move(ctx), err);
    if (!s) {
        ADD_FAILURE() << "make_skill(" << id << ") failed: " << err;
    }
    return s;
}

}  // namespace

TEST(SkillBasic, HandOpenMovesTo255) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("hand.open", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    // 未连接 -> validate 失败
    SkillParams p;
    Result v = skill->validate(p);
    EXPECT_FALSE(v.success);

    ASSERT_TRUE(hand->connect().success);
    safety->set_real_motion_enabled(true);
    v = skill->validate(p);
    ASSERT_TRUE(v.success) << v.error.to_string();

    SkillResult r = skill->execute(p);
    ASSERT_TRUE(r.success) << r.error.to_string();
    const auto st = hand->get_state();
    for (std::size_t j = 0; j < kHandDof; ++j) {
        EXPECT_EQ(st.position[j], 255);
    }
}

TEST(SkillBasic, HandCloseMovesToZero) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("hand.close", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(hand->connect().success);
    safety->set_real_motion_enabled(true);
    ASSERT_TRUE(skill->validate(SkillParams{}).success);

    SkillResult r = skill->execute(SkillParams{});
    ASSERT_TRUE(r.success) << r.error.to_string();
    const auto st = hand->get_state();
    for (std::size_t j = 0; j < kHandDof; ++j) {
        EXPECT_EQ(st.position[j], 0);
    }
}

TEST(SkillBasic, HandApplyPresetParamOverridesManifest) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("hand.apply_preset", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(hand->connect().success);
    safety->set_real_motion_enabled(true);

    // preset=close
    SkillParams p;
    p.parameters_json = R"({"preset":"close"})";
    ASSERT_TRUE(skill->validate(p).success);
    SkillResult r = skill->execute(p);
    ASSERT_TRUE(r.success) << r.error.to_string();
    const auto st = hand->get_state();
    for (std::size_t j = 0; j < kHandDof; ++j) {
        EXPECT_EQ(st.position[j], 0);
    }
}

TEST(SkillBasic, HandApplyPresetRejectsUnknown) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("hand.apply_preset", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(hand->connect().success);
    safety->set_real_motion_enabled(true);

    SkillParams p;
    p.parameters_json = R"({"preset":"bogus"})";
    Result v = skill->validate(p);
    EXPECT_FALSE(v.success);
    EXPECT_EQ(v.error.category, ErrorCategory::Validation);
}

TEST(SkillBasic, ArmMoveToSafePoseUsesManifestPose) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("arm.move_to_safe_pose", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(arm->connect().success);
    safety->set_real_motion_enabled(true);
    ASSERT_TRUE(skill->validate(SkillParams{}).success);

    SkillResult r = skill->execute(SkillParams{});
    ASSERT_TRUE(r.success) << r.error.to_string();
    const auto st = arm->get_state();
    for (std::size_t j = 0; j < kArmDof; ++j) {
        EXPECT_DOUBLE_EQ(st.joint_state.position[j], 0.1 + 0.1 * j);
    }
}

TEST(SkillBasic, ArmMoveToSafePoseParamOverridesManifest) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("arm.move_to_safe_pose", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(arm->connect().success);
    safety->set_real_motion_enabled(true);

    SkillParams p;
    p.parameters_json =
        R"({"joints":[0.9,0.8,0.7,0.6,0.5,0.4,0.3]})";
    ASSERT_TRUE(skill->validate(p).success);
    SkillResult r = skill->execute(p);
    ASSERT_TRUE(r.success) << r.error.to_string();
    const auto st = arm->get_state();
    for (std::size_t j = 0; j < kArmDof; ++j) {
        EXPECT_DOUBLE_EQ(st.joint_state.position[j], 0.9 - 0.1 * j);
    }
}

TEST(SkillBasic, RealMotionWithoutPermissionIsSafetyBlocked) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("hand.open", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(hand->connect().success);
    // 未 enable_real_motion -> execute 在安全门拒绝
    SkillParams p;
    ASSERT_TRUE(skill->validate(p).success);
    const auto before = hand->get_state();
    SkillResult r = skill->execute(p);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.failed_stage, "safety");
    EXPECT_EQ(r.error.category, ErrorCategory::Safety);
    EXPECT_TRUE(r.safety_stopped);
    // 手未被移动（保持执行前状态）
    const auto after = hand->get_state();
    for (std::size_t j = 0; j < kHandDof; ++j) {
        EXPECT_EQ(after.position[j], before.position[j]);
    }
}

TEST(SkillBasic, SafeReleaseOpensHandThenArm) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("combined.safe_release", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);
    safety->set_real_motion_enabled(true);
    ASSERT_TRUE(skill->validate(SkillParams{}).success);

    // 预置：手合拢、臂在别处
    HandJointVector closed;
    ASSERT_TRUE(hand->set_joint_positions(closed).success);

    SkillResult r = skill->execute(SkillParams{});
    ASSERT_TRUE(r.success) << r.error.to_string();

    // 手已张开
    const auto hst = hand->get_state();
    for (std::size_t j = 0; j < kHandDof; ++j) {
        EXPECT_EQ(hst.position[j], 255);
    }
}

TEST(SkillBasic, DragTeachRecordRequiresOutDir) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg());
    auto skill = build("arm.drag_teach_record", make_ctx(arm, hand, safety));
    ASSERT_NE(skill, nullptr);

    ASSERT_TRUE(arm->connect().success);
    safety->set_real_motion_enabled(true);

    // 缺 out_dir -> validate 失败
    SkillParams p;
    Result v = skill->validate(p);
    EXPECT_FALSE(v.success);
    EXPECT_EQ(v.error.category, ErrorCategory::Validation);

    // 有 out_dir + duration_s=0 会阻塞到取消；用 duration_s=0.05 快速结束
    p.parameters_json = R"({"out_dir":"/tmp","duration_s":0.05,"import":false})";
    v = skill->validate(p);
    ASSERT_TRUE(v.success) << v.error.to_string();
    SkillResult r = skill->execute(p);
    // 无 recording hook 时跳过录制，直接示教结束
    ASSERT_TRUE(r.success) << r.error.to_string();
}
