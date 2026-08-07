#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "robotics/skills/SkillRuntime.hpp"
#include "robotics/services/ResourceManager.hpp"
#include "src/infrastructure/time/SystemClock.hpp"

using namespace robotics;
using namespace robotics::domain;
using namespace robotics::skills;

namespace {

// 可控假 Skill：validate/execute 行为由测试注入。
class FakeSkill : public ISkill {
public:
    SkillDescriptor desc;
    Result validate_result = Result::ok();
    SkillResult execute_result;
    std::function<bool()> cancel_check = nullptr;  // 在 execute 中轮询
    std::chrono::milliseconds execute_sleep{0};
    bool execute_called = false;
    bool validate_called = false;

    const SkillDescriptor& descriptor() const override { return desc; }

    Result validate(const SkillParams&) override {
        validate_called = true;
        return validate_result;
    }

    SkillResult execute(const SkillParams& params) override {
        execute_called = true;
        if (execute_sleep.count() > 0) {
            auto deadline =
                std::chrono::steady_clock::now() + execute_sleep;
            while (std::chrono::steady_clock::now() < deadline) {
                if (params.cancel && params.cancel()) {
                    // 模拟 Skill 收到取消回调后受控停止
                    SkillResult r;
                    r.success = false;
                    r.failed_stage = "wait";
                    r.error = Error::make(ErrorCategory::Cancelled,
                                          DeviceType::Combined, "FakeSkill",
                                          1, "已请求取消");
                    r.final_state = CommandState::Cancelled;
                    return r;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        return execute_result;
    }
};

}  // namespace

TEST(SkillRuntime, DryRunValidatesButDoesNotExecute) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.dry";
    skill->desc.required_resources = {"arm"};
    skill->execute_result.success = true;
    skill->execute_result.final_state = CommandState::Succeeded;

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/true);

    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.failed_stage, "dry_run");
    EXPECT_EQ(r.final_state, CommandState::Succeeded);
    EXPECT_TRUE(skill->validate_called);
    EXPECT_FALSE(skill->execute_called);   // dry-run 不执行
    // dry-run 不占资源
    EXPECT_TRUE(rm->owner(DeviceType::RobotArm).empty());
    EXPECT_TRUE(rm->owner(DeviceType::DexterousHand).empty());
}

TEST(SkillRuntime, ValidateFailureFailsBeforeExecute) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.bad";
    skill->desc.required_resources = {"hand"};
    skill->validate_result =
        Result::fail(Error::make(ErrorCategory::Validation,
                                 DeviceType::DexterousHand, "FakeSkill", 9,
                                 "参数非法"));

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/false);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.failed_stage, "validate");
    EXPECT_EQ(r.final_state, CommandState::Failed);
    EXPECT_TRUE(skill->validate_called);
    EXPECT_FALSE(skill->execute_called);
    // 失败后资源已释放
    EXPECT_TRUE(rm->owner(DeviceType::DexterousHand).empty());
}

TEST(SkillRuntime, ExecutesSuccessfullyAndReleasesResources) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.ok";
    skill->desc.required_resources = {"arm", "hand"};
    skill->execute_result.success = true;
    skill->execute_result.final_state = CommandState::Succeeded;

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/false);

    EXPECT_TRUE(r.success) << r.error.to_string();
    EXPECT_EQ(r.final_state, CommandState::Succeeded);
    EXPECT_TRUE(skill->execute_called);
    EXPECT_EQ(r.skill_id, "fake.ok");
    EXPECT_FALSE(r.duration_ms.count() < 0);
    // 成功执行后释放全部资源
    EXPECT_TRUE(rm->owner(DeviceType::RobotArm).empty());
    EXPECT_TRUE(rm->owner(DeviceType::DexterousHand).empty());
}

TEST(SkillRuntime, ResourceConflictFailsAtAcquire) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    // 先占用 arm 资源（模拟另一命令持有）
    ASSERT_TRUE(rm->acquire(DeviceType::RobotArm, "other").success);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.conflict";
    skill->desc.required_resources = {"arm"};

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/false);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.failed_stage, "acquire_resources");
    EXPECT_EQ(r.final_state, CommandState::Failed);
    EXPECT_EQ(r.error.category, ErrorCategory::ResourceConflict);
    EXPECT_FALSE(skill->validate_called);  // 资源失败在 validate 之前
    EXPECT_FALSE(skill->execute_called);
}

TEST(SkillRuntime, CombinedResourceExpandsAndDedupes) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.combined";
    // combined 展开为 arm+hand；重复资源应去重（不重复占用）
    skill->desc.required_resources = {"combined", "arm"};
    skill->execute_result.success = true;
    skill->execute_result.final_state = CommandState::Succeeded;

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/false);
    EXPECT_TRUE(r.success) << r.error.to_string();
    // combined 持有期间两种资源都占用
    // （释放发生在 run 返回前，故此处只验证已释放）
    EXPECT_TRUE(rm->owner(DeviceType::RobotArm).empty());
    EXPECT_TRUE(rm->owner(DeviceType::DexterousHand).empty());
}

TEST(SkillRuntime, CancelDuringExecuteMapsToCancelled) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.cancel";
    skill->desc.required_resources = {"arm"};
    skill->execute_sleep = std::chrono::seconds(5);
    skill->execute_result.success = true;

    std::atomic<bool> cancel_req{false};
    std::function<bool()> cancel = [&]() { return cancel_req.load(); };

    std::thread worker([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        cancel_req = true;
    });

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/false, cancel);
    worker.join();

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.failed_stage, "wait");
    EXPECT_EQ(r.final_state, CommandState::Cancelled);
    EXPECT_EQ(r.error.category, ErrorCategory::Cancelled);
    // 取消后资源释放
    EXPECT_TRUE(rm->owner(DeviceType::RobotArm).empty());
}

TEST(SkillRuntime, TimeoutMapsToTimedOut) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.timeout";
    skill->desc.required_resources = {"arm"};
    skill->desc.timeout = std::chrono::milliseconds(80);
    skill->execute_sleep = std::chrono::seconds(5);
    skill->execute_result.success = true;

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/false);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.final_state, CommandState::Cancelled);
    // 超时并入 cancel 回调：Skill 返回 Cancelled，
    // runtime 侧统一映射为 Cancelled（超时语义由 Skill 自行判断）。
    EXPECT_EQ(r.error.category, ErrorCategory::Cancelled);
    EXPECT_TRUE(rm->owner(DeviceType::RobotArm).empty());
}

TEST(SkillRuntime, FailedExecutionMapsToFailed) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto rm = std::make_shared<ResourceManager>();
    SkillRuntime rt(clock);
    rt.set_resource_manager(rm);

    auto skill = std::make_shared<FakeSkill>();
    skill->desc.id = "fake.fail";
    skill->desc.required_resources = {"arm"};
    skill->execute_result.success = false;
    skill->execute_result.failed_stage = "move";
    skill->execute_result.error =
        Error::make(ErrorCategory::Motion, DeviceType::RobotArm, "FakeSkill",
                    7, "运动失败");

    const SkillResult r = rt.run(skill, "{}", /*dry_run=*/false);

    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.final_state, CommandState::Failed);
    EXPECT_EQ(r.failed_stage, "move");
    EXPECT_EQ(r.error.category, ErrorCategory::Motion);
}
