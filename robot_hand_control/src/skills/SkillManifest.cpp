#include "robotics/skills/SkillManifest.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

namespace robotics::skills {

namespace {

// 读取必填字符串字段；缺失返回 false
bool required_str(const YAML::Node& n, const char* key, std::string& out,
                  std::string& error, const std::string& path) {
    if (!n[key] || !n[key].IsScalar()) {
        error = path + ": 缺少必填字段 " + std::string(key);
        return false;
    }
    out = n[key].as<std::string>();
    return true;
}

std::vector<std::string> read_str_list(const YAML::Node& n, const char* key) {
    std::vector<std::string> out;
    if (n[key] && n[key].IsSequence()) {
        for (const auto& item : n[key]) {
            out.push_back(item.as<std::string>());
        }
    }
    return out;
}

std::vector<double> read_double_list(const YAML::Node& n, const char* key) {
    std::vector<double> out;
    if (n[key] && n[key].IsSequence()) {
        for (const auto& item : n[key]) {
            out.push_back(item.as<double>());
        }
    }
    return out;
}

}  // namespace

bool load_skill_manifest(const std::string& yaml_path,
                         robotics::domain::SkillDescriptor& out,
                         std::string& error) {
    YAML::Node n;
    try {
        n = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        error = yaml_path + ": YAML 解析失败: " + e.what();
        return false;
    }
    if (!n || !n.IsMap()) {
        error = yaml_path + ": 不是有效的 YAML 映射";
        return false;
    }

    const std::string& path = yaml_path;
    if (!required_str(n, "id", out.id, error, path)) return false;
    if (!required_str(n, "version", out.version, error, path)) return false;

    out.name = n["name"] ? n["name"].as<std::string>() : out.id;
    out.description =
        n["description"] ? n["description"].as<std::string>() : std::string{};

    out.required_resources = read_str_list(n, "required_resources");
    if (out.required_resources.empty()) {
        error = path + ": required_resources 不能为空（至少一个设备资源）";
        return false;
    }

    if (n["parameters"] && n["parameters"].IsSequence()) {
        for (const auto& p : n["parameters"]) {
            robotics::domain::SkillParameter sp;
            if (p["name"]) sp.name = p["name"].as<std::string>();
            if (p["type"]) sp.type = p["type"].as<std::string>();
            if (p["required"]) sp.required = p["required"].as<bool>();
            if (p["default"]) sp.default_value = p["default"].as<std::string>();
            if (p["description"]) sp.description = p["description"].as<std::string>();
            if (!sp.name.empty()) out.parameters.push_back(std::move(sp));
        }
    }

    out.preconditions = read_str_list(n, "preconditions");
    out.execution_stages = read_str_list(n, "execution_stages");
    out.success_conditions = read_str_list(n, "success_conditions");
    out.failure_conditions = read_str_list(n, "failure_conditions");
    out.trajectory_references = read_str_list(n, "trajectory_references");
    out.calibration_references = read_str_list(n, "calibration_references");

    if (n["timeout_ms"]) {
        out.timeout = std::chrono::milliseconds(n["timeout_ms"].as<long long>());
    }
    if (n["cancellation_policy"]) {
        out.cancellation_policy = n["cancellation_policy"].as<std::string>();
    }
    if (n["recovery_policy"]) {
        out.recovery_policy = n["recovery_policy"].as<std::string>();
    }
    if (n["safety_profile"] && n["safety_profile"]["real_motion"]) {
        out.real_motion = n["safety_profile"]["real_motion"].as<bool>();
    }

    // ---- 技能专属数据 ----
    out.safe_pose = read_double_list(n, "safe_pose");
    if (n["safe_pose_speed"]) {
        out.safe_pose_speed = n["safe_pose_speed"].as<double>();
    }
    if (n["preset"]) out.preset = n["preset"].as<std::string>();

    out.manifest_path = yaml_path;
    return true;
}

}  // namespace robotics::skills
