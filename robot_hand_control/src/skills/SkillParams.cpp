#include "robotics/skills/SkillParams.hpp"

#include <yaml-cpp/yaml.h>

#include <cctype>

namespace robotics::skills {

namespace {
// parameters_json 为空串或非法时返回空 Node（参数一律走默认值）
YAML::Node try_load(const std::string& json) {
    if (json.empty()) return YAML::Node();
    try {
        YAML::Node n = YAML::Load(json);
        return n;
    } catch (const YAML::Exception&) {
        return YAML::Node();
    }
}
}  // namespace

bool param_bool(const std::string& json, const std::string& key, bool dflt) {
    const YAML::Node n = try_load(json);
    if (n && n[key]) return n[key].as<bool>(dflt);
    return dflt;
}

double param_double(const std::string& json, const std::string& key,
                    double dflt) {
    const YAML::Node n = try_load(json);
    if (n && n[key]) return n[key].as<double>(dflt);
    return dflt;
}

long long param_int(const std::string& json, const std::string& key,
                    long long dflt) {
    const YAML::Node n = try_load(json);
    if (n && n[key]) return n[key].as<long long>(dflt);
    return dflt;
}

std::string param_string(const std::string& json, const std::string& key,
                         const std::string& dflt) {
    const YAML::Node n = try_load(json);
    if (n && n[key]) return n[key].as<std::string>(dflt);
    return dflt;
}

std::vector<double> param_double_list(const std::string& json,
                                      const std::string& key) {
    const YAML::Node n = try_load(json);
    std::vector<double> out;
    if (n && n[key] && n[key].IsSequence()) {
        for (const auto& v : n[key]) out.push_back(v.as<double>());
    }
    return out;
}

bool parse_hand_preset(const std::string& preset, domain::HandPreset& out,
                       std::string& error) {
    if (preset == "open") {
        out = domain::HandPreset::Open;
        return true;
    }
    if (preset == "close") {
        out = domain::HandPreset::Close;
        return true;
    }
    if (preset == "pregrasp") {
        out = domain::HandPreset::PreGrasp;
        return true;
    }
    if (preset == "custom") {
        out = domain::HandPreset::Custom;
        return true;
    }
    error = "非法 hand 预设名: '" + preset +
            "'（可用 open/close/pregrasp/custom）";
    return false;
}

}  // namespace robotics::skills
