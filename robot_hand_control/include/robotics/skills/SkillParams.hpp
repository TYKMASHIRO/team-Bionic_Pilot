#pragma once

#include <string>
#include <vector>

#include "robotics/domain/types/JointVector.hpp"
#include "robotics/interfaces/IDexterousHand.hpp"

namespace robotics::skills {

/// 从 parameters_json（JSON/YAML 对象）读取参数。缺字段用默认值。
bool param_bool(const std::string& json, const std::string& key, bool dflt);
double param_double(const std::string& json, const std::string& key,
                    double dflt);
long long param_int(const std::string& json, const std::string& key,
                    long long dflt);
std::string param_string(const std::string& json, const std::string& key,
                         const std::string& dflt);
/// 读取数值数组（如 safe_pose / joint 目标）。
std::vector<double> param_double_list(const std::string& json,
                                      const std::string& key);

/// 解析 hand 预设名（open/close/pregrasp/custom）。
/// @return false 且 error 说明非法值时。
bool parse_hand_preset(const std::string& preset,
                       domain::HandPreset& out, std::string& error);

}  // namespace robotics::skills
