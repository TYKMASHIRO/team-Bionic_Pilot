#include "robotics/domain/results/SkillResult.hpp"

#include <sstream>

namespace robotics::domain {

std::string SkillResult::to_string() const {
    std::ostringstream oss;
    oss << "skill=" << skill_id
        << " ver=" << skill_version
        << " success=" << (success ? "true" : "false")
        << " state=" << robotics::domain::to_string(final_state)
        << " duration_ms=" << duration_ms.count();
    if (!failed_stage.empty()) {
        oss << " failed_stage=" << failed_stage;
    }
    if (!error.is_ok()) {
        oss << " err=" << error.to_string();
    }
    if (safety_stopped) {
        oss << " SAFETY_STOPPED";
    }
    if (!final_device_summary.empty()) {
        oss << " summary=" << final_device_summary;
    }
    return oss.str();
}

}  // namespace robotics::domain
