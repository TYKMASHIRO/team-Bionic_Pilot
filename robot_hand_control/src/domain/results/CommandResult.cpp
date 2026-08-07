#include "robotics/domain/results/CommandResult.hpp"

#include <sstream>

namespace robotics::domain {

std::string CommandResult::to_string() const {
    std::ostringstream oss;
    oss << "cmd=" << command_id
        << " type=" << robotics::domain::to_string(type)
        << " success=" << (success ? "true" : "false")
        << " state=" << robotics::domain::to_string(final_state)
        << " duration_ms=" << duration_ms.count();
    if (!error.is_ok()) {
        oss << " err=" << error.to_string();
    }
    if (!summary.empty()) {
        oss << " summary=" << summary;
    }
    return oss.str();
}

}  // namespace robotics::domain
