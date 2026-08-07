#include "robotics/domain/errors/Error.hpp"

#include <sstream>

namespace robotics::domain {

std::string to_string(ErrorCategory cat) {
    switch (cat) {
        case ErrorCategory::None: return "none";
        case ErrorCategory::Configuration: return "configuration";
        case ErrorCategory::Communication: return "communication";
        case ErrorCategory::Protocol: return "protocol";
        case ErrorCategory::Device: return "device";
        case ErrorCategory::Motion: return "motion";
        case ErrorCategory::Safety: return "safety";
        case ErrorCategory::Cancelled: return "cancelled";
        case ErrorCategory::Timeout: return "timeout";
        case ErrorCategory::Unsupported: return "unsupported";
        case ErrorCategory::Internal: return "internal";
        case ErrorCategory::NotConnected: return "not_connected";
        case ErrorCategory::StateStale: return "state_stale";
        case ErrorCategory::ResourceConflict: return "resource_conflict";
        case ErrorCategory::Validation: return "validation";
    }
    return "unknown";
}

std::string to_string(Severity sev) {
    switch (sev) {
        case Severity::Info: return "info";
        case Severity::Warning: return "warning";
        case Severity::Error: return "error";
        case Severity::Critical: return "critical";
    }
    return "unknown";
}

std::string Error::to_string() const {
    std::ostringstream oss;
    oss << "[cat=" << ::robotics::domain::to_string(category)
        << " dev=" << ::robotics::domain::to_string(device)
        << " mod=" << module << " code=" << code
        << " sev=" << ::robotics::domain::to_string(severity)
        << " retry=" << (retryable ? "y" : "n");
    if (raw_vendor_code.has_value()) {
        oss << " vendor=" << raw_vendor_code.value();
    }
    oss << "] " << message;
    return oss.str();
}

}  // namespace robotics::domain
