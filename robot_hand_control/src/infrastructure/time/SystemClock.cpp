#include "src/infrastructure/time/SystemClock.hpp"

#include <ctime>
#include <iomanip>
#include <sstream>

namespace robotics::infra {

std::chrono::steady_clock::time_point SystemClock::steady_now() {
    return std::chrono::steady_clock::now();
}

std::chrono::system_clock::time_point SystemClock::system_now() {
    return std::chrono::system_clock::now();
}

domain::Timestamp SystemClock::now() {
    domain::Timestamp ts;
    ts.steady = steady_now();
    ts.wall = system_now();
    return ts;
}

}  // namespace robotics::infra

namespace robotics::domain {

Timestamp make_timestamp() {
    Timestamp ts;
    ts.steady = std::chrono::steady_clock::now();
    ts.wall = std::chrono::system_clock::now();
    return ts;
}

std::string Timestamp::to_iso8601() const {
    const std::time_t t = std::chrono::system_clock::to_time_t(wall);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
}

}  // namespace robotics::domain
