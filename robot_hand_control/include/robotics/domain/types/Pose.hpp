#pragma once

#include <array>
#include <cmath>
#include <string>

namespace robotics::domain {

/**
 * @brief 三维位置（单位：m）
 */
struct Position3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief 姿态（单位：rad），采用欧拉角 RPY（rx/ry/rz）
 */
struct Orientation {
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

/**
 * @brief 位姿：位置(m) + 姿态(rad)
 */
struct Pose {
    Position3 position;
    Orientation orientation;

    double euclidean_distance_to(const Pose& other) const {
        const double dx = position.x - other.position.x;
        const double dy = position.y - other.position.y;
        const double dz = position.z - other.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool is_finite() const {
        return std::isfinite(position.x) && std::isfinite(position.y) &&
               std::isfinite(position.z) && std::isfinite(orientation.rx) &&
               std::isfinite(orientation.ry) && std::isfinite(orientation.rz);
    }
};

}  // namespace robotics::domain
