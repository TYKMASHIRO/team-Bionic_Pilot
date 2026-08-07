#pragma once

#include "robotics/domain/types/JointVector.hpp"

namespace robotics::domain {

/**
 * @brief 六维力/力矩读数。
 * force 单位为 N；torque 单位为 N·m。数组顺序 [X, Y, Z]。
 */
struct ForceTorque {
    std::array<double, 3> force{};    ///< Fx, Fy, Fz (N)
    std::array<double, 3> torque{};   ///< Mx, My, Mz (N·m)

    ForceTorqueVector to_vector() const {
        return {force[0], force[1], force[2], torque[0], torque[1], torque[2]};
    }

    static ForceTorque from_vector(const ForceTorqueVector& v) {
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5]}};
    }
};

}  // namespace robotics::domain
