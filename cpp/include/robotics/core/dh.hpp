#pragma once

#include <cmath>

#include "robotics/core/types.hpp"

namespace robotics {

/**
 * @brief One row of a modified (Craig) Denavit-Hartenberg table.
 *
 * Some vendors, Franka among them, publish their kinematics this way rather than
 * as screw axes. Walking the table once at zero configuration yields the screw
 * axes directly, so the published numbers are the only ones written down.
 */
struct ModifiedDhRow {
    Scalar a{};      ///< `a_{i-1}`, link length [m].
    Scalar d{};      ///< `d_i`, link offset [m].
    Scalar alpha{};  ///< `alpha_{i-1}`, link twist [rad].
};

/**
 * @brief Link transform for a modified-DH row.
 *
 * `T = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)`
 *
 * @param row The DH row for this link.
 * @param theta Joint angle [rad].
 * @return The transform from frame `i-1` to frame `i`.
 */
[[nodiscard]] inline Pose modified_dh_transform(const ModifiedDhRow& row, Scalar theta) {
    const Scalar ca = std::cos(row.alpha);
    const Scalar sa = std::sin(row.alpha);
    const Scalar ct = std::cos(theta);
    const Scalar st = std::sin(theta);

    Matrix3 rotation;
    // clang-format off
    rotation << ct,      -st,      Scalar{0},
                st * ca,  ct * ca, -sa,
                st * sa,  ct * sa,  ca;
    // clang-format on

    const Vector3 translation{row.a, -row.d * sa, row.d * ca};
    return Pose{Sophus::SO3<Scalar>::fitToSO3(rotation), translation};
}

}  // namespace robotics
