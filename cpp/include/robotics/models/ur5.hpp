#pragma once

#include "robotics/kinematics/serial_chain.hpp"

namespace robotics::models {

inline constexpr int kUr5Dof = 6;

using Ur5Chain = SerialChain<kUr5Dof>;

/**
 * @brief UR5 link dimensions [m], named as in Modern Robotics chapter 4.
 *
 * The defaults are the manufacturer's values from the official `ur_description`
 * kinematics. Modern Robotics prints them rounded to three decimals (0.109,
 * 0.082, 0.392, 0.089, 0.095); the full-precision values keep the rendered
 * vendor meshes aligned with the computed frames to within a micrometre, and
 * still reproduce the book's tabulated poses to the three decimals it prints.
 */
struct Ur5Dimensions {
    Scalar shoulder_offset{Scalar{0.10915}};  ///< `W1`, shoulder-to-wrist offset along y.
    Scalar wrist_offset{Scalar{0.0823}};      ///< `W2`, wrist-to-flange offset along y.
    Scalar upper_arm{Scalar{0.425}};          ///< `L1`.
    Scalar forearm{Scalar{0.39225}};          ///< `L2`.
    Scalar base_height{Scalar{0.089159}};     ///< `H1`.
    Scalar wrist_drop{Scalar{0.09465}};       ///< `H2`.
};

/**
 * @brief Builds the UR5 chain.
 *
 * Each joint is described by its rotation axis and a point on that axis, which
 * doubles as the origin of the frame used to draw the link. The screw axes this
 * produces are the ones tabulated in Modern Robotics.
 *
 * @param dims Link dimensions to use.
 */
[[nodiscard]] inline Ur5Chain ur5(const Ur5Dimensions& dims = {}) {
    const Scalar w1 = dims.shoulder_offset;
    const Scalar w2 = dims.wrist_offset;
    const Scalar l1 = dims.upper_arm;
    const Scalar l2 = dims.forearm;
    const Scalar h1 = dims.base_height;
    const Scalar h2 = dims.wrist_drop;

    const Vector3 z{0, 0, 1};
    const Vector3 y{0, 1, 0};

    // Rotation axis and a point on it, per joint, at zero configuration.
    const std::array<std::pair<Vector3, Vector3>, kUr5Dof> axes{{
        {z, Vector3{0, 0, 0}},
        {y, Vector3{0, 0, h1}},
        {y, Vector3{l1, 0, h1}},
        {y, Vector3{l1 + l2, 0, h1}},
        {-z, Vector3{l1 + l2, w1, h1}},
        {y, Vector3{l1 + l2, w1, h1 - h2}},
    }};

    constexpr std::array<const char*, kUr5Dof> names{"shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"};
    constexpr JointLimit kFullTurn{-2 * kPi, 2 * kPi};

    Ur5Chain::Specs specs{};
    for (std::size_t i = 0; i < specs.size(); ++i) {
        const auto& [axis, point] = axes[i];
        specs[i] = JointSpec<kUr5Dof>{.screw = ScrewAxis::revolute(axis, point),
                                      .link_home = Pose{Sophus::SO3<Scalar>{}, point},
                                      .limit = kFullTurn,
                                      .name = names[i]};
    }

    // Flange pose at zero configuration.
    Matrix3 flange_rotation;
    // clang-format off
    flange_rotation << -1, 0, 0,
                        0, 0, 1,
                        0, 1, 0;
    // clang-format on
    const Pose flange{Sophus::SO3<Scalar>::fitToSO3(flange_rotation), Vector3{l1 + l2, w1 + w2, h1 - h2}};

    return Ur5Chain{specs, flange};
}

}  // namespace robotics::models
