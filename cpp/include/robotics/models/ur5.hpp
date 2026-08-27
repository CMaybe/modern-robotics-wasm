#pragma once

#include "robotics/collision/robot_geometry.hpp"
#include "robotics/dynamics/newton_euler.hpp"
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

/// One radius per skeleton span: base->j0 (zero length), base column, upper
/// arm, forearm, wrist 1, wrist 2, wrist 3 + flange. Shared by the collision
/// capsules and the dynamics inertias, so both describe the same body.
inline constexpr std::array<Scalar, kUr5Dof + 1> kUr5CapsuleRadii{
    Scalar{0.075}, Scalar{0.075}, Scalar{0.06}, Scalar{0.05}, Scalar{0.045}, Scalar{0.045}, Scalar{0.045}};

/**
 * @brief UR5 collision body: capsules spanning the joint-frame origins.
 *
 * The radii are padded to enclose the vendor meshes, so the model errs
 * conservative: it may flag a near miss, but a configuration it accepts is
 * clear on the real geometry too.
 */
[[nodiscard]] inline collision::RobotGeometry ur5_collision(const Ur5Chain& chain = ur5()) {
    auto geometry = collision::geometry_from_link_frames(chain, kUr5CapsuleRadii);
    // Pairs whose gap no joint can close: the shoulder offset holds the forearm
    // and the wrist-2 drop exactly 0.109 m apart, and the wrist drop holds
    // wrist 1 and the flange 0.095 m apart, at every configuration. Their
    // capsule distance is a constant near-touch, so checking them could only
    // ever produce false positives.
    geometry.disabled_self_pairs = {{2, 4}, {3, 5}};
    return geometry;
}

/**
 * @brief UR5 link inertias: manufacturer masses on the capsule skeleton.
 *
 * The masses are the `ur_description` values (shoulder to wrist 3); each is
 * spread uniformly over the link's capsule, so the tensors are approximate but
 * consistent with the geometry everything else uses.
 */
[[nodiscard]] inline dynamics::DynamicsModel<kUr5Dof> ur5_dynamics(const Ur5Chain& chain = ur5()) {
    constexpr std::array<Scalar, kUr5Dof> kMasses{
        Scalar{3.7}, Scalar{8.393}, Scalar{2.275}, Scalar{1.219}, Scalar{1.219}, Scalar{0.1879}};
    return dynamics::DynamicsModel<kUr5Dof>{chain, dynamics::inertias_from_link_frames(chain, kUr5CapsuleRadii, kMasses)};
}

}  // namespace robotics::models
