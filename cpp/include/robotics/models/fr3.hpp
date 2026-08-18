#pragma once

#include "robotics/core/dh.hpp"
#include "robotics/kinematics/serial_chain.hpp"

namespace robotics::models {

inline constexpr int kFr3Dof = 7;

using Fr3Chain = SerialChain<kFr3Dof>;

/// FR3 modified-DH table, base to joint 7, as published by Franka.
inline constexpr std::array<ModifiedDhRow, kFr3Dof> kFr3DhTable{{
    {.a = 0, .d = Scalar{0.333}, .alpha = 0},
    {.a = 0, .d = 0, .alpha = -kHalfPi},
    {.a = 0, .d = Scalar{0.316}, .alpha = kHalfPi},
    {.a = Scalar{0.0825}, .d = 0, .alpha = kHalfPi},
    {.a = Scalar{-0.0825}, .d = Scalar{0.384}, .alpha = -kHalfPi},
    {.a = 0, .d = 0, .alpha = kHalfPi},
    {.a = Scalar{0.088}, .d = 0, .alpha = kHalfPi},
}};

/// Joint 7 to the flange, the frame FR3 poses are normally reported in.
inline constexpr ModifiedDhRow kFr3FlangeRow{.a = 0, .d = Scalar{0.107}, .alpha = 0};

/**
 * @brief FR3 joint position limits, from the Franka Control Interface.
 *
 * Joints 4 and 6 are asymmetric and exclude zero, so the all-zero configuration
 * is unreachable on real hardware. That is why `fr3_ready()` is the default pose.
 */
inline constexpr std::array<JointLimit, kFr3Dof> kFr3JointLimits{{
    {Scalar{-2.7437}, Scalar{2.7437}},
    {Scalar{-1.7837}, Scalar{1.7837}},
    {Scalar{-2.9007}, Scalar{2.9007}},
    {Scalar{-3.0421}, Scalar{-0.1518}},
    {Scalar{-2.8065}, Scalar{2.8065}},
    {Scalar{0.5445}, Scalar{4.5169}},
    {Scalar{-3.0159}, Scalar{3.0159}},
}};

/// The "ready" configuration used throughout the Franka stack.
[[nodiscard]] inline JointVector<kFr3Dof> fr3_ready() {
    JointVector<kFr3Dof> ready;
    ready << 0, -kQuarterPi, 0, -3 * kQuarterPi, 0, kHalfPi, kQuarterPi;
    return ready;
}

/**
 * @brief Builds the FR3 chain from the published DH table.
 *
 * The table is walked once at zero configuration. Each joint rotates about the
 * z axis of its own DH frame, so its screw axis is `revolute(z_i, o_i)` with both
 * taken in the base frame, and that same frame is the link's home pose.
 */
[[nodiscard]] inline Fr3Chain fr3() {
    constexpr std::array<const char*, kFr3Dof> names{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

    Fr3Chain::Specs specs{};
    Pose frame;
    for (std::size_t i = 0; i < specs.size(); ++i) {
        frame = frame * modified_dh_transform(kFr3DhTable[i], Scalar{0});

        const Vector3 axis = frame.so3().matrix().col(2);
        specs[i] = JointSpec<kFr3Dof>{.screw = ScrewAxis::revolute(axis, frame.translation()),
                                      .link_home = frame,
                                      .limit = kFr3JointLimits[i],
                                      .name = names[i]};
    }

    return Fr3Chain{specs, frame * modified_dh_transform(kFr3FlangeRow, Scalar{0})};
}

}  // namespace robotics::models
