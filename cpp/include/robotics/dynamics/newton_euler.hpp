#pragma once

#include <array>
#include <vector>

#include "robotics/dynamics/inertia.hpp"
#include "robotics/kinematics/serial_chain.hpp"

/// Recursive Newton-Euler dynamics on the product-of-exponentials chain —
/// Modern Robotics chapter 8, in Sophus `[linear; angular]` twist ordering.
namespace robotics::dynamics {

/**
 * @brief Everything the recursion needs, precomputed once per robot.
 *
 * The chain stores space-frame screws `S_i` and link home poses `M_i`; the
 * body-frame recursion wants each joint's screw in its own link frame,
 * `A_i = Ad_{M_i^{-1}} S_i`, and the home transform between consecutive link
 * frames, `B_i = M_{i-1}^{-1} M_i`. At a configuration the transform that pulls
 * quantities from frame i-1 into frame i is then `(B_i exp([A_i] theta_i))^{-1}`.
 */
template <int Dof>
class DynamicsModel {
public:
    using Joints = JointVector<Dof>;
    using Inertias = std::array<RigidBodyInertia, static_cast<std::size_t>(Dof)>;

    DynamicsModel(const SerialChain<Dof>& chain, const Inertias& inertias) : chain_{chain}, inertias_{inertias} {
        Pose previous;  // Identity: the base frame.
        for (int i = 0; i < Dof; ++i) {
            const Pose& home = chain.joints()[static_cast<std::size_t>(i)].link_home;
            screws_[static_cast<std::size_t>(i)] =
                home.inverse().Adj() * chain.joints()[static_cast<std::size_t>(i)].screw.to_twist();
            between_[static_cast<std::size_t>(i)] = previous.inverse() * home;
            spatial_[static_cast<std::size_t>(i)] = spatial_inertia(inertias[static_cast<std::size_t>(i)]);
            previous = home;
        }
        to_tip_ = previous.inverse() * chain.end_effector_home();
    }

    [[nodiscard]] const SerialChain<Dof>& chain() const noexcept { return chain_; }
    [[nodiscard]] const Inertias& inertias() const noexcept { return inertias_; }

    /**
     * @brief Inverse dynamics: the joint torques that realise a motion.
     *
     * Forward pass propagates twists and accelerations from the base outward;
     * backward pass propagates wrenches from the tip inward. Gravity enters as
     * the standard fictitious base acceleration, so it needs no separate term.
     *
     * @param angles Joint angles [rad].
     * @param velocities Joint rates [rad/s].
     * @param accelerations Joint accelerations [rad/s^2].
     * @param gravity Gravity vector in the space frame, e.g. (0, 0, -9.81).
     * @param tip_wrench External wrench on the end-effector, `[force; moment]`
     *        in the end-effector frame; zero when the arm moves freely.
     * @return Torques satisfying `tau = M(q) qdd + C(q, qd) qd + g(q) - J^T F_tip`.
     */
    [[nodiscard]] Joints inverse_dynamics(const JointsArg<Dof>& angles,
                                          const JointsArg<Dof>& velocities,
                                          const JointsArg<Dof>& accelerations,
                                          const Vector3& gravity,
                                          const Twist& tip_wrench = Twist::Zero()) const {
        std::array<Eigen::Matrix<Scalar, 6, 6>, static_cast<std::size_t>(Dof)> pull_from_parent;
        std::array<Twist, static_cast<std::size_t>(Dof)> twists;
        std::array<Twist, static_cast<std::size_t>(Dof)> rates;

        Twist parent_twist = Twist::Zero();
        // Accelerating the base at -g makes every link feel gravity for free.
        Twist parent_rate = Twist::Zero();
        parent_rate.head<3>() = -gravity;

        for (int i = 0; i < Dof; ++i) {
            const auto index = static_cast<std::size_t>(i);
            const Twist& screw = screws_[index];

            const Pose transform = (between_[index] * Pose::exp(angles(i) * screw)).inverse();
            pull_from_parent[index] = transform.Adj();

            const Twist joint_twist = velocities(i) * screw;
            twists[index] = pull_from_parent[index] * parent_twist + joint_twist;
            rates[index] =
                pull_from_parent[index] * parent_rate + Pose::lieBracket(twists[index], joint_twist) + accelerations(i) * screw;

            parent_twist = twists[index];
            parent_rate = rates[index];
        }

        Joints torques;
        Twist wrench = to_tip_.inverse().Adj().transpose() * tip_wrench;
        for (int i = Dof - 1; i >= 0; --i) {
            const auto index = static_cast<std::size_t>(i);
            wrench = wrench + spatial_[index] * rates[index] - adjoint_transpose(twists[index], spatial_[index] * twists[index]);
            torques(i) = screws_[index].dot(wrench);
            if (i > 0) {
                wrench = pull_from_parent[index].transpose() * wrench;
            }
        }
        return torques;
    }

    /// @return `g(q)`: the torques that hold the arm still under gravity.
    [[nodiscard]] Joints gravity_torque(const JointsArg<Dof>& angles, const Vector3& gravity) const {
        return inverse_dynamics(angles, Joints::Zero(), Joints::Zero(), gravity);
    }

    /// @return `C(q, qd) qd + g(q)`: every torque that is not inertial.
    [[nodiscard]] Joints bias_torque(const JointsArg<Dof>& angles,
                                     const JointsArg<Dof>& velocities,
                                     const Vector3& gravity) const {
        return inverse_dynamics(angles, velocities, Joints::Zero(), gravity);
    }

    /// @return The joint-space mass matrix `M(q)`, built one RNEA column at a time.
    [[nodiscard]] JointMatrix<Dof> mass_matrix(const JointsArg<Dof>& angles) const {
        JointMatrix<Dof> mass;
        for (int j = 0; j < Dof; ++j) {
            Joints unit = Joints::Zero();
            unit(j) = 1;
            mass.col(j) = inverse_dynamics(angles, Joints::Zero(), unit, Vector3::Zero());
        }
        return mass;
    }

    /// @return `qdd` from `M(q) qdd = tau - C(q, qd) qd - g(q)`.
    [[nodiscard]] Joints forward_dynamics(const JointsArg<Dof>& angles,
                                          const JointsArg<Dof>& velocities,
                                          const JointsArg<Dof>& torques,
                                          const Vector3& gravity) const {
        const JointMatrix<Dof> mass = mass_matrix(angles);
        const Joints bias = bias_torque(angles, velocities, gravity);
        return mass.ldlt().solve(torques - bias);
    }

    /// @return `0.5 qd^T M(q) qd` [J].
    [[nodiscard]] Scalar kinetic_energy(const JointsArg<Dof>& angles, const JointsArg<Dof>& velocities) const {
        return Scalar{0.5} * velocities.dot(mass_matrix(angles) * velocities);
    }

    /// @return Gravitational potential energy [J], zero level at the base.
    [[nodiscard]] Scalar potential_energy(const JointsArg<Dof>& angles, const Vector3& gravity) const {
        const std::vector<Pose> poses = chain_.link_poses(angles);
        Scalar energy{0};
        for (int i = 0; i < Dof; ++i) {
            const auto index = static_cast<std::size_t>(i);
            const Vector3 com_world = poses[index] * inertias_[index].com;
            energy -= inertias_[index].mass * gravity.dot(com_world);
        }
        return energy;
    }

private:
    /// @return `ad_V^T F` — the wrench-side dual of the twist bracket.
    [[nodiscard]] static Twist adjoint_transpose(const Twist& twist, const Twist& wrench) {
        // ad in [v; w] ordering is [[w]x [v]x; 0 [w]x]; this is its transpose
        // applied to a wrench, written out to avoid forming the 6x6.
        const Vector3 v = twist.head<3>();
        const Vector3 w = twist.tail<3>();
        const Vector3 f = wrench.head<3>();
        const Vector3 m = wrench.tail<3>();
        Twist result;
        result.head<3>() = -w.cross(f);
        result.tail<3>() = -v.cross(f) - w.cross(m);
        return result;
    }

    SerialChain<Dof> chain_;
    Inertias inertias_;
    std::array<Twist, static_cast<std::size_t>(Dof)> screws_{};  ///< `A_i`, in the link frame.
    std::array<Pose, static_cast<std::size_t>(Dof)> between_{};  ///< `B_i = M_{i-1}^{-1} M_i`.
    std::array<SpatialInertia, static_cast<std::size_t>(Dof)> spatial_{};
    Pose to_tip_{};  ///< `M_{n-1}^{-1} M_ee`.
};

}  // namespace robotics::dynamics
