#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "robotics/core/screw.hpp"
#include "robotics/core/types.hpp"

namespace robotics {

/// Everything needed to describe one joint of a serial chain.
template <int Dof>
struct JointSpec {
    ScrewAxis screw{};            ///< Screw axis in the space frame at zero configuration.
    Pose link_home{};             ///< Home pose of the frame this joint carries; used for rendering.
    JointLimit limit{-kPi, kPi};  ///< Travel range [rad].
    const char* name{"joint"};    ///< Human-readable name.
};

/**
 * @class SerialChain
 * @brief Product-of-exponentials model of a serial manipulator.
 *
 * `T(theta) = exp([S_0] theta_0) ... exp([S_n-1] theta_n-1) * M`
 *
 * The chain is a pure kinematic model: it answers where the arm is, and nothing
 * more. Inverse kinematics and manipulability are free functions built on top
 * (see `solvers/inverse_kinematics.hpp` and `kinematics/manipulability.hpp`), so
 * each algorithm can evolve without touching the model.
 *
 * Joints are supplied whole at construction, which makes a partially specified
 * chain unrepresentable.
 *
 * @tparam Dof Number of joints.
 */
template <int Dof>
class SerialChain {
public:
    static_assert(Dof > 0, "a serial chain needs at least one joint");

    using Joints = JointVector<Dof>;
    using Jacobian = JacobianMatrix<Dof>;
    using Specs = std::array<JointSpec<Dof>, Dof>;

    /**
     * @param joints Joint specifications, ordered base to tip.
     * @param end_effector_home The end-effector pose `M` at zero configuration.
     */
    constexpr SerialChain(const Specs& joints, const Pose& end_effector_home)
        : joints_{joints}, end_effector_home_{end_effector_home} {
        for (int i = 0; i < Dof; ++i) {
            twists_[static_cast<std::size_t>(i)] = joints_[static_cast<std::size_t>(i)].screw.to_twist();
        }
    }

    /// @return Number of joints, known at compile time.
    [[nodiscard]] static constexpr int dof() noexcept { return Dof; }

    /// @return The joint specifications, base to tip.
    [[nodiscard]] constexpr const Specs& joints() const noexcept { return joints_; }

    /// @return The end-effector home pose `M`.
    [[nodiscard]] constexpr const Pose& end_effector_home() const noexcept { return end_effector_home_; }

    /// @return Lower joint limits, as a vector.
    [[nodiscard]] Joints lower_limits() const {
        Joints limits;
        for (int i = 0; i < Dof; ++i) {
            limits(i) = joints_[static_cast<std::size_t>(i)].limit.lower;
        }
        return limits;
    }

    /// @return Upper joint limits, as a vector.
    [[nodiscard]] Joints upper_limits() const {
        Joints limits;
        for (int i = 0; i < Dof; ++i) {
            limits(i) = joints_[static_cast<std::size_t>(i)].limit.upper;
        }
        return limits;
    }

    /// @return `angles` projected into the joint limits.
    [[nodiscard]] Joints clamp_to_limits(const Joints& angles) const {
        Joints clamped = angles;
        for (int i = 0; i < Dof; ++i) {
            clamped(i) = joints_[static_cast<std::size_t>(i)].limit.clamp(clamped(i));
        }
        return clamped;
    }

    /// @return True if every joint of `angles` lies within its limit.
    [[nodiscard]] bool within_limits(const Joints& angles, Scalar tolerance = Scalar{1e-5}) const {
        for (int i = 0; i < Dof; ++i) {
            if (!joints_[static_cast<std::size_t>(i)].limit.contains(angles(i), tolerance)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Forward kinematics.
     * @param angles Joint angles [rad].
     * @return The end-effector pose in the space frame.
     */
    [[nodiscard]] Pose forward(const Joints& angles) const {
        Pose pose;
        for (int i = Dof - 1; i >= 0; --i) {
            pose = joint_exp(i, angles(i)) * pose;
        }
        return pose * end_effector_home_;
    }

    /**
     * @brief Poses of every joint frame plus the end-effector.
     *
     * `T_i(theta) = exp([S_0] theta_0) ... exp([S_i] theta_i) * M_i`
     *
     * Not needed for kinematics, but a renderer cannot draw the arm from the
     * end-effector pose alone.
     *
     * @param angles Joint angles [rad].
     * @return `Dof + 1` poses: one per joint frame, end-effector last.
     */
    [[nodiscard]] std::vector<Pose> link_poses(const Joints& angles) const {
        std::vector<Pose> poses;
        poses.reserve(static_cast<std::size_t>(Dof) + 1);

        Pose prefix;
        for (int i = 0; i < Dof; ++i) {
            prefix = prefix * joint_exp(i, angles(i));
            poses.push_back(prefix * joints_[static_cast<std::size_t>(i)].link_home);
        }
        poses.push_back(prefix * end_effector_home_);
        return poses;
    }

    /**
     * @brief Each joint's rotation axis in the space frame at the given configuration.
     * @param angles Joint angles [rad].
     * @return `Dof` vectors; zero-length for a prismatic joint.
     */
    [[nodiscard]] std::vector<Vector3> joint_axes(const Joints& angles) const {
        std::vector<Vector3> axes;
        axes.reserve(static_cast<std::size_t>(Dof));

        // Axis i is carried by the joints before it, so the prefix excludes theta_i.
        Pose prefix;
        for (int i = 0; i < Dof; ++i) {
            axes.push_back(prefix.so3() * joints_[static_cast<std::size_t>(i)].screw.angular);
            prefix = prefix * joint_exp(i, angles(i));
        }
        return axes;
    }

    /**
     * @brief Space Jacobian, in Sophus `[linear; angular]` ordering.
     *
     * `J_i = Ad_(exp([S_0] theta_0) ... exp([S_i-1] theta_i-1)) * S_i`
     *
     * @param angles Joint angles [rad].
     * @return The 6 x Dof space Jacobian.
     */
    [[nodiscard]] Jacobian space_jacobian(const Joints& angles) const {
        Jacobian jacobian;
        jacobian.col(0) = twists_[0];

        Pose prefix;
        for (int i = 1; i < Dof; ++i) {
            prefix = prefix * joint_exp(i - 1, angles(i - 1));
            jacobian.col(i) = prefix.Adj() * twists_[static_cast<std::size_t>(i)];
        }
        return jacobian;
    }

private:
    /// @return `exp([S_i] theta)` for joint `i`.
    [[nodiscard]] Pose joint_exp(int index, Scalar theta) const {
        return Pose::exp(theta * twists_[static_cast<std::size_t>(index)]);
    }

    Specs joints_{};
    std::array<Twist, static_cast<std::size_t>(Dof)> twists_{};
    Pose end_effector_home_{};
};

}  // namespace robotics
