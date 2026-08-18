#pragma once

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <type_traits>

#include "robotics/kinematics/serial_chain.hpp"

namespace robotics {

/**
 * @brief Principal axes of a manipulability ellipsoid.
 *
 * The ellipsoid is the image of the unit ball of joint velocities under a
 * Jacobian block: the singular values are the semi-axis lengths and the left
 * singular vectors are the directions they point in.
 */
struct Ellipsoid {
    Matrix3 axes{Matrix3::Identity()};  ///< Columns are the principal directions, right-handed.
    Vector3 radii{Vector3::Zero()};     ///< Semi-axis lengths, descending.
    Scalar volume{};                    ///< Product of the semi-axes, i.e. the Yoshikawa measure.
    Scalar isotropy{};                  ///< `sigma_min / sigma_max` in [0, 1]; 1 is a sphere, 0 is singular.
};

/// Translational and rotational manipulability at the end-effector.
struct ManipulabilityEllipsoids {
    Ellipsoid linear;   ///< End-effector linear velocity, in m/rad.
    Ellipsoid angular;  ///< End-effector angular velocity, dimensionless.
};

namespace detail {

/// Builds an Ellipsoid from the SVD of a 3xN velocity Jacobian block.
template <typename Derived>
[[nodiscard]] Ellipsoid ellipsoid_from(const Eigen::MatrixBase<Derived>& block) {
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>> svd(block, Eigen::ComputeFullU);

    Ellipsoid ellipsoid;
    ellipsoid.axes = svd.matrixU();
    // A left-handed basis would mirror the ellipsoid when handed to a renderer.
    if (ellipsoid.axes.determinant() < Scalar{0}) {
        ellipsoid.axes.col(2) *= Scalar{-1};
    }

    ellipsoid.radii = svd.singularValues().template head<3>();
    ellipsoid.volume = ellipsoid.radii.prod();

    const Scalar largest = ellipsoid.radii(0);
    ellipsoid.isotropy = largest > Scalar{0} ? ellipsoid.radii(2) / largest : Scalar{0};
    return ellipsoid;
}

}  // namespace detail

/**
 * @brief Yoshikawa manipulability of the full twist Jacobian, `sqrt(det(J J^T))`.
 * @param chain The arm.
 * @param angles Joint angles [rad].
 * @return A value approaching zero near a singularity.
 */
template <int Dof>
[[nodiscard]] Scalar manipulability(const SerialChain<Dof>& chain, const JointsArg<Dof>& angles) {
    const auto jacobian = chain.space_jacobian(angles);
    const Eigen::Matrix<Scalar, 6, 6> gram = jacobian * jacobian.transpose();
    return std::sqrt(std::max(Scalar{0}, gram.determinant()));
}

/**
 * @brief Manipulability ellipsoids at the end-effector.
 *
 * The space Jacobian's linear block gives the velocity of the body point at the
 * origin, not at the tool, so it is first shifted to the end-effector:
 * `p_dot = v_s + w_s x p = (J_v - [p]_x J_w) * theta_dot`. Angular velocity is
 * the same for every point of a rigid body, so that block is used as-is.
 *
 * @param chain The arm.
 * @param angles Joint angles [rad].
 * @return Principal axes and shape measures for both blocks.
 */
template <int Dof>
[[nodiscard]] ManipulabilityEllipsoids manipulability_ellipsoids(const SerialChain<Dof>& chain, const JointsArg<Dof>& angles) {
    const auto jacobian = chain.space_jacobian(angles);
    const Vector3 tool = chain.forward(angles).translation();

    const Eigen::Matrix<Scalar, 3, Dof> linear_block = jacobian.template topRows<3>();
    const Eigen::Matrix<Scalar, 3, Dof> angular_block = jacobian.template bottomRows<3>();
    const Eigen::Matrix<Scalar, 3, Dof> tool_block = linear_block - Sophus::SO3<Scalar>::hat(tool) * angular_block;

    return ManipulabilityEllipsoids{.linear = detail::ellipsoid_from(tool_block),
                                    .angular = detail::ellipsoid_from(angular_block)};
}

}  // namespace robotics
