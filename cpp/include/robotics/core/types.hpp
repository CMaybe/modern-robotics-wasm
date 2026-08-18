#pragma once

#include <sophus/se3.hpp>

#include <Eigen/Dense>
#include <numbers>
#include <type_traits>

/// Kinematics for serial manipulators, built on the product-of-exponentials formulation.
namespace robotics {

/// Scalar type used throughout; single precision is plenty for kinematics and
/// keeps the WebAssembly build small.
using Scalar = float;

inline constexpr Scalar kPi = std::numbers::pi_v<Scalar>;
inline constexpr Scalar kHalfPi = kPi / Scalar{2};
inline constexpr Scalar kQuarterPi = kPi / Scalar{4};

/// A rigid transform. Sophus keeps it on the manifold, so poses cannot drift into
/// non-orthogonal matrices the way a bare 4x4 can.
using Pose = Sophus::SE3<Scalar>;

/// A spatial velocity in Sophus ordering, `[linear; angular]`.
using Twist = Eigen::Vector<Scalar, 6>;

using Vector3 = Eigen::Vector<Scalar, 3>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

/// Joint-space vector for a chain with `Dof` joints.
template <int Dof>
using JointVector = Eigen::Vector<Scalar, Dof>;

/**
 * @brief A joint vector in a position where `Dof` must not be deduced.
 *
 * Free functions take the chain and the configuration together, so `Dof` is
 * already fixed by the chain. Wrapping the second parameter keeps deduction out
 * of it, which is what lets callers pass an Eigen expression such as
 * `angles + step` without an explicit `.eval()`.
 */
template <int Dof>
using JointsArg = std::type_identity_t<JointVector<Dof>>;

/// Geometric Jacobian, mapping joint rates to a twist.
template <int Dof>
using JacobianMatrix = Eigen::Matrix<Scalar, 6, Dof>;

/// Symmetric joint-space matrix, e.g. a Gauss-Newton Hessian.
template <int Dof>
using JointMatrix = Eigen::Matrix<Scalar, Dof, Dof>;

/// Inclusive joint range, in radians.
struct JointLimit {
    Scalar lower{};
    Scalar upper{};

    [[nodiscard]] constexpr bool contains(Scalar value, Scalar tolerance = Scalar{0}) const noexcept {
        return value >= lower - tolerance && value <= upper + tolerance;
    }

    [[nodiscard]] constexpr Scalar clamp(Scalar value) const noexcept {
        return value < lower ? lower : (value > upper ? upper : value);
    }
};

}  // namespace robotics
