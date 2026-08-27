#pragma once

#include <array>
#include <cmath>

#include "robotics/core/types.hpp"
#include "robotics/kinematics/serial_chain.hpp"

/// Rigid-body inertia for the dynamics. The tensors are not taken from a URDF:
/// each link's published mass is spread uniformly over the same capsule the
/// collision model uses, so geometry, collision and dynamics share one source
/// of truth. That is an approximation — real links are not uniform — but a
/// self-consistent one, and exact for the demo's purposes.
namespace robotics::dynamics {

/// Mass, centre of mass and rotational inertia of one link, in its link frame.
struct RigidBodyInertia {
    Scalar mass{};
    Vector3 com{Vector3::Zero()};      ///< Centre of mass [m], link frame.
    Matrix3 inertia{Matrix3::Zero()};  ///< About the COM [kg m^2], link frame axes.
};

/**
 * @brief Inertia of a solid capsule of uniform density.
 *
 * Cylinder of length `L` plus two hemispherical caps, all of radius `r`. About
 * the COM (the midpoint), with the symmetry axis `u = (end - start)/L`:
 *
 *   I_axial = m_cy r^2/2                    + m_hs (2/5) r^2
 *   I_perp  = m_cy (L^2/12 + r^2/4)         + m_hs (2r^2/5 + L^2/4 + 3Lr/8)
 *
 * where `m_cy` and `m_hs` split the mass by volume between the cylinder and the
 * two caps. A zero-length capsule degenerates cleanly to a solid sphere.
 */
[[nodiscard]] inline RigidBodyInertia capsule_inertia(Scalar mass, const Vector3& start, const Vector3& end, Scalar radius) {
    const Vector3 offset = end - start;
    const Scalar length = offset.norm();

    const Scalar cylinder_volume = kPi * radius * radius * length;
    const Scalar caps_volume = Scalar{4} / Scalar{3} * kPi * radius * radius * radius;
    const Scalar cylinder_mass = mass * cylinder_volume / (cylinder_volume + caps_volume);
    const Scalar caps_mass = mass - cylinder_mass;

    const Scalar r2 = radius * radius;
    const Scalar axial = cylinder_mass * r2 / 2 + caps_mass * Scalar{0.4} * r2;
    const Scalar perpendicular = cylinder_mass * (length * length / 12 + r2 / 4) +
                                 caps_mass * (Scalar{0.4} * r2 + length * length / 4 + Scalar{3} * length * radius / 8);

    // diag(I_perp, I_perp, I_axial) in a frame whose z axis is `u`, expressed
    // without building that frame: I = I_perp (1 - u u^T) + I_axial u u^T.
    const Vector3 axis = length > Scalar{1e-9} ? Vector3{offset / length} : Vector3::UnitZ();
    const Matrix3 axis_outer = axis * axis.transpose();

    RigidBodyInertia body;
    body.mass = mass;
    body.com = Scalar{0.5} * (start + end);
    body.inertia = perpendicular * (Matrix3::Identity() - axis_outer) + axial * axis_outer;
    return body;
}

/// A 6x6 spatial inertia in Sophus `[linear; angular]` twist ordering, so that
/// the kinetic energy is `0.5 * V^T G V`.
using SpatialInertia = Eigen::Matrix<Scalar, 6, 6>;

/// @return The skew-symmetric matrix `[v]x` with `[v]x w = v x w`.
[[nodiscard]] inline Matrix3 skew(const Vector3& v) {
    Matrix3 matrix;
    // clang-format off
    matrix <<       0, -v.z(),  v.y(),
                v.z(),      0, -v.x(),
               -v.y(),  v.x(),      0;
    // clang-format on
    return matrix;
}

/// @return The spatial inertia of `body` about its link-frame origin.
[[nodiscard]] inline SpatialInertia spatial_inertia(const RigidBodyInertia& body) {
    const Matrix3 com_skew = skew(body.com);

    SpatialInertia inertia = SpatialInertia::Zero();
    inertia.topLeftCorner<3, 3>() = body.mass * Matrix3::Identity();
    inertia.topRightCorner<3, 3>() = -body.mass * com_skew;
    inertia.bottomLeftCorner<3, 3>() = body.mass * com_skew;
    inertia.bottomRightCorner<3, 3>() = body.inertia - body.mass * com_skew * com_skew;
    return inertia;
}

/**
 * @brief Builds one inertia per link from the chain's own skeleton.
 *
 * Link `i` is the body between joint `i` and joint `i+1` (the end-effector for
 * the last). Its geometry is the capsule spanning those frame origins with
 * `radii[i+1]` — the same span the collision model uses — carrying `masses[i]`
 * spread uniformly. A zero-length span degenerates to a solid sphere.
 */
template <int Dof>
[[nodiscard]] std::array<RigidBodyInertia, static_cast<std::size_t>(Dof)> inertias_from_link_frames(
    const SerialChain<Dof>& chain,
    const std::array<Scalar, Dof + 1>& radii,
    const std::array<Scalar, static_cast<std::size_t>(Dof)>& masses) {
    std::array<RigidBodyInertia, static_cast<std::size_t>(Dof)> inertias;
    for (int i = 0; i < Dof; ++i) {
        const Pose& link_home = chain.joints()[static_cast<std::size_t>(i)].link_home;
        const Vector3 start_space = link_home.translation();
        const Vector3 end_space = i + 1 < Dof ? chain.joints()[static_cast<std::size_t>(i) + 1].link_home.translation()
                                              : chain.end_effector_home().translation();

        // Express the span in the link frame, where the inertia must live.
        const Pose to_link = link_home.inverse();
        inertias[static_cast<std::size_t>(i)] = capsule_inertia(masses[static_cast<std::size_t>(i)],
                                                                to_link * start_space,
                                                                to_link * end_space,
                                                                radii[static_cast<std::size_t>(i) + 1]);
    }
    return inertias;
}

}  // namespace robotics::dynamics
