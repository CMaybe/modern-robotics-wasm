#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "robotics/collision/world.hpp"
#include "robotics/kinematics/serial_chain.hpp"
#include "robotics/planning/rrt_connect.hpp"
#include "robotics/solvers/box_qp.hpp"

/// Optimisation-based trajectory smoothing, CHOMP-style: the shortcut path is a
/// polyline with kinks, this pass turns it into a curve that trades path length
/// against obstacle clearance. Every ingredient is reused: the per-waypoint
/// subproblem is the same box-constrained QP the IK solver runs, and the
/// obstacle gradient comes from the witness point and normal that every
/// collision query already returns.
namespace robotics::planning {

/// Tuning for `optimize_path`.
struct TrajectoryOptions {
    Scalar spacing{Scalar{0.15}};         ///< Waypoint spacing after densification [rad].
    int max_sweeps{60};                   ///< Full passes over the waypoints.
    Scalar tolerance{Scalar{1e-4}};       ///< Stop once the largest waypoint move falls below this [rad].
    Scalar smoothness_weight{Scalar{1}};  ///< Weight of the squared segment lengths.
    Scalar obstacle_weight{Scalar{100}};  ///< Weight of the squared clearance violation.
    Scalar safe_distance{Scalar{0.05}};   ///< Clearance below which the obstacle cost activates [m].
    Scalar damping{Scalar{1e-3}};         ///< Levenberg regularisation of each step.
    Scalar max_step{Scalar{0.1}};         ///< Per-sweep clamp on a waypoint's move [rad].
    Scalar resolution{Scalar{0.05}};      ///< Collision-check spacing for the final validation [rad].
    Scalar margin{Scalar{0.01}};          ///< Clearance the validated path must keep [m].
};

/// What `optimize_path` produced.
template <int Dof>
struct TrajectoryResult {
    std::vector<JointVector<Dof>> path;  ///< The optimised path, or the densified input when infeasible.
    bool feasible{false};                ///< Whether the optimised path passed full edge validation.
    int sweeps{0};                       ///< Sweeps actually run.
    Scalar smoothness{0};                ///< Final sum of squared segment lengths [rad^2].
    Scalar min_clearance{0};             ///< Tightest waypoint clearance of the returned path [m].
};

/// @return `path` with waypoints inserted so no segment exceeds `spacing`. The
/// optimiser moves waypoints, never endpoints, so resolution must exist upfront.
template <int Dof>
[[nodiscard]] std::vector<JointVector<Dof>> densify_path(const std::vector<JointVector<Dof>>& path, Scalar spacing) {
    std::vector<JointVector<Dof>> dense;
    if (path.empty()) {
        return dense;
    }
    dense.push_back(path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
        const JointVector<Dof>& from = path[i - 1];
        const JointVector<Dof>& to = path[i];
        const int steps = std::max(1, static_cast<int>(std::ceil((to - from).norm() / std::max(spacing, Scalar{1e-4}))));
        for (int s = 1; s <= steps; ++s) {
            dense.push_back(from + (static_cast<Scalar>(s) / static_cast<Scalar>(steps)) * (to - from));
        }
    }
    return dense;
}

/// @return Sum of squared segment lengths — the smoothness objective [rad^2].
template <int Dof>
[[nodiscard]] Scalar path_smoothness(const std::vector<JointVector<Dof>>& path) {
    Scalar total{0};
    for (std::size_t i = 1; i < path.size(); ++i) {
        total += (path[i] - path[i - 1]).squaredNorm();
    }
    return total;
}

/**
 * @brief Positional Jacobian of a point riding on a link.
 *
 * Column `j` is how the point moves per unit rate of joint `j`: for a revolute
 * joint, `axis_j x (point - origin_j)`; joints beyond the link do not move the
 * point at all. Both robot models are all-revolute — a prismatic joint reports
 * a zero axis and thus contributes nothing here.
 */
template <int Dof>
[[nodiscard]] Eigen::Matrix<Scalar, 3, Dof> point_jacobian(const SerialChain<Dof>& chain,
                                                           const JointsArg<Dof>& angles,
                                                           int link,
                                                           const Vector3& point) {
    const std::vector<Vector3> axes = chain.joint_axes(angles);
    const std::vector<Pose> poses = chain.link_poses(angles);

    Eigen::Matrix<Scalar, 3, Dof> jacobian = Eigen::Matrix<Scalar, 3, Dof>::Zero();
    for (int j = 0; j <= link && j < Dof; ++j) {
        // The link frame origin lies on joint j's axis in both models.
        const Vector3 origin = poses[static_cast<std::size_t>(j)].translation();
        jacobian.col(j) = axes[static_cast<std::size_t>(j)].cross(point - origin);
    }
    return jacobian;
}

/**
 * @brief Smooths a collision-free path by local optimisation.
 *
 * The path is densified, then each interior waypoint is repeatedly re-solved
 * with its neighbours held fixed (coordinate descent over waypoints). One
 * waypoint's subproblem minimises
 *
 *     w_s (|q - q_prev|^2 + |q_next - q|^2)          — stretch: pulls the path straight
 *   + w_o max(0, d_safe - d(q))^2                    — pushes it off the obstacles
 *   + damping |dq|^2                                 — keeps the Gauss-Newton step honest
 *
 * subject to the joint limits and a per-sweep trust region — exactly the
 * quadratic-plus-box shape `solve_box_qp` handles, so the IK solver is reused
 * verbatim. The obstacle term is linearised through the contact witness: the
 * signed distance grows along `normal` at the witness `point`, and
 * `point_jacobian` maps that direction into joint space.
 *
 * Like every local method this cannot guarantee feasibility, so the result is
 * validated edge by edge afterwards; when validation fails the densified input
 * (which was feasible by construction) is returned with `feasible = false`.
 */
template <int Dof>
[[nodiscard]] TrajectoryResult<Dof> optimize_path(const SerialChain<Dof>& chain,
                                                  const collision::RobotGeometry& geometry,
                                                  const collision::CollisionWorld& world,
                                                  const std::vector<JointVector<Dof>>& input,
                                                  const TrajectoryOptions& options = {}) {
    using JointMat = JointMatrix<Dof>;
    using Joints = JointVector<Dof>;

    TrajectoryResult<Dof> result;
    std::vector<Joints> path = densify_path(input, options.spacing);
    const std::vector<Joints> fallback = path;
    if (path.size() < 3) {
        result.path = std::move(path);
        result.feasible = true;
        result.smoothness = path_smoothness(result.path);
        result.min_clearance = std::numeric_limits<Scalar>::infinity();
        return result;
    }

    const Joints lower = chain.lower_limits();
    const Joints upper = chain.upper_limits();

    for (int sweep = 0; sweep < options.max_sweeps; ++sweep) {
        result.sweeps = sweep + 1;
        Scalar largest_move{0};

        for (std::size_t i = 1; i + 1 < path.size(); ++i) {
            const Joints& current = path[i];

            // Stretch terms against both fixed neighbours.
            JointMat hessian = (2 * options.smoothness_weight + options.damping) * JointMat::Identity();
            Joints gradient = options.smoothness_weight * ((current - path[i - 1]) + (current - path[i + 1]));

            // Obstacle term, linearised at the tightest contact of this waypoint.
            const collision::Contact contact = collision::nearest_contact(chain, geometry, current, world);
            const Scalar violation = options.safe_distance - contact.distance;
            if (contact.link >= 0 && violation > Scalar{0}) {
                // d grows along the normal at the witness point, so the violation
                // c = d_safe - d(q) has joint-space gradient -J_p^T n.
                const Eigen::Matrix<Scalar, 3, Dof> jacobian = point_jacobian(chain, current, contact.link, contact.point);
                const Joints direction = -(jacobian.transpose() * contact.normal);
                hessian += options.obstacle_weight * (direction * direction.transpose());
                gradient += options.obstacle_weight * violation * direction;
            }

            Joints low;
            Joints high;
            for (int j = 0; j < Dof; ++j) {
                low(j) = std::max(lower(j) - current(j), -options.max_step);
                high(j) = std::min(upper(j) - current(j), options.max_step);
            }

            Joints step = Joints::Zero();
            (void)solvers::solve_box_qp<Dof>(hessian, gradient, low, high, step);
            path[i] = current + step;
            largest_move = std::max(largest_move, step.template lpNorm<Eigen::Infinity>());
        }

        if (largest_move < options.tolerance) {
            break;
        }
    }

    // A local method promises nothing; the validation pass does.
    bool feasible = true;
    for (std::size_t i = 1; i < path.size() && feasible; ++i) {
        feasible = motion_is_free(chain, geometry, world, path[i - 1], path[i], options.resolution, options.margin);
    }

    result.feasible = feasible;
    result.path = feasible ? std::move(path) : fallback;
    result.smoothness = path_smoothness(result.path);

    result.min_clearance = std::numeric_limits<Scalar>::infinity();
    for (const Joints& waypoint : result.path) {
        result.min_clearance =
            std::min(result.min_clearance, collision::nearest_contact(chain, geometry, waypoint, world).distance);
    }
    return result;
}

}  // namespace robotics::planning
