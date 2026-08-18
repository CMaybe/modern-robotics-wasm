#pragma once

#include <limits>
#include <type_traits>

#include "robotics/kinematics/serial_chain.hpp"
#include "robotics/solvers/box_qp.hpp"

namespace robotics::ik {

/// How each iteration turns the task error into a joint-space step.
enum class Method {
    /**
     * Damped least squares, then clamp the iterate into the joint limits. The
     * step is chosen as if the joints were unbounded, so a joint that reaches a
     * limit simply loses its contribution: the others were solved assuming it
     * would move, and nothing hands the task back to them.
     */
    DampedLeastSquares,
    /**
     * Box-constrained QP: the limits constrain the step itself, so free joints
     * take over the motion a saturated joint can no longer provide. The
     * per-iteration step cap folds into the same box.
     */
    BoxQp,
};

/**
 * @brief Solver tuning.
 *
 * The defaults suit interactive dragging: non-zero damping keeps the iteration
 * stable as the target passes through singular configurations, and `max_step`
 * stops a single frame from jumping.
 */
struct Options {
    Method method{Method::BoxQp};
    int max_iterations{100};
    Scalar position_tolerance{Scalar{1e-4}};     ///< [m]
    Scalar orientation_tolerance{Scalar{1e-4}};  ///< [rad]
    Scalar damping{Scalar{1e-2}};                ///< Levenberg-Marquardt lambda.
    Scalar max_step{Scalar{0.2}};                ///< Per-iteration cap on |delta_theta|_inf [rad].
    bool respect_joint_limits{true};
};

/// Outcome of a solve.
template <int Dof>
struct Result {
    JointVector<Dof> joints{JointVector<Dof>::Zero()};
    bool converged{false};
    int iterations{};
    Scalar position_error{};     ///< [m]
    Scalar orientation_error{};  ///< [rad]
};

/**
 * @brief Solves inverse kinematics for a target pose.
 *
 * Both methods minimise the same local model of the task error,
 *
 *     0.5 * ||J dtheta - e||^2 + 0.5 * lambda^2 ||dtheta||^2,   e = log(T_d T^-1)
 *
 * and differ only in whether the joint limits take part in that minimisation or
 * are applied afterwards. The error twist is taken in the space frame so that it
 * pairs with the space Jacobian.
 *
 * @param chain The arm.
 * @param target Desired end-effector pose.
 * @param seed Initial guess; pass the current configuration when dragging.
 * @param options Solver tuning.
 * @return The solution together with convergence diagnostics.
 */
template <int Dof>
[[nodiscard]] Result<Dof> solve(const SerialChain<Dof>& chain,
                                const Pose& target,
                                const JointsArg<Dof>& seed,
                                const Options& options = {}) {
    Result<Dof> result;
    result.joints = options.respect_joint_limits ? chain.clamp_to_limits(seed) : seed;

    const Scalar lambda = std::max(options.damping, Scalar{1e-6});
    const JointMatrix<Dof> damping_term = (lambda * lambda) * JointMatrix<Dof>::Identity();

    const JointVector<Dof> lower = chain.lower_limits();
    const JointVector<Dof> upper = chain.upper_limits();

    for (int iteration = 0; iteration <= options.max_iterations; ++iteration) {
        const Pose current = chain.forward(result.joints);
        const Pose error = target * current.inverse();
        const Twist error_twist = error.log();

        result.iterations = iteration;
        result.position_error = (target.translation() - current.translation()).norm();
        result.orientation_error = error.so3().log().norm();

        if (result.position_error < options.position_tolerance && result.orientation_error < options.orientation_tolerance) {
            result.converged = true;
            break;
        }
        if (iteration == options.max_iterations) {
            break;
        }

        const auto jacobian = chain.space_jacobian(result.joints);
        const JointMatrix<Dof> hessian = jacobian.transpose() * jacobian + damping_term;
        const JointVector<Dof> gradient = -(jacobian.transpose() * error_twist);

        JointVector<Dof> step;
        if (options.method == Method::BoxQp) {
            // How far each joint may still travel, intersected with the trust region.
            constexpr Scalar kUnbounded = std::numeric_limits<Scalar>::max();
            JointVector<Dof> lower_step = JointVector<Dof>::Constant(-kUnbounded);
            JointVector<Dof> upper_step = JointVector<Dof>::Constant(kUnbounded);

            if (options.respect_joint_limits) {
                lower_step = lower - result.joints;
                upper_step = upper - result.joints;
            }
            if (options.max_step > Scalar{0}) {
                // The cap applies per joint, so one saturated joint does not
                // scale the whole step down with it.
                lower_step = lower_step.cwiseMax(-options.max_step);
                upper_step = upper_step.cwiseMin(options.max_step);
            }

            step.setZero();
            solvers::solve_box_qp<Dof>(hessian, gradient, lower_step, upper_step, step);
        } else {
            step = hessian.ldlt().solve(-gradient);

            // Trust region: a large step early on can throw the iterate across a singularity.
            const Scalar largest = step.cwiseAbs().maxCoeff();
            if (options.max_step > Scalar{0} && largest > options.max_step) {
                step *= options.max_step / largest;
            }
        }

        if (!step.allFinite()) {
            break;
        }

        result.joints += step;

        // The QP step is feasible by construction; this also repairs the damped
        // least-squares step, which knew nothing about the limits.
        if (options.respect_joint_limits) {
            result.joints = chain.clamp_to_limits(result.joints);
        }
    }

    return result;
}

}  // namespace robotics::ik
