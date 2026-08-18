#pragma once

#include <algorithm>
#include <cmath>

#include "robotics/core/types.hpp"

namespace robotics::solvers {

/// Tuning for the box-constrained QP solver.
struct BoxQpOptions {
    int max_sweeps{60};               ///< Projected Gauss-Seidel sweeps.
    Scalar tolerance{Scalar{1e-10}};  ///< Stop once the largest coordinate move falls below this.
};

struct BoxQpResult {
    bool converged{false};
    int sweeps{0};
};

/**
 * @brief Minimises a strictly convex quadratic subject to simple bounds.
 *
 *     minimise    0.5 * x^T H x + g^T x
 *     subject to  lower <= x <= upper   (element-wise)
 *
 * Solved by projected Gauss-Seidel: coordinate descent with each coordinate
 * clamped back into its interval. For positive-definite `H` the objective is
 * strictly convex, so the minimiser is unique and the sweeps converge to it.
 *
 * Working on the primal problem keeps the solver allocation-free and forms no
 * inverse, which is what this needs when it runs on every mouse move.
 *
 * @tparam Dof Number of variables.
 * @param hessian Positive-definite `H`; only its diagonal must be non-zero.
 * @param gradient Linear term `g`.
 * @param lower Element-wise lower bounds.
 * @param upper Element-wise upper bounds.
 * @param x [in,out] Warm start on entry, minimiser on exit; projected into the box first.
 * @param options Sweep limit and tolerance.
 * @return Whether the sweeps converged, and how many ran.
 */
template <int Dof>
BoxQpResult solve_box_qp(const JointMatrix<Dof>& hessian,
                         const JointVector<Dof>& gradient,
                         const JointVector<Dof>& lower,
                         const JointVector<Dof>& upper,
                         JointVector<Dof>& x,
                         const BoxQpOptions& options = {}) {
    BoxQpResult result;

    // A point already outside its bounds would give an empty interval; collapsing
    // it to the midpoint keeps the problem well posed.
    JointVector<Dof> low = lower;
    JointVector<Dof> high = upper;
    for (int i = 0; i < Dof; ++i) {
        if (low(i) > high(i)) {
            const Scalar middle = Scalar{0.5} * (low(i) + high(i));
            low(i) = middle;
            high(i) = middle;
        }
        x(i) = std::clamp(x(i), low(i), high(i));
    }

    for (int sweep = 0; sweep < options.max_sweeps; ++sweep) {
        Scalar largest_move{0};

        for (int i = 0; i < Dof; ++i) {
            const Scalar diagonal = hessian(i, i);
            if (!(diagonal > Scalar{0})) {
                continue;  // Regularisation should prevent this; skip rather than divide by zero.
            }

            // Exact minimiser along coordinate i, projected back into its interval.
            const Scalar partial = gradient(i) + hessian.row(i).dot(x);
            const Scalar candidate = std::clamp(x(i) - partial / diagonal, low(i), high(i));

            largest_move = std::max(largest_move, std::abs(candidate - x(i)));
            x(i) = candidate;
        }

        result.sweeps = sweep + 1;
        if (largest_move < options.tolerance) {
            result.converged = true;
            break;
        }
    }

    return result;
}

}  // namespace robotics::solvers
