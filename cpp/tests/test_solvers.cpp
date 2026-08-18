#include <gtest/gtest.h>

#include "robotics/models/fr3.hpp"
#include "robotics/models/ur5.hpp"
#include "robotics/solvers/box_qp.hpp"
#include "robotics/solvers/inverse_kinematics.hpp"
#include "test_helpers.hpp"

namespace {

using robotics::Pose;
using robotics::Scalar;
using robotics::models::kUr5Dof;
using robotics::testing::joints;

using Ur5Joints = robotics::JointVector<kUr5Dof>;

/// The quadratic both IK step rules minimise: 0.5 x^T H x + g^T x.
template <int Dof>
[[nodiscard]] Scalar quadratic_cost(const robotics::JointMatrix<Dof>& hessian,
                                    const robotics::JointVector<Dof>& gradient,
                                    const robotics::JointVector<Dof>& x) {
    return Scalar{0.5} * x.dot(hessian * x) + gradient.dot(x);
}

}  // namespace

// ---------------------------------------------------------------------------
// Box-constrained QP
// ---------------------------------------------------------------------------

TEST(BoxQp, MatchesTheUnconstrainedSolutionWhenTheBoxIsInactive) {
    robotics::JointMatrix<3> hessian;
    // clang-format off
    hessian << 4.0F, 1.0F, 0.5F,
               1.0F, 3.0F, 0.7F,
               0.5F, 0.7F, 2.0F;
    // clang-format on
    const robotics::JointVector<3> gradient{-1.0F, 0.4F, -0.2F};

    robotics::JointVector<3> x = robotics::JointVector<3>::Zero();
    const auto result = robotics::solvers::solve_box_qp<3>(
        hessian, gradient, robotics::JointVector<3>::Constant(-10.0F), robotics::JointVector<3>::Constant(10.0F), x);

    EXPECT_TRUE(result.converged);
    const robotics::JointVector<3> unconstrained = hessian.ldlt().solve(-gradient);
    EXPECT_TRUE(x.isApprox(unconstrained, Scalar{1e-4})) << x.transpose() << " vs " << unconstrained.transpose();
}

TEST(BoxQp, SatisfiesTheKktConditionsOnAnActiveBox) {
    robotics::JointMatrix<3> hessian;
    // clang-format off
    hessian << 4.0F, 1.0F, 0.5F,
               1.0F, 3.0F, 0.7F,
               0.5F, 0.7F, 2.0F;
    // clang-format on
    const robotics::JointVector<3> gradient{-9.0F, 0.4F, -0.2F};
    const robotics::JointVector<3> lower{-1.0F, -1.0F, -1.0F};
    const robotics::JointVector<3> upper{0.25F, 1.0F, 1.0F};

    robotics::JointVector<3> x = robotics::JointVector<3>::Zero();
    robotics::solvers::solve_box_qp<3>(hessian, gradient, lower, upper, x);

    for (int i = 0; i < 3; ++i) {
        EXPECT_GE(x(i), lower(i) - Scalar{1e-5});
        EXPECT_LE(x(i), upper(i) + Scalar{1e-5});
    }

    // KKT for a box: the gradient pushes each coordinate into its active bound and
    // vanishes wherever the coordinate sits strictly inside.
    const robotics::JointVector<3> residual = hessian * x + gradient;
    for (int i = 0; i < 3; ++i) {
        if (x(i) > lower(i) + Scalar{1e-4} && x(i) < upper(i) - Scalar{1e-4}) {
            EXPECT_NEAR(residual(i), Scalar{0}, Scalar{1e-3}) << "free coordinate " << i;
        } else if (x(i) <= lower(i) + Scalar{1e-4}) {
            EXPECT_GE(residual(i), Scalar{-1e-3}) << "at lower bound " << i;
        } else {
            EXPECT_LE(residual(i), Scalar{1e-3}) << "at upper bound " << i;
        }
    }
}

// ---------------------------------------------------------------------------
// Inverse kinematics
// ---------------------------------------------------------------------------

TEST(InverseKinematics, RecoversAReachablePose) {
    const auto chain = robotics::models::ur5();
    const auto truth = joints<kUr5Dof>({0.35F, -1.2F, 0.9F, 0.4F, 1.1F, -0.3F});
    const Pose target = chain.forward(truth);
    const Ur5Joints seed = truth + joints<kUr5Dof>({0.25F, 0.2F, -0.3F, 0.15F, -0.2F, 0.1F});

    for (const auto method : {robotics::ik::Method::BoxQp, robotics::ik::Method::DampedLeastSquares}) {
        const auto result = robotics::ik::solve(chain, target, seed, {.method = method});

        EXPECT_TRUE(result.converged) << "position error " << result.position_error;
        EXPECT_LT(result.position_error, Scalar{1e-3});
        EXPECT_LT(result.orientation_error, Scalar{1e-3});

        // Multiple configurations reach the same pose, so only the pose must match.
        robotics::testing::expect_pose_near(chain.forward(result.joints), target, Scalar{1e-3});
    }
}

TEST(InverseKinematics, TerminatesOnAnUnreachableTarget) {
    const auto chain = robotics::models::ur5();
    const Pose target{Sophus::SO3<Scalar>{}, robotics::Vector3{12, 0, 0}};

    for (const auto method : {robotics::ik::Method::BoxQp, robotics::ik::Method::DampedLeastSquares}) {
        const auto result = robotics::ik::solve(chain, target, Ur5Joints::Zero(), {.method = method, .max_iterations = 40});

        EXPECT_FALSE(result.converged);
        EXPECT_LE(result.iterations, 40);
        EXPECT_TRUE(result.joints.allFinite());
    }
}

TEST(InverseKinematics, RespectsJointLimits) {
    const auto chain = robotics::models::fr3();
    const Pose target{Sophus::SO3<Scalar>{}, robotics::Vector3{5, 0, 5}};

    for (const auto method : {robotics::ik::Method::BoxQp, robotics::ik::Method::DampedLeastSquares}) {
        const auto result = robotics::ik::solve(chain, target, robotics::models::fr3_ready(), {.method = method});
        EXPECT_TRUE(chain.within_limits(result.joints, Scalar{1e-4})) << "solution " << result.joints.transpose();
    }
}

TEST(InverseKinematics, RedundantArmSolvesWithSevenJoints) {
    const auto chain = robotics::models::fr3();
    const auto truth = joints<robotics::models::kFr3Dof>({0.25F, -0.6F, 0.3F, -1.7F, 0.4F, 1.6F, 0.5F});
    const Pose target = chain.forward(truth);
    const auto seed = truth + joints<robotics::models::kFr3Dof>({0.2F, 0.15F, -0.2F, 0.25F, -0.15F, 0.1F, 0.2F});

    const auto result = robotics::ik::solve(chain, target, seed);

    EXPECT_TRUE(result.converged) << "position error " << result.position_error;
    robotics::testing::expect_pose_near(chain.forward(result.joints), target, Scalar{1e-3});
}

TEST(InverseKinematics, QpStepIsNeverWorseThanClampedDampedLeastSquares) {
    // Tight limits so the unconstrained step wants to leave the feasible box.
    auto specs = robotics::models::ur5().joints();
    const std::array<robotics::JointLimit, kUr5Dof> tight{{
        {-0.20F, 0.20F},
        {-1.60F, -0.40F},
        {-0.10F, 1.60F},
        {-1.00F, 1.00F},
        {-1.00F, 1.00F},
        {-1.00F, 1.00F},
    }};
    for (std::size_t i = 0; i < specs.size(); ++i) {
        specs[i].limit = tight[i];
    }
    const robotics::SerialChain<kUr5Dof> chain{specs, robotics::models::ur5().end_effector_home()};

    const Ur5Joints start = joints<kUr5Dof>({0.18F, -0.5F, 1.4F, 0.0F, 0.5F, 0.0F});
    const Pose target = chain.forward(joints<kUr5Dof>({-0.15F, -1.4F, 0.3F, -0.6F, -0.6F, 0.4F}));

    // Reproduce one iteration of each step rule from the same state.
    const robotics::Twist error_twist = (target * chain.forward(start).inverse()).log();
    const auto jacobian = chain.space_jacobian(start);

    constexpr Scalar kLambda{0.02};
    constexpr Scalar kMaxStep{0.25};
    const robotics::JointMatrix<kUr5Dof> hessian =
        jacobian.transpose() * jacobian + (kLambda * kLambda) * robotics::JointMatrix<kUr5Dof>::Identity();
    const Ur5Joints gradient = -(jacobian.transpose() * error_twist);

    const Ur5Joints lower_step = (chain.lower_limits() - start).cwiseMax(-kMaxStep);
    const Ur5Joints upper_step = (chain.upper_limits() - start).cwiseMin(kMaxStep);

    Ur5Joints qp_step = Ur5Joints::Zero();
    robotics::solvers::solve_box_qp<kUr5Dof>(hessian, gradient, lower_step, upper_step, qp_step);

    // Damped least squares: unconstrained solve, global rescale, then clamp.
    Ur5Joints dls_step = hessian.ldlt().solve(-gradient);
    const Scalar largest = dls_step.cwiseAbs().maxCoeff();
    if (largest > kMaxStep) {
        dls_step *= kMaxStep / largest;
    }
    dls_step = chain.clamp_to_limits((start + dls_step).eval()) - start;

    const Scalar qp_cost = quadratic_cost<kUr5Dof>(hessian, gradient, qp_step);
    const Scalar dls_cost = quadratic_cost<kUr5Dof>(hessian, gradient, dls_step);

    // The QP returns the minimiser over the feasible box, so it cannot be beaten.
    EXPECT_LE(qp_cost, dls_cost + Scalar{1e-6}) << "qp " << qp_cost << " vs dls " << dls_cost;
    // On this deliberately constrained setup it should be strictly better.
    EXPECT_LT(qp_cost, dls_cost) << "qp " << qp_cost << " vs dls " << dls_cost;

    EXPECT_TRUE(chain.within_limits((start + qp_step).eval()));
}
