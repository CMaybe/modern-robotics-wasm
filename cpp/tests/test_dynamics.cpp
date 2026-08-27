#include <cmath>
#include <gtest/gtest.h>

#include "robotics/dynamics/simulation.hpp"
#include "robotics/models/fr3.hpp"
#include "robotics/models/ur5.hpp"
#include "test_helpers.hpp"

namespace {

using robotics::JointVector;
using robotics::kHalfPi;
using robotics::Scalar;
using robotics::Vector3;
using robotics::models::kFr3Dof;
using robotics::models::kUr5Dof;
using robotics::testing::joints;

namespace dynamics = robotics::dynamics;

const Vector3 kGravity{0, 0, Scalar{-9.81}};

[[nodiscard]] JointVector<kUr5Dof> ur5_pose() { return joints<kUr5Dof>({0.4F, -1.1F, 1.3F, -0.2F, 1.0F, 0.3F}); }

}  // namespace

// ---------------------------------------------------------------------------
// Inertias
// ---------------------------------------------------------------------------

TEST(Inertia, AZeroLengthCapsuleIsASolidSphere) {
    const auto body = dynamics::capsule_inertia(Scalar{2}, Vector3{1, 2, 3}, Vector3{1, 2, 3}, Scalar{0.1});

    EXPECT_TRUE(body.com.isApprox(Vector3{1, 2, 3}, Scalar{1e-6}));
    const Scalar sphere = Scalar{0.4} * 2 * Scalar{0.01};  // 2/5 m r^2
    EXPECT_TRUE(body.inertia.isApprox(sphere * robotics::Matrix3::Identity(), Scalar{1e-5})) << body.inertia;
}

TEST(Inertia, TheCapsuleTensorIsPrincipalAlongItsAxis) {
    const Vector3 start{0, 0, 0};
    const Vector3 end{0.4F, 0, 0};
    const auto body = dynamics::capsule_inertia(Scalar{3}, start, end, Scalar{0.05});

    // The x axis is the symmetry axis: smallest moment, and an eigenvector.
    EXPECT_TRUE((body.inertia * Vector3::UnitX()).isApprox(body.inertia(0, 0) * Vector3::UnitX(), Scalar{1e-6}));
    EXPECT_LT(body.inertia(0, 0), body.inertia(1, 1));
    EXPECT_NEAR(body.inertia(1, 1), body.inertia(2, 2), Scalar{1e-6});
    EXPECT_TRUE(body.com.isApprox(Vector3{0.2F, 0, 0}, Scalar{1e-6}));
}

// ---------------------------------------------------------------------------
// Newton-Euler
// ---------------------------------------------------------------------------

TEST(NewtonEuler, GravityTorqueIsTheGradientOfThePotentialEnergy) {
    const auto chain = robotics::models::ur5();
    const auto model = robotics::models::ur5_dynamics(chain);
    const auto q = ur5_pose();

    const auto torque = model.gravity_torque(q, kGravity);

    constexpr Scalar kDelta = Scalar{1e-3};
    for (int j = 0; j < kUr5Dof; ++j) {
        auto forward = q;
        auto backward = q;
        forward(j) += kDelta;
        backward(j) -= kDelta;
        const Scalar numeric =
            (model.potential_energy(forward, kGravity) - model.potential_energy(backward, kGravity)) / (2 * kDelta);
        EXPECT_NEAR(torque(j), numeric, Scalar{2e-2}) << "joint " << j;
    }
}

TEST(NewtonEuler, TheMassMatrixIsSymmetricPositiveDefinite) {
    const auto model = robotics::models::fr3_dynamics();
    const auto q = robotics::models::fr3_ready();

    const auto mass = model.mass_matrix(q);
    EXPECT_TRUE(mass.isApprox(mass.transpose(), Scalar{1e-4})) << mass;

    const auto eigenvalues = mass.template selfadjointView<Eigen::Lower>().eigenvalues();
    EXPECT_GT(eigenvalues.minCoeff(), Scalar{0}) << eigenvalues.transpose();
}

TEST(NewtonEuler, ForwardAndInverseDynamicsRoundTrip) {
    const auto model = robotics::models::ur5_dynamics();
    const auto q = ur5_pose();
    const auto qd = joints<kUr5Dof>({0.3F, -0.5F, 0.7F, 0.2F, -0.4F, 0.6F});
    const auto qdd = joints<kUr5Dof>({-0.8F, 0.4F, -0.2F, 0.9F, 0.1F, -0.5F});

    const auto torque = model.inverse_dynamics(q, qd, qdd, kGravity);
    const auto recovered = model.forward_dynamics(q, qd, torque, kGravity);

    EXPECT_TRUE(recovered.isApprox(qdd, Scalar{1e-3})) << recovered.transpose();
}

TEST(NewtonEuler, KineticEnergyMatchesTheVelocityRecursion) {
    // 0.5 qd^T M qd must equal the energy summed link by link from the twists —
    // two independent code paths over the same model.
    const auto chain = robotics::models::ur5();
    const auto model = robotics::models::ur5_dynamics(chain);
    const auto q = ur5_pose();
    const auto qd = joints<kUr5Dof>({0.3F, -0.5F, 0.7F, 0.2F, -0.4F, 0.6F});

    // Independent evaluation: energy of each link from its world COM velocity,
    // via finite differences of the COM positions and rotations.
    constexpr Scalar kDelta = Scalar{1e-4};
    const auto poses_now = chain.link_poses(q);
    const auto poses_next = chain.link_poses((q + kDelta * qd).eval());
    Scalar summed{0};
    for (int i = 0; i < kUr5Dof; ++i) {
        const auto index = static_cast<std::size_t>(i);
        const auto& body = model.inertias()[index];
        const Vector3 com_velocity = (poses_next[index] * body.com - poses_now[index] * body.com) / kDelta;
        const robotics::Matrix3 rotation_rate = (poses_next[index].so3().matrix() - poses_now[index].so3().matrix()) / kDelta;
        const robotics::Matrix3 omega_skew = rotation_rate * poses_now[index].so3().matrix().transpose();
        const Vector3 omega{omega_skew(2, 1), omega_skew(0, 2), omega_skew(1, 0)};
        const robotics::Matrix3 world_inertia =
            poses_now[index].so3().matrix() * body.inertia * poses_now[index].so3().matrix().transpose();
        summed += Scalar{0.5} * body.mass * com_velocity.squaredNorm() + Scalar{0.5} * omega.dot(world_inertia * omega);
    }

    EXPECT_NEAR(model.kinetic_energy(q, qd), summed, Scalar{5e-3} * std::max(Scalar{1}, summed));
}

// ---------------------------------------------------------------------------
// Simulation and controllers
// ---------------------------------------------------------------------------

TEST(Simulation, ThePassiveArmConservesEnergyWithoutDampingOrGravity) {
    const auto model = robotics::models::ur5_dynamics();

    dynamics::SimState<kUr5Dof> state;
    state.position = ur5_pose();
    state.velocity = joints<kUr5Dof>({0.5F, -0.3F, 0.4F, 0.6F, -0.2F, 0.3F});

    dynamics::SimulationOptions options;
    options.gravity = Vector3::Zero();
    options.damping = 0;

    const Scalar initial = model.kinetic_energy(state.position, state.velocity);
    for (int i = 0; i < 500; ++i) {
        (void)dynamics::simulate(model, state, {}, dynamics::Controller::kPassive, Scalar{1e-3}, options);
    }
    const Scalar final_energy = model.kinetic_energy(state.position, state.velocity);

    // Symplectic Euler keeps the energy bounded; allow a few percent of drift.
    EXPECT_NEAR(final_energy, initial, Scalar{0.05} * initial) << initial << " -> " << final_energy;
}

TEST(Simulation, GravityCompensationHoldsTheArmStill) {
    const auto model = robotics::models::ur5_dynamics();

    dynamics::SimState<kUr5Dof> state;
    state.position = ur5_pose();

    for (int i = 0; i < 200; ++i) {
        (void)dynamics::simulate(model, state, {}, dynamics::Controller::kGravity, Scalar{1e-3});
    }

    EXPECT_TRUE(state.position.isApprox(ur5_pose(), Scalar{1e-3})) << state.position.transpose();
    EXPECT_LT(state.velocity.norm(), Scalar{1e-2});
}

TEST(Simulation, ThePassiveArmFallsUnderGravity) {
    const auto model = robotics::models::ur5_dynamics();

    dynamics::SimState<kUr5Dof> state;
    state.position = ur5_pose();
    const Scalar initial_potential = model.potential_energy(state.position, kGravity);

    for (int i = 0; i < 300; ++i) {
        (void)dynamics::simulate(model, state, {}, dynamics::Controller::kPassive, Scalar{1e-3});
    }

    EXPECT_GT((state.position - ur5_pose()).norm(), Scalar{0.05});
    EXPECT_LT(model.potential_energy(state.position, kGravity), initial_potential);
}

TEST(Simulation, PdWithGravityCompensationReachesTheTarget) {
    const auto model = robotics::models::ur5_dynamics();

    dynamics::SimState<kUr5Dof> state;
    state.position = ur5_pose();

    dynamics::Reference<kUr5Dof> reference;
    reference.position = joints<kUr5Dof>({0.0F, -0.9F, 1.0F, 0.1F, 0.8F, 0.0F});

    // The wrist converges slowest: its shaped torque scales with its tiny
    // inertia while the viscous friction does not, so give the loop 4 s.
    for (int i = 0; i < 4000; ++i) {
        (void)dynamics::simulate(model, state, reference, dynamics::Controller::kPd, Scalar{1e-3});
    }

    EXPECT_LT((state.position - reference.position).norm(), Scalar{2e-2}) << state.position.transpose();
    EXPECT_LT(state.velocity.norm(), Scalar{5e-2});
}

TEST(Simulation, ComputedTorqueTracksAMovingReference) {
    const auto model = robotics::models::fr3_dynamics();

    const auto start = robotics::models::fr3_ready();
    auto goal = start;
    goal(0) += Scalar{0.8};
    goal(1) += Scalar{0.4};
    goal(3) += Scalar{0.5};

    dynamics::SimState<kFr3Dof> state;
    state.position = start;

    // Quintic point-to-point reference over 1.5 s: zero end velocities.
    const Scalar horizon{1.5};
    Scalar worst_error{0};
    const int frames = 150;
    for (int frame = 0; frame < frames; ++frame) {
        const Scalar t = (static_cast<Scalar>(frame) + 1) / static_cast<Scalar>(frames) * horizon;
        const Scalar s = t / horizon;
        const Scalar shape = 10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s;
        const Scalar shape_rate = (30 * s * s - 60 * s * s * s + 30 * s * s * s * s) / horizon;
        const Scalar shape_accel = (60 * s - 180 * s * s + 120 * s * s * s) / (horizon * horizon);

        dynamics::Reference<kFr3Dof> reference;
        reference.position = start + shape * (goal - start);
        reference.velocity = shape_rate * (goal - start);
        reference.acceleration = shape_accel * (goal - start);

        (void)dynamics::simulate(model, state, reference, dynamics::Controller::kComputedTorque, horizon / frames);
        worst_error = std::max(worst_error, (state.position - reference.position).norm());
    }

    EXPECT_LT(worst_error, Scalar{0.05}) << worst_error;
    EXPECT_TRUE(state.position.isApprox(goal, Scalar{2e-2})) << state.position.transpose();
}

TEST(Simulation, JointLimitsActAsInelasticStops) {
    const auto model = robotics::models::fr3_dynamics();
    const auto chain = robotics::models::fr3();

    dynamics::SimState<kFr3Dof> state;
    state.position = robotics::models::fr3_ready();
    state.velocity = JointVector<kFr3Dof>::Zero();
    state.velocity(3) = Scalar{-3};  // Slam joint 4 toward its lower limit.

    dynamics::SimulationOptions options;
    options.damping = 0;
    for (int i = 0; i < 1000; ++i) {
        (void)dynamics::simulate(model, state, {}, dynamics::Controller::kGravity, Scalar{1e-3}, options);
    }

    EXPECT_TRUE(chain.within_limits(state.position)) << state.position.transpose();
}
