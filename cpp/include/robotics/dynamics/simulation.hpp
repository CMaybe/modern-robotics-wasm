#pragma once

#include <algorithm>
#include <cmath>

#include "robotics/dynamics/newton_euler.hpp"

/// Fixed-substep rigid-body simulation with the torque controllers layered on
/// top. Semi-implicit (symplectic) Euler keeps the energy of the unforced arm
/// bounded instead of exploding, which is what lets the "passive" mode swing
/// indefinitely at 1 ms substeps.
namespace robotics::dynamics {

/// How the commanded torque is produced each substep.
enum class Controller {
    kPassive,         ///< No actuation; only damping and gravity act.
    kGravity,         ///< Gravity compensation: the arm floats where it is left.
    kPd,              ///< PD about a target configuration + gravity compensation.
    kComputedTorque,  ///< Full inverse-dynamics tracking of a moving reference.
};

/// Gains and physical constants for the controllers and the integrator.
struct SimulationOptions {
    Vector3 gravity{0, 0, Scalar{-9.81}};
    Scalar damping{Scalar{0.02}};    ///< Viscous joint friction [N m s/rad].
    Scalar kp{Scalar{100}};          ///< Proportional gain [1/s^2] of the shaped error dynamics.
    Scalar kd{Scalar{20}};           ///< Derivative gain [1/s]; 2*sqrt(kp) is critical.
    Scalar max_torque{Scalar{300}};  ///< Per-joint actuation limit [N m].
    Scalar substep{Scalar{1e-3}};    ///< Integration step [s].
};

/// The reference a controller regulates to; only `kPd` and `kComputedTorque` read it.
template <int Dof>
struct Reference {
    JointVector<Dof> position{JointVector<Dof>::Zero()};
    JointVector<Dof> velocity{JointVector<Dof>::Zero()};
    JointVector<Dof> acceleration{JointVector<Dof>::Zero()};
};

/// Simulation state, joint space only.
template <int Dof>
struct SimState {
    JointVector<Dof> position{JointVector<Dof>::Zero()};
    JointVector<Dof> velocity{JointVector<Dof>::Zero()};
};

/**
 * @brief The commanded torque for one substep, before the actuation clamp.
 *
 * - `kPassive`: zero — damping and gravity do whatever they do.
 * - `kGravity`: `g(q)` exactly cancels gravity; damping then bleeds velocity off.
 * - `kPd`: `g(q) + M(q)(Kp e - Kd qd)`. Gravity compensation plus a spring and
 *   damper to the target, with the gains shaped by the mass matrix so every
 *   joint closes the loop at the same rate. A fixed scalar gain would either
 *   crawl on the heavy shoulder or blow up the nearly massless wrist — the
 *   wrist's `Kd/M` exceeds the explicit integrator's stability bound.
 * - `kComputedTorque`: `M(q)(qdd_ref + Kp e + Kd de) + C qd + g` — feedback
 *   linearisation. The model cancels the real dynamics, leaving each joint a
 *   unit-mass double integrator with error dynamics `dde + Kd de + Kp e = 0`.
 *   Versus `kPd`: it also feeds forward the reference motion and cancels
 *   Coriolis forces, which is what lets it *track* rather than just arrive.
 */
template <int Dof>
[[nodiscard]] JointVector<Dof> control_torque(const DynamicsModel<Dof>& model,
                                              const SimState<Dof>& state,
                                              const Reference<Dof>& reference,
                                              Controller controller,
                                              const SimulationOptions& options) {
    using Joints = JointVector<Dof>;
    switch (controller) {
        case Controller::kPassive:
            return Joints::Zero();
        case Controller::kGravity:
            return model.gravity_torque(state.position, options.gravity);
        case Controller::kPd:
            return model.gravity_torque(state.position, options.gravity) +
                   model.mass_matrix(state.position) *
                       (options.kp * (reference.position - state.position) - options.kd * state.velocity).eval();
        case Controller::kComputedTorque: {
            const Joints error = reference.position - state.position;
            const Joints error_rate = reference.velocity - state.velocity;
            const Joints command = reference.acceleration + options.kp * error + options.kd * error_rate;
            return model.mass_matrix(state.position) * command +
                   model.bias_torque(state.position, state.velocity, options.gravity);
        }
    }
    return Joints::Zero();
}

/**
 * @brief Advances the state by `duration` in fixed substeps.
 *
 * Each substep recomputes the controller, clamps the torque to the actuation
 * limit, adds viscous damping, integrates semi-implicitly (velocity first, then
 * position with the new velocity), and resolves joint limits as inelastic
 * stops: the position clamps and the velocity component into the stop is
 * zeroed.
 *
 * @return The torque commanded on the last substep, for display.
 */
template <int Dof>
[[nodiscard]] JointVector<Dof> simulate(const DynamicsModel<Dof>& model,
                                        SimState<Dof>& state,
                                        const Reference<Dof>& reference,
                                        Controller controller,
                                        Scalar duration,
                                        const SimulationOptions& options = {}) {
    using Joints = JointVector<Dof>;

    const auto& chain = model.chain();
    const Joints lower = chain.lower_limits();
    const Joints upper = chain.upper_limits();

    const int substeps = std::max(1, static_cast<int>(std::ceil(duration / std::max(options.substep, Scalar{1e-5}))));
    const Scalar dt = duration / static_cast<Scalar>(substeps);

    Joints commanded = Joints::Zero();
    for (int step = 0; step < substeps; ++step) {
        commanded = control_torque(model, state, reference, controller, options);
        commanded = commanded.cwiseMax(-options.max_torque).cwiseMin(options.max_torque);

        const Joints applied = commanded - options.damping * state.velocity;
        const Joints acceleration = model.forward_dynamics(state.position, state.velocity, applied, options.gravity);

        state.velocity += dt * acceleration;
        state.position += dt * state.velocity;

        for (int j = 0; j < Dof; ++j) {
            if (state.position(j) < lower(j)) {
                state.position(j) = lower(j);
                state.velocity(j) = std::max(state.velocity(j), Scalar{0});
            } else if (state.position(j) > upper(j)) {
                state.position(j) = upper(j);
                state.velocity(j) = std::min(state.velocity(j), Scalar{0});
            }
        }
    }
    return commanded;
}

}  // namespace robotics::dynamics
