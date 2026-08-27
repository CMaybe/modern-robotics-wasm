#include "robotics/robot.hpp"

#include <algorithm>
#include <array>

#include "robotics/models/fr3.hpp"
#include "robotics/models/ur5.hpp"

namespace robotics {
namespace {

/// Builds a preset from a list of joint angles.
[[nodiscard]] RobotPreset preset(std::string label, std::initializer_list<Scalar> joints) {
    RobotPreset entry{.label = std::move(label), .joints = Eigen::VectorXf(joints.size())};
    int index = 0;
    for (const Scalar value : joints) {
        entry.joints(index++) = value;
    }
    return entry;
}

/**
 * @brief Adapts a fixed-size SerialChain to the Robot interface.
 * @tparam Dof Degrees of freedom.
 */
template <int Dof>
class ChainRobot final : public Robot {
public:
    ChainRobot(std::string_view id,
               std::string_view label,
               SerialChain<Dof> chain,
               collision::RobotGeometry geometry,
               dynamics::DynamicsModel<Dof> dynamics_model,
               const JointVector<Dof>& default_joints,
               std::vector<RobotPreset> presets)
        : id_{id}
        , label_{label}
        , chain_{std::move(chain)}
        , geometry_{std::move(geometry)}
        , dynamics_{std::move(dynamics_model)}
        , presets_{std::move(presets)}
        , lower_{chain_.lower_limits()}
        , upper_{chain_.upper_limits()}
        , default_joints_{default_joints} {
        names_.reserve(static_cast<std::size_t>(Dof));
        for (const auto& joint : chain_.joints()) {
            names_.emplace_back(joint.name);
        }
    }

    [[nodiscard]] std::string_view id() const noexcept override { return id_; }
    [[nodiscard]] std::string_view label() const noexcept override { return label_; }
    [[nodiscard]] int dof() const noexcept override { return Dof; }
    [[nodiscard]] std::span<const std::string> joint_names() const noexcept override { return names_; }
    [[nodiscard]] const Eigen::VectorXf& lower_limits() const noexcept override { return lower_; }
    [[nodiscard]] const Eigen::VectorXf& upper_limits() const noexcept override { return upper_; }
    [[nodiscard]] const Eigen::VectorXf& default_joints() const noexcept override { return default_joints_; }
    [[nodiscard]] std::span<const RobotPreset> presets() const noexcept override { return presets_; }

    [[nodiscard]] Pose forward(const Eigen::VectorXf& joints) const override { return chain_.forward(to_fixed(joints)); }

    [[nodiscard]] std::vector<Pose> link_poses(const Eigen::VectorXf& joints) const override {
        return chain_.link_poses(to_fixed(joints));
    }

    [[nodiscard]] std::vector<Vector3> joint_axes(const Eigen::VectorXf& joints) const override {
        return chain_.joint_axes(to_fixed(joints));
    }

    [[nodiscard]] Eigen::MatrixXf space_jacobian(const Eigen::VectorXf& joints) const override {
        return chain_.space_jacobian(to_fixed(joints));
    }

    [[nodiscard]] Scalar manipulability(const Eigen::VectorXf& joints) const override {
        return robotics::manipulability(chain_, to_fixed(joints));
    }

    [[nodiscard]] ManipulabilityEllipsoids ellipsoids(const Eigen::VectorXf& joints) const override {
        return manipulability_ellipsoids(chain_, to_fixed(joints));
    }

    [[nodiscard]] DynamicIkResult inverse(const Pose& target,
                                          const Eigen::VectorXf& seed,
                                          const ik::Options& options) const override {
        const ik::Result<Dof> result = ik::solve(chain_, target, to_fixed(seed), options);
        return DynamicIkResult{.joints = result.joints,
                               .converged = result.converged,
                               .iterations = result.iterations,
                               .position_error = result.position_error,
                               .orientation_error = result.orientation_error};
    }

    [[nodiscard]] std::vector<collision::PosedCapsule> collision_capsules(const Eigen::VectorXf& joints) const override {
        return collision::pose_geometry(geometry_, chain_.link_poses(to_fixed(joints)));
    }

    [[nodiscard]] collision::Contact self_contact(const Eigen::VectorXf& joints) const override {
        return collision::self_contact(geometry_, collision_capsules(joints));
    }

    [[nodiscard]] DynamicSimResult simulate(const Eigen::VectorXf& position,
                                            const Eigen::VectorXf& velocity,
                                            dynamics::Controller controller,
                                            const DynamicReference& reference,
                                            Scalar duration,
                                            const dynamics::SimulationOptions& options) const override {
        dynamics::SimState<Dof> state{to_fixed(position), to_fixed(velocity)};
        const dynamics::Reference<Dof> fixed_reference{
            to_fixed(reference.position), to_fixed(reference.velocity), to_fixed(reference.acceleration)};

        const JointVector<Dof> torque = dynamics::simulate(dynamics_, state, fixed_reference, controller, duration, options);
        return DynamicSimResult{.position = state.position, .velocity = state.velocity, .torque = torque};
    }

    [[nodiscard]] collision::Contact nearest_contact(const Eigen::VectorXf& joints,
                                                     const collision::CollisionWorld& world) const override {
        return collision::nearest_contact(chain_, geometry_, to_fixed(joints), world);
    }

    [[nodiscard]] DynamicPlanResult plan(const Eigen::VectorXf& start,
                                         const Eigen::VectorXf& goal,
                                         const collision::CollisionWorld& world,
                                         const planning::Options& options,
                                         const planning::TrajectoryOptions& trajectory) const override {
        const planning::Result<Dof> result =
            planning::plan_rrt_connect(chain_, geometry_, world, to_fixed(start), to_fixed(goal), options);

        DynamicPlanResult dynamic;
        dynamic.status = result.status;
        dynamic.iterations = result.iterations;
        dynamic.nodes = result.nodes;
        dynamic.path.assign(result.path.begin(), result.path.end());
        dynamic.raw_path.assign(result.raw_path.begin(), result.raw_path.end());

        if (result.success()) {
            const planning::TrajectoryResult<Dof> optimized =
                planning::optimize_path(chain_, geometry_, world, result.path, trajectory);
            dynamic.optimized_path.assign(optimized.path.begin(), optimized.path.end());
            dynamic.optimized_feasible = optimized.feasible;
        }
        return dynamic;
    }

private:
    /// Truncates or zero-pads a runtime-sized vector to this arm's joint count.
    [[nodiscard]] static JointVector<Dof> to_fixed(const Eigen::VectorXf& joints) {
        JointVector<Dof> fixed = JointVector<Dof>::Zero();
        const auto count = std::min<Eigen::Index>(Dof, joints.size());
        fixed.head(count) = joints.head(count);
        return fixed;
    }

    std::string id_;
    std::string label_;
    SerialChain<Dof> chain_;
    collision::RobotGeometry geometry_;
    dynamics::DynamicsModel<Dof> dynamics_;
    std::vector<RobotPreset> presets_;
    std::vector<std::string> names_;
    Eigen::VectorXf lower_;
    Eigen::VectorXf upper_;
    Eigen::VectorXf default_joints_;
};

[[nodiscard]] std::unique_ptr<Robot> make_ur5_robot() {
    JointVector<models::kUr5Dof> ready;
    ready << Scalar{0.4}, Scalar{-1.1}, Scalar{1.3}, Scalar{-0.2}, Scalar{1.0}, Scalar{0.0};

    std::vector<RobotPreset> presets;
    presets.push_back(preset("Ready", {0.4F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F}));
    presets.push_back(preset("Home", {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}));
    // The configuration tabulated in Modern Robotics, Example 4.5.
    presets.push_back(preset("MR Example 4.5", {0.0F, -kHalfPi, 0.0F, 0.0F, kHalfPi, 0.0F}));
    presets.push_back(preset("Wrist singularity", {0.0F, -0.6F, 0.9F, 0.0F, 0.0F, 0.0F}));

    auto chain = models::ur5();
    auto geometry = models::ur5_collision(chain);
    auto dynamics_model = models::ur5_dynamics(chain);
    return std::make_unique<ChainRobot<models::kUr5Dof>>("ur5",
                                                         "Universal Robots UR5",
                                                         std::move(chain),
                                                         std::move(geometry),
                                                         std::move(dynamics_model),
                                                         ready,
                                                         std::move(presets));
}

[[nodiscard]] std::unique_ptr<Robot> make_fr3_robot() {
    // Every preset must respect the asymmetric limits: joint 4 stays negative and
    // joint 6 stays positive, so neither can be zero.
    std::vector<RobotPreset> presets;
    presets.push_back(preset("Ready", {0.0F, -kQuarterPi, 0.0F, -3 * kQuarterPi, 0.0F, kHalfPi, kQuarterPi}));
    presets.push_back(preset("Folded", {0.0F, -1.0F, 0.0F, -2.6F, 0.0F, 1.6F, 0.8F}));
    presets.push_back(preset("Side reach", {1.2F, 0.6F, -0.5F, -1.8F, 0.4F, 1.9F, 0.5F}));
    // Joint 4 at its upper limit is as straight as an FR3 gets: an elbow singularity.
    presets.push_back(preset("Elbow singularity", {0.0F, 0.0F, 0.0F, -0.1518F, 0.0F, kHalfPi, 0.0F}));

    auto chain = models::fr3();
    auto geometry = models::fr3_collision(chain);
    auto dynamics_model = models::fr3_dynamics(chain);
    return std::make_unique<ChainRobot<models::kFr3Dof>>("fr3",
                                                         "Franka Research 3",
                                                         std::move(chain),
                                                         std::move(geometry),
                                                         std::move(dynamics_model),
                                                         models::fr3_ready(),
                                                         std::move(presets));
}

}  // namespace

std::span<const RobotDescription> robot_catalog() noexcept {
    static constexpr std::array<RobotDescription, 2> kCatalog{{
        {.id = "ur5", .label = "Universal Robots UR5", .dof = models::kUr5Dof},
        {.id = "fr3", .label = "Franka Research 3", .dof = models::kFr3Dof},
    }};
    return kCatalog;
}

std::unique_ptr<Robot> make_robot(std::string_view id) {
    if (id == "ur5") {
        return make_ur5_robot();
    }
    if (id == "fr3") {
        return make_fr3_robot();
    }
    return nullptr;
}

}  // namespace robotics
