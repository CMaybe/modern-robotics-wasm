#pragma once

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "robotics/collision/world.hpp"
#include "robotics/kinematics/manipulability.hpp"
#include "robotics/kinematics/serial_chain.hpp"
#include "robotics/solvers/inverse_kinematics.hpp"

namespace robotics {

/// A named configuration worth jumping to from a UI.
struct RobotPreset {
    std::string label;
    Eigen::VectorXf joints;
};

/// Inverse-kinematics result with a runtime-sized solution.
struct DynamicIkResult {
    Eigen::VectorXf joints;
    bool converged{false};
    int iterations{};
    Scalar position_error{};
    Scalar orientation_error{};
};

/**
 * @class Robot
 * @brief Runtime-polymorphic view of a SerialChain of any size.
 *
 * The kinematics stays templated so the linear algebra remains fixed-size; this
 * interface erases the joint count so that bindings and UIs can hold arms of
 * different DOF behind one type.
 */
class Robot {
public:
    virtual ~Robot() = default;

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot(Robot&&) = delete;
    Robot& operator=(Robot&&) = delete;

    /// Stable identifier this robot was built from, e.g. "ur5".
    [[nodiscard]] virtual std::string_view id() const noexcept = 0;
    /// Human-readable name, e.g. "Universal Robots UR5".
    [[nodiscard]] virtual std::string_view label() const noexcept = 0;
    [[nodiscard]] virtual int dof() const noexcept = 0;
    [[nodiscard]] virtual std::span<const std::string> joint_names() const noexcept = 0;
    [[nodiscard]] virtual const Eigen::VectorXf& lower_limits() const noexcept = 0;
    [[nodiscard]] virtual const Eigen::VectorXf& upper_limits() const noexcept = 0;
    /// Configuration a viewer should open in; always inside the joint limits.
    [[nodiscard]] virtual const Eigen::VectorXf& default_joints() const noexcept = 0;
    [[nodiscard]] virtual std::span<const RobotPreset> presets() const noexcept = 0;

    [[nodiscard]] virtual Pose forward(const Eigen::VectorXf& joints) const = 0;
    [[nodiscard]] virtual std::vector<Pose> link_poses(const Eigen::VectorXf& joints) const = 0;
    [[nodiscard]] virtual std::vector<Vector3> joint_axes(const Eigen::VectorXf& joints) const = 0;
    /// @return The 6 x dof space Jacobian in Sophus `[linear; angular]` ordering.
    [[nodiscard]] virtual Eigen::MatrixXf space_jacobian(const Eigen::VectorXf& joints) const = 0;
    [[nodiscard]] virtual Scalar manipulability(const Eigen::VectorXf& joints) const = 0;
    [[nodiscard]] virtual ManipulabilityEllipsoids ellipsoids(const Eigen::VectorXf& joints) const = 0;
    [[nodiscard]] virtual DynamicIkResult inverse(const Pose& target,
                                                  const Eigen::VectorXf& seed,
                                                  const ik::Options& options) const = 0;

    /// The collision body posed at `joints`, in the space frame.
    [[nodiscard]] virtual std::vector<collision::PosedCapsule> collision_capsules(const Eigen::VectorXf& joints) const = 0;
    /// The tightest self-collision pair at `joints` (infinite when none is checked).
    [[nodiscard]] virtual collision::Contact self_contact(const Eigen::VectorXf& joints) const = 0;

protected:
    Robot() = default;
};

/// Everything the registry knows about a robot before building it.
struct RobotDescription {
    std::string_view id;
    std::string_view label;
    int dof{};
};

/// @return Every robot `make_robot()` accepts, in menu order.
[[nodiscard]] std::span<const RobotDescription> robot_catalog() noexcept;

/**
 * @brief Builds a robot by identifier.
 * @param id One of the ids in `robot_catalog()`.
 * @return The robot, or nullptr when the identifier is unknown.
 */
[[nodiscard]] std::unique_ptr<Robot> make_robot(std::string_view id);

}  // namespace robotics
