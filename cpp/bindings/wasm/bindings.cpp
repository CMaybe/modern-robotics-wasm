/**
 * @file bindings.cpp
 * @brief Emscripten/embind surface exposing the robot kinematics to JavaScript.
 *
 * Everything crosses the boundary as plain JS arrays and objects, so the frontend
 * never touches WASM heap pointers. Poses are returned both as a column-major
 * 16-element matrix (three.js `Matrix4.fromArray` order) and as a
 * position/quaternion pair, since the viewer wants the latter.
 *
 * Arms of different DOF are served by one `Robot` class: the kinematics stays
 * templated in C++, and `robotics::Robot` erases the joint count for this layer.
 */

#include <cstdint>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robotics/robot.hpp"

namespace {

using emscripten::val;
using robotics::Scalar;

/// Reads a numeric field from a JS options object, falling back when absent.
[[nodiscard]] Scalar read_number(const val& options, const char* key, Scalar fallback) {
    if (options.isUndefined() || options.isNull()) {
        return fallback;
    }
    const val value = options[key];
    return (value.isUndefined() || value.isNull()) ? fallback : value.as<Scalar>();
}

/// Reads a string field from a JS options object, falling back when absent.
[[nodiscard]] std::string read_string(const val& options, const char* key, const char* fallback) {
    if (options.isUndefined() || options.isNull()) {
        return fallback;
    }
    const val value = options[key];
    return (value.isUndefined() || value.isNull()) ? fallback : value.as<std::string>();
}

/// Converts a JS array of numbers into a runtime-sized joint vector.
[[nodiscard]] Eigen::VectorXf to_joints(const val& array, int dof) {
    Eigen::VectorXf joints = Eigen::VectorXf::Zero(dof);
    const std::vector<Scalar> values = emscripten::vecFromJSArray<Scalar>(array);
    const auto count = std::min<std::size_t>(static_cast<std::size_t>(dof), values.size());
    for (std::size_t i = 0; i < count; ++i) {
        joints(static_cast<Eigen::Index>(i)) = values[i];
    }
    return joints;
}

/// Converts an Eigen vector into a JS array.
[[nodiscard]] val from_vector(const Eigen::VectorXf& values) {
    val array = val::array();
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        array.set(static_cast<int>(i), values(i));
    }
    return array;
}

/// Converts a 3-vector into a JS array.
[[nodiscard]] val from_vector3(const robotics::Vector3& value) {
    val array = val::array();
    array.set(0, value.x());
    array.set(1, value.y());
    array.set(2, value.z());
    return array;
}

/// Packs a pose as `{ position, quaternion, matrix }` for direct use by three.js.
[[nodiscard]] val to_pose_object(const robotics::Pose& pose) {
    const Eigen::Quaternion<Scalar> rotation = pose.so3().unit_quaternion();

    // three.js quaternion ordering is (x, y, z, w).
    val quaternion = val::array();
    quaternion.set(0, rotation.x());
    quaternion.set(1, rotation.y());
    quaternion.set(2, rotation.z());
    quaternion.set(3, rotation.w());

    // Column-major, matching THREE.Matrix4.fromArray().
    const robotics::Matrix4 transform = pose.matrix();
    val matrix = val::array();
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            matrix.set(column * 4 + row, transform(row, column));
        }
    }

    val object = val::object();
    object.set("position", from_vector3(pose.translation()));
    object.set("quaternion", quaternion);
    object.set("matrix", matrix);
    return object;
}

/// Packs an ellipsoid as `{ quaternion, radii, volume, isotropy }` for three.js.
[[nodiscard]] val to_ellipsoid_object(const robotics::Ellipsoid& ellipsoid) {
    // The axes are orthonormal and right-handed, so they convert cleanly to a quaternion.
    const Eigen::Quaternion<Scalar> orientation{ellipsoid.axes};

    val quaternion = val::array();
    quaternion.set(0, orientation.x());
    quaternion.set(1, orientation.y());
    quaternion.set(2, orientation.z());
    quaternion.set(3, orientation.w());

    val radii = val::array();
    for (int i = 0; i < 3; ++i) {
        radii.set(i, ellipsoid.radii(i));
    }

    val object = val::object();
    object.set("quaternion", quaternion);
    object.set("radii", radii);
    object.set("volume", ellipsoid.volume);
    object.set("isotropy", ellipsoid.isotropy);
    return object;
}

/// Packs a collision contact as `{ distance, link, point, normal }`.
[[nodiscard]] val from_contact(const robotics::collision::Contact& contact) {
    val object = val::object();
    object.set("distance", contact.distance);
    object.set("link", contact.link);
    object.set("point", from_vector3(contact.point));
    object.set("normal", from_vector3(contact.normal));
    return object;
}

/// Reads a `[x, y, z]` JS array; missing entries stay zero.
[[nodiscard]] robotics::Vector3 to_vector3(const val& array) {
    const std::vector<Scalar> values = emscripten::vecFromJSArray<Scalar>(array);
    robotics::Vector3 vector = robotics::Vector3::Zero();
    for (std::size_t i = 0; i < 3 && i < values.size(); ++i) {
        vector(static_cast<Eigen::Index>(i)) = values[i];
    }
    return vector;
}

/// Rebuilds an obstacle world from `{ spheres: [{ center, radius }] }`; absent
/// or malformed fields simply contribute nothing, matching the empty world.
[[nodiscard]] robotics::collision::CollisionWorld to_world(const val& world) {
    robotics::collision::CollisionWorld result;
    if (world.isUndefined() || world.isNull()) {
        return result;
    }
    const val spheres = world["spheres"];
    if (spheres.isUndefined() || spheres.isNull()) {
        return result;
    }
    const auto count = spheres["length"].as<int>();
    for (int i = 0; i < count; ++i) {
        const val entry = spheres[i];
        result.spheres.push_back({to_vector3(entry["center"]), entry["radius"].as<Scalar>()});
    }
    return result;
}

/// Rebuilds a pose from a JS position array and a three.js-ordered quaternion array.
[[nodiscard]] robotics::Pose to_pose(const val& position, const val& quaternion) {
    const std::vector<Scalar> p = emscripten::vecFromJSArray<Scalar>(position);
    const std::vector<Scalar> q = emscripten::vecFromJSArray<Scalar>(quaternion);

    Eigen::Quaternion<Scalar> rotation = Eigen::Quaternion<Scalar>::Identity();
    if (q.size() >= 4) {
        rotation = Eigen::Quaternion<Scalar>{q[3], q[0], q[1], q[2]};  // (w, x, y, z) from (x, y, z, w)
        rotation.normalize();
    }

    robotics::Vector3 translation = robotics::Vector3::Zero();
    for (std::size_t i = 0; i < 3 && i < p.size(); ++i) {
        translation(static_cast<Eigen::Index>(i)) = p[i];
    }
    return robotics::Pose{Sophus::SO3<Scalar>{rotation}, translation};
}

/**
 * @brief JS-facing facade over any robotics::Robot.
 *
 * Instances come from the module-level `createRobot()` factory, which returns
 * null for an unknown identifier rather than throwing: the module is built with
 * exceptions disabled.
 */
class RobotHandle {
public:
    explicit RobotHandle(std::unique_ptr<robotics::Robot> model) : model_{std::move(model)} {}

    [[nodiscard]] std::string id() const { return std::string{model_->id()}; }
    [[nodiscard]] std::string label() const { return std::string{model_->label()}; }
    [[nodiscard]] int dof() const { return model_->dof(); }

    /// @return Joint names, base to tip.
    [[nodiscard]] val jointNames() const {
        val array = val::array();
        int index = 0;
        for (const auto& name : model_->joint_names()) {
            array.set(index++, name);
        }
        return array;
    }

    /// @return `[{ name, lower, upper }]` for every joint.
    [[nodiscard]] val jointLimits() const {
        val array = val::array();
        const auto names = model_->joint_names();
        for (int i = 0; i < model_->dof(); ++i) {
            val limit = val::object();
            limit.set("name", names[static_cast<std::size_t>(i)]);
            limit.set("lower", model_->lower_limits()(i));
            limit.set("upper", model_->upper_limits()(i));
            array.set(i, limit);
        }
        return array;
    }

    /// @return The configuration a viewer should open in.
    [[nodiscard]] val defaultConfiguration() const { return from_vector(model_->default_joints()); }

    /// @return `[{ label, angles }]`, all within the joint limits.
    [[nodiscard]] val presets() const {
        val array = val::array();
        int index = 0;
        for (const auto& preset : model_->presets()) {
            val entry = val::object();
            entry.set("label", preset.label);
            entry.set("angles", from_vector(preset.joints));
            array.set(index++, entry);
        }
        return array;
    }

    /// @return The end-effector pose for `angles`.
    [[nodiscard]] val forward(const val& angles) const { return to_pose_object(model_->forward(joints(angles))); }

    /// @return `{ frames: [pose x (dof + 1)], axes: [[x, y, z] x dof] }` for drawing the arm.
    [[nodiscard]] val linkFrames(const val& angles) const {
        const Eigen::VectorXf configuration = joints(angles);

        val frames = val::array();
        int index = 0;
        for (const auto& pose : model_->link_poses(configuration)) {
            frames.set(index++, to_pose_object(pose));
        }

        val axes = val::array();
        index = 0;
        for (const auto& axis : model_->joint_axes(configuration)) {
            axes.set(index++, from_vector3(axis));
        }

        val result = val::object();
        result.set("frames", frames);
        result.set("axes", axes);
        return result;
    }

    /// @return The 6 x dof space Jacobian, row-major, in `[linear; angular]` ordering.
    [[nodiscard]] val jacobian(const val& angles) const {
        const Eigen::MatrixXf matrix = model_->space_jacobian(joints(angles));

        val array = val::array();
        for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
            for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
                array.set(static_cast<int>(row * matrix.cols() + column), matrix(row, column));
            }
        }
        return array;
    }

    /// @return Yoshikawa manipulability; near zero means near-singular.
    [[nodiscard]] Scalar manipulability(const val& angles) const { return model_->manipulability(joints(angles)); }

    /// @return `{ linear, angular }`, each `{ quaternion, radii, volume, isotropy }`.
    [[nodiscard]] val manipulabilityEllipsoids(const val& angles) const {
        const auto ellipsoids = model_->ellipsoids(joints(angles));

        val result = val::object();
        result.set("linear", to_ellipsoid_object(ellipsoids.linear));
        result.set("angular", to_ellipsoid_object(ellipsoids.angular));
        return result;
    }

    /**
     * @brief The collision body posed at `angles`, plus the tightest self pair.
     *
     * Capsule endpoints are in the space frame, so the viewer can draw them
     * without touching link transforms. `selfContact.distance` is the signed
     * clearance in metres (negative = penetration, Infinity = nothing checked).
     *
     * @return `{ capsules: [{ start, end, radius, link }], selfContact: { distance, link, point, normal } }`.
     */
    [[nodiscard]] val collisionBody(const val& angles) const {
        const Eigen::VectorXf configuration = joints(angles);

        val capsules = val::array();
        int index = 0;
        for (const auto& capsule : model_->collision_capsules(configuration)) {
            val entry = val::object();
            entry.set("start", from_vector3(capsule.world.start));
            entry.set("end", from_vector3(capsule.world.end));
            entry.set("radius", capsule.world.radius);
            entry.set("link", capsule.link);
            capsules.set(index++, entry);
        }

        val result = val::object();
        result.set("capsules", capsules);
        result.set("selfContact", from_contact(model_->self_contact(configuration)));
        return result;
    }

    /**
     * @brief The tightest approach at `angles` — obstacles and self both.
     * @param world Optional `{ spheres: [{ center, radius }] }` in the space frame.
     */
    [[nodiscard]] val nearestContact(const val& angles, const val& world) const {
        return from_contact(model_->nearest_contact(joints(angles), to_world(world)));
    }

    /**
     * @brief Plans a collision-free joint path with RRT-Connect + shortcutting.
     *
     * @param start Start joint angles; must be legal and clear of the obstacles.
     * @param goal Goal joint angles, same requirements.
     * @param world Optional `{ spheres: [{ center, radius }] }`.
     * @param options Optional `{ maxIterations, step, resolution, margin, shortcutRounds, seed }`.
     * @return `{ status, path, rawPath, iterations, nodes, pathLength, rawLength }`;
     *         paths are arrays of joint vectors and lengths are radians of joint motion.
     */
    [[nodiscard]] val plan(const val& start, const val& goal, const val& world, const val& options) const {
        const robotics::planning::Options settings{
            .max_iterations = static_cast<int>(read_number(options, "maxIterations", 3000)),
            .step = read_number(options, "step", Scalar{0.2}),
            .resolution = read_number(options, "resolution", Scalar{0.05}),
            .margin = read_number(options, "margin", Scalar{0.01}),
            .shortcut_rounds = static_cast<int>(read_number(options, "shortcutRounds", 150)),
            .seed = static_cast<std::uint32_t>(read_number(options, "seed", 2026)),
        };

        const robotics::DynamicPlanResult result = model_->plan(joints(start), joints(goal), to_world(world), settings);

        const auto pack = [](const std::vector<Eigen::VectorXf>& path) {
            val array = val::array();
            int index = 0;
            for (const auto& waypoint : path) {
                array.set(index++, from_vector(waypoint));
            }
            return array;
        };
        const auto length = [](const std::vector<Eigen::VectorXf>& path) {
            Scalar total{0};
            for (std::size_t i = 1; i < path.size(); ++i) {
                total += (path[i] - path[i - 1]).norm();
            }
            return total;
        };

        const char* status = "not_found";
        switch (result.status) {
            case robotics::planning::Status::kSuccess:
                status = "success";
                break;
            case robotics::planning::Status::kStartInvalid:
                status = "start_invalid";
                break;
            case robotics::planning::Status::kGoalInvalid:
                status = "goal_invalid";
                break;
            case robotics::planning::Status::kNotFound:
                break;
        }

        val object = val::object();
        object.set("status", std::string{status});
        object.set("path", pack(result.path));
        object.set("rawPath", pack(result.raw_path));
        object.set("iterations", result.iterations);
        object.set("nodes", result.nodes);
        object.set("pathLength", length(result.path));
        object.set("rawLength", length(result.raw_path));
        return object;
    }

    /**
     * @brief Solves IK for a target pose.
     * @param initialGuess Seed joint angles; pass the current configuration when dragging.
     * @param position Target position `[x, y, z]`.
     * @param quaternion Target orientation `[x, y, z, w]`.
     * @param options Optional `{ method, maxIterations, positionTolerance, orientationTolerance, damping, maxStep }`.
     * @return `{ angles, converged, iterations, positionError, orientationError }`.
     */
    [[nodiscard]] val inverse(const val& initialGuess, const val& position, const val& quaternion, const val& options) const {
        // "dls" selects damped least squares with post-step clamping; anything
        // else, including the default, uses the box-constrained QP.
        const robotics::ik::Options settings{
            .method = read_string(options, "method", "qp") == "dls" ? robotics::ik::Method::DampedLeastSquares
                                                                    : robotics::ik::Method::BoxQp,
            .max_iterations = static_cast<int>(read_number(options, "maxIterations", 100)),
            .position_tolerance = read_number(options, "positionTolerance", Scalar{1e-4}),
            .orientation_tolerance = read_number(options, "orientationTolerance", Scalar{1e-4}),
            .damping = read_number(options, "damping", Scalar{1e-2}),
            .max_step = read_number(options, "maxStep", Scalar{0.2}),
        };

        const auto result = model_->inverse(to_pose(position, quaternion), joints(initialGuess), settings);

        val object = val::object();
        object.set("angles", from_vector(result.joints));
        object.set("converged", result.converged);
        object.set("iterations", result.iterations);
        object.set("positionError", result.position_error);
        object.set("orientationError", result.orientation_error);
        return object;
    }

private:
    [[nodiscard]] Eigen::VectorXf joints(const val& angles) const { return to_joints(angles, model_->dof()); }

    std::unique_ptr<robotics::Robot> model_;
};

/// @return `[{ id, label, dof }]` for every robot the module can build.
[[nodiscard]] val available_robots() {
    val array = val::array();
    int index = 0;
    for (const auto& description : robotics::robot_catalog()) {
        val entry = val::object();
        entry.set("id", std::string{description.id});
        entry.set("label", std::string{description.label});
        entry.set("dof", description.dof);
        array.set(index++, entry);
    }
    return array;
}

/**
 * @brief Builds a robot by identifier.
 * @param id One of the ids from `availableRobots()`.
 * @return A new robot, or null when the identifier is unknown. The caller owns it
 *         and must call `.delete()`.
 */
[[nodiscard]] RobotHandle* create_robot(const std::string& id) {
    auto model = robotics::make_robot(id);
    return model ? new RobotHandle{std::move(model)} : nullptr;
}

}  // namespace

EMSCRIPTEN_BINDINGS(kinematics_module) {
    emscripten::class_<RobotHandle>("Robot")
        .function("id", &RobotHandle::id)
        .function("label", &RobotHandle::label)
        .function("dof", &RobotHandle::dof)
        .function("jointNames", &RobotHandle::jointNames)
        .function("jointLimits", &RobotHandle::jointLimits)
        .function("defaultConfiguration", &RobotHandle::defaultConfiguration)
        .function("presets", &RobotHandle::presets)
        .function("forward", &RobotHandle::forward)
        .function("linkFrames", &RobotHandle::linkFrames)
        .function("jacobian", &RobotHandle::jacobian)
        .function("manipulability", &RobotHandle::manipulability)
        .function("manipulabilityEllipsoids", &RobotHandle::manipulabilityEllipsoids)
        .function("collisionBody", &RobotHandle::collisionBody)
        .function("nearestContact", &RobotHandle::nearestContact)
        .function("plan", &RobotHandle::plan)
        .function("inverse", &RobotHandle::inverse);

    emscripten::function("availableRobots", &available_robots);
    emscripten::function("createRobot", &create_robot, emscripten::allow_raw_pointers());
}
