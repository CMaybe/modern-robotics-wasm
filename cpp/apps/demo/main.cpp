/**
 * @file main.cpp
 * @brief Native walkthrough of the library: FK, link frames, manipulability and IK.
 *
 * This runs the same code the browser does, so the kinematics can be stepped
 * through in gdb instead of across the WebAssembly boundary.
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "robotics/robot.hpp"

namespace {

void print_pose(std::string_view label, const robotics::Pose& pose) {
    std::cout << label << '\n' << std::fixed << std::setprecision(4) << pose.matrix() << "\n\n";
}

void print_joints(std::string_view label, const Eigen::VectorXf& joints) {
    std::cout << label << " [" << std::fixed << std::setprecision(4);
    for (Eigen::Index i = 0; i < joints.size(); ++i) {
        std::cout << joints(i) << (i + 1 < joints.size() ? ", " : "");
    }
    std::cout << "]\n";
}

int report(const robotics::Robot& robot) {
    std::cout << "=== " << robot.label() << " (" << robot.dof() << " DOF) ===\n\n";

    const Eigen::VectorXf home = Eigen::VectorXf::Zero(robot.dof());
    print_pose("FK at the zero configuration:", robot.forward(home));

    const Eigen::VectorXf ready = robot.default_joints();
    print_joints("Default configuration:", ready);
    print_pose("FK there:", robot.forward(ready));

    std::cout << "Manipulability: " << robot.manipulability(ready) << '\n';
    const auto ellipsoids = robot.ellipsoids(ready);
    std::cout << "Linear ellipsoid semi-axes:  " << ellipsoids.linear.radii.transpose() << "  (isotropy "
              << ellipsoids.linear.isotropy << ")\n"
              << "Angular ellipsoid semi-axes: " << ellipsoids.angular.radii.transpose() << "  (isotropy "
              << ellipsoids.angular.isotropy << ")\n\n";

    std::cout << "Link frame origins (base to tip, end-effector last):\n";
    const auto poses = robot.link_poses(ready);
    for (std::size_t i = 0; i < poses.size(); ++i) {
        std::cout << "  [" << i << "] " << poses[i].translation().transpose() << '\n';
    }
    std::cout << '\n';

    // Round trip: perturb the seed, then recover a pose we know is reachable.
    const robotics::Pose target = robot.forward(ready);
    Eigen::VectorXf seed = ready;
    for (Eigen::Index i = 0; i < seed.size(); ++i) {
        seed(i) += (i % 2 == 0) ? 0.20F : -0.15F;
    }

    int failures = 0;
    for (const auto method : {robotics::ik::Method::BoxQp, robotics::ik::Method::DampedLeastSquares}) {
        const robotics::ik::Options options{.method = method};

        const auto start = std::chrono::steady_clock::now();
        const auto result = robot.inverse(target, seed, options);
        const auto elapsed = std::chrono::steady_clock::now() - start;

        const auto* name = method == robotics::ik::Method::BoxQp ? "box QP     " : "DLS + clamp";
        std::cout << "IK (" << name << "): converged=" << std::boolalpha << result.converged
                  << " iterations=" << result.iterations << " position error=" << result.position_error << " m"
                  << " time=" << std::chrono::duration<double, std::milli>(elapsed).count() << " ms\n";
        print_joints("  solution:", result.joints);

        failures += result.converged ? 0 : 1;
    }
    std::cout << '\n';
    return failures;
}

}  // namespace

int main() {
    int failures = 0;
    for (const auto& description : robotics::robot_catalog()) {
        const auto robot = robotics::make_robot(description.id);
        if (!robot) {
            std::cerr << "unknown robot: " << description.id << '\n';
            return 1;
        }
        failures += report(*robot);
    }
    return failures == 0 ? 0 : 1;
}
