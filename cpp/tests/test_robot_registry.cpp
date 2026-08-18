#include <gtest/gtest.h>

#include "robotics/models/fr3.hpp"
#include "robotics/robot.hpp"

namespace {

using robotics::Scalar;

}  // namespace

TEST(RobotRegistry, BuildsEveryAdvertisedRobot) {
    ASSERT_FALSE(robotics::robot_catalog().empty());

    for (const auto& description : robotics::robot_catalog()) {
        const auto robot = robotics::make_robot(description.id);
        ASSERT_NE(robot, nullptr) << description.id;

        EXPECT_EQ(robot->id(), description.id);
        EXPECT_EQ(robot->label(), description.label);
        EXPECT_EQ(robot->dof(), description.dof);
        EXPECT_EQ(static_cast<int>(robot->joint_names().size()), robot->dof());
        EXPECT_EQ(robot->lower_limits().size(), robot->dof());
        EXPECT_EQ(robot->upper_limits().size(), robot->dof());
        EXPECT_EQ(robot->default_joints().size(), robot->dof());
        EXPECT_FALSE(robot->presets().empty());

        const auto in_limits = [&](const Eigen::VectorXf& joints) {
            for (int i = 0; i < robot->dof(); ++i) {
                if (joints(i) < robot->lower_limits()(i) || joints(i) > robot->upper_limits()(i)) {
                    return false;
                }
            }
            return true;
        };

        EXPECT_TRUE(in_limits(robot->default_joints())) << description.id << " default configuration";
        for (const auto& preset : robot->presets()) {
            ASSERT_EQ(preset.joints.size(), robot->dof()) << description.id << " preset " << preset.label;
            EXPECT_TRUE(in_limits(preset.joints)) << description.id << " preset " << preset.label;
        }
    }
}

TEST(RobotRegistry, RejectsUnknownIdentifiers) {
    EXPECT_EQ(robotics::make_robot("nope"), nullptr);
    EXPECT_EQ(robotics::make_robot(""), nullptr);
}

TEST(RobotRegistry, DynamicInterfaceAgreesWithTheTemplate) {
    const auto robot = robotics::make_robot("fr3");
    ASSERT_NE(robot, nullptr);

    const auto chain = robotics::models::fr3();
    const robotics::JointVector<robotics::models::kFr3Dof> fixed =
        (robotics::JointVector<robotics::models::kFr3Dof>() << 0.3F, -0.7F, 0.4F, -1.9F, 0.5F, 1.8F, 0.6F).finished();
    const Eigen::VectorXf dynamic = fixed;

    EXPECT_TRUE(robot->forward(dynamic).matrix().isApprox(chain.forward(fixed).matrix(), Scalar{1e-6}));
    EXPECT_EQ(robot->space_jacobian(dynamic).rows(), 6);
    EXPECT_EQ(robot->space_jacobian(dynamic).cols(), robotics::models::kFr3Dof);
    EXPECT_NEAR(robot->manipulability(dynamic), robotics::manipulability(chain, fixed), Scalar{1e-6});

    const auto poses = robot->link_poses(dynamic);
    EXPECT_EQ(static_cast<int>(poses.size()), robotics::models::kFr3Dof + 1);
    EXPECT_TRUE(poses.back().matrix().isApprox(chain.forward(fixed).matrix(), Scalar{1e-5}));
}

TEST(RobotRegistry, InverseKinematicsStaysLegalForEveryRobotAndMethod) {
    for (const auto& description : robotics::robot_catalog()) {
        const auto robot = robotics::make_robot(description.id);
        ASSERT_NE(robot, nullptr) << description.id;

        // Far outside the workspace: the solver must stop and stay legal.
        const robotics::Pose target{Sophus::SO3<Scalar>{}, robotics::Vector3{9, 0, 0}};

        for (const auto method : {robotics::ik::Method::BoxQp, robotics::ik::Method::DampedLeastSquares}) {
            const auto result = robot->inverse(target, robot->default_joints(), {.method = method, .max_iterations = 40});

            EXPECT_FALSE(result.converged) << description.id;
            EXPECT_LE(result.iterations, 40) << description.id;
            ASSERT_EQ(result.joints.size(), robot->dof()) << description.id;
            for (int i = 0; i < robot->dof(); ++i) {
                EXPECT_TRUE(std::isfinite(result.joints(i))) << description.id << " joint " << i;
                EXPECT_GE(result.joints(i), robot->lower_limits()(i) - Scalar{1e-4}) << description.id;
                EXPECT_LE(result.joints(i), robot->upper_limits()(i) + Scalar{1e-4}) << description.id;
            }
        }
    }
}
