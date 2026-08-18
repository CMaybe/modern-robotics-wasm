#include <cmath>
#include <gtest/gtest.h>

#include "robotics/kinematics/manipulability.hpp"
#include "robotics/models/fr3.hpp"
#include "robotics/models/ur5.hpp"
#include "test_helpers.hpp"

namespace {

using robotics::Matrix3;
using robotics::Pose;
using robotics::Scalar;
using robotics::Vector3;
using robotics::models::kFr3Dof;
using robotics::models::kUr5Dof;
using robotics::testing::joints;

}  // namespace

// ---------------------------------------------------------------------------
// UR5
// ---------------------------------------------------------------------------

TEST(Ur5, HomeConfigurationMatchesTheFlangeFrame) {
    const robotics::models::Ur5Dimensions dims;
    const auto chain = robotics::models::ur5(dims);
    const Pose pose = chain.forward(robotics::JointVector<kUr5Dof>::Zero());

    EXPECT_NEAR(pose.translation().x(), dims.upper_arm + dims.forearm, Scalar{1e-5});
    EXPECT_NEAR(pose.translation().y(), dims.shoulder_offset + dims.wrist_offset, Scalar{1e-5});
    EXPECT_NEAR(pose.translation().z(), dims.base_height - dims.wrist_drop, Scalar{1e-5});
}

TEST(Ur5, MatchesModernRoboticsExample) {
    const auto chain = robotics::models::ur5();
    const Pose pose = chain.forward(joints<kUr5Dof>({0, -robotics::kHalfPi, 0, 0, robotics::kHalfPi, 0}));

    // Modern Robotics, Example 4.5. The book prints three decimals and this model
    // uses the manufacturer's full-precision link lengths, so the comparison is
    // made at the precision the book actually states.
    EXPECT_NEAR(pose.translation().x(), Scalar{0.095}, Scalar{5e-4});
    EXPECT_NEAR(pose.translation().y(), Scalar{0.109}, Scalar{5e-4});
    EXPECT_NEAR(pose.translation().z(), Scalar{0.988}, Scalar{1e-3});

    Matrix3 expected;
    // clang-format off
    expected << 0, -1, 0,
                1,  0, 0,
                0,  0, 1;
    // clang-format on
    EXPECT_TRUE(pose.so3().matrix().isApprox(expected, Scalar{1e-3})) << pose.so3().matrix();
}

TEST(Ur5, AngularEllipsoidIsADiscAtTheHomeConfiguration) {
    const auto chain = robotics::models::ur5();
    const auto ellipsoids = robotics::manipulability_ellipsoids(chain, robotics::JointVector<kUr5Dof>::Zero());

    // At home every UR5 joint axis is +/-y or +/-z, so the reachable angular
    // velocities span a plane and the ellipsoid flattens into a disc exactly.
    EXPECT_NEAR(ellipsoids.angular.radii(2), Scalar{0}, Scalar{1e-5});
    EXPECT_NEAR(ellipsoids.angular.isotropy, Scalar{0}, Scalar{1e-5});
    EXPECT_NEAR(ellipsoids.angular.volume, Scalar{0}, Scalar{1e-5});
    EXPECT_GT(ellipsoids.angular.radii(1), Scalar{1});

    // The collapsed direction is the one no joint axis reaches: world x.
    EXPECT_NEAR(std::abs(ellipsoids.angular.axes.col(2).x()), Scalar{1}, Scalar{1e-4});
}

TEST(Ur5, WristSingularityShrinksTheEllipsoidAndTheManipulability) {
    const auto chain = robotics::models::ur5();
    const auto singular_joints = joints<kUr5Dof>({0, -0.6F, 0.9F, 0, 0, 0});
    const auto regular_joints = joints<kUr5Dof>({0, -0.6F, 0.9F, 0, 1.2F, 0});

    EXPECT_LT(robotics::manipulability(chain, singular_joints), robotics::manipulability(chain, regular_joints));
    EXPECT_LT(robotics::manipulability(chain, singular_joints), Scalar{1e-3});

    // Aligning axes 4 and 6 does not drop the angular block's rank -- the other
    // four axes still span 3D -- but it squashes the ellipsoid noticeably.
    const auto singular = robotics::manipulability_ellipsoids(chain, singular_joints).angular;
    const auto regular = robotics::manipulability_ellipsoids(chain, regular_joints).angular;
    EXPECT_LT(singular.isotropy, Scalar{0.5} * regular.isotropy);
    EXPECT_LT(singular.volume, Scalar{0.5} * regular.volume);
}

// ---------------------------------------------------------------------------
// FR3
// ---------------------------------------------------------------------------

TEST(Fr3, ZeroConfigurationMatchesTheDhChain) {
    const auto chain = robotics::models::fr3();
    const Pose pose = chain.forward(robotics::JointVector<kFr3Dof>::Zero());

    // Walking the DH table directly is an independent path to the same pose.
    Pose expected;
    for (const auto& row : robotics::models::kFr3DhTable) {
        expected = expected * robotics::modified_dh_transform(row, Scalar{0});
    }
    expected = expected * robotics::modified_dh_transform(robotics::models::kFr3FlangeRow, Scalar{0});

    robotics::testing::expect_pose_near(pose, expected, Scalar{1e-5});
    EXPECT_NEAR(pose.translation().x(), Scalar{0.088}, Scalar{1e-4});
    EXPECT_NEAR(pose.translation().y(), Scalar{0}, Scalar{1e-4});
    EXPECT_NEAR(pose.translation().z(), Scalar{0.926}, Scalar{1e-4});
}

TEST(Fr3, ReadyPoseMatchesFrankaDocumentation) {
    const auto chain = robotics::models::fr3();
    const Pose flange = chain.forward(robotics::models::fr3_ready());

    // Franka quotes the ready pose for the *gripper* frame as (0.307, 0, 0.487).
    // This model ends at the flange, so applying the Franka Hand's F_T_EE -- a
    // 0.1034 m offset along z plus a -45 degree rotation about z -- must reproduce it.
    const Pose hand{Sophus::SO3<Scalar>::rotZ(-robotics::kQuarterPi), Vector3{0, 0, Scalar{0.1034}}};
    const Pose gripper = flange * hand;

    EXPECT_NEAR(gripper.translation().x(), Scalar{0.307}, Scalar{1e-3});
    EXPECT_NEAR(gripper.translation().y(), Scalar{0}, Scalar{1e-3});
    EXPECT_NEAR(gripper.translation().z(), Scalar{0.487}, Scalar{1e-3});

    // The familiar "tool pointing straight down" orientation.
    Matrix3 expected;
    // clang-format off
    expected << 1,  0,  0,
                0, -1,  0,
                0,  0, -1;
    // clang-format on
    EXPECT_TRUE(gripper.so3().matrix().isApprox(expected, Scalar{1e-3})) << gripper.so3().matrix();
}

TEST(Fr3, ZeroConfigurationViolatesTheJointLimits) {
    const auto chain = robotics::models::fr3();

    // Joint 4 is capped below zero and joint 6 above it, so the all-zero pose is
    // unreachable on real hardware. This is why the FR3 default is the ready pose.
    EXPECT_LT(chain.upper_limits()(3), Scalar{0});
    EXPECT_GT(chain.lower_limits()(5), Scalar{0});

    EXPECT_FALSE(chain.within_limits(robotics::JointVector<kFr3Dof>::Zero()));
    EXPECT_TRUE(chain.within_limits(robotics::models::fr3_ready()));
}

TEST(Fr3, SpaceJacobianMatchesNumericalDifferentiation) {
    const auto chain = robotics::models::fr3();
    const auto angles = joints<kFr3Dof>({0.3F, -0.7F, 0.4F, -1.9F, 0.5F, 1.8F, 0.6F});

    const auto analytic = chain.space_jacobian(angles);
    const Pose base = chain.forward(angles);

    constexpr Scalar kEpsilon{1e-4};
    for (int i = 0; i < kFr3Dof; ++i) {
        auto perturbed = angles;
        perturbed(i) += kEpsilon;
        const robotics::Twist numeric = (chain.forward(perturbed) * base.inverse()).log() / kEpsilon;

        for (int row = 0; row < 6; ++row) {
            EXPECT_NEAR(analytic(row, i), numeric(row), Scalar{1e-2}) << "column " << i << ", row " << row;
        }
    }
}
