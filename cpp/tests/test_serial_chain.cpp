#include <gtest/gtest.h>

#include "robotics/kinematics/manipulability.hpp"
#include "robotics/kinematics/serial_chain.hpp"
#include "robotics/models/ur5.hpp"
#include "test_helpers.hpp"

namespace {

using robotics::Pose;
using robotics::Scalar;
using robotics::Vector3;
using robotics::models::kUr5Dof;
using robotics::testing::joints;

using Ur5Joints = robotics::JointVector<kUr5Dof>;

}  // namespace

TEST(ScrewAxis, RevoluteMatchesTheClosedForm) {
    const Vector3 axis{0, 1, 0};
    const Vector3 point{Scalar{0.4}, Scalar{-0.2}, Scalar{0.9}};

    const auto screw = robotics::ScrewAxis::revolute(axis, point);

    EXPECT_TRUE(screw.angular.isApprox(axis));
    EXPECT_TRUE(screw.linear.isApprox(-axis.cross(point)));

    // The Sophus twist swaps the two halves.
    const auto twist = screw.to_twist();
    EXPECT_TRUE(twist.head<3>().isApprox(screw.linear));
    EXPECT_TRUE(twist.tail<3>().isApprox(screw.angular));
}

TEST(ScrewAxis, RevoluteIsInvariantToThePointChosenOnTheAxis) {
    const Vector3 axis{0, 0, 1};
    const Vector3 point{Scalar{0.3}, Scalar{0.1}, Scalar{0.0}};

    const auto at_point = robotics::ScrewAxis::revolute(axis, point);
    const auto shifted = robotics::ScrewAxis::revolute(axis, point + Scalar{2.5} * axis);

    EXPECT_TRUE(at_point.linear.isApprox(shifted.linear, Scalar{1e-5}));
}

TEST(SerialChain, ZeroConfigurationIsTheEndEffectorHome) {
    const auto chain = robotics::models::ur5();
    robotics::testing::expect_pose_near(chain.forward(Ur5Joints::Zero()), chain.end_effector_home(), Scalar{1e-5});
}

TEST(SerialChain, LinkPosesEndWithTheEndEffector) {
    const auto chain = robotics::models::ur5();
    const auto angles = joints<kUr5Dof>({0.3F, -0.7F, 1.1F, 0.2F, -0.5F, 0.9F});

    const auto poses = chain.link_poses(angles);

    ASSERT_EQ(poses.size(), static_cast<std::size_t>(kUr5Dof) + 1);
    robotics::testing::expect_pose_near(poses.back(), chain.forward(angles), Scalar{1e-5});

    // Joint 0's axis passes through the base, so its frame never leaves the origin.
    EXPECT_NEAR(poses.front().translation().norm(), Scalar{0}, Scalar{1e-5});
}

TEST(SerialChain, JointAxesAreUnitAndIndependentOfTheirOwnAngle) {
    const auto chain = robotics::models::ur5();
    const auto base = joints<kUr5Dof>({0.4F, -0.9F, 0.8F, 0.35F, -0.6F, 0.2F});

    const auto axes = chain.joint_axes(base);
    ASSERT_EQ(axes.size(), static_cast<std::size_t>(kUr5Dof));
    for (const auto& axis : axes) {
        EXPECT_NEAR(axis.norm(), Scalar{1}, Scalar{1e-4});
    }

    // Turning joint i cannot move axis i, only the axes after it.
    for (int i = 0; i < kUr5Dof; ++i) {
        Ur5Joints turned = base;
        turned(i) += Scalar{0.7};
        const auto rotated = chain.joint_axes(turned);
        for (int j = 0; j <= i; ++j) {
            EXPECT_TRUE(rotated[static_cast<std::size_t>(j)].isApprox(axes[static_cast<std::size_t>(j)], Scalar{1e-4}))
                << "axis " << j << " moved when joint " << i << " turned";
        }
    }
}

TEST(SerialChain, SpaceJacobianMatchesNumericalDifferentiation) {
    const auto chain = robotics::models::ur5();
    const auto angles = joints<kUr5Dof>({0.4F, -0.9F, 0.8F, 0.35F, -0.6F, 0.2F});

    const auto analytic = chain.space_jacobian(angles);
    const Pose base = chain.forward(angles);

    constexpr Scalar kEpsilon{1e-4};
    for (int i = 0; i < kUr5Dof; ++i) {
        Ur5Joints perturbed = angles;
        perturbed(i) += kEpsilon;

        // Space Jacobian column i is the twist joint i generates: [J_i] = (dT/dtheta_i) T^-1.
        const robotics::Twist numeric = (chain.forward(perturbed) * base.inverse()).log() / kEpsilon;

        for (int row = 0; row < 6; ++row) {
            EXPECT_NEAR(analytic(row, i), numeric(row), Scalar{1e-2}) << "column " << i << ", row " << row;
        }
    }
}

TEST(SerialChain, ClampAndLimitQueriesAgree) {
    const auto chain = robotics::models::ur5();
    const Ur5Joints far_outside = Ur5Joints::Constant(Scalar{100});

    EXPECT_FALSE(chain.within_limits(far_outside));
    const Ur5Joints clamped = chain.clamp_to_limits(far_outside);
    EXPECT_TRUE(chain.within_limits(clamped));
    EXPECT_TRUE(clamped.isApprox(chain.upper_limits()));
}

TEST(Manipulability, EllipsoidAxesFormARightHandedRotation) {
    const auto chain = robotics::models::ur5();
    const auto ellipsoids = robotics::manipulability_ellipsoids(chain, joints<kUr5Dof>({0.4F, -0.9F, 0.8F, 0.35F, -0.6F, 0.2F}));

    for (const auto* ellipsoid : {&ellipsoids.linear, &ellipsoids.angular}) {
        const robotics::Matrix3 identity = ellipsoid->axes.transpose() * ellipsoid->axes;
        EXPECT_TRUE(identity.isApprox(robotics::Matrix3::Identity(), Scalar{1e-4})) << ellipsoid->axes;
        EXPECT_NEAR(ellipsoid->axes.determinant(), Scalar{1}, Scalar{1e-4});

        EXPECT_GE(ellipsoid->radii(0), ellipsoid->radii(1));
        EXPECT_GE(ellipsoid->radii(1), ellipsoid->radii(2));
        EXPECT_GE(ellipsoid->radii(2), Scalar{0});
        EXPECT_LE(ellipsoid->isotropy, Scalar{1} + Scalar{1e-5});
    }
}

TEST(Manipulability, LinearEllipsoidBoundsSampledToolVelocities) {
    const auto chain = robotics::models::ur5();
    const auto angles = joints<kUr5Dof>({0.3F, -1.0F, 0.7F, 0.25F, -0.8F, 0.4F});
    const auto ellipsoid = robotics::manipulability_ellipsoids(chain, angles).linear;

    // Independent check: finite-difference the tool for unit joint-velocity
    // directions and confirm each sampled speed lands inside the ellipsoid.
    const Vector3 origin = chain.forward(angles).translation();
    constexpr Scalar kEpsilon{1e-4};

    const std::array<Ur5Joints, 5> directions{
        joints<kUr5Dof>({1, 0, 0, 0, 0, 0}),
        joints<kUr5Dof>({0, 1, 0, 0, 0, 0}),
        joints<kUr5Dof>({0.5F, -0.5F, 0.5F, -0.5F, 0, 0}),
        joints<kUr5Dof>({0.4F, 0.4F, 0.4F, 0.4F, 0.4F, 0.4F}),
        joints<kUr5Dof>({-0.3F, 0.6F, 0, 0.2F, -0.7F, 0.1F}),
    };

    for (const auto& raw : directions) {
        const Ur5Joints direction = raw.normalized();
        const Vector3 velocity = (chain.forward((angles + kEpsilon * direction).eval()).translation() - origin) / kEpsilon;

        // sum((x_i / r_i)^2) <= 1, since the ellipsoid is the image of the unit ball.
        const Vector3 local = ellipsoid.axes.transpose() * velocity;
        Scalar squared_norm{0};
        for (int i = 0; i < 3; ++i) {
            if (ellipsoid.radii(i) > Scalar{1e-6}) {
                squared_norm += (local(i) / ellipsoid.radii(i)) * (local(i) / ellipsoid.radii(i));
            }
        }
        EXPECT_LE(squared_norm, Scalar{1} + Scalar{1e-2}) << "direction " << direction.transpose();
    }

    EXPECT_GT(ellipsoid.radii(0), Scalar{0.1});
}
