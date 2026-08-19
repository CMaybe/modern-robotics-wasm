#include <array>
#include <cmath>
#include <gtest/gtest.h>

#include "robotics/collision/world.hpp"
#include "robotics/models/fr3.hpp"
#include "robotics/models/ur5.hpp"
#include "test_helpers.hpp"

namespace {

using robotics::JointVector;
using robotics::kHalfPi;
using robotics::kPi;
using robotics::Scalar;
using robotics::Vector3;
using robotics::models::kFr3Dof;
using robotics::models::kUr5Dof;
using robotics::testing::joints;

namespace collision = robotics::collision;

constexpr Scalar kTolerance = Scalar{1e-5};

}  // namespace

// ---------------------------------------------------------------------------
// Shape primitives
// ---------------------------------------------------------------------------

TEST(CollisionShapes, ClosestPointOnSegmentClampsToTheEndpoints) {
    const Vector3 a{0, 0, 0};
    const Vector3 b{1, 0, 0};

    EXPECT_TRUE(collision::closest_point_on_segment(a, b, Vector3{0.25F, 1, 0}).isApprox(Vector3{0.25F, 0, 0}, kTolerance));
    EXPECT_TRUE(collision::closest_point_on_segment(a, b, Vector3{-2, 1, 0}).isApprox(a, kTolerance));
    EXPECT_TRUE(collision::closest_point_on_segment(a, b, Vector3{5, -3, 2}).isApprox(b, kTolerance));
}

TEST(CollisionShapes, CrossingSegmentsMeetAtTheCrossing) {
    const auto pair =
        collision::closest_points_between_segments(Vector3{-1, 0, 0}, Vector3{1, 0, 0}, Vector3{0, -1, 1}, Vector3{0, 1, 1});

    EXPECT_TRUE(pair.on_a.isApprox(Vector3{0, 0, 0}, kTolerance)) << pair.on_a;
    EXPECT_TRUE(pair.on_b.isApprox(Vector3{0, 0, 1}, kTolerance)) << pair.on_b;
}

TEST(CollisionShapes, ParallelSegmentsReportTheirTrueDistance) {
    const auto pair =
        collision::closest_points_between_segments(Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0.5F, 2, 0}, Vector3{1.5F, 2, 0});

    EXPECT_NEAR((pair.on_a - pair.on_b).norm(), Scalar{2}, kTolerance);
}

TEST(CollisionShapes, DegenerateSegmentsActAsPoints) {
    const Vector3 point{0, 3, 0};
    const auto pair = collision::closest_points_between_segments(point, point, Vector3{-1, 0, 0}, Vector3{1, 0, 0});

    EXPECT_TRUE(pair.on_a.isApprox(point, kTolerance));
    EXPECT_TRUE(pair.on_b.isApprox(Vector3{0, 0, 0}, kTolerance)) << pair.on_b;
}

TEST(CollisionShapes, CapsuleSphereSeparationIsExact) {
    const collision::Capsule capsule{Vector3{0, 0, 0}, Vector3{0, 0, 1}, Scalar{0.1}};
    const collision::Sphere sphere{Vector3{0.5F, 0, 0.5F}, Scalar{0.1}};

    const auto result = collision::signed_distance(capsule, sphere);
    EXPECT_NEAR(result.distance, Scalar{0.3}, kTolerance);
    EXPECT_TRUE(result.point.isApprox(Vector3{0, 0, 0.5F}, kTolerance)) << result.point;
    EXPECT_TRUE(result.normal.isApprox(Vector3{-1, 0, 0}, kTolerance)) << result.normal;
}

TEST(CollisionShapes, PenetrationIsNegativeByTheOverlapDepth) {
    const collision::Capsule capsule{Vector3{0, 0, 0}, Vector3{0, 0, 1}, Scalar{0.1}};
    const collision::Sphere sphere{Vector3{0.15F, 0, 0.5F}, Scalar{0.1}};

    EXPECT_NEAR(collision::signed_distance(capsule, sphere).distance, Scalar{-0.05}, kTolerance);
}

TEST(CollisionShapes, CapsuleCapsuleMatchesTheAxisDistance) {
    const collision::Capsule first{Vector3{-1, 0, 0}, Vector3{1, 0, 0}, Scalar{0.2}};
    const collision::Capsule second{Vector3{0, -1, 1}, Vector3{0, 1, 1}, Scalar{0.3}};

    EXPECT_NEAR(collision::signed_distance(first, second).distance, Scalar{0.5}, kTolerance);
}

TEST(CollisionShapes, NormalIsTheGradientOfTheDistance) {
    const collision::Capsule capsule{Vector3{0, 0, 0}, Vector3{0, 0, 1}, Scalar{0.1}};
    const collision::Sphere sphere{Vector3{0.4F, 0.3F, 0.7F}, Scalar{0.05}};

    const auto at = collision::signed_distance(capsule, sphere);
    // Shift the capsule along the reported normal; the distance must grow by the
    // same amount, which is what an optimisation-based planner relies on.
    constexpr Scalar kStep = Scalar{0.01};
    const Vector3 shift = kStep * at.normal;
    const collision::Capsule moved{capsule.start + shift, capsule.end + shift, capsule.radius};

    EXPECT_NEAR(collision::signed_distance(moved, sphere).distance, at.distance + kStep, Scalar{1e-4});
}

// ---------------------------------------------------------------------------
// Robot geometry
// ---------------------------------------------------------------------------

TEST(RobotGeometry, Ur5SkipsTheZeroLengthSpanAndCoversEveryLink) {
    const auto geometry = robotics::models::ur5_collision();

    // The base-to-joint-0 span has zero length, leaving one capsule per link.
    ASSERT_EQ(geometry.capsules.size(), 6U);
    for (std::size_t i = 0; i < geometry.capsules.size(); ++i) {
        EXPECT_EQ(geometry.capsules[i].segment, static_cast<int>(i));
        EXPECT_EQ(geometry.capsules[i].link, static_cast<int>(i));
    }
}

TEST(RobotGeometry, PosedCapsulesFollowTheLinks) {
    const robotics::models::Ur5Dimensions dims;
    const auto chain = robotics::models::ur5(dims);
    const auto geometry = robotics::models::ur5_collision(chain);

    // Rotating the shoulder pan by 90 degrees swings the upper arm from +x to +y.
    const auto posed = collision::pose_geometry(geometry, chain.link_poses(joints<kUr5Dof>({kHalfPi, 0, 0, 0, 0, 0})));
    const Vector3 expected{0, dims.upper_arm, dims.base_height};

    EXPECT_TRUE(posed[1].world.end.isApprox(expected, Scalar{1e-4})) << posed[1].world.end;
}

TEST(RobotGeometry, AdjacentSegmentsAreExcludedFromSelfChecks) {
    const auto geometry = robotics::models::ur5_collision();

    EXPECT_FALSE(geometry.self_pair_enabled(geometry.capsules[0], geometry.capsules[1]));
    EXPECT_TRUE(geometry.self_pair_enabled(geometry.capsules[0], geometry.capsules[2]));
}

// ---------------------------------------------------------------------------
// World queries
// ---------------------------------------------------------------------------

TEST(CollisionWorld, AnEmptyWorldReportsInfiniteClearance) {
    const auto posed = collision::pose_geometry(robotics::models::ur5_collision(),
                                                robotics::models::ur5().link_poses(JointVector<kUr5Dof>::Zero()));

    EXPECT_TRUE(std::isinf(collision::obstacle_contact(posed, {}).distance));
}

TEST(CollisionWorld, ASphereAtTheFlangeCollidesWithTheLastLink) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto home = JointVector<kUr5Dof>::Zero();

    collision::CollisionWorld world;
    world.spheres.push_back({chain.forward(home).translation(), Scalar{0.05}});

    const auto contact = collision::nearest_contact(chain, geometry, home, world);
    EXPECT_LT(contact.distance, Scalar{0});
    EXPECT_EQ(contact.link, kUr5Dof - 1);
    EXPECT_TRUE(collision::in_collision(chain, geometry, home, world));
}

TEST(CollisionWorld, ClearanceShrinksAsTheObstacleApproaches) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto home = JointVector<kUr5Dof>::Zero();

    collision::CollisionWorld far_world;
    far_world.spheres.push_back({Vector3{0, 0, 2}, Scalar{0.1}});
    collision::CollisionWorld near_world;
    near_world.spheres.push_back({Vector3{0, 0, 1}, Scalar{0.1}});

    const auto posed = collision::pose_geometry(geometry, chain.link_poses(home));
    const Scalar far_clearance = collision::obstacle_contact(posed, far_world).distance;
    const Scalar near_clearance = collision::obstacle_contact(posed, near_world).distance;

    EXPECT_GT(far_clearance, near_clearance);
    EXPECT_GT(near_clearance, Scalar{0});
}

TEST(CollisionWorld, TheMarginTurnsANearMissIntoACollision) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto home = JointVector<kUr5Dof>::Zero();

    // 2 cm above the upper-arm capsule surface. The margin must stay below the
    // arm's own tightest self-clearance (~5 cm at home) to isolate the obstacle.
    collision::CollisionWorld world;
    world.spheres.push_back({Vector3{0.2F, 0, chain.joints()[1].link_home.translation().z() + Scalar{0.08}}, Scalar{0}});

    EXPECT_FALSE(collision::in_collision(chain, geometry, home, world));
    EXPECT_TRUE(collision::in_collision(chain, geometry, home, world, Scalar{0.04}));
}

TEST(CollisionWorld, CapsuleObstaclesAreCheckedToo) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto home = JointVector<kUr5Dof>::Zero();

    // A horizontal bar crossing the arm at the elbow height.
    collision::CollisionWorld world;
    const Scalar height = chain.joints()[1].link_home.translation().z();
    world.capsules.push_back({Vector3{0.4F, -1, height}, Vector3{0.4F, 1, height}, Scalar{0.05}});

    EXPECT_TRUE(collision::in_collision(chain, geometry, home, world));
}

// ---------------------------------------------------------------------------
// Robot models
// ---------------------------------------------------------------------------

TEST(Ur5Collision, ThePresetConfigurationsAreSelfCollisionFree) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    const std::array<JointVector<kUr5Dof>, 4> presets{joints<kUr5Dof>({0.4F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F}),
                                                      JointVector<kUr5Dof>::Zero(),
                                                      joints<kUr5Dof>({0, -kHalfPi, 0, 0, kHalfPi, 0}),
                                                      joints<kUr5Dof>({0, -0.6F, 0.9F, 0, 0, 0})};
    for (const auto& configuration : presets) {
        const auto posed = collision::pose_geometry(geometry, chain.link_poses(configuration));
        EXPECT_GT(collision::self_contact(geometry, posed).distance, Scalar{0.02}) << configuration.transpose();
    }
}

TEST(Ur5Collision, FoldingTheElbowOntoTheBaseIsDetected) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    // Elbow at pi folds the forearm back over the upper arm; the wrist ends up
    // inside the base column.
    const auto folded = joints<kUr5Dof>({0, 0, kPi, 0, 0, 0});
    const auto posed = collision::pose_geometry(geometry, chain.link_poses(folded));

    EXPECT_LT(collision::self_contact(geometry, posed).distance, Scalar{0});
    EXPECT_TRUE(collision::in_collision(chain, geometry, folded, {}));
}

TEST(Fr3Collision, ThePresetConfigurationsAreSelfCollisionFree) {
    const auto chain = robotics::models::fr3();
    const auto geometry = robotics::models::fr3_collision(chain);

    const std::array<JointVector<kFr3Dof>, 4> presets{robotics::models::fr3_ready(),
                                                      joints<kFr3Dof>({0, -1.0F, 0, -2.6F, 0, 1.6F, 0.8F}),
                                                      joints<kFr3Dof>({1.2F, 0.6F, -0.5F, -1.8F, 0.4F, 1.9F, 0.5F}),
                                                      joints<kFr3Dof>({0, 0, 0, -0.1518F, 0, kHalfPi, 0})};
    for (const auto& configuration : presets) {
        const auto posed = collision::pose_geometry(geometry, chain.link_poses(configuration));
        EXPECT_GT(collision::self_contact(geometry, posed).distance, Scalar{0.02}) << configuration.transpose();
    }
}

TEST(Fr3Collision, ADeepElbowFoldBringsTheFlangeIntoTheBaseColumn) {
    const auto chain = robotics::models::fr3();
    const auto geometry = robotics::models::fr3_collision(chain);

    const auto folded = joints<kFr3Dof>({0, 0, 0, -3.0F, 0, kHalfPi, 0});
    const auto contact = collision::self_contact(geometry, collision::pose_geometry(geometry, chain.link_poses(folded)));

    EXPECT_LT(contact.distance, Scalar{0});
    EXPECT_EQ(contact.link, 0);  // The base column is one side of the pair.
}
