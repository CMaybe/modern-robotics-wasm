#include <gtest/gtest.h>

#include "robotics/models/fr3.hpp"
#include "robotics/models/ur5.hpp"
#include "robotics/planning/rrt_connect.hpp"
#include "test_helpers.hpp"

namespace {

using robotics::JointVector;
using robotics::kHalfPi;
using robotics::Scalar;
using robotics::Vector3;
using robotics::models::kFr3Dof;
using robotics::models::kUr5Dof;
using robotics::testing::joints;

namespace collision = robotics::collision;
namespace planning = robotics::planning;

/// A sphere blocking the pan sweep between mirrored UR5 poses: probing showed
/// both endpoints keep 49 mm of clearance while the sweep penetrates it 157 mm.
[[nodiscard]] collision::CollisionWorld ur5_wall() {
    collision::CollisionWorld world;
    world.spheres.push_back({Vector3{0.55F, 0.0F, 0.35F}, Scalar{0.15}});
    return world;
}

/// Every consecutive pair of waypoints must be a valid motion on its own.
template <int Dof>
void expect_path_is_valid(const robotics::SerialChain<Dof>& chain,
                          const collision::RobotGeometry& geometry,
                          const collision::CollisionWorld& world,
                          const std::vector<JointVector<Dof>>& path,
                          const planning::Options& options) {
    ASSERT_GE(path.size(), 2U);
    for (std::size_t i = 1; i < path.size(); ++i) {
        EXPECT_TRUE(chain.within_limits(path[i - 1]));
        EXPECT_TRUE(planning::motion_is_free(chain, geometry, world, path[i - 1], path[i], options.resolution, options.margin))
            << "segment " << i << " collides";
    }
}

}  // namespace

TEST(MotionCheck, AStraightSegmentThroughAnObstacleIsRejected) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    // Swinging the pan joint from Ready to its mirror sweeps the arm through the wall.
    const auto from = joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});
    const auto to = joints<kUr5Dof>({-0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});

    EXPECT_TRUE(planning::motion_is_free(chain, geometry, {}, from, to, Scalar{0.05}, Scalar{0.01}));
    EXPECT_FALSE(planning::motion_is_free(chain, geometry, ur5_wall(), from, to, Scalar{0.05}, Scalar{0.01}));
}

TEST(RrtConnect, FreeSpaceKeepsTheEndpointsAndStaysLegal) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const planning::Options options{};

    const auto start = joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});
    const auto goal = joints<kUr5Dof>({-0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});

    const auto result = planning::plan_rrt_connect(chain, geometry, {}, start, goal, options);
    ASSERT_TRUE(result.success());
    EXPECT_TRUE(result.path.front().isApprox(start, Scalar{1e-5}));
    EXPECT_TRUE(result.path.back().isApprox(goal, Scalar{1e-5}));
    expect_path_is_valid(chain, geometry, {}, result.path, options);
}

TEST(RrtConnect, PlansAroundAnObstacleTheStraightLineHits) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto world = ur5_wall();
    const planning::Options options{};

    const auto start = joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});
    const auto goal = joints<kUr5Dof>({-0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});

    const auto result = planning::plan_rrt_connect(chain, geometry, world, start, goal, options);
    ASSERT_TRUE(result.success()) << "iterations=" << result.iterations << " nodes=" << result.nodes;
    EXPECT_TRUE(result.path.front().isApprox(start, Scalar{1e-5}));
    EXPECT_TRUE(result.path.back().isApprox(goal, Scalar{1e-5}));
    expect_path_is_valid(chain, geometry, world, result.path, options);
    expect_path_is_valid(chain, geometry, world, result.raw_path, options);

    // The straight line is blocked, so any valid path must be longer than it.
    EXPECT_GT(planning::path_length(result.path), (goal - start).norm());
}

TEST(RrtConnect, ShortcuttingNeverLengthensThePath) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    const auto start = joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});
    const auto goal = joints<kUr5Dof>({-0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});

    const auto result = planning::plan_rrt_connect(chain, geometry, ur5_wall(), start, goal, {});
    ASSERT_TRUE(result.success());
    EXPECT_LE(planning::path_length(result.path), planning::path_length(result.raw_path) + Scalar{1e-4});
    EXPECT_LE(result.path.size(), result.raw_path.size());
}

TEST(RrtConnect, TheSameSeedReproducesThePath) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto world = ur5_wall();

    const auto start = joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});
    const auto goal = joints<kUr5Dof>({-0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});

    const auto first = planning::plan_rrt_connect(chain, geometry, world, start, goal, {});
    const auto second = planning::plan_rrt_connect(chain, geometry, world, start, goal, {});
    ASSERT_TRUE(first.success());
    ASSERT_EQ(first.path.size(), second.path.size());
    for (std::size_t i = 0; i < first.path.size(); ++i) {
        EXPECT_TRUE(first.path[i].isApprox(second.path[i], Scalar{1e-6}));
    }
}

TEST(RrtConnect, AnInvalidStartOrGoalFailsFast) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    const auto legal = joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});
    const auto folded = joints<kUr5Dof>({0, 0, robotics::kPi, 0, 0, 0});  // Self-collides.

    const auto from_bad = planning::plan_rrt_connect(chain, geometry, {}, folded, legal, {});
    EXPECT_EQ(from_bad.status, planning::Status::kStartInvalid);
    EXPECT_TRUE(from_bad.path.empty());

    const auto to_bad = planning::plan_rrt_connect(chain, geometry, {}, legal, folded, {});
    EXPECT_EQ(to_bad.status, planning::Status::kGoalInvalid);

    // A goal buried inside an obstacle is just as invalid as one out of limits.
    collision::CollisionWorld world;
    world.spheres.push_back({chain.forward(legal).translation(), Scalar{0.1}});
    const auto buried = planning::plan_rrt_connect(chain, geometry, world, legal, legal, {});
    EXPECT_EQ(buried.status, planning::Status::kStartInvalid);
}

TEST(RrtConnect, AnExhaustedBudgetReportsNotFound) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    const auto start = joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});
    const auto goal = joints<kUr5Dof>({-0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F});

    planning::Options options;
    options.max_iterations = 1;  // The wall cannot be rounded in one extend.
    const auto result = planning::plan_rrt_connect(chain, geometry, ur5_wall(), start, goal, options);
    EXPECT_EQ(result.status, planning::Status::kNotFound);
    EXPECT_TRUE(result.path.empty());
    EXPECT_LE(result.iterations, 1);
}

TEST(RrtConnect, PlansForTheFr3BetweenPresets) {
    const auto chain = robotics::models::fr3();
    const auto geometry = robotics::models::fr3_collision(chain);
    const planning::Options options{};

    // Probed like the UR5 wall: Ready and Side reach clear this sphere by more
    // than 0.1 m each, while the straight-line sweep penetrates it 46 mm.
    collision::CollisionWorld world;
    world.spheres.push_back({Vector3{0.45F, 0.1F, 0.4F}, Scalar{0.1}});

    const auto start = robotics::models::fr3_ready();
    const auto goal = joints<kFr3Dof>({1.2F, 0.6F, -0.5F, -1.8F, 0.4F, 1.9F, 0.5F});

    const auto result = planning::plan_rrt_connect(chain, geometry, world, start, goal, options);
    ASSERT_TRUE(result.success()) << "iterations=" << result.iterations;
    expect_path_is_valid(chain, geometry, world, result.path, options);
}
