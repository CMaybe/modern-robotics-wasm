#include <gtest/gtest.h>

#include "robotics/models/ur5.hpp"
#include "robotics/planning/rrt_connect.hpp"
#include "robotics/planning/trajectory_optimizer.hpp"
#include "test_helpers.hpp"

namespace {

using robotics::JointVector;
using robotics::Scalar;
using robotics::Vector3;
using robotics::models::kUr5Dof;
using robotics::testing::joints;

namespace collision = robotics::collision;
namespace planning = robotics::planning;

/// The probed wall from the planner tests: endpoints clear, straight sweep blocked.
[[nodiscard]] collision::CollisionWorld ur5_wall() {
    collision::CollisionWorld world;
    world.spheres.push_back({Vector3{0.55F, 0.0F, 0.35F}, Scalar{0.15}});
    return world;
}

[[nodiscard]] JointVector<kUr5Dof> sweep_start() { return joints<kUr5Dof>({0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F}); }
[[nodiscard]] JointVector<kUr5Dof> sweep_goal() { return joints<kUr5Dof>({-0.9F, -1.1F, 1.3F, -0.2F, 1.0F, 0.0F}); }

/// Plans through the wall and returns the shortcut path the optimiser starts from.
[[nodiscard]] std::vector<JointVector<kUr5Dof>> planned_shortcut(const robotics::models::Ur5Chain& chain,
                                                                 const collision::RobotGeometry& geometry) {
    const auto plan = planning::plan_rrt_connect(chain, geometry, ur5_wall(), sweep_start(), sweep_goal(), {});
    EXPECT_TRUE(plan.success());
    return plan.path;
}

}  // namespace

TEST(TrajectoryOptimizer, DensifyBoundsTheSegmentLengthAndKeepsTheEndpoints) {
    const std::vector<JointVector<kUr5Dof>> path{sweep_start(), sweep_goal()};
    const auto dense = planning::densify_path(path, Scalar{0.15});

    ASSERT_GE(dense.size(), 2U);
    EXPECT_TRUE(dense.front().isApprox(path.front(), Scalar{1e-6}));
    EXPECT_TRUE(dense.back().isApprox(path.back(), Scalar{1e-6}));
    for (std::size_t i = 1; i < dense.size(); ++i) {
        EXPECT_LE((dense[i] - dense[i - 1]).norm(), Scalar{0.15} + Scalar{1e-5});
    }
}

TEST(TrajectoryOptimizer, PointJacobianMatchesFiniteDifferences) {
    const auto chain = robotics::models::ur5();
    const auto q = sweep_start();
    const int link = 4;
    // A point on the wrist-2 capsule axis, taken from the posed geometry.
    const auto posed = collision::pose_geometry(robotics::models::ur5_collision(chain), chain.link_poses(q));
    const Vector3 point = posed[static_cast<std::size_t>(link)].world.start;

    const auto jacobian = planning::point_jacobian(chain, q, link, point);

    // Move each joint a little and track the same material point: it is rigid on
    // the link, so its world motion is T_new * T_old^{-1} applied to it.
    const auto poses = chain.link_poses(q);
    const Vector3 local = poses[static_cast<std::size_t>(link)].inverse() * point;
    constexpr Scalar kDelta = Scalar{1e-3};
    for (int j = 0; j < kUr5Dof; ++j) {
        auto perturbed = q;
        perturbed(j) += kDelta;
        const Vector3 moved = chain.link_poses(perturbed)[static_cast<std::size_t>(link)] * local;
        const Vector3 numeric = (moved - point) / kDelta;
        EXPECT_TRUE((jacobian.col(j) - numeric).norm() < Scalar{5e-3})
            << "joint " << j << ": analytic " << jacobian.col(j).transpose() << " vs numeric " << numeric.transpose();
    }
}

TEST(TrajectoryOptimizer, FreeSpacePullsThePathStraight) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    // A deliberately kinked but collision-free polyline between the endpoints.
    const auto detour = joints<kUr5Dof>({0.0F, -1.4F, 1.8F, -0.4F, 1.2F, 0.3F});
    const std::vector<JointVector<kUr5Dof>> input{sweep_start(), detour, sweep_goal()};

    const auto result = planning::optimize_path(chain, geometry, {}, input, {});
    ASSERT_TRUE(result.feasible);
    EXPECT_TRUE(result.path.front().isApprox(sweep_start(), Scalar{1e-5}));
    EXPECT_TRUE(result.path.back().isApprox(sweep_goal(), Scalar{1e-5}));

    // With nothing to avoid, the optimum is the straight line; allow slack for
    // the finite sweep budget.
    const Scalar straight = (sweep_goal() - sweep_start()).norm();
    EXPECT_LT(planning::path_length(result.path), Scalar{1.1} * straight) << "input length " << planning::path_length(input);
}

TEST(TrajectoryOptimizer, SmoothsThePlannedPathWithoutBreakingIt) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto world = ur5_wall();
    const planning::TrajectoryOptions options{};

    const auto shortcut = planned_shortcut(chain, geometry);
    const auto baseline = planning::densify_path(shortcut, options.spacing);
    const auto result = planning::optimize_path(chain, geometry, world, shortcut, options);

    ASSERT_TRUE(result.feasible);
    EXPECT_TRUE(result.path.front().isApprox(shortcut.front(), Scalar{1e-5}));
    EXPECT_TRUE(result.path.back().isApprox(shortcut.back(), Scalar{1e-5}));

    // The objective the optimiser actually minimises must not get worse.
    EXPECT_LE(result.smoothness, planning::path_smoothness(baseline) + Scalar{1e-4});

    // And the result must still be a valid motion end to end.
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        EXPECT_TRUE(planning::motion_is_free(
            chain, geometry, world, result.path[i - 1], result.path[i], options.resolution, options.margin))
            << "segment " << i;
    }
}

TEST(TrajectoryOptimizer, TradesExcessClearanceForLengthButKeepsABuffer) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);
    const auto world = ur5_wall();
    const planning::TrajectoryOptions options{};

    const auto shortcut = planned_shortcut(chain, geometry);

    // Tightest obstacle-only clearance along a path. The combined min_clearance
    // is dominated by the UR5's structural wrist self-clearance (~49 mm), which
    // the optimiser has no reason to preserve beyond safe_distance; the obstacle
    // side is what this pass exists to improve.
    const auto obstacle_clearance = [&](const std::vector<JointVector<kUr5Dof>>& path) {
        Scalar tightest = std::numeric_limits<Scalar>::infinity();
        for (const auto& waypoint : path) {
            const auto posed = collision::pose_geometry(geometry, chain.link_poses(waypoint));
            tightest = std::min(tightest, collision::obstacle_contact(posed, world).distance);
        }
        return tightest;
    };

    const auto baseline = planning::densify_path(shortcut, options.spacing);
    const auto result = planning::optimize_path(chain, geometry, world, shortcut, options);

    ASSERT_TRUE(result.feasible);
    const Scalar after = obstacle_clearance(result.path);
    // The stretch term pulls the path straight until the obstacle term pushes
    // back, so the equilibrium sits just below safe_distance (0.05 m) — the path
    // gets shorter, and the clearance settles at a real buffer, not the margin.
    EXPECT_GT(after, Scalar{0.03}) << after;
    EXPECT_GT(result.min_clearance, options.margin);
    EXPECT_LE(planning::path_length(result.path), planning::path_length(baseline) + Scalar{1e-4})
        << "clearance " << obstacle_clearance(baseline) << " -> " << after;
}

TEST(TrajectoryOptimizer, ATrivialPathPassesThrough) {
    const auto chain = robotics::models::ur5();
    const auto geometry = robotics::models::ur5_collision(chain);

    const std::vector<JointVector<kUr5Dof>> single{sweep_start(), sweep_start()};
    const auto result = planning::optimize_path(chain, geometry, {}, single, {});
    EXPECT_TRUE(result.feasible);
    EXPECT_TRUE(result.path.front().isApprox(sweep_start(), Scalar{1e-6}));
}
