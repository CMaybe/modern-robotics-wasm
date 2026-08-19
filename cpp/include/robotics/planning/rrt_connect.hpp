#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "robotics/collision/world.hpp"
#include "robotics/kinematics/serial_chain.hpp"

/// Sampling-based motion planning in joint space. RRT-Connect grows one tree
/// from the start and one from the goal, greedily connecting them; it is
/// probabilistically complete, so a path that exists is eventually found, at the
/// price of a jagged result — which is why a shortcut pass follows it.
namespace robotics::planning {

/// Tuning for `plan_rrt_connect`.
struct Options {
    int max_iterations{3000};         ///< Extend attempts across both trees.
    Scalar step{Scalar{0.2}};         ///< Joint-space extension step [rad].
    Scalar resolution{Scalar{0.05}};  ///< Collision-check spacing along an edge [rad].
    Scalar margin{Scalar{0.01}};      ///< Clearance every configuration must keep [m].
    int shortcut_rounds{150};         ///< Random shortcut attempts on the found path.
    std::uint32_t seed{2026};         ///< RNG seed; fixed so runs reproduce.
};

/// Why a plan ended the way it did, for error messages a UI can act on.
enum class Status {
    kSuccess,
    kStartInvalid,  ///< Start violates the limits or the required clearance.
    kGoalInvalid,   ///< Goal violates the limits or the required clearance.
    kNotFound,      ///< Iteration budget exhausted before the trees met.
};

template <int Dof>
struct Result {
    Status status{Status::kNotFound};
    std::vector<JointVector<Dof>> path;      ///< Shortcut waypoints, start to goal; empty on failure.
    std::vector<JointVector<Dof>> raw_path;  ///< The path as the trees found it, before shortcutting.
    int iterations{0};                       ///< Extend attempts actually spent.
    int nodes{0};                            ///< Vertices grown across both trees.

    [[nodiscard]] bool success() const noexcept { return status == Status::kSuccess; }
};

/// @return Total joint-space length of a waypoint path [rad].
template <int Dof>
[[nodiscard]] Scalar path_length(const std::vector<JointVector<Dof>>& path) {
    Scalar length{0};
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

namespace detail {

/**
 * @brief Deterministic uniform sampler on top of a small xorshift generator.
 *
 * `std::uniform_real_distribution` is implementation-defined, which would let
 * the same seed produce different paths natively and under WebAssembly. Drawing
 * 24 explicit mantissa bits keeps every platform bit-identical.
 */
class Sampler {
public:
    explicit Sampler(std::uint32_t seed) : state_{seed != 0 ? seed : 1} {}

    /// @return A uniform draw from [0, 1).
    [[nodiscard]] Scalar uniform() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return static_cast<Scalar>(state_ >> 8) / static_cast<Scalar>(1U << 24);
    }

    /// @return A uniform integer in [0, bound).
    [[nodiscard]] int below(int bound) { return std::min(bound - 1, static_cast<int>(uniform() * static_cast<Scalar>(bound))); }

private:
    std::uint32_t state_;
};

}  // namespace detail

/**
 * @brief Whether the straight joint-space segment between two configurations stays valid.
 *
 * The segment is sampled every `resolution` radians (endpoints included), and each
 * sample must keep `margin` clearance from the obstacles and from the robot itself.
 * The capsules are conservative, so a segment accepted here is clear on the real
 * geometry too; the discretisation gap is what `margin` is for.
 */
template <int Dof>
[[nodiscard]] bool motion_is_free(const SerialChain<Dof>& chain,
                                  const collision::RobotGeometry& geometry,
                                  const collision::CollisionWorld& world,
                                  const JointsArg<Dof>& from,
                                  const JointsArg<Dof>& to,
                                  Scalar resolution,
                                  Scalar margin) {
    const Scalar distance = (to - from).norm();
    const int checks = std::max(1, static_cast<int>(std::ceil(distance / std::max(resolution, Scalar{1e-4}))));
    for (int i = 0; i <= checks; ++i) {
        const Scalar t = static_cast<Scalar>(i) / static_cast<Scalar>(checks);
        const JointVector<Dof> sample = from + t * (to - from);
        if (collision::in_collision(chain, geometry, sample, world, margin)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Plans a collision-free joint path with RRT-Connect, then shortcuts it.
 *
 * Kuffner & LaValle (2000): each iteration extends one tree a single `step`
 * toward a uniform sample, then the other tree extends greedily toward the new
 * vertex until it either reaches it (the trees meet) or hits an obstacle. The
 * trees swap roles every iteration, so the search stays balanced.
 *
 * The raw path is then shortcut: random waypoint pairs are joined by a straight
 * segment whenever that segment is collision-free. This removes most of the
 * detours sampling introduces, at no cost to feasibility — every surviving edge
 * has been checked the same way the tree edges were.
 *
 * @param chain The kinematic model; samples are drawn inside its joint limits.
 * @param geometry Its collision body.
 * @param world The obstacles.
 * @param start Start configuration.
 * @param goal Goal configuration.
 * @param options Budget, step sizes, clearance margin, and RNG seed.
 */
template <int Dof>
[[nodiscard]] Result<Dof> plan_rrt_connect(const SerialChain<Dof>& chain,
                                           const collision::RobotGeometry& geometry,
                                           const collision::CollisionWorld& world,
                                           const JointsArg<Dof>& start,
                                           const JointsArg<Dof>& goal,
                                           const Options& options = {}) {
    Result<Dof> result;

    const auto valid = [&](const JointVector<Dof>& q) {
        return chain.within_limits(q) && !collision::in_collision(chain, geometry, q, world, options.margin);
    };
    if (!valid(start)) {
        result.status = Status::kStartInvalid;
        return result;
    }
    if (!valid(goal)) {
        result.status = Status::kGoalInvalid;
        return result;
    }

    const auto edge_is_free = [&](const JointVector<Dof>& from, const JointVector<Dof>& to) {
        return motion_is_free(chain, geometry, world, from, to, options.resolution, options.margin);
    };

    struct Node {
        JointVector<Dof> configuration;
        int parent;
    };
    // Tree 0 roots at the start, tree 1 at the goal.
    std::array<std::vector<Node>, 2> trees;
    trees[0].push_back({start, -1});
    trees[1].push_back({goal, -1});

    const auto nearest = [](const std::vector<Node>& tree, const JointVector<Dof>& target) {
        std::size_t best = 0;
        Scalar best_distance = (tree[0].configuration - target).squaredNorm();
        for (std::size_t i = 1; i < tree.size(); ++i) {
            const Scalar distance = (tree[i].configuration - target).squaredNorm();
            if (distance < best_distance) {
                best_distance = distance;
                best = i;
            }
        }
        return best;
    };

    enum class Extend { kTrapped, kAdvanced, kReached };

    // One bounded step from the tree's nearest node toward the target; appends the
    // new node on success. kReached means the step arrived at the target itself.
    const auto extend = [&](std::vector<Node>& tree, const JointVector<Dof>& target) {
        const std::size_t from_index = nearest(tree, target);
        const JointVector<Dof>& from = tree[from_index].configuration;

        const Scalar distance = (target - from).norm();
        const bool reaches = distance <= options.step;
        const JointVector<Dof> next = reaches ? target : JointVector<Dof>{from + (options.step / distance) * (target - from)};

        if (!chain.within_limits(next) || !edge_is_free(from, next)) {
            return Extend::kTrapped;
        }
        tree.push_back({next, static_cast<int>(from_index)});
        return reaches ? Extend::kReached : Extend::kAdvanced;
    };

    detail::Sampler sampler{options.seed};
    const JointVector<Dof> lower = chain.lower_limits();
    const JointVector<Dof> upper = chain.upper_limits();

    int grower = 0;  // The tree that extends toward the sample this iteration.
    std::array<int, 2> meet{-1, -1};

    for (int iteration = 0; iteration < options.max_iterations && meet[0] < 0; ++iteration) {
        result.iterations = iteration + 1;

        JointVector<Dof> sample;
        for (int j = 0; j < Dof; ++j) {
            sample(j) = lower(j) + sampler.uniform() * (upper(j) - lower(j));
        }

        auto& expanding = trees[static_cast<std::size_t>(grower)];
        auto& connecting = trees[static_cast<std::size_t>(1 - grower)];

        if (extend(expanding, sample) != Extend::kTrapped) {
            // Greedily connect the other tree to the vertex just added.
            const JointVector<Dof> bridge = expanding.back().configuration;
            Extend outcome = Extend::kAdvanced;
            while (outcome == Extend::kAdvanced) {
                outcome = extend(connecting, bridge);
            }
            if (outcome == Extend::kReached) {
                meet[static_cast<std::size_t>(grower)] = static_cast<int>(expanding.size()) - 1;
                meet[static_cast<std::size_t>(1 - grower)] = static_cast<int>(connecting.size()) - 1;
            }
        }
        grower = 1 - grower;
    }

    result.nodes = static_cast<int>(trees[0].size() + trees[1].size());
    if (meet[0] < 0) {
        return result;  // kNotFound
    }

    // Walk each tree from its meeting vertex back to the root; the two branches
    // share the meeting configuration, so one copy is dropped.
    const auto branch = [&](const std::vector<Node>& tree, int leaf) {
        std::vector<JointVector<Dof>> chain_to_root;
        for (int index = leaf; index >= 0; index = tree[static_cast<std::size_t>(index)].parent) {
            chain_to_root.push_back(tree[static_cast<std::size_t>(index)].configuration);
        }
        return chain_to_root;
    };
    std::vector<JointVector<Dof>> path = branch(trees[0], meet[0]);
    std::reverse(path.begin(), path.end());
    const std::vector<JointVector<Dof>> to_goal = branch(trees[1], meet[1]);
    path.insert(path.end(), to_goal.begin() + 1, to_goal.end());

    result.raw_path = path;

    // Shortcut: replace path[i..j] with a straight segment whenever that segment
    // is free. Endpoints never move, so start and goal are preserved exactly.
    for (int round = 0; round < options.shortcut_rounds && path.size() > 2; ++round) {
        const int count = static_cast<int>(path.size());
        int i = sampler.below(count - 1);
        int j = sampler.below(count);
        if (i > j) {
            std::swap(i, j);
        }
        if (j - i < 2) {
            continue;
        }
        if (edge_is_free(path[static_cast<std::size_t>(i)], path[static_cast<std::size_t>(j)])) {
            path.erase(path.begin() + i + 1, path.begin() + j);
        }
    }

    result.path = std::move(path);
    result.status = Status::kSuccess;
    return result;
}

}  // namespace robotics::planning
