#pragma once

#include <array>
#include <cstdlib>
#include <utility>
#include <vector>

#include "robotics/collision/shapes.hpp"
#include "robotics/kinematics/serial_chain.hpp"

namespace robotics::collision {

/// One capsule rigidly attached to a link, stored in that link's frame.
struct LinkCapsule {
    int link{};       ///< Index into the poses returned by `SerialChain::link_poses`.
    int segment{};    ///< Position along the chain; consecutive segments share an endpoint.
    Capsule local{};  ///< In the link frame.
};

/**
 * @brief The robot's collision body: capsules attached to links.
 *
 * Plain data once built — no `Dof` template parameter — so the same queries can
 * serve every robot behind the type-erased `Robot` interface.
 */
struct RobotGeometry {
    std::vector<LinkCapsule> capsules;

    /// Segment pairs excluded from self-collision checks on top of the adjacency
    /// rule, the way an SRDF disables pairs that touch by construction.
    std::vector<std::pair<int, int>> disabled_self_pairs;

    /**
     * @brief Whether a pair of capsules takes part in self-collision checks.
     *
     * Consecutive segments share an endpoint, so their distance is negative at
     * every configuration; checking them would report a permanent collision.
     */
    [[nodiscard]] bool self_pair_enabled(const LinkCapsule& a, const LinkCapsule& b) const {
        if (std::abs(a.segment - b.segment) <= 1) {
            return false;
        }
        for (const auto& [first, second] : disabled_self_pairs) {
            if ((a.segment == first && b.segment == second) || (a.segment == second && b.segment == first)) {
                return false;
            }
        }
        return true;
    }
};

/// A robot capsule placed in the space frame at some configuration.
struct PosedCapsule {
    int link{};
    int segment{};
    Capsule world{};
};

/**
 * @brief Places the collision body at a configuration.
 * @param geometry The robot's collision body.
 * @param link_poses World pose per link, as returned by `SerialChain::link_poses`.
 */
[[nodiscard]] inline std::vector<PosedCapsule> pose_geometry(const RobotGeometry& geometry, const std::vector<Pose>& link_poses) {
    std::vector<PosedCapsule> posed;
    posed.reserve(geometry.capsules.size());
    for (const auto& capsule : geometry.capsules) {
        const Pose& pose = link_poses[static_cast<std::size_t>(capsule.link)];
        posed.push_back(
            {capsule.link, capsule.segment, Capsule{pose * capsule.local.start, pose * capsule.local.end, capsule.local.radius}});
    }
    return posed;
}

/**
 * @brief Builds a collision body from the chain's own frames.
 *
 * The capsules span consecutive joint-frame origins at zero configuration —
 * base to joint 0, joint i to joint i+1, and joint Dof-1 to the end-effector —
 * which is the same skeleton the schematic renderer draws. `radii[j]` pads the
 * j-th of those `Dof + 1` spans; pick each radius to *enclose* the link's mesh,
 * so the approximation errs conservative. Zero-length spans (consecutive joints
 * with coincident origins) produce no capsule, since their neighbours already
 * cover the shared point.
 *
 * Each capsule is attached to the last joint at or before its start, so it
 * moves with the physical link it approximates.
 */
template <int Dof>
[[nodiscard]] RobotGeometry geometry_from_link_frames(const SerialChain<Dof>& chain, const std::array<Scalar, Dof + 1>& radii) {
    std::array<Vector3, Dof + 2> points;
    points[0] = Vector3::Zero();
    for (int i = 0; i < Dof; ++i) {
        points[static_cast<std::size_t>(i) + 1] = chain.joints()[static_cast<std::size_t>(i)].link_home.translation();
    }
    points[Dof + 1] = chain.end_effector_home().translation();

    constexpr Scalar kDegenerate = Scalar{1e-10};

    RobotGeometry geometry;
    for (int span = 0; span <= Dof; ++span) {
        const Vector3& start = points[static_cast<std::size_t>(span)];
        const Vector3& end = points[static_cast<std::size_t>(span) + 1];
        if ((end - start).squaredNorm() <= kDegenerate) {
            continue;
        }

        const int link = std::max(0, span - 1);
        // Skipped spans had zero length, so consecutive kept capsules still share
        // an endpoint; numbering the kept ones keeps the adjacency rule exact.
        const int segment = static_cast<int>(geometry.capsules.size());
        // The endpoints are authored in the space frame at zero configuration;
        // re-express them in the link frame so the capsule rides the link.
        const Pose to_link = chain.joints()[static_cast<std::size_t>(link)].link_home.inverse();
        geometry.capsules.push_back(
            {link, segment, Capsule{to_link * start, to_link * end, radii[static_cast<std::size_t>(span)]}});
    }
    return geometry;
}

}  // namespace robotics::collision
