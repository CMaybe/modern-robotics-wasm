#pragma once

#include <limits>
#include <vector>

#include "robotics/collision/robot_geometry.hpp"
#include "robotics/collision/shapes.hpp"
#include "robotics/kinematics/serial_chain.hpp"

namespace robotics::collision {

/// Static obstacles, in the space frame.
struct CollisionWorld {
    std::vector<Sphere> spheres;
    std::vector<Capsule> capsules;
};

/**
 * @brief The closest approach a query found, with the witness a gradient needs.
 *
 * `distance` is signed (negative = penetration). `point` lies on the involved
 * robot capsule's axis and `normal` points away from the other body, so pushing
 * link `link` along `normal` increases the clearance at unit rate. An empty
 * query — no obstacles, or no enabled pair — leaves the infinite default, which
 * every consumer already treats as "nothing nearby".
 */
struct Contact {
    Scalar distance{std::numeric_limits<Scalar>::infinity()};
    int link{-1};
    Vector3 point{Vector3::Zero()};
    Vector3 normal{Vector3::UnitZ()};
};

namespace detail {

inline void keep_closer(Contact& contact, const SignedDistance& candidate, int link) {
    if (candidate.distance < contact.distance) {
        contact = Contact{candidate.distance, link, candidate.point, candidate.normal};
    }
}

}  // namespace detail

/// @return The closest approach between the posed robot and the obstacles.
[[nodiscard]] inline Contact obstacle_contact(const std::vector<PosedCapsule>& robot, const CollisionWorld& world) {
    Contact contact;
    for (const auto& capsule : robot) {
        for (const auto& sphere : world.spheres) {
            detail::keep_closer(contact, signed_distance(capsule.world, sphere), capsule.link);
        }
        for (const auto& obstacle : world.capsules) {
            detail::keep_closer(contact, signed_distance(capsule.world, obstacle), capsule.link);
        }
    }
    return contact;
}

/// @return The closest approach between two robot capsules the geometry allows checking.
[[nodiscard]] inline Contact self_contact(const RobotGeometry& geometry, const std::vector<PosedCapsule>& robot) {
    Contact contact;
    for (std::size_t i = 0; i < robot.size(); ++i) {
        for (std::size_t j = i + 1; j < robot.size(); ++j) {
            if (!geometry.self_pair_enabled(geometry.capsules[i], geometry.capsules[j])) {
                continue;
            }
            detail::keep_closer(contact, signed_distance(robot[i].world, robot[j].world), robot[i].link);
        }
    }
    return contact;
}

/**
 * @brief The single closest approach at a configuration, obstacles and self both.
 * @param chain The kinematic model.
 * @param geometry Its collision body.
 * @param angles Joint angles [rad].
 * @param world The obstacles.
 */
template <int Dof>
[[nodiscard]] Contact nearest_contact(const SerialChain<Dof>& chain,
                                      const RobotGeometry& geometry,
                                      const JointsArg<Dof>& angles,
                                      const CollisionWorld& world) {
    const auto posed = pose_geometry(geometry, chain.link_poses(angles));
    const Contact against_world = obstacle_contact(posed, world);
    const Contact against_self = self_contact(geometry, posed);
    return against_self.distance < against_world.distance ? against_self : against_world;
}

/**
 * @brief Whether a configuration collides, with an optional safety margin.
 *
 * The margin is the clearance a planner keeps in hand for the capsule
 * approximation; `margin = 0` tests the capsules themselves.
 */
template <int Dof>
[[nodiscard]] bool in_collision(const SerialChain<Dof>& chain,
                                const RobotGeometry& geometry,
                                const JointsArg<Dof>& angles,
                                const CollisionWorld& world,
                                Scalar margin = Scalar{0}) {
    return nearest_contact(chain, geometry, angles, world).distance <= margin;
}

}  // namespace robotics::collision
