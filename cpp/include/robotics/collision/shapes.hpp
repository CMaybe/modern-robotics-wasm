#pragma once

#include <algorithm>
#include <cmath>

#include "robotics/core/types.hpp"

/// Collision geometry: convex shapes with analytic signed distances. Everything
/// here returns a *signed* distance rather than a boolean, because the planned
/// motion planners need it twice over — sampling planners want a safety margin,
/// and optimisation-based ones differentiate the distance itself.
namespace robotics::collision {

/// A sphere, in whatever frame its user keeps it in.
struct Sphere {
    Vector3 center{Vector3::Zero()};
    Scalar radius{};
};

/**
 * @brief A capsule: every point within `radius` of the segment `[start, end]`.
 *
 * A zero-length segment makes it a sphere; every routine here accepts that, so
 * degenerate links need no special casing by callers.
 */
struct Capsule {
    Vector3 start{Vector3::Zero()};
    Vector3 end{Vector3::Zero()};
    Scalar radius{};
};

/// @return The point of the segment `[a, b]` closest to `point`.
[[nodiscard]] inline Vector3 closest_point_on_segment(const Vector3& a, const Vector3& b, const Vector3& point) {
    const Vector3 direction = b - a;
    const Scalar squared_length = direction.squaredNorm();
    if (squared_length <= Scalar{0}) {
        return a;
    }
    const Scalar t = std::clamp(direction.dot(point - a) / squared_length, Scalar{0}, Scalar{1});
    return a + t * direction;
}

/// Closest pair of points between two segments.
struct SegmentClosestPoints {
    Vector3 on_a{Vector3::Zero()};
    Vector3 on_b{Vector3::Zero()};
};

/**
 * @brief Closest points between segments `[a0, a1]` and `[b0, b1]`.
 *
 * Ericson, Real-Time Collision Detection, section 5.1.9: minimise the quadratic
 * distance over the unit square of segment parameters, clamping each parameter
 * and re-solving the other. Degenerate (zero-length) segments fall out of the
 * same clamping, so points and segments mix freely.
 */
[[nodiscard]] inline SegmentClosestPoints closest_points_between_segments(const Vector3& a0,
                                                                          const Vector3& a1,
                                                                          const Vector3& b0,
                                                                          const Vector3& b1) {
    // Below this squared length a segment is treated as a point (~1e-5 m).
    constexpr Scalar kDegenerate = Scalar{1e-10};

    const Vector3 direction_a = a1 - a0;
    const Vector3 direction_b = b1 - b0;
    const Vector3 offset = a0 - b0;
    const Scalar length_a = direction_a.squaredNorm();
    const Scalar length_b = direction_b.squaredNorm();
    const Scalar b_dot_offset = direction_b.dot(offset);

    Scalar s{0};
    Scalar t{0};
    if (length_a <= kDegenerate && length_b <= kDegenerate) {
        // Both are points; s = t = 0.
    } else if (length_a <= kDegenerate) {
        t = std::clamp(b_dot_offset / length_b, Scalar{0}, Scalar{1});
    } else {
        const Scalar a_dot_offset = direction_a.dot(offset);
        if (length_b <= kDegenerate) {
            s = std::clamp(-a_dot_offset / length_a, Scalar{0}, Scalar{1});
        } else {
            const Scalar a_dot_b = direction_a.dot(direction_b);
            const Scalar denominator = length_a * length_b - a_dot_b * a_dot_b;
            // Parallel segments have a zero denominator; any s works, so keep 0.
            if (denominator > kDegenerate) {
                s = std::clamp((a_dot_b * b_dot_offset - a_dot_offset * length_b) / denominator, Scalar{0}, Scalar{1});
            }
            t = (a_dot_b * s + b_dot_offset) / length_b;
            if (t < Scalar{0}) {
                t = 0;
                s = std::clamp(-a_dot_offset / length_a, Scalar{0}, Scalar{1});
            } else if (t > Scalar{1}) {
                t = 1;
                s = std::clamp((a_dot_b - a_dot_offset) / length_a, Scalar{0}, Scalar{1});
            }
        }
    }
    return {a0 + s * direction_a, b0 + t * direction_b};
}

/**
 * @brief A signed distance together with the witness a gradient needs.
 *
 * `distance` is between the *surfaces*: positive means separated by that much,
 * negative means penetrating by that depth. `point` lies on the first shape's
 * axis and `normal` points from the second shape toward the first, so moving the
 * first shape along `normal` increases the distance at unit rate.
 */
struct SignedDistance {
    Scalar distance{};
    Vector3 point{Vector3::Zero()};
    Vector3 normal{Vector3::UnitZ()};
};

namespace detail {

/// Distance between axis points, minus the summed radii; coincident axes get an
/// arbitrary but fixed normal so penetration depth stays well defined.
[[nodiscard]] inline SignedDistance from_axis_points(const Vector3& on_first, const Vector3& on_second, Scalar summed_radii) {
    const Vector3 offset = on_first - on_second;
    const Scalar length = offset.norm();
    constexpr Scalar kCoincident = Scalar{1e-6};
    const Vector3 normal = length > kCoincident ? Vector3{offset / length} : Vector3::UnitZ();
    return {length - summed_radii, on_first, normal};
}

}  // namespace detail

/// @return Signed distance from a capsule's surface to a sphere's surface.
[[nodiscard]] inline SignedDistance signed_distance(const Capsule& capsule, const Sphere& sphere) {
    const Vector3 on_axis = closest_point_on_segment(capsule.start, capsule.end, sphere.center);
    return detail::from_axis_points(on_axis, sphere.center, capsule.radius + sphere.radius);
}

/// @return Signed distance between two capsules' surfaces.
[[nodiscard]] inline SignedDistance signed_distance(const Capsule& first, const Capsule& second) {
    const auto [on_first, on_second] = closest_points_between_segments(first.start, first.end, second.start, second.end);
    return detail::from_axis_points(on_first, on_second, first.radius + second.radius);
}

/// @return Signed distance between two spheres' surfaces.
[[nodiscard]] inline SignedDistance signed_distance(const Sphere& first, const Sphere& second) {
    return detail::from_axis_points(first.center, second.center, first.radius + second.radius);
}

}  // namespace robotics::collision
