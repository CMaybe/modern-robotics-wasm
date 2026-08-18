#pragma once

#include "robotics/core/types.hpp"

namespace robotics {

/**
 * @brief A screw axis in the space (base) frame.
 *
 * Stored in the `[angular; linear]` ordering that Modern Robotics uses, and
 * converted to the `[linear; angular]` ordering Sophus expects only at the
 * boundary. Keeping the two apart is what stops the orderings being mixed up.
 */
struct ScrewAxis {
    Vector3 angular{Vector3::Zero()};  ///< Rotation axis `w`, unit length for a revolute joint.
    Vector3 linear{Vector3::Zero()};   ///< Linear term `v`.

    /**
     * @brief Builds the screw axis of a revolute joint.
     *
     * For a joint rotating about `axis` through `point`, the screw axis is
     * `[w; -w x q]`, which is where every model in this project gets its numbers.
     *
     * @param axis Rotation axis in the space frame; normalised internally.
     * @param point Any point on the axis, in the space frame.
     */
    [[nodiscard]] static ScrewAxis revolute(const Vector3& axis, const Vector3& point) noexcept {
        const Vector3 unit = axis.normalized();
        return ScrewAxis{.angular = unit, .linear = -unit.cross(point)};
    }

    /**
     * @brief Builds the screw axis of a prismatic joint.
     * @param direction Translation direction in the space frame; normalised internally.
     */
    [[nodiscard]] static ScrewAxis prismatic(const Vector3& direction) noexcept {
        return ScrewAxis{.angular = Vector3::Zero(), .linear = direction.normalized()};
    }

    /// @return The same screw in Sophus `[linear; angular]` ordering.
    [[nodiscard]] Twist to_twist() const noexcept {
        Twist twist;
        twist.head<3>() = linear;
        twist.tail<3>() = angular;
        return twist;
    }
};

}  // namespace robotics
