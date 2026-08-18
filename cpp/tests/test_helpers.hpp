#pragma once

#include <gtest/gtest.h>

#include "robotics/core/types.hpp"

namespace robotics::testing {

/// Builds a joint vector from a brace list, so tests read as tables of numbers.
template <int Dof>
[[nodiscard]] JointVector<Dof> joints(std::initializer_list<Scalar> values) {
    EXPECT_EQ(values.size(), static_cast<std::size_t>(Dof));
    JointVector<Dof> vector = JointVector<Dof>::Zero();
    int index = 0;
    for (const Scalar value : values) {
        vector(index++) = value;
    }
    return vector;
}

/// Matcher-free pose comparison with a readable failure message.
inline void expect_pose_near(const Pose& actual, const Pose& expected, Scalar tolerance) {
    EXPECT_TRUE(actual.matrix().isApprox(expected.matrix(), tolerance)) << "got\n"
                                                                        << actual.matrix() << "\nexpected\n"
                                                                        << expected.matrix();
}

}  // namespace robotics::testing
