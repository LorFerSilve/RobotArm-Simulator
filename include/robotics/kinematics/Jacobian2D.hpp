#pragma once

#include "robotics/kinematics/Joint2D.hpp"
#include "robotics/math/Vec2.hpp"

#include <span>
#include <vector>

namespace robotics::kinematics {

// Returns the columns of the 2 x N end-effector Jacobian for a planar
// revolute arm. 'positions' contains N + 1 points from forwardKinematics:
// base, then each link endpoint. Column i is d(end effector)/d(angle_i).
// Throws std::invalid_argument for empty or non-finite positions, and
// std::overflow_error if a Jacobian component overflows.
[[nodiscard]] std::vector<math::Vec2> endEffectorJacobian(
    std::span<const math::Vec2> positions);

// Multiplies the 2 x N Jacobian by the N joint angular velocities (rad/s).
// Returns the end-effector linear velocity (m/s).
[[nodiscard]] math::Vec2 endEffectorVelocity(
    std::span<const math::Vec2> jacobianColumns,
    std::span<const Joint2D> joints);

// Determinant is only defined for a square 2 x 2 Jacobian (two joints).
// Throws std::invalid_argument if the column count is not exactly two.
[[nodiscard]] double jacobianDeterminant2D(
    std::span<const math::Vec2> jacobianColumns);

 // Spectral 2-norm condition number of a 2 x N translational Jacobian.
 // Returns +infinity at rank-deficient poses (including an empty arm).
 // Meaningful for both two-joint and redundant planar robots.
 [[nodiscard]] double jacobianConditionNumber2D(
     std::span<const math::Vec2> jacobianColumns);

} // namespace robotics::kinematics
