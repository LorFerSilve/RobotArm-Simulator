#pragma once

#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/Joint2D.hpp"
#include "robotics/kinematics/TargetObject2D.hpp"

#include <span>
#include <stop_token>

namespace robotics::kinematics {

// Conservative continuous collision test for a LINEAR JOINT-SPACE edge.
//
// Proves that all intermediate link poses avoid static scene obstacles
// and nonadjacent link self-contact. The given clearances are surface gaps
// in addition to the physical link radii carried by the joints.
// Uses a bound on the maximum endpoint displacement over each time interval,
// subdividing until all intervals are certified clear.
//
// Returns false for a collision, cancellation, or an interval that cannot
// be certified within the recursion limit. That failure is conservative:
// it does not prove a collision or that no alternate route exists.
//
// This overload checks scene and self-contact. Neither overload certifies
// curved spline arcs.
[[nodiscard]] bool sweptArmClear2D(
    std::span<const Joint2D> joints,
    std::span<const double> startAngles,
    std::span<const double> endAngles,
    std::span<const geometry::Obstacle2D> obstacles,
    double selfClearance,
    double obstacleClearance,
    std::stop_token cancellation = {});

// Also certifies the explicit target against every protected shaft segment.
// Only the distal tool section is exempt. A shaft already in contact may
// retreat while its penetration is nonincreasing and must not re-enter.
[[nodiscard]] bool sweptArmClear2D(
    std::span<const Joint2D> joints,
    std::span<const double> startAngles,
    std::span<const double> endAngles,
    std::span<const geometry::Obstacle2D> obstacles,
    double selfClearance,
    double obstacleClearance,
    math::Vec2 target,
    const TargetObject2D& targetObject,
    std::stop_token cancellation = {});

} // namespace robotics::kinematics
