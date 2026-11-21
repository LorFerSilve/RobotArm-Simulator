#pragma once

#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/Joint2D.hpp"

#include <span>
#include <stop_token>

namespace robotics::kinematics {

// Conservative continuous collision test for a LINEAR JOINT-SPACE edge.
//
// Proves that all intermediate link poses avoid static scene obstacles
// (with robot thickness/clearance) and nonadjacent link self-contact.
// Uses a bound on the maximum endpoint displacement over each time interval,
// subdividing until all intervals are certified clear.
//
// Returns false for a collision, cancellation, or an interval that cannot
// be certified within the recursion limit. That failure is conservative:
// it does not prove a collision or that no alternate route exists.
//
// Does not model target/tool contact or curved spline arcs; the planner's
// target-specific approach rules and spline-edge sampling remain separate.
[[nodiscard]] bool sweptArmClear2D(
    std::span<const Joint2D> joints,
    std::span<const double> startAngles,
    std::span<const double> endAngles,
    std::span<const geometry::Obstacle2D> obstacles,
    double selfClearance,
    double obstacleClearance,
    std::stop_token cancellation = {});

} // namespace robotics::kinematics
