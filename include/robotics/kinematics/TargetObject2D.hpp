#pragma once

#include "robotics/geometry/Obstacles2D.hpp"

#include <cstdint>

namespace robotics::kinematics {

// A solid physical target object centered on the requested Cartesian goal.
// Only the explicitly defined terminal tool section may touch this object;
// the other links and the final link's shaft remain outside it.
enum class TargetShape2D : std::uint8_t { Circle, Rectangle };

struct TargetObject2D {
    TargetShape2D shape{TargetShape2D::Circle};
    double radius{0.16};          // circle radius, metres
    double halfWidth{0.16};       // rectangle half width, metres
    double halfHeight{0.12};      // rectangle half height, metres
    double clearance{0.0};       // additional shaft/object separation, metres
    double toolLength{0.30};     // length of terminal link reserved for tool contact
};

[[nodiscard]] bool validTargetObject2D(const TargetObject2D& object) noexcept;
[[nodiscard]] geometry::Obstacle2D targetBody2D(
    const TargetObject2D& object, math::Vec2 center);
// Bounding radius of solid object plus clearance (for conservative tests).
[[nodiscard]] double targetObjectBound2D(const TargetObject2D& object) noexcept;

} // namespace robotics::kinematics
