#pragma once

#include "robotics/math/Vec2.hpp"

#include <span>
#include <variant>

namespace robotics::geometry {

// The obstacle geometry is expressed in world metres, independently of raylib.
struct Circle2D {
    math::Vec2 center;
    double radius;
};

struct Box2D {
    math::Vec2 center;
    double halfWidth;
    double halfHeight;
};

using Obstacle2D = std::variant<Circle2D, Box2D>;

// All obstacle dimensions must be positive and finite.
[[nodiscard]] bool validObstacle(const Obstacle2D& obstacle) noexcept;

// 'clearance' expands the solid obstacle by an isotropic radius (e.g. robot
// link thickness / 2 + margin), not just by an axis-aligned rectangle.
[[nodiscard]] bool pointHitsObstacle(math::Vec2 point,
                                     const Obstacle2D& obstacle,
                                     double clearance = 0.0);
[[nodiscard]] bool segmentHitsObstacle(math::Vec2 from, math::Vec2 to,
                                       const Obstacle2D& obstacle,
                                       double clearance = 0.0);

// Signed minimum distance from a segment to a solid circle/rectangle.
// Positive outside, zero at contact, negative when crossing the interior.
// Useful for measuring (rather than merely testing) target penetration.
[[nodiscard]] double segmentSignedDistanceToObstacle(
    math::Vec2 from, math::Vec2 to, const Obstacle2D& obstacle);

// All links are considered, including the final tool link. The target-specific
// tool-tip exception does not apply to scene obstacles.
[[nodiscard]] bool hasObstacleCollision(
    std::span<const math::Vec2> positions,
    std::span<const Obstacle2D> obstacles,
    double clearance);

// Each capsule radius is added to the gap between the link and the obstacle.
// Empty radii use zero-thickness links.
[[nodiscard]] bool hasObstacleCollision(
    std::span<const math::Vec2> positions,
    std::span<const Obstacle2D> obstacles,
    double clearance,
    std::span<const double> linkRadii);

} // namespace robotics::geometry
