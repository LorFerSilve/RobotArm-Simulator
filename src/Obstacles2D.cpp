#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/geometry/LinkRadii2D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace robotics::geometry {
namespace {

bool finite(math::Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

double squaredDistance(math::Vec2 a, math::Vec2 b) {
    const double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double pointSegmentSquared(math::Vec2 p, math::Vec2 a, math::Vec2 b) {
    const math::Vec2 ab = b - a;
    const auto offset = p - a;
    const double length = ab.length();
    if (length == 0.0) return squaredDistance(p, a);
    // Avoid a squared-length denominator that can overflow even when the
    // true distance to a long segment is zero. Unknown distances fail closed.
    if (!std::isfinite(length) || !finite(offset)) return 0.0;
    const math::Vec2 direction{ab.x / length, ab.y / length};
    const double t = std::clamp(offset.dot(direction), 0.0, length);
    return squaredDistance(offset, direction * t);
}

double pointBoxSquared(math::Vec2 p, const Box2D& box) {
    const double dx = std::max(std::abs(p.x - box.center.x) - box.halfWidth, 0.0);
    const double dy = std::max(std::abs(p.y - box.center.y) - box.halfHeight, 0.0);
    return dx * dx + dy * dy;
}

bool segmentIntersectsBox(math::Vec2 a, math::Vec2 b, const Box2D& box) {
    double near = 0.0, far = 1.0;
    const math::Vec2 d = b - a;
    for (int axis = 0; axis != 2; ++axis) {
        const double start = axis == 0 ? a.x : a.y;
        const double delta = axis == 0 ? d.x : d.y;
        const double center = axis == 0 ? box.center.x : box.center.y;
        const double extent = axis == 0 ? box.halfWidth : box.halfHeight;
        if (delta == 0.0) {
            if (start < center - extent || start > center + extent)
                return false;
        } else {
            const double t0 = (center - extent - start) / delta;
            const double t1 = (center + extent - start) / delta;
            near = std::max(near, std::min(t0, t1));
            far = std::min(far, std::max(t0, t1));
            if (near > far) return false;
        }
    }
    return true;
}

double segmentBoxSquared(math::Vec2 a, math::Vec2 b, const Box2D& box) {
    if (segmentIntersectsBox(a, b, box)) return 0.0;
    double best = std::min(pointBoxSquared(a, box), pointBoxSquared(b, box));
    const double x0 = box.center.x - box.halfWidth;
    const double x1 = box.center.x + box.halfWidth;
    const double y0 = box.center.y - box.halfHeight;
    const double y1 = box.center.y + box.halfHeight;
    for (const math::Vec2 corner : {math::Vec2{x0, y0}, {x0, y1},
                                    {x1, y0}, {x1, y1}}) {
        best = std::min(best, pointSegmentSquared(corner, a, b));
    }
    return best;
}

// Collision geometry after validation. Keeping the per-link inner loop free
// of repeated shape and point checks improves planning with multiple obstacles.
bool segmentHitsObstacleUnchecked(math::Vec2 from, math::Vec2 to,
                                  const Obstacle2D& obstacle,
                                  double clearance) {
    return std::visit([&](const auto& value) {
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, Circle2D>) {
            const double radius = value.radius + clearance;
            return pointSegmentSquared(value.center, from, to) <= radius * radius;
        } else {
            return segmentBoxSquared(from, to, value) <= clearance * clearance;
        }
    }, obstacle);
}

// Signed distance to a box: negative inside its solid interior.
double pointBoxSignedDistance(math::Vec2 point, const Box2D& box) {
    const double dx = std::abs(point.x - box.center.x) - box.halfWidth;
    const double dy = std::abs(point.y - box.center.y) - box.halfHeight;
    return std::hypot(std::max(dx, 0.0), std::max(dy, 0.0)) +
           std::min(std::max(dx, dy), 0.0);
}

double segmentBoxSignedDistance(math::Vec2 from, math::Vec2 to, const Box2D& box) {
    if (!segmentIntersectsBox(from, to, box))
        return std::sqrt(segmentBoxSquared(from, to, box));

    // The signed distance to an axis-aligned box is convex along any
    // straight segment. Minimize it in one dimension to also measure
    // intersections where both endpoints are outside the rectangle.
    // Fixed iteration count keeps deterministic accuracy (~1e-10 in t).
    const auto diff = to - from;
    double lo = 0.0, hi = 1.0;
    constexpr double ratio = 0.6180339887498948482;
    double a = hi - ratio * (hi - lo);
    double b = lo + ratio * (hi - lo);
    double fa = pointBoxSignedDistance(from + diff * a, box);
    double fb = pointBoxSignedDistance(from + diff * b, box);
    for (int i = 0; i < 56; ++i) {
        if (fa < fb) {
            hi = b;
            b = a; fb = fa;
            a = hi - ratio * (hi - lo);
            fa = pointBoxSignedDistance(from + diff * a, box);
        } else {
            lo = a;
            a = b; fa = fb;
            b = lo + ratio * (hi - lo);
            fb = pointBoxSignedDistance(from + diff * b, box);
        }
    }
    return std::min({fa, fb, pointBoxSignedDistance(from, box),
                     pointBoxSignedDistance(to, box)});
}

void requireValid(const Obstacle2D& obstacle, double clearance) {
    if (!validObstacle(obstacle) || !std::isfinite(clearance) || clearance < 0.0)
        throw std::invalid_argument("Invalid obstacle geometry or clearance");
}

} // namespace

bool validObstacle(const Obstacle2D& obstacle) noexcept {
    return std::visit([](const auto& value) {
        if (!finite(value.center)) return false;
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, Circle2D>) {
            return std::isfinite(value.radius) && value.radius > 0.0;
        } else {
            return std::isfinite(value.halfWidth) && value.halfWidth > 0.0 &&
                   std::isfinite(value.halfHeight) && value.halfHeight > 0.0;
        }
    }, obstacle);
}

bool pointHitsObstacle(math::Vec2 point, const Obstacle2D& obstacle,
                       double clearance) {
    requireValid(obstacle, clearance);
    if (!finite(point))
        throw std::invalid_argument("Invalid obstacle query point");
    return std::visit([&](const auto& value) {
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, Circle2D>) {
            const double radius = value.radius + clearance;
            return squaredDistance(point, value.center) <= radius * radius;
        } else {
            return pointBoxSquared(point, value) <= clearance * clearance;
        }
    }, obstacle);
}

bool segmentHitsObstacle(math::Vec2 from, math::Vec2 to,
                         const Obstacle2D& obstacle, double clearance) {
    requireValid(obstacle, clearance);
    if (!finite(from) || !finite(to))
        throw std::invalid_argument("Invalid arm segment");
    return segmentHitsObstacleUnchecked(from, to, obstacle, clearance);
}

double segmentSignedDistanceToObstacle(
    math::Vec2 from, math::Vec2 to, const Obstacle2D& obstacle) {
    requireValid(obstacle, 0.0);
    if (!finite(from) || !finite(to))
        throw std::invalid_argument("Invalid segment distance query");
    return std::visit([&](const auto& shape) -> double {
        if constexpr (std::is_same_v<std::decay_t<decltype(shape)>, Circle2D>)
            return std::sqrt(pointSegmentSquared(shape.center, from, to)) -
                   shape.radius;
        else
            return segmentBoxSignedDistance(from, to, shape);
    }, obstacle);
}

bool hasObstacleCollision(std::span<const math::Vec2> positions,
                          std::span<const Obstacle2D> obstacles,
                          double clearance) {
    return hasObstacleCollision(positions, obstacles, clearance, {});
}

bool hasObstacleCollision(std::span<const math::Vec2> positions,
                          std::span<const Obstacle2D> obstacles,
                          double clearance, std::span<const double> linkRadii) {
    if (!std::isfinite(clearance) || clearance < 0.0)
        throw std::invalid_argument("Invalid obstacle clearance");
    validateLinkRadii(linkRadii, positions.empty() ? 0 : positions.size() - 1);
    for (const auto& p : positions)
        if (!finite(p)) throw std::invalid_argument("Invalid arm position");
    for (const auto& obstacle : obstacles) {
        requireValid(obstacle, clearance);
        for (std::size_t i = 1; i < positions.size(); ++i)
            if (segmentHitsObstacleUnchecked(positions[i - 1], positions[i],
                                             obstacle,
                                             clearance + linkRadiusAt(linkRadii, i - 1)))
                return true;
    }
    return false;
}

} // namespace robotics::geometry
