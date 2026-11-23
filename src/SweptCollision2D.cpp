#include "robotics/kinematics/SweptCollision2D.hpp"

#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/SmoothMotion2D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace robotics::kinematics {
namespace {

struct ShaftMotion {
    math::Vec2 start;
    math::Vec2 direction;
    math::Vec2 startVelocity;
    math::Vec2 directionVelocity;
    double length;
    double angleAtStart;
    double startSpeedBound;
    double speedBound;
    double accelerationBound;

    [[nodiscard]] math::Vec2 point(double offset) const {
        return start + direction * offset;
    }

    [[nodiscard]] math::Vec2 velocity(double offset) const {
        return startVelocity + directionVelocity * offset;
    }
};

bool distanceNondecreasingAtFixedCoordinate(const ShaftMotion& shaft, math::Vec2 center,
                                            double first, double last,
                                            double halfInterval, double displacement);

bool circleDistanceNondecreasing(const ShaftMotion& shaft,
                                 const geometry::Circle2D& circle,
                                 double halfInterval, double displacement,
                                 double guard) {
    const auto radial = shaft.start - circle.center;
    const double projection = -radial.dot(shaft.direction);
    const double projectionChange =
        halfInterval * (shaft.startSpeedBound +
                        radial.length() * shaft.directionVelocity.length()) + guard;
    const double first = std::clamp(projection - projectionChange, 0.0, shaft.length);
    const double last = std::clamp(projection + projectionChange, 0.0, shaft.length);
    const auto a = radial + shaft.direction * first;
    const auto b = radial + shaft.direction * last;
    const auto va = shaft.velocity(first);
    const auto vb = shaft.velocity(last);
    const double velocityChange = halfInterval * shaft.accelerationBound;

    // The nearest material point lies in [first,last]. Half the time
    // derivative of its squared distance is radial.dot(velocity). At the
    // midpoint that dot product is affine in the offset, since direction
    // is orthogonal to directionVelocity. Bound it over the time interval.
    const double lower = std::min(a.dot(va), b.dot(vb)) -
        std::max(a.length(), b.length()) * velocityChange -
        (std::max(va.length(), vb.length()) + velocityChange) * displacement;
    return lower > 0.0 || distanceNondecreasingAtFixedCoordinate(
        shaft, circle.center, first, last, halfInterval, displacement);
}

bool clipToSlab(double origin, double direction, double extent,
                double& first, double& last) {
    if (!std::isfinite(extent) || extent < 0.0) return false;
    if (direction == 0.0) return std::abs(origin) <= extent;
    const double a = (-extent - origin) / direction;
    const double b = (extent - origin) / direction;
    first = std::max(first, std::min(a, b));
    last = std::min(last, std::max(a, b));
    return first <= last;
}

bool distanceNondecreasingAtFixedCoordinate(const ShaftMotion& shaft, math::Vec2 center,
                                            double first, double last,
                                            double halfInterval, double displacement) {
    // When both ends straddle the closest region, compare the shaft at
    // the same world coordinate over time. This also handles a shaft
    // parallel to a face, where several material points can tie for closest.
    const bool swapAxes = std::abs(shaft.direction.x) < std::abs(shaft.direction.y);
    const auto coordinates = [swapAxes](math::Vec2 p) {
        return swapAxes ? math::Vec2{p.y, p.x} : p;
    };
    const auto direction = coordinates(shaft.direction);
    const double directionChange = halfInterval * shaft.directionVelocity.length();
    if (!(std::abs(direction.x) > directionChange)) return false;

    const auto start = coordinates(shaft.start - center);
    const auto end = start + direction * shaft.length;
    const double a = start.x + direction.x * first;
    const double b = start.x + direction.x * last;
    const double left = std::min(a, b) - displacement;
    const double right = std::max(a, b) + displacement;
    if (left < std::min(start.x, end.x) + displacement ||
        right > std::max(start.x, end.x) - displacement)
        return false;

    const double l = (left - displacement - start.x) / direction.x;
    const double r = (right + displacement - start.x) / direction.x;
    const double firstOffset = std::max(0.0, std::min(l, r));
    const double lastOffset = std::min(shaft.length, std::max(l, r));
    const double nearY = start.y + direction.y * firstOffset;
    const double farY = start.y + direction.y * lastOffset;
    bool above = std::min(nearY, farY) > displacement;
    bool below = std::max(nearY, farY) < -displacement;
    if (!above && !below && shaft.startSpeedBound == 0.0 && start.y == 0.0) {
        const auto initialDirection = coordinates(
            {std::cos(shaft.angleAtStart), std::sin(shaft.angleAtStart)});
        // A shaft exactly on the centre line at this interval's start
        // may retreat to either side when the derivative proves that side.
        above = below = initialDirection.y == 0.0;
    }
    if (!above && !below) return false;

    // At fixed x the derivative of y is cross(direction, velocity) / ux.
    // Both shapes have nondecreasing signed distance as |y| grows at fixed x.
    const auto va = coordinates(shaft.velocity(firstOffset));
    const auto vb = coordinates(shaft.velocity(lastOffset));
    const double sign = std::copysign(1.0, direction.x);
    const double firstDerivative = sign * (direction.x * va.y - direction.y * va.x);
    const double lastDerivative = sign * (direction.x * vb.y - direction.y * vb.x);
    const double velocityChange = halfInterval * shaft.accelerationBound;
    const double uncertainty = velocityChange +
        directionChange * (std::max(va.length(), vb.length()) + velocityChange);
    return (above && std::min(firstDerivative, lastDerivative) > uncertainty) ||
           (below && std::max(firstDerivative, lastDerivative) < -uncertainty);
}

bool boxDistanceNondecreasing(const ShaftMotion& shaft,
                              const geometry::Box2D& box,
                              double halfInterval, double displacement) {
    const auto relativeStart = shaft.start - box.center;
    const double minimum = geometry::segmentSignedDistanceToObstacle(
        shaft.start, shaft.point(shaft.length), box);
    // Signed distance is 1-Lipschitz. A point that can minimize it at any
    // time is at most 2 * displacement above the midpoint minimum.
    const double band = minimum + 2.0 * displacement;
    double first = 0.0, last = shaft.length;
    if (!clipToSlab(relativeStart.x, shaft.direction.x, box.halfWidth + band,
                    first, last) ||
        !clipToSlab(relativeStart.y, shaft.direction.y, box.halfHeight + band,
                    first, last))
        return false;

    const auto a = relativeStart + shaft.direction * first;
    const auto b = relativeStart + shaft.direction * last;
    const auto va = shaft.velocity(first);
    const auto vb = shaft.velocity(last);
    const double velocityChange = halfInterval * shaft.accelerationBound;
    const std::array faceMinimum{
        std::min(a.x, b.x) - box.halfWidth - displacement,
        -std::max(a.x, b.x) - box.halfWidth - displacement,
        std::min(a.y, b.y) - box.halfHeight - displacement,
        -std::max(a.y, b.y) - box.halfHeight - displacement
    };
    const std::array faceMaximum{
        std::max(a.x, b.x) - box.halfWidth + displacement,
        -std::min(a.x, b.x) - box.halfWidth + displacement,
        std::max(a.y, b.y) - box.halfHeight + displacement,
        -std::min(a.y, b.y) - box.halfHeight + displacement
    };
    const std::array outwardVelocity{
        std::min(va.x, vb.x), -std::max(va.x, vb.x),
        std::min(va.y, vb.y), -std::max(va.y, vb.y)
    };
    // Inside, a face can be active if it can attain the largest signed
    // face distance. Outside, any positive face distance can contribute
    // to the Euclidean distance, including at rounded clearance corners.
    const double activeThreshold = std::min(
        0.0, *std::max_element(faceMinimum.begin(), faceMinimum.end()));
    for (std::size_t i = 0; i < faceMaximum.size(); ++i) {
        if (faceMaximum[i] >= activeThreshold &&
            !(outwardVelocity[i] > velocityChange))
            return distanceNondecreasingAtFixedCoordinate(
                shaft, box.center, first, last, halfInterval, displacement);
    }
    return true;
}

bool sweptArmClear(
    std::span<const Joint2D> joints,
    std::span<const double> startAngles,
    std::span<const double> endAngles,
    std::span<const geometry::Obstacle2D> obstacles,
    double selfClearance,
    double obstacleClearance,
    const geometry::Obstacle2D* targetBody,
    double shaftClearance,
    double toolLength,
    std::stop_token cancellation) {
    if (joints.empty() || joints.size() != startAngles.size() ||
        joints.size() != endAngles.size() ||
        !std::isfinite(selfClearance) || selfClearance < 0.0 ||
        !std::isfinite(obstacleClearance) || obstacleClearance < 0.0)
        throw std::invalid_argument("Invalid swept collision parameters");

    double cumulativeAngularRate = 0.0;
    double maximumPointSpeed = 0.0;
    double reach = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (!std::isfinite(joints[i].length) || joints[i].length <= 0.0 ||
            !std::isfinite(startAngles[i]) || !std::isfinite(endAngles[i]))
            throw std::invalid_argument("Invalid swept joint geometry");
        reach += joints[i].length;
        cumulativeAngularRate += std::abs(endAngles[i] - startAngles[i]);
        maximumPointSpeed += joints[i].length * cumulativeAngularRate;
    }
    if (!std::isfinite(reach) || !std::isfinite(maximumPointSpeed))
        throw std::overflow_error("Swept movement bound overflow");
    for (const auto& obstacle : obstacles)
        if (!geometry::validObstacle(obstacle))
            throw std::invalid_argument("Invalid swept obstacle");

    if (cancellation.stop_requested()) return false;
    // With two links, there are no nonadjacent links to collide.
    if (obstacles.empty() && joints.size() <= 2 && !targetBody) return true;

    constexpr int maxDepth = 24;
    int intervalsRemaining = 4096;
    std::vector<double> angles(joints.size());
    std::vector<math::Vec2> positions;
    positions.reserve(joints.size() + 1);
    std::vector<unsigned char> initialTargetContact;
    if (targetBody) {
        if (toolLength >= joints.back().length)
            throw std::invalid_argument("Tool contact length must be shorter than final link");
        forwardKinematics(joints, startAngles, positions);
        initialTargetContact.resize(joints.size());
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const auto offset = positions[i + 1] - positions[i];
            const double length = joints[i].length;
            const double protectedLength = i + 1 == joints.size() ? length - toolLength : length;
            const auto end = positions[i] + offset * (protectedLength / length);
            initialTargetContact[i] = geometry::segmentHitsObstacle(
                positions[i], end, *targetBody, shaftClearance);
        }
    }

    // A simple Lipschitz bound: for any t in [lo,hi], any link endpoint
    // moves at most (hi-lo)/2 * maximumPointSpeed relative to the midpoint.
    // An entire link segment stays within the same displacement radius
    // because each point is a convex combination of its two endpoints.
    //
    // Therefore if every midpoint segment is farther than that radius
    // from scene obstacles, and nonadjacent links are separated by more
    // than twice the radius, ALL intervening poses are collision-free.
    // Small numerical guard errs on the side of rejecting near-contact.
    const double guard = 1e-12 * std::max(1.0, reach);

    const auto certify = [&](auto&& self, double lo, double hi, int depth) -> bool {
        if (cancellation.stop_requested() || intervalsRemaining == 0) return false;
        --intervalsRemaining;
        const double halfInterval = (hi - lo) * 0.5;
        const double mid = lo + halfInterval;
        for (std::size_t i = 0; i < angles.size(); ++i)
            angles[i] = std::lerp(startAngles[i], endAngles[i], mid);
        forwardKinematics(joints, angles, positions);

        // Midpoint collision means the entire edge is not safe.
        if (!obstacles.empty() &&
            geometry::hasObstacleCollision(
                positions, obstacles, obstacleClearance))
            return false;
        if (joints.size() >= 3 &&
            hasSelfCollision(positions, selfClearance))
            return false;

        const double displacement = halfInterval * maximumPointSpeed + guard;
        const bool sceneCertified = obstacles.empty() ||
            !geometry::hasObstacleCollision(
                positions, obstacles, obstacleClearance + displacement);
        const bool selfCertified = joints.size() <= 2 ||
            !hasSelfCollision(positions, selfClearance + 2.0 * displacement);
        bool targetCertified = true;
        if (targetBody) {
            double angle = 0.0, angleAtStart = 0.0, rate = 0.0, rateBound = 0.0;
            double speedBound = 0.0, accelerationBound = 0.0;
            math::Vec2 velocity{};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                angle += angles[i];
                angleAtStart += std::lerp(startAngles[i], endAngles[i], lo);
                const double delta = endAngles[i] - startAngles[i];
                rate += delta;
                rateBound += std::abs(delta);
                const math::Vec2 direction{std::cos(angle), std::sin(angle)};
                const math::Vec2 directionVelocity{-direction.y * rate, direction.x * rate};
                const double length = joints[i].length;
                const double protectedLength = i + 1 == joints.size() ? length - toolLength : length;
                const ShaftMotion shaft{
                    positions[i], direction, velocity, directionVelocity, protectedLength,
                    angleAtStart,
                    speedBound, speedBound + protectedLength * rateBound,
                    accelerationBound + protectedLength * rateBound * rateBound
                };
                const double shaftDisplacement = halfInterval * shaft.speedBound + guard;
                if (geometry::segmentHitsObstacle(shaft.start, shaft.point(shaft.length),
                                                   *targetBody, shaftClearance + shaftDisplacement)) {
                    if (initialTargetContact[i]) {
                        // Each interval is either entirely clear or has
                        // nondecreasing distance. A link that becomes clear
                        // cannot re-enter, even across interval boundaries.
                        if (shaft.speedBound > 0.0) {
                            const auto* circle = std::get_if<geometry::Circle2D>(targetBody);
                            const bool retreats = circle ?
                                circleDistanceNondecreasing(shaft, *circle, halfInterval,
                                                            shaftDisplacement, guard) :
                                boxDistanceNondecreasing(shaft, std::get<geometry::Box2D>(*targetBody),
                                                         halfInterval, shaftDisplacement);
                            targetCertified = targetCertified && retreats;
                        }
                    } else {
                        if (geometry::segmentHitsObstacle(shaft.start, shaft.point(shaft.length),
                                                          *targetBody, shaftClearance))
                            return false;
                        targetCertified = false;
                    }
                }
                velocity = velocity + directionVelocity * length;
                speedBound += length * rateBound;
                accelerationBound += length * rateBound * rateBound;
            }
        }
        if (sceneCertified && selfCertified && targetCertified) return true;

        // No maximum-interval assumption: if we cannot prove safety,
        // conservatively reject instead of allowing a missed collision.
        if (depth == maxDepth) return false;
        return self(self, lo, mid, depth + 1) &&
               self(self, mid, hi, depth + 1);
    };

    return certify(certify, 0.0, 1.0, 0);
}

} // namespace

bool sweptArmClear2D(
    std::span<const Joint2D> joints, std::span<const double> startAngles,
    std::span<const double> endAngles, std::span<const geometry::Obstacle2D> obstacles,
    double selfClearance, double obstacleClearance, std::stop_token cancellation) {
    return sweptArmClear(joints, startAngles, endAngles, obstacles, selfClearance,
                         obstacleClearance, nullptr, 0.0, 0.0, cancellation);
}

bool sweptArmClear2D(
    std::span<const Joint2D> joints, std::span<const double> startAngles,
    std::span<const double> endAngles, std::span<const geometry::Obstacle2D> obstacles,
    double selfClearance, double obstacleClearance, math::Vec2 target,
    const TargetObject2D& targetObject, std::stop_token cancellation) {
    const auto body = targetBody2D(targetObject, target);
    return sweptArmClear(joints, startAngles, endAngles, obstacles, selfClearance,
                         obstacleClearance, &body, targetObject.clearance,
                         targetObject.toolLength, cancellation);
}

} // namespace robotics::kinematics
