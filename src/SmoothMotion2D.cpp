#include "robotics/kinematics/SmoothMotion2D.hpp"

#include "robotics/geometry/LinkRadii2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/InverseKinematics2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace robotics::kinematics {
namespace {
constexpr double pi = std::numbers::pi;
constexpr double twicePi = 2.0 * pi;

bool finite(math::Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

int sideOfLine(math::Vec2 a, math::Vec2 b, math::Vec2 p) {
    const auto direction = b - a;
    const auto offset = p - a;
    const double directionScale = std::max(std::abs(direction.x), std::abs(direction.y));
    const double offsetScale = std::max(std::abs(offset.x), std::abs(offset.y));
    if (directionScale == 0.0 || offsetScale == 0.0 ||
        !std::isfinite(directionScale) || !std::isfinite(offsetScale))
        return 0;

    // Normalize before taking the determinant. A fixed epsilon in square
    // metres can hide crossings of short links. Ambiguous signs remain
    // possible contact if the segments' bounding boxes overlap.
    const double left = (direction.x / directionScale) * (offset.y / offsetScale);
    const double right = (direction.y / directionScale) * (offset.x / offsetScale);
    const double determinant = left - right;
    const double uncertainty = 8.0 * std::numeric_limits<double>::epsilon() *
                               (std::abs(left) + std::abs(right));
    if (determinant > uncertainty) return 1;
    if (determinant < -uncertainty) return -1;
    return 0;
}

double pointDistance(math::Vec2 p, math::Vec2 a, math::Vec2 b) {
    const auto ab = b - a;
    const auto offset = p - a;
    const double length = ab.length();
    if (length == 0.0) return offset.length();
    if (!std::isfinite(length)) return 0.0;
    const math::Vec2 direction{ab.x / length, ab.y / length};
    const double projected = std::clamp(offset.dot(direction), 0.0, length);
    const double distance = (offset - direction * projected).length();
    return std::isnan(distance) ? 0.0 : distance;
}

double segmentDistance(math::Vec2 a, math::Vec2 b, math::Vec2 c, math::Vec2 d) {
    const auto overlaps = [](double a, double b, double c, double d) {
        return std::max(a, b) >= std::min(c, d) &&
               std::max(c, d) >= std::min(a, b);
    };
    if (overlaps(a.x, b.x, c.x, d.x) && overlaps(a.y, b.y, c.y, d.y) &&
        sideOfLine(a, b, c) * sideOfLine(a, b, d) <= 0 &&
        sideOfLine(c, d, a) * sideOfLine(c, d, b) <= 0)
        return 0.0;

    return std::min({pointDistance(a, c, d), pointDistance(b, c, d),
                     pointDistance(c, a, b), pointDistance(d, a, b)});
}

// Keep each internal joint on its original branch and away from collinearity.
bool allowedAngles(std::span<const Joint2D> current,
                   std::span<const Joint2D> next, double straightMargin,
                   double foldMargin) {
    for (std::size_t i = 1; i < current.size(); ++i) {
        const double sign = std::signbit(current[i].angle) ? -1.0 : 1.0;
        const double folded = sign * next[i].angle;
        if (folded < straightMargin || folded > pi - foldMargin) return false;
    }
    return true;
}

bool withinLimits(std::span<const Joint2D> joints,
                  std::span<const JointLimits2D> limits) {
    if (limits.empty()) return true;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (joints[i].angle < limits[i].minAngle - 1e-9 ||
            joints[i].angle > limits[i].maxAngle + 1e-9)
            return false;
    }
    return true;
}

double slewVelocity(double current, double desired, double acceleration, double dt) {
    return current + std::clamp(desired - current, -acceleration * dt,
                                acceleration * dt);
}

// Use proportional feedback, a braking-distance bound and acceleration limiting.
double servoVelocity(double angle, double goal, double velocity,
                     double dt, double maxSpeed, double maxAcceleration) {
    const double delta = goal - angle;
    const double braking = std::sqrt(2.0 * maxAcceleration * std::abs(delta));
    const double targetSpeed = std::copysign(
        std::min({maxSpeed, 4.0 * std::abs(delta), braking}), delta);
    return slewVelocity(velocity, targetSpeed, maxAcceleration, dt);
}

// Closed-form two-link IK, preserving the current elbow branch.
std::vector<double> analyticTwoLinkPose(std::span<const Joint2D> joints,
                                        math::Vec2 target, double straightMargin,
                                        double foldMargin, math::Vec2 tip) {
    const double l1 = joints[0].length, l2 = joints[1].length;
    const double r = target.length();
    const double minR = std::hypot(l1 + l2 * std::cos(pi - foldMargin),
                                  l2 * std::sin(pi - foldMargin));
    const double maxR = std::hypot(l1 + l2 * std::cos(straightMargin),
                                  l2 * std::sin(straightMargin));
    const double clampedR = std::clamp(r, minR, maxR);
    const double bearing = r > 0.01 ? std::atan2(target.y, target.x)
                                    : std::atan2(tip.y, tip.x);
    const double cosine = std::clamp(
        (clampedR * clampedR - l1 * l1 - l2 * l2) / (2.0 * l1 * l2), -1.0, 1.0);
    const double sign = std::signbit(joints[1].angle) ? -1.0 : 1.0;
    const double elbow = sign * std::acos(cosine);
    const double shoulder = bearing - std::atan2(l2 * std::sin(elbow),
                                                l1 + l2 * std::cos(elbow));
    return {joints[0].angle + std::remainder(shoulder - joints[0].angle, twicePi),
            elbow};
}

} // namespace

namespace {

void validateTargetGeometry(std::span<const math::Vec2> positions,
                            math::Vec2 target, double clearance) {
    if (positions.size() < 2 || !finite(target) || !std::isfinite(clearance) ||
        clearance < 0.0)
        throw std::invalid_argument("Invalid target approach geometry");
    for (const auto& point : positions)
        if (!finite(point)) throw std::invalid_argument("Invalid arm point");
}

// Shared point-to-link calculation: the *same* tool-tip allowance is used
// by both the allocation-free safety predicate and penetration-depth queries.
double targetDepthAt(std::span<const math::Vec2> positions, std::size_t index,
                     math::Vec2 target, double clearance, double linkRadius,
                     bool toolNearTarget) {
    auto shaftEnd = positions[index + 1];
    if (index + 2 == positions.size() && toolNearTarget) {
        const double length = (shaftEnd - positions[index]).length();
        const double toolLength = std::min(2.0 * clearance, 0.4 * length);
        shaftEnd = toolContactStart2D(positions[index], shaftEnd, toolLength);
    }
    return std::max(0.0,
        clearance + linkRadius - pointDistance(target, positions[index], shaftEnd));
}

} // namespace

void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, double clearance,
                            std::span<double> depths) {
    targetLinkPenetrations(positions, target, clearance, depths, {});
}

void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, double clearance,
                            std::span<double> depths, std::span<const double> linkRadii) {
    validateTargetGeometry(positions, target, clearance);
    geometry::validateLinkRadii(linkRadii, positions.size() - 1);
    if (depths.size() != positions.size() - 1)
        throw std::invalid_argument("Target-depth buffer must match link count");
    std::fill(depths.begin(), depths.end(), 0.0);
    if (clearance == 0.0) return;

    const bool toolNearTarget = (target - positions.back()).length() <=
        2.0 * clearance + geometry::linkRadiusAt(linkRadii, depths.size() - 1);
    for (std::size_t i = 0; i < depths.size(); ++i)
        depths[i] = targetDepthAt(positions, i, target, clearance,
                                  geometry::linkRadiusAt(linkRadii, i), toolNearTarget);
}

std::vector<double> targetLinkPenetrations(
    std::span<const math::Vec2> positions, math::Vec2 target, double clearance) {
    std::vector<double> depths(positions.empty() ? 0 : positions.size() - 1);
    targetLinkPenetrations(positions, target, clearance, depths);
    return depths;
}

bool targetApproachClear(std::span<const math::Vec2> positions,
                         math::Vec2 target, double clearance) {
    return targetApproachClear(positions, target, clearance, {});
}

bool targetApproachClear(std::span<const math::Vec2> positions,
                         math::Vec2 target, double clearance,
                         std::span<const double> linkRadii) {
    validateTargetGeometry(positions, target, clearance);
    const std::size_t linkCount = positions.size() - 1;
    geometry::validateLinkRadii(linkRadii, linkCount);
    if (clearance == 0.0) return true;

    const bool toolNearTarget = (target - positions.back()).length() <=
        2.0 * clearance + geometry::linkRadiusAt(linkRadii, linkCount - 1);
    for (std::size_t i = 0; i < linkCount; ++i)
        if (targetDepthAt(positions, i, target, clearance,
                          geometry::linkRadiusAt(linkRadii, i), toolNearTarget) != 0.0)
            return false;
    return true;
}


bool validTargetObject2D(const TargetObject2D& object) noexcept {
    return (object.shape == TargetShape2D::Circle ||
            object.shape == TargetShape2D::Rectangle) &&
           std::isfinite(object.radius) && object.radius > 0.0 &&
           std::isfinite(object.halfWidth) && object.halfWidth > 0.0 &&
           std::isfinite(object.halfHeight) && object.halfHeight > 0.0 &&
           std::isfinite(object.clearance) && object.clearance >= 0.0 &&
           std::isfinite(object.toolLength) && object.toolLength > 0.0;
}

geometry::Obstacle2D targetBody2D(const TargetObject2D& object,
                                  math::Vec2 center) {
    if (!validTargetObject2D(object) || !finite(center))
        throw std::invalid_argument("Invalid target object geometry");
    if (object.shape == TargetShape2D::Circle)
        return geometry::Circle2D{center, object.radius};
    return geometry::Box2D{center, object.halfWidth, object.halfHeight};
}

double targetObjectBound2D(const TargetObject2D& object) noexcept {
    return object.shape == TargetShape2D::Circle ?
        object.radius + object.clearance :
        std::hypot(object.halfWidth, object.halfHeight) + object.clearance;
}

math::Vec2 toolContactStart2D(math::Vec2 start, math::Vec2 tip, double toolLength) {
    const auto offset = tip - start;
    const double length = offset.length();
    if (!std::isfinite(length) || !std::isfinite(toolLength) || toolLength < 0.0)
        throw std::invalid_argument("Invalid tool contact geometry");
    if (length <= toolLength) return start;
    return start + offset * ((length - toolLength) / length);
}

namespace {

void validateObjectContact(std::span<const math::Vec2> positions,
                           math::Vec2 center, const TargetObject2D& object,
                           std::size_t depthCount) {
    if (!validTargetObject2D(object) || positions.size() < 2 ||
        depthCount != positions.size() - 1 || !finite(center))
        throw std::invalid_argument("Invalid target object or arm geometry");
    for (const auto& point : positions)
        if (!finite(point))
            throw std::invalid_argument("Invalid arm point");
    const double lastLinkLength =
        (positions.back() - positions[positions.size() - 2]).length();
    if (lastLinkLength <= object.toolLength)
        throw std::invalid_argument("Tool contact length must be shorter than final link");
}

math::Vec2 protectedEnd(std::span<const math::Vec2> positions,
                        std::size_t index, const TargetObject2D& object) {
    const auto end = positions[index + 1];
    if (index + 2 != positions.size()) return end;
    return toolContactStart2D(positions[index], end, object.toolLength);
}

} // namespace

void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, const TargetObject2D& object,
                            std::span<double> depths) {
    targetLinkPenetrations(positions, target, object, depths, {});
}

void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, const TargetObject2D& object,
                            std::span<double> depths, std::span<const double> linkRadii) {
    validateObjectContact(positions, target, object, depths.size());
    geometry::validateLinkRadii(linkRadii, depths.size());
    const auto body = targetBody2D(object, target);
    for (std::size_t i = 0; i < depths.size(); ++i) {
        const double signedDistance = geometry::segmentSignedDistanceToObstacle(
            positions[i], protectedEnd(positions, i, object), body);
        depths[i] = std::max(0.0,
            object.clearance + geometry::linkRadiusAt(linkRadii, i) - signedDistance);
    }
}

std::vector<double> targetLinkPenetrations(
    std::span<const math::Vec2> positions, math::Vec2 target,
    const TargetObject2D& object) {
    std::vector<double> depths(positions.empty() ? 0 : positions.size() - 1);
    targetLinkPenetrations(positions, target, object, depths);
    return depths;
}

bool targetApproachClear(std::span<const math::Vec2> positions,
                         math::Vec2 target, const TargetObject2D& object) {
    return targetApproachClear(positions, target, object, {});
}

bool targetApproachClear(std::span<const math::Vec2> positions,
                         math::Vec2 target, const TargetObject2D& object,
                         std::span<const double> linkRadii) {
    validateObjectContact(positions, target, object, positions.size() - 1);
    geometry::validateLinkRadii(linkRadii, positions.size() - 1);
    const auto body = targetBody2D(object, target);
    for (std::size_t i = 0; i + 1 < positions.size(); ++i)
        if (geometry::segmentHitsObstacle(
                positions[i], protectedEnd(positions, i, object),
                body, object.clearance + geometry::linkRadiusAt(linkRadii, i)))
            return false;
    return true;
}

bool hasSelfCollision(std::span<const math::Vec2> positions, double clearance) {
    return hasSelfCollision(positions, clearance, {});
}

bool hasSelfCollision(std::span<const math::Vec2> positions, double clearance,
                      std::span<const double> linkRadii) {
    if (!std::isfinite(clearance) || clearance < 0.0)
        throw std::invalid_argument("Self collision clearance must be finite and nonnegative");
    for (const auto& position : positions) {
        if (!finite(position)) throw std::invalid_argument("Invalid position");
    }
    const std::size_t links = positions.empty() ? 0 : positions.size() - 1;
    geometry::validateLinkRadii(linkRadii, links);
    for (std::size_t i = 0; i < links; ++i) {
        for (std::size_t j = i + 2; j < links; ++j) {
            if (segmentDistance(positions[i], positions[i + 1],
                                positions[j], positions[j + 1]) <=
                clearance + geometry::linkRadiusAt(linkRadii, i) +
                    geometry::linkRadiusAt(linkRadii, j))
                return true;
        }
    }
    return false;
}

std::vector<double> linkRadii2D(std::span<const Joint2D> joints) {
    if (std::all_of(joints.begin(), joints.end(),
                    [](const Joint2D& joint) { return joint.linkRadius == 0.0; }))
        return {};
    std::vector<double> radii;
    radii.reserve(joints.size());
    for (const auto& joint : joints) {
        if (!geometry::validLinkRadius(joint.linkRadius))
            throw std::invalid_argument("Link radii must be finite and nonnegative");
        radii.push_back(joint.linkRadius);
    }
    return radii;
}

MotionStatus2D advanceTowardTarget(std::span<Joint2D> joints, math::Vec2 target,
                                   double dt, const MotionSettings2D& options,
                                   std::span<const JointLimits2D> limits) {
    if (joints.size() < 2 || !finite(target) || !std::isfinite(dt) ||
        dt <= 0.0 || dt > 0.1 ||
        !std::isfinite(options.maxAngularSpeed) || options.maxAngularSpeed <= 0.0 ||
        !std::isfinite(options.maxAngularAcceleration) || options.maxAngularAcceleration <= 0.0 ||
        !std::isfinite(options.jointClearance) || options.jointClearance <= 0.0 ||
        options.jointClearance >= pi / 2.0 ||
        !std::isfinite(options.foldClearance) ||
        options.foldClearance < options.jointClearance ||
        options.foldClearance >= pi / 2.0 ||
        !std::isfinite(options.targetClearance) || options.targetClearance < 0.0 ||
        !std::isfinite(options.linkClearance) ||
        options.linkClearance < 0.0 || !std::isfinite(options.positionTolerance) ||
        options.positionTolerance <= 0.0) {
        throw std::invalid_argument("Invalid motion parameters");
    }
    if (!limits.empty() && limits.size() != joints.size())
        throw std::invalid_argument("Joint limits must match the number of joints");
    double totalLength = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        if (!std::isfinite(joint.length) || joint.length <= 0.0 ||
            !std::isfinite(joint.angle) || !std::isfinite(joint.angularVelocity) ||
            std::abs(joint.angularVelocity) > options.maxAngularSpeed + 1e-9)
            throw std::invalid_argument("Invalid joint state");
        if (!limits.empty()) {
            const auto& lim = limits[i];
            if (!std::isfinite(lim.minAngle) || !std::isfinite(lim.maxAngle) ||
                lim.minAngle >= lim.maxAngle ||
                !std::isfinite(lim.maxAngularSpeed) || lim.maxAngularSpeed <= 0.0 ||
                !std::isfinite(lim.maxAngularAcceleration) || lim.maxAngularAcceleration <= 0.0 ||
                std::abs(joint.angularVelocity) > lim.maxAngularSpeed + 1e-9 ||
                joint.angle < lim.minAngle - 1e-9 || joint.angle > lim.maxAngle + 1e-9)
                throw std::invalid_argument("Invalid individual joint limits or state");
        }
        totalLength += joint.length;
    }
    if (!std::isfinite(totalLength)) throw std::overflow_error("Invalid arm reach");
    if (options.targetObject &&
        (!validTargetObject2D(*options.targetObject) ||
         options.targetObject->toolLength >= joints.back().length))
        throw std::invalid_argument("Invalid target object or tool contact length");
    const auto positions = forwardKinematics(joints);
    const auto radii = linkRadii2D(joints);
    if (!allowedAngles(joints, joints, options.jointClearance, options.foldClearance) ||
        hasSelfCollision(positions, options.linkClearance, radii))
        throw std::invalid_argument("Robot starts in an unsafe configuration");

    MotionStatus2D status;
    status.positionError = (target - positions.back()).length();
    if (!std::isfinite(status.positionError))
        throw std::overflow_error("Target position error overflow");
    status.reached = status.positionError <= options.positionTolerance;
    status.outOfReach = target.length() > totalLength;
    if (!options.targetObject &&
        !targetApproachClear(positions, target, options.targetClearance, radii)) {
        status.blocked = true;
        for (auto& joint : joints) joint.angularVelocity = 0.0;
        return status;
    }
    std::vector<double> startAngles;
    std::vector<double> endAngles;
    if (options.targetObject || joints.size() > 2) {
        startAngles.reserve(joints.size());
        for (const auto& joint : joints) startAngles.push_back(joint.angle);
        endAngles.resize(joints.size());
    }
    const auto sweptStepClear = [&](std::span<const Joint2D> next) {
        for (std::size_t i = 0; i < endAngles.size(); ++i)
            endAngles[i] = next[i].angle;
        if (options.targetObject)
            return sweptArmClear2D(joints, startAngles, endAngles, {},
                                   options.linkClearance, 0.0, target,
                                   *options.targetObject);
        return sweptArmClear2D(joints, startAngles, endAngles, {},
                               options.linkClearance, 0.0);
    };
    if (joints.size() == 2) {
        const auto pose = analyticTwoLinkPose(
            joints, target, options.jointClearance, options.foldClearance, positions.back());
        const double targetRadius = target.length();
        const double minRadius = std::hypot(joints[0].length + joints[1].length *
                         std::cos(pi - options.foldClearance),
                         joints[1].length * std::sin(pi - options.foldClearance));
        const double maxRadius = std::hypot(joints[0].length + joints[1].length *
                         std::cos(options.jointClearance),
                         joints[1].length * std::sin(options.jointClearance));
        status.outOfReach = targetRadius < minRadius || targetRadius > maxRadius;

        std::vector<Joint2D> proposal(joints.begin(), joints.end());
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const double goal = limits.empty() ? pose[i] :
                std::clamp(pose[i], limits[i].minAngle, limits[i].maxAngle);
            if (goal != pose[i]) status.constrained = true;
            const double maxSpeed = limits.empty() ? options.maxAngularSpeed :
                std::min(options.maxAngularSpeed, limits[i].maxAngularSpeed);
            const double maxAcceleration = limits.empty() ? options.maxAngularAcceleration :
                std::min(options.maxAngularAcceleration, limits[i].maxAngularAcceleration);
            const double delta = goal - joints[i].angle;
            double speed = servoVelocity(joints[i].angle, goal,
                joints[i].angularVelocity, dt, maxSpeed, maxAcceleration);
            if (std::abs(delta) < 1e-8 && std::abs(speed) < maxAcceleration * dt)
                speed = 0.0;
            double nextAngle = joints[i].angle + speed * dt;
            if (std::abs(speed) <= maxAcceleration * dt &&
                ((delta >= 0.0 && nextAngle > goal) ||
                 (delta < 0.0 && nextAngle < goal))) {
                nextAngle = goal;
                speed = 0.0;
            }
            if (!limits.empty() &&
                (nextAngle < limits[i].minAngle || nextAngle > limits[i].maxAngle)) {
                nextAngle = std::clamp(nextAngle, limits[i].minAngle, limits[i].maxAngle);
                speed = (nextAngle - joints[i].angle) / dt;
                status.constrained = true;
            }
            proposal[i].angle = nextAngle;
            proposal[i].angularVelocity = speed;
        }
        if (!allowedAngles(joints, proposal, options.jointClearance - 1e-9,
                           options.foldClearance - 1e-9)) {
            status.constrained = true;
            // Handle roundoff at the configured elbow margin.
            proposal[1].angle = std::clamp(proposal[1].angle,
                std::signbit(joints[1].angle) ? -pi + options.foldClearance : options.jointClearance,
                std::signbit(joints[1].angle) ? -options.jointClearance : pi - options.foldClearance);
            proposal[1].angularVelocity = (proposal[1].angle - joints[1].angle) / dt;
        }
        // Guard the entire per-frame sweep, not only the proposed endpoint.
        // If the selected target lies on the arm, stop safely instead of
        // pushing an intermediate link through the object.
        if (options.targetObject) {
            if (!sweptStepClear(proposal)) {
                status.blocked = true;
                status.constrained = true;
                for (auto& joint : joints) joint.angularVelocity = 0.0;
                return status;
            }
        } else if (options.targetClearance > 0.0) {
            std::vector<Joint2D> sweep(joints.begin(), joints.end());
            for (int sub = 1; sub <= 8; ++sub) {
                for (std::size_t i = 0; i < joints.size(); ++i)
                    sweep[i].angle = joints[i].angle +
                        (proposal[i].angle - joints[i].angle) *
                        (static_cast<double>(sub) / 8.0);
                if (!targetApproachClear(forwardKinematics(sweep), target,
                                         options.targetClearance, radii)) {
                    status.blocked = true;
                    status.constrained = true;
                    for (auto& joint : joints) joint.angularVelocity = 0.0;
                    return status;
                }
            }
        }
        std::copy(proposal.begin(), proposal.end(), joints.begin());
    } else {
        // Damped least squares for redundant arms, plus a collision-checked step.
        math::Vec2 effectiveTarget = target;
        const double targetRadius = target.length();
        if (targetRadius > totalLength - 0.01) {
            effectiveTarget = target * ((totalLength - 0.01) / targetRadius);
        }
        const auto command = computeIKCommand(joints, effectiveTarget);
        std::vector<Joint2D> proposal(joints.begin(), joints.end());
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const double maxAcceleration = limits.empty() ? options.maxAngularAcceleration :
                std::min(options.maxAngularAcceleration, limits[i].maxAngularAcceleration);
            const double maxSpeed = limits.empty() ? options.maxAngularSpeed :
                std::min(options.maxAngularSpeed, limits[i].maxAngularSpeed);
            proposal[i].angularVelocity = slewVelocity(
                joints[i].angularVelocity, command.angularVelocities[i],
                maxAcceleration, dt);
            proposal[i].angularVelocity = std::clamp(proposal[i].angularVelocity,
                -maxSpeed, maxSpeed);
        }
        // Backtrack the step if a joint limit or self-collision is predicted.
        // Substeps also guard against passing through thin links in one frame.
        double scale = 1.0;
        bool safe = false;
        std::vector<Joint2D> intermediate(joints.begin(), joints.end());
        for (int attempt = 0; attempt < 18; ++attempt, scale *= 0.5) {
            safe = true;
            for (int sub = 1; sub <= 4; ++sub) {
                for (std::size_t i = 0; i < joints.size(); ++i) {
                    intermediate[i] = joints[i];
                    intermediate[i].angle += proposal[i].angularVelocity * dt * scale *
                                             (static_cast<double>(sub) / 4.0);
                }
                if (!allowedAngles(joints, intermediate, options.jointClearance,
                                   options.foldClearance) ||
                    !withinLimits(intermediate, limits)) {
                    safe = false;
                    break;
                }
                if (!options.targetObject &&
                    !targetApproachClear(forwardKinematics(intermediate), target,
                                         options.targetClearance, radii)) {
                    safe = false;
                    break;
                }
            }
            if (safe && sweptStepClear(intermediate)) break;
            safe = false;
        }
        if (scale < 1.0 || !safe) status.constrained = true;
        if (!safe) {
            status.blocked = true;
            for (auto& joint : joints) joint.angularVelocity = 0.0;
        } else {
            for (std::size_t i = 0; i < joints.size(); ++i) {
                joints[i].angularVelocity = proposal[i].angularVelocity * scale;
                joints[i].angle += joints[i].angularVelocity * dt;
            }
        }
    }
    status.positionError = (target - forwardKinematics(joints).back()).length();
    status.reached = status.positionError <= options.positionTolerance;
    return status;
}

} // namespace robotics::kinematics
