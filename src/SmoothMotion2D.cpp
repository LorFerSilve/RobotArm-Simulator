#include "robotics/kinematics/SmoothMotion2D.hpp"

#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/InverseKinematics2D.hpp"

#include <algorithm>
#include <cmath>
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

double cross(math::Vec2 a, math::Vec2 b) {
    return a.x * b.y - a.y * b.x;
}

double pointDistance(math::Vec2 p, math::Vec2 a, math::Vec2 b) {
    const auto ab = b - a;
    const double d = ab.dot(ab);
    const double t = d > 0.0 ? std::clamp((p - a).dot(ab) / d, 0.0, 1.0) : 0.0;
    return (p - (a + ab * t)).length();
}

double segmentDistance(math::Vec2 a, math::Vec2 b, math::Vec2 c, math::Vec2 d) {
    const auto ab = b - a;
    const auto cd = d - c;
    const double abCrossCd = cross(ab, cd);
    if (std::abs(abCrossCd) > 1e-12) {
        const double t = cross(c - a, cd) / abCrossCd;
        const double u = cross(c - a, ab) / abCrossCd;
        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
            return 0.0;
        }
    }
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

std::vector<double> targetLinkPenetrations(std::span<const math::Vec2> positions,
                                           math::Vec2 target, double clearance) {
    if (positions.size() < 2 || !finite(target) || !std::isfinite(clearance) ||
        clearance < 0.0)
        throw std::invalid_argument("Invalid target approach geometry");
    for (const auto& point : positions)
        if (!finite(point)) throw std::invalid_argument("Invalid arm point");

    std::vector<double> depths(positions.size() - 1, 0.0);
    if (clearance == 0.0) return depths;

    const std::size_t last = depths.size() - 1;
    const bool toolNearTarget = (target - positions.back()).length() <=
                                2.0 * clearance;
    for (std::size_t i = 0; i < depths.size(); ++i) {
        // Only the final link gets the tool-tip allowance.
        if (i == last && toolNearTarget) continue;
        const double distance = pointDistance(target, positions[i],
                                               positions[i + 1]);
        depths[i] = std::max(0.0, clearance - distance);
    }
    return depths;
}

bool targetApproachClear(std::span<const math::Vec2> positions,
                         math::Vec2 target, double clearance) {
    const auto depths = targetLinkPenetrations(positions, target, clearance);
    return std::all_of(depths.begin(), depths.end(),
                       [](double depth) { return depth == 0.0; });
}

bool hasSelfCollision(std::span<const math::Vec2> positions, double clearance) {
    if (!std::isfinite(clearance) || clearance < 0.0)
        throw std::invalid_argument("Self collision clearance must be finite and nonnegative");
    for (const auto& position : positions) {
        if (!finite(position)) throw std::invalid_argument("Invalid position");
    }
    const std::size_t links = positions.empty() ? 0 : positions.size() - 1;
    for (std::size_t i = 0; i < links; ++i) {
        for (std::size_t j = i + 2; j < links; ++j) {
            if (segmentDistance(positions[i], positions[i + 1],
                                positions[j], positions[j + 1]) <= clearance)
                return true;
        }
    }
    return false;
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
    const auto positions = forwardKinematics(joints);
    if (!allowedAngles(joints, joints, options.jointClearance, options.foldClearance) ||
        hasSelfCollision(positions, options.linkClearance))
        throw std::invalid_argument("Robot starts in an unsafe configuration");

    MotionStatus2D status;
    status.positionError = (target - positions.back()).length();
    if (!std::isfinite(status.positionError))
        throw std::overflow_error("Target position error overflow");
    status.reached = status.positionError <= options.positionTolerance;
    status.outOfReach = target.length() > totalLength;
    if (!targetApproachClear(positions, target, options.targetClearance)) {
        status.blocked = true;
        for (auto& joint : joints) joint.angularVelocity = 0.0;
        return status;
    }
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
        if (options.targetClearance > 0.0) {
            std::vector<Joint2D> sweep(joints.begin(), joints.end());
            for (int sub = 1; sub <= 8; ++sub) {
                for (std::size_t i = 0; i < joints.size(); ++i)
                    sweep[i].angle = joints[i].angle +
                        (proposal[i].angle - joints[i].angle) *
                        (static_cast<double>(sub) / 8.0);
                if (!targetApproachClear(forwardKinematics(sweep), target,
                                         options.targetClearance)) {
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
                    !withinLimits(intermediate, limits) ||
                    hasSelfCollision(forwardKinematics(intermediate), options.linkClearance) ||
                    !targetApproachClear(forwardKinematics(intermediate), target,
                                         options.targetClearance)) {
                    safe = false;
                    break;
                }
            }
            if (safe) break;
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
