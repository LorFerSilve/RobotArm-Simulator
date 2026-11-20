#include "robotics/kinematics/PathPlanning2D.hpp"

#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <utility>

namespace robotics::kinematics {
namespace {

using Angles = std::vector<double>;
constexpr double pi = std::numbers::pi;
constexpr double edgeResolution = 0.04; // maximum angular displacement per check
constexpr double searchStep = 0.32;

struct SearchContext {
    std::vector<Joint2D> joints;
    std::vector<JointLimits2D> limits;
    const MotionSettings2D& settings;
    math::Vec2 target;

    explicit SearchContext(std::span<const Joint2D> state,
                           std::span<const JointLimits2D> bounds,
                           const MotionSettings2D& s, math::Vec2 goal)
        : joints(state.begin(), state.end()), limits(bounds.begin(), bounds.end()), settings(s), target(goal) {}

    [[nodiscard]] Angles angles() const {
        Angles q;
        q.reserve(joints.size());
        for (const auto& joint : joints) q.push_back(joint.angle);
        return q;
    }

    [[nodiscard]] std::vector<math::Vec2> positions(const Angles& q) const {
        auto robot = joints;
        for (std::size_t i = 0; i < q.size(); ++i) robot[i].angle = q[i];
        return forwardKinematics(robot);
    }

    [[nodiscard]] bool validRobot(const Angles& q) const {
        if (q.size() != joints.size()) return false;
        for (std::size_t i = 0; i < q.size(); ++i) {
            if (!std::isfinite(q[i]) || q[i] < limits[i].minAngle ||
                q[i] > limits[i].maxAngle) return false;
            if (i > 0) {
                const bool negative = std::signbit(joints[i].angle);
                if ((negative && (q[i] > -settings.jointClearance ||
                                  q[i] < -pi + settings.foldClearance)) ||
                    (!negative && (q[i] < settings.jointClearance ||
                                   q[i] > pi - settings.foldClearance)))
                    return false;
            }
        }
        return !hasSelfCollision(positions(q), settings.linkClearance);
    }

    [[nodiscard]] bool valid(const Angles& q) const {
        return validRobot(q) &&
               targetApproachClear(positions(q), target, settings.targetClearance);
    }

    // Only a geometric IK search: its intermediate iterations are not real
    // robot movements, so the target-object keep-out zone is not imposed here.
    [[nodiscard]] bool edgeRobot(const Angles& a, const Angles& b) const {
        if (!validRobot(a)) return false;
        double maxDelta = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i)
            maxDelta = std::max(maxDelta, std::abs(a[i] - b[i]));
        const auto steps = std::max(1, static_cast<int>(std::ceil(maxDelta / edgeResolution)));
        Angles q(a.size());
        for (int step = 1; step <= steps; ++step) {
            const double alpha = static_cast<double>(step) / steps;
            for (std::size_t i = 0; i < q.size(); ++i)
                q[i] = a[i] + alpha * (b[i] - a[i]);
            if (!validRobot(q)) return false;
        }
        return true;
    }

    // A *physical* path must keep the target safe. If a newly selected target
    // is already touching a link, allow a retreat only while penetration
    // decreases monotonically for every link (never introduce new contact).
    // Once outside the keep-out zone, re-entry is forbidden.
    [[nodiscard]] bool edge(const Angles& a, const Angles& b) const {
        if (!validRobot(a)) return false;
        auto previous = targetLinkPenetrations(positions(a), target,
                                                settings.targetClearance);
        double maxDelta = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i)
            maxDelta = std::max(maxDelta, std::abs(a[i] - b[i]));
        const auto steps = std::max(1, static_cast<int>(std::ceil(maxDelta / edgeResolution)));
        Angles q(a.size());
        for (int step = 1; step <= steps; ++step) {
            const double alpha = static_cast<double>(step) / steps;
            for (std::size_t i = 0; i < q.size(); ++i)
                q[i] = a[i] + alpha * (b[i] - a[i]);
            if (!validRobot(q)) return false;
            const auto next = targetLinkPenetrations(positions(q), target,
                                                     settings.targetClearance);
            for (std::size_t i = 0; i < next.size(); ++i)
                if (next[i] > previous[i] + 1e-9) return false;
            previous = next;
        }
        return true;
    }
};

double distanceSquared(const Angles& a, const Angles& b) {
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    return sum;
}

// A constrained, damped Jacobian iteration; unlike the per-frame DLS motion
// controller, this searches for a static *goal configuration* before moving.
// Random restarts can find alternative redundant configurations.
[[nodiscard]] bool refineIK(const SearchContext& c, Angles& q, math::Vec2 target) {
    for (int step = 0; step < 180; ++step) {
        const auto positions = c.positions(q);
        const auto e = target - positions.back();
        const double error = e.length();
        if (error <= c.settings.positionTolerance) return c.valid(q);
        const auto jacobian = endEffectorJacobian(positions);

        // Exclude columns that can only move farther outside a saturated bound.
        // Retry with line search to preserve joint limits and self-collision.
        double xx = 0.0025, xy = 0.0, yy = 0.0025;
        for (std::size_t i = 0; i < q.size(); ++i) {
            const auto& column = jacobian[i];
            const double gradient = column.x * e.x + column.y * e.y;
            if ((q[i] <= c.limits[i].minAngle + 1e-6 && gradient < 0.0) ||
                (q[i] >= c.limits[i].maxAngle - 1e-6 && gradient > 0.0)) continue;
            xx += column.x * column.x;
            xy += column.x * column.y;
            yy += column.y * column.y;
        }
        const double determinant = xx * yy - xy * xy;
        if (!std::isfinite(determinant) || determinant <= 1e-15) return false;
        const math::Vec2 w{(yy * e.x - xy * e.y) / determinant,
                           (xx * e.y - xy * e.x) / determinant};
        Angles direction(q.size());
        for (std::size_t i = 0; i < q.size(); ++i)
            direction[i] = std::clamp(jacobian[i].dot(w), -0.3, 0.3);

        bool improved = false;
        for (double alpha = 1.0; alpha >= 1.0 / 128.0; alpha *= 0.5) {
            Angles next = q;
            for (std::size_t i = 0; i < q.size(); ++i)
                next[i] = std::clamp(q[i] + alpha * direction[i],
                                     c.limits[i].minAngle, c.limits[i].maxAngle);
            if (!c.edgeRobot(q, next)) continue;
            const double nextError = (target - c.positions(next).back()).length();
            if (nextError < error - 1e-9) {
                q = std::move(next);
                improved = true;
                break;
            }
        }
        if (!improved) return false;
    }
    return (target - c.positions(q).back()).length() <= c.settings.positionTolerance;
}

struct Node {
    Angles q;
    int parent;
};

[[nodiscard]] int nearest(const std::vector<Node>& tree, const Angles& q) {
    double best = std::numeric_limits<double>::infinity();
    int index = 0;
    for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
        const double d = distanceSquared(tree[static_cast<std::size_t>(i)].q, q);
        if (d < best) { best = d; index = i; }
    }
    return index;
}

[[nodiscard]] Angles steer(const Angles& from, const Angles& to) {
    const double d = std::sqrt(distanceSquared(from, to));
    const double scale = d > searchStep ? searchStep / d : 1.0;
    Angles next(from.size());
    for (std::size_t i = 0; i < from.size(); ++i)
        next[i] = from[i] + scale * (to[i] - from[i]);
    return next;
}

// Returns the newly added node index, or -1 if blocked.
int extend(std::vector<Node>& tree, const Angles& target, const SearchContext& c) {
    const int index = nearest(tree, target);
    const auto& from = tree[static_cast<std::size_t>(index)].q;
    const Angles next = steer(from, target);
    if (distanceSquared(from, next) < 1e-12 || !c.edge(from, next)) return -1;
    tree.push_back({next, index});
    return static_cast<int>(tree.size()) - 1;
}

[[nodiscard]] std::vector<Angles> trace(const std::vector<Node>& tree, int index) {
    std::vector<Angles> result;
    while (index >= 0) {
        const auto& node = tree[static_cast<std::size_t>(index)];
        result.push_back(node.q);
        index = node.parent;
    }
    std::reverse(result.begin(), result.end());
    return result;
}

// Bidirectional RRT-Connect in configuration space. Path only contains
// collision-checked edges; deterministic seed makes regressions reproducible.
[[nodiscard]] std::vector<Angles> connectPath(
    const SearchContext& c, const Angles& start, const Angles& goal,
    std::mt19937& rng) {
    if (c.edge(start, goal)) return {start, goal};
    std::vector<Node> fromStart{{start, -1}};
    std::vector<Node> fromGoal{{goal, -1}};
    for (int iter = 0; iter < 1800; ++iter) {
        const bool forward = iter % 2 == 0;
        auto& growing = forward ? fromStart : fromGoal;
        auto& matching = forward ? fromGoal : fromStart;
        Angles sample(start.size());
        for (std::size_t i = 0; i < sample.size(); ++i) {
            std::uniform_real_distribution<double> angle(c.limits[i].minAngle,
                                                         c.limits[i].maxAngle);
            sample[i] = angle(rng);
        }
        if (iter % 9 == 0) sample = forward ? goal : start;
        const int grown = extend(growing, sample, c);
        if (grown < 0) continue;
        const auto& destination = growing[static_cast<std::size_t>(grown)].q;
        int matchIndex = -1;
        for (int k = 0; k < 80; ++k) {
            const int newer = extend(matching, destination, c);
            if (newer < 0) break;
            matchIndex = newer;
            const auto& endpoint = matching[static_cast<std::size_t>(newer)].q;
            if (distanceSquared(endpoint, destination) < 1e-10) {
                auto prefix = forward ? trace(growing, grown) : trace(matching, matchIndex);
                auto suffix = forward ? trace(matching, matchIndex) : trace(growing, grown);
                std::reverse(suffix.begin(), suffix.end());
                prefix.insert(prefix.end(), suffix.begin() + 1, suffix.end());
                return prefix;
            }
        }
    }
    return {};
}

[[nodiscard]] std::vector<Angles> shorten(const SearchContext& c,
                                          const std::vector<Angles>& path) {
    if (path.size() < 3) return path;
    std::vector<Angles> simplified{path.front()};
    std::size_t i = 0;
    while (i + 1 < path.size()) {
        std::size_t next = path.size() - 1;
        while (next > i + 1 && !c.edge(path[i], path[next])) --next;
        simplified.push_back(path[next]);
        i = next;
    }
    return simplified;
}

void validate(const SearchContext& c, math::Vec2 target) {
    if (c.joints.size() < 3 || c.joints.size() != c.limits.size() ||
        !std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(c.settings.positionTolerance) || c.settings.positionTolerance <= 0.0 ||
        !std::isfinite(c.settings.jointClearance) || c.settings.jointClearance <= 0.0 ||
        c.settings.jointClearance >= pi / 2.0 ||
        !std::isfinite(c.settings.foldClearance) ||
        c.settings.foldClearance < c.settings.jointClearance ||
        c.settings.foldClearance >= pi / 2.0 ||
        !std::isfinite(c.settings.targetClearance) ||
        c.settings.targetClearance < 0.0 ||
        !std::isfinite(c.settings.linkClearance) || c.settings.linkClearance < 0.0)
        throw std::invalid_argument("Planning requires >=3 joints, matching limits and finite target");
    for (std::size_t i = 0; i < c.joints.size(); ++i) {
        const auto& bound = c.limits[i];
        if (!std::isfinite(c.joints[i].length) || c.joints[i].length <= 0.0 ||
            !std::isfinite(c.joints[i].angle) ||
            !std::isfinite(bound.minAngle) || !std::isfinite(bound.maxAngle) ||
            bound.minAngle >= bound.maxAngle) throw std::invalid_argument("Invalid robot geometry or limits");
    }
    if (!c.validRobot(c.angles())) throw std::invalid_argument("Cannot plan from an invalid starting pose");
}

} // namespace

PlannedJointPath2D planJointPath2D(std::span<const Joint2D> joints,
                                   std::span<const JointLimits2D> limits,
                                   math::Vec2 target,
                                   const MotionSettings2D& settings) {
    SearchContext c(joints, limits, settings, target);
    validate(c, target);
    const Angles start = c.angles();
    PlannedJointPath2D output;
    // An already contacted target needs an escape path, not an immediate failure.
    output.terminalError = (target - c.positions(start).back()).length();
    if (output.terminalError <= settings.positionTolerance && c.valid(start)) {
        output.waypoints = {start};
        return output;
    }
    double reach = 0.0;
    for (const auto& joint : joints) reach += joint.length;
    if (target.length() > reach + settings.positionTolerance) return output;
    // The base is fixed at the origin; a target containing the base can
    // never be cleared by the first link, regardless of reconfiguration.
    if (settings.targetClearance > 0.0 &&
        target.length() < settings.targetClearance) return output;

    std::mt19937 rng{0x526f626fU};
    // Prioritize the original configuration; use other sampled joint states
    // when the straightforward IK basin or a direct edge is blocked.
    for (int attempt = 0; attempt < 70; ++attempt) {
        Angles seed = start;
        if (attempt != 0) {
            for (std::size_t i = 0; i < seed.size(); ++i) {
                const auto& bound = limits[i];
                const double low = (attempt % 3 == 0)
                    ? std::max(bound.minAngle, start[i] - 1.5) : bound.minAngle;
                const double high = (attempt % 3 == 0)
                    ? std::min(bound.maxAngle, start[i] + 1.5) : bound.maxAngle;
                seed[i] = std::uniform_real_distribution<double>(low, high)(rng);
            }
        }
        if (!c.validRobot(seed) || !refineIK(c, seed, target)) continue;
        const auto path = connectPath(c, start, seed, rng);
        if (path.empty()) continue;
        output.waypoints = shorten(c, path);
        output.terminalError = (target - c.positions(seed).back()).length();
        return output;
    }
    return output;
}

void PlannedMotion2D::reset() noexcept {
    path_.clear();
    index_ = 0;
    hasTarget_ = false;
    failed_ = false;
}

void PlannedMotion2D::setTarget(std::span<const Joint2D> joints,
                                std::span<const JointLimits2D> limits,
                                math::Vec2 target, const MotionSettings2D& settings) {
    const auto plan = planJointPath2D(joints, limits, target, settings);
    target_ = target;
    hasTarget_ = true;
    failed_ = !plan.found();
    path_ = plan.waypoints;
    index_ = path_.size() > 1 ? 1 : 0;
}

MotionStatus2D PlannedMotion2D::advance(std::span<Joint2D> joints,
                                        std::span<const JointLimits2D> limits,
                                        double dt, const MotionSettings2D& settings) {
    if (!hasTarget_ || joints.size() != limits.size() || !std::isfinite(dt) ||
        dt <= 0.0 || dt > 0.1)
        throw std::invalid_argument("Follower requires target, joint limits and valid timestep");
    MotionStatus2D status;
    status.positionError = (target_ - forwardKinematics(joints).back()).length();
    status.blocked = failed_;
    if (failed_) {
        for (auto& joint : joints) joint.angularVelocity = 0.0;
        return status;
    }
    if (path_.empty()) return status;
    // Do not stop mid-motion just because the tip briefly passes through the
    // tolerance circle; the joint-space trajectory must settle as well.
    bool settled = true;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (std::abs(joints[i].angle - path_.back()[i]) > 0.002 ||
            std::abs(joints[i].angularVelocity) > 0.01) settled = false;
    }
    status.reached = status.positionError <= settings.positionTolerance && settled;
    if (status.reached) return status;

    const auto& goal = path_[index_];
    bool close = true;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (std::abs(joints[i].angle - goal[i]) > 0.012 ||
            std::abs(joints[i].angularVelocity) > 0.035) {
            close = false;
            break;
        }
    }
    if (close && index_ + 1 < path_.size()) ++index_;
    const auto& waypoint = path_[index_];
    std::vector<Joint2D> candidate(joints.begin(), joints.end());
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const double maxSpeed = std::min(settings.maxAngularSpeed, limits[i].maxAngularSpeed);
        const double maxAcceleration = std::min(settings.maxAngularAcceleration,
                                                limits[i].maxAngularAcceleration);
        const double delta = waypoint[i] - joints[i].angle;
        const double braking = std::sqrt(2.0 * maxAcceleration * std::abs(delta));
        const double desiredSpeed = std::copysign(std::min({maxSpeed, braking,
                                                            4.0 * std::abs(delta)}), delta);
        candidate[i].angularVelocity = joints[i].angularVelocity + std::clamp(
            desiredSpeed - joints[i].angularVelocity,
            -maxAcceleration * dt, maxAcceleration * dt);
        candidate[i].angle += candidate[i].angularVelocity * dt;
        if (candidate[i].angle < limits[i].minAngle || candidate[i].angle > limits[i].maxAngle) {
            candidate[i].angle = std::clamp(candidate[i].angle,
                                            limits[i].minAngle, limits[i].maxAngle);
            candidate[i].angularVelocity = (candidate[i].angle - joints[i].angle) / dt;
        }
    }
    SearchContext c(joints, limits, settings, target_);
    if (!c.edge(c.angles(), [&]() {
            Angles q;
            for (const auto& joint : candidate) q.push_back(joint.angle);
            return q;
        }())) {
        // Never move through a collision: report a blocked path instead of
        // silently stalling and repeatedly applying an invalid DLS command.
        failed_ = true;
        status.blocked = true;
        status.constrained = true;
        for (auto& joint : joints) joint.angularVelocity = 0.0;
        return status;
    }
    std::copy(candidate.begin(), candidate.end(), joints.begin());
    status.positionError = (target_ - forwardKinematics(joints).back()).length();
    settled = true;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (std::abs(joints[i].angle - path_.back()[i]) > 0.002 ||
            std::abs(joints[i].angularVelocity) > 0.01) settled = false;
    }
    status.reached = status.positionError <= settings.positionTolerance && settled;
    return status;
}

} // namespace robotics::kinematics
