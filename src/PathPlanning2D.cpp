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
    std::span<const geometry::Obstacle2D> obstacles;
    std::stop_token cancellation;

    [[nodiscard]] bool cancelled() const noexcept { return cancellation.stop_requested(); }

    explicit SearchContext(std::span<const Joint2D> state,
                           std::span<const JointLimits2D> bounds,
                           const MotionSettings2D& s, math::Vec2 goal,
                           std::span<const geometry::Obstacle2D> scene,
                           std::stop_token stop = {})
        : joints(state.begin(), state.end()), limits(bounds.begin(), bounds.end()),
          settings(s), target(goal), obstacles(scene), cancellation(stop) {}

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
        if (cancelled() || q.size() != joints.size()) return false;
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
        const auto points = positions(q);
        return !hasSelfCollision(points, settings.linkClearance) &&
               !geometry::hasObstacleCollision(points, obstacles,
                                                settings.obstacleClearance);
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
            if (cancelled()) return false;
            const double alpha = static_cast<double>(step) / steps;
            for (std::size_t i = 0; i < q.size(); ++i)
                q[i] = a[i] + alpha * (b[i] - a[i]);
            if (!validRobot(q)) return false;
        }
        return true;
    }

    // Distance to the entire arm, including its tool, used only to prove
    // that a portion of the path is too far away to hit the target.
    [[nodiscard]] double nearestLinkDistance(const Angles& q) const {
        const auto points = positions(q);
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 1; i < points.size(); ++i) {
            const auto ab = points[i] - points[i - 1];
            const double length2 = ab.dot(ab);
            const double t = length2 > 0.0 ? std::clamp(
                (target - points[i - 1]).dot(ab) / length2, 0.0, 1.0) : 0.0;
            best = std::min(best, (target - (points[i - 1] + ab * t)).length());
        }
        return best;
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
        Angles from = a;
        Angles to(a.size());
        Angles q(a.size());

        for (int step = 1; step <= steps; ++step) {
            if (cancelled()) return false;
            const double alpha = static_cast<double>(step) / steps;
            for (std::size_t i = 0; i < q.size(); ++i)
                to[i] = a[i] + alpha * (b[i] - a[i]);
            if (!validRobot(to)) return false;

            // An angular step of 0.04 rad can move the end of a long
            // multi-link arm by much more than the 0.16 m target diameter.
            // Bound the travel of any point on the arm using downstream
            // link lengths. Only subdivide further when contact is possible.
            int checks = 1;
            if (settings.targetClearance > 0.0) {
                double downstream = 0.0;
                double maximumTravel = 0.0;
                for (std::size_t i = q.size(); i-- > 0;) {
                    downstream += joints[i].length;
                    maximumTravel += downstream * std::abs(to[i] - from[i]);
                }
                if (nearestLinkDistance(from) <=
                    settings.targetClearance + maximumTravel) {
                    const double maxTravelPerCheck =
                        std::max(0.02, settings.targetClearance * 0.4);
                    checks = std::max(1, static_cast<int>(
                        std::ceil(maximumTravel / maxTravelPerCheck)));
                }
            }
            // An obstacle can also be skipped by a wide Cartesian sweep
            // between adjacent joint samples. Constrain the maximum travel
            // of any link point when scene obstacles are present.
            if (!obstacles.empty()) {
                double downstream = 0.0, maximumTravel = 0.0;
                for (std::size_t i = q.size(); i-- > 0;) {
                    downstream += joints[i].length;
                    maximumTravel += downstream * std::abs(to[i] - from[i]);
                }
                checks = std::max(checks, std::max(1,
                    static_cast<int>(std::ceil(maximumTravel / 0.04))));
            }
            for (int sub = 1; sub <= checks; ++sub) {
                if (cancelled()) return false;
                const double fraction = static_cast<double>(sub) / checks;
                for (std::size_t i = 0; i < q.size(); ++i)
                    q[i] = from[i] + fraction * (to[i] - from[i]);
                if (!validRobot(q)) return false;
                const auto next = targetLinkPenetrations(positions(q), target,
                                                         settings.targetClearance);
                for (std::size_t i = 0; i < next.size(); ++i)
                    if (next[i] > previous[i] + 1e-9) return false;
                previous = next;
            }
            from = to;
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
        if (c.cancelled()) return false;
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
        if (c.cancelled()) return {};
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
            if (c.cancelled()) return {};
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
        if (c.cancelled()) return {};
        std::size_t next = path.size() - 1;
        while (next > i + 1 && !c.edge(path[i], path[next])) --next;
        simplified.push_back(path[next]);
        i = next;
    }
    return simplified;
}

// Quintic Hermite curve in joint space with position, shared tangent and
// zero second derivative at each waypoint. Shared tangents yield C2 motion
// through safe corners; unsafe corners get zero tangent and a safe stop.
[[nodiscard]] Angles splinePose(const std::vector<Angles>& waypoints,
                                const std::vector<Angles>& tangents,
                                const std::vector<double>& distances,
                                std::size_t segment, double u) {
    const auto& a = waypoints[segment];
    const auto& b = waypoints[segment + 1];
    const double h = distances[segment + 1] - distances[segment];
    Angles result(a.size());
    for (std::size_t j = 0; j < a.size(); ++j) {
        const double p = h * tangents[segment][j];
        const double q = h * tangents[segment + 1][j];
        const double d = b[j] - a[j];
        const double A = 10.0 * d - 6.0 * p - 4.0 * q;
        const double B = -15.0 * d + 8.0 * p + 7.0 * q;
        const double C = 6.0 * d - 3.0 * p - 3.0 * q;
        const double u2 = u * u;
        result[j] = a[j] + p * u +
                    u2 * u * (A + u * (B + u * C));
    }
    return result;
}

[[nodiscard]] PlannedJointPath2D prepareTrajectory(
    const SearchContext& c, PlannedJointPath2D plan) {
    if (plan.waypoints.size() < 2) return plan;

    // Consecutive duplicate RRT endpoints are harmless geometrically but
    // would produce a zero denominator in the spline parameterization.
    std::vector<Angles> unique;
    for (const auto& q : plan.waypoints) {
        if (unique.empty() || distanceSquared(unique.back(), q) > 1e-12)
            unique.push_back(q);
    }
    plan.waypoints = std::move(unique);
    if (plan.waypoints.size() < 2) return plan;

    const std::size_t count = plan.waypoints.size();
    const std::size_t joints = plan.waypoints.front().size();
    plan.pathDistances.assign(count, 0.0);
    for (std::size_t i = 1; i < count; ++i)
        plan.pathDistances[i] = plan.pathDistances[i - 1] +
            std::sqrt(distanceSquared(plan.waypoints[i - 1], plan.waypoints[i]));
    const double length = plan.pathDistances.back();

    plan.splineTangents.assign(count, Angles(joints, 0.0));
    for (std::size_t i = 1; i + 1 < count; ++i) {
        double beforeAfterDot = 0.0;
        for (std::size_t j = 0; j < joints; ++j) {
            beforeAfterDot +=
                (plan.waypoints[i][j] - plan.waypoints[i - 1][j]) *
                (plan.waypoints[i + 1][j] - plan.waypoints[i][j]);
        }
        if (beforeAfterDot <= 0.0) continue;
        const double span = plan.pathDistances[i + 1] -
                            plan.pathDistances[i - 1];
        for (std::size_t j = 0; j < joints; ++j)
            plan.splineTangents[i][j] =
                0.45 * (plan.waypoints[i + 1][j] -
                        plan.waypoints[i - 1][j]) / span;
    }

    // Each candidate curve is validated using the same edge checker as the
    // original RRT solution, including target-retreat and scene obstacles.
    // If rounding a corner would leave free space, stop at that waypoint.
    for (std::size_t attempt = 0; attempt <= count; ++attempt) {
        if (c.cancelled()) {
            plan.waypoints.clear();
            plan.failure = PlanningFailure2D::Cancelled;
            return plan;
        }
        bool safe = true;
        bool changed = false;
        for (std::size_t k = 0; k + 1 < count; ++k) {
            const double h = plan.pathDistances[k + 1] - plan.pathDistances[k];
            const int subdivisions = std::max(24,
                static_cast<int>(std::ceil(h / 0.025)));
            Angles previous = plan.waypoints[k];
            for (int sample = 1; sample <= subdivisions; ++sample) {
                const auto next = splinePose(
                    plan.waypoints, plan.splineTangents, plan.pathDistances,
                    k, static_cast<double>(sample) / subdivisions);
                if (!c.edge(previous, next)) {
                    safe = false;
                    for (const std::size_t waypoint : {k, k + 1}) {
                        if (waypoint == 0 || waypoint + 1 == count) continue;
                        for (double& tangent : plan.splineTangents[waypoint]) {
                            if (tangent != 0.0) changed = true;
                            tangent = 0.0;
                        }
                    }
                    break;
                }
                previous = next;
            }
            if (!safe) break;
        }
        if (safe) break;
        if (!changed) {
            // The original RRT polyline has already passed collision checks.
            // A rounded curve can fail extra tests from numerical differences;
            // NEVER discard a valid path merely because smoothing is unsafe.
            // Reuse the earlier safe, stop-at-waypoint quintic follower.
            plan.splineTangents.clear();
            plan.pathDistances.clear();
            plan.trajectoryDuration = 0.0;
            return plan;
        }
    }

    // Analytic first/second/third derivatives of each quintic segment with
    // respect to joint-space arc length. The global trajectory uses s=L*t/T.
    // Conservative margins account for 60 Hz finite-difference estimates.
    double duration = 2.0 / 60.0;
    for (std::size_t k = 0; k + 1 < count; ++k) {
        const double h = plan.pathDistances[k + 1] - plan.pathDistances[k];
        for (std::size_t j = 0; j < joints; ++j) {
            const double d = plan.waypoints[k + 1][j] -
                             plan.waypoints[k][j];
            const double m0 = h * plan.splineTangents[k][j];
            const double m1 = h * plan.splineTangents[k + 1][j];
            const double A = 10.0 * d - 6.0 * m0 - 4.0 * m1;
            const double B = -15.0 * d + 8.0 * m0 + 7.0 * m1;
            const double C = 6.0 * d - 3.0 * m0 - 3.0 * m1;
            const double speed = std::min(c.settings.maxAngularSpeed,
                                          c.limits[j].maxAngularSpeed);
            const double acceleration =
                std::min(c.settings.maxAngularAcceleration,
                         c.limits[j].maxAngularAcceleration);
            if (speed <= 0.0 || acceleration <= 0.0)
                throw std::invalid_argument("Joint limits must be positive");

            for (int sample = 0; sample <= 64; ++sample) {
                const double u = static_cast<double>(sample) / 64.0;
                const double u2 = u * u, u3 = u2 * u, u4 = u3 * u;
                const double ds = std::abs(
                    (m0 + 3.0 * A * u2 + 4.0 * B * u3 + 5.0 * C * u4) / h);
                const double dss = std::abs(
                    (6.0 * A * u + 12.0 * B * u2 + 20.0 * C * u3) / (h*h));
                const double dsss = std::abs(
                    (6.0 * A + 24.0 * B * u + 60.0 * C * u2) / (h*h*h));
                duration = std::max(duration, 1.35 * length * ds / speed);
                duration = std::max(duration, std::sqrt(
                    1.8 * length * length * dss / acceleration));
                duration = std::max(duration, std::cbrt(
                    1.8 * length * length * length * dsss /
                    c.settings.maxAngularJerk));
            }
        }
    }
    if (!std::isfinite(duration))
        throw std::overflow_error("Trajectory duration overflow");
    plan.trajectoryDuration = duration;
    return plan;
}

void validate(const SearchContext& c, math::Vec2 target) {
    if (c.joints.size() < 2 || c.joints.size() != c.limits.size() ||
        !std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(c.settings.positionTolerance) || c.settings.positionTolerance <= 0.0 ||
        !std::isfinite(c.settings.jointClearance) || c.settings.jointClearance <= 0.0 ||
        c.settings.jointClearance >= pi / 2.0 ||
        !std::isfinite(c.settings.foldClearance) ||
        c.settings.foldClearance < c.settings.jointClearance ||
        c.settings.foldClearance >= pi / 2.0 ||
        !std::isfinite(c.settings.targetClearance) ||
        c.settings.targetClearance < 0.0 ||
        !std::isfinite(c.settings.obstacleClearance) ||
        c.settings.obstacleClearance < 0.0 ||
        !std::isfinite(c.settings.maxAngularJerk) ||
        c.settings.maxAngularJerk <= 0.0 ||
        !std::isfinite(c.settings.linkClearance) || c.settings.linkClearance < 0.0)
        throw std::invalid_argument("Planning requires >=2 joints, matching limits and finite target");
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
                                   const MotionSettings2D& settings,
                                   std::span<const geometry::Obstacle2D> obstacles,
                                   std::stop_token cancellation) {
    SearchContext c(joints, limits, settings, target, obstacles, cancellation);
    PlannedJointPath2D output;
    if (c.cancelled()) { output.failure = PlanningFailure2D::Cancelled; return output; }
    validate(c, target);
    const Angles start = c.angles();
    // An already contacted target needs an escape path, not an immediate failure.
    output.terminalError = (target - c.positions(start).back()).length();
    if (output.terminalError <= settings.positionTolerance && c.valid(start)) {
        output.waypoints = {start};
        return output;
    }
    double reach = 0.0;
    for (const auto& joint : joints) reach += joint.length;
    if (target.length() > reach + settings.positionTolerance) {
        output.failure = PlanningFailure2D::OutOfReach;
        return output;
    }
    // A tool endpoint inside a solid obstacle cannot be a valid goal.
    for (const auto& obstacle : obstacles) {
        if (geometry::pointHitsObstacle(target, obstacle,
                                        settings.obstacleClearance)) {
            output.failure = PlanningFailure2D::GoalInObstacle;
            return output;
        }
    }
    // The base is fixed at the origin; a target containing the base can
    // never be cleared by the first link, regardless of reconfiguration.
    if (settings.targetClearance > 0.0 &&
        target.length() < settings.targetClearance) {
        output.failure = PlanningFailure2D::GoalInObstacle;
        return output;
    }

    std::mt19937 rng{0x526f626fU};
    if (joints.size() == 2) {
        // The closed-form IK solution is periodic in the shoulder angle.
        // A short shoulder rotation may sweep an intermediate link across the
        // target, while the equivalent angle +/- 2*pi is collision-free.
        // Preserve the elbow branch (switching branches would require
        // crossing a forbidden straight/folded configuration).
        const double l1 = joints[0].length;
        const double l2 = joints[1].length;
        const double radius2 = target.x * target.x + target.y * target.y;
        const double cosine = (radius2 - l1 * l1 - l2 * l2) /
                              (2.0 * l1 * l2);
        if (!std::isfinite(cosine) || cosine < -1.0 - 1e-12 ||
            cosine > 1.0 + 1e-12) {
            output.failure = PlanningFailure2D::OutOfReach;
            return output;
        }

        const double elbow = (std::signbit(start[1]) ? -1.0 : 1.0) *
                             std::acos(std::clamp(cosine, -1.0, 1.0));
        const double shoulder = std::atan2(target.y, target.x) -
                                std::atan2(l2 * std::sin(elbow),
                                           l1 + l2 * std::cos(elbow));
        constexpr double fullTurn = 2.0 * pi;
        const double nearestTurn = std::round((start[0] - shoulder) / fullTurn);

        std::vector<Angles> goals;
        for (int offset = -3; offset <= 3; ++offset) {
            const double angle = shoulder + (nearestTurn + offset) * fullTurn;
            Angles candidate{angle, elbow};
            if (c.valid(candidate)) goals.push_back(std::move(candidate));
        }
        std::sort(goals.begin(), goals.end(),
                  [&](const Angles& a, const Angles& b) {
                      return distanceSquared(start, a) <
                             distanceSquared(start, b);
                  });

        // Prefer a direct, fully checked transition. This keeps ordinary
        // two-joint movements as smooth as the previous controller.
        for (const auto& goal : goals) {
            if (!c.edge(start, goal)) continue;
            output.waypoints = {start, goal};
            output.terminalError = (target - c.positions(goal).back()).length();
            return prepareTrajectory(c, std::move(output));
        }
        // When no direct transition is safe, search joint space for a detour.
        for (const auto& goal : goals) {
            const auto path = connectPath(c, start, goal, rng);
            if (path.empty()) continue;
            output.waypoints = shorten(c, path);
            output.terminalError = (target - c.positions(goal).back()).length();
            return prepareTrajectory(c, std::move(output));
        }
        output.failure = c.cancelled() ? PlanningFailure2D::Cancelled :
                         goals.empty() ? PlanningFailure2D::NoIKSolution :
                                         PlanningFailure2D::PathSearchExhausted;
        return output;
    }

    // Prioritize the original configuration; use other sampled joint states
    // when the straightforward IK basin or a direct edge is blocked.
    bool foundGoal = false;
    for (int attempt = 0; attempt < 70; ++attempt) {
        if (c.cancelled()) {
            output.failure = PlanningFailure2D::Cancelled;
            return output;
        }
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
        foundGoal = true;
        const auto path = connectPath(c, start, seed, rng);
        if (path.empty()) continue;
        output.waypoints = shorten(c, path);
        output.terminalError = (target - c.positions(seed).back()).length();
        return prepareTrajectory(c, std::move(output));
    }
    output.failure = c.cancelled() ? PlanningFailure2D::Cancelled :
                     foundGoal ? PlanningFailure2D::PathSearchExhausted :
                                 PlanningFailure2D::NoIKSolution;
    return output;
}

const char* planningFailureMessage(PlanningFailure2D reason) noexcept {
    switch (reason) {
    case PlanningFailure2D::None: return "READY";
    case PlanningFailure2D::Cancelled: return "PLANNING CANCELLED";
    case PlanningFailure2D::OutOfReach: return "OUT OF REACH";
    case PlanningFailure2D::GoalInObstacle: return "TARGET BLOCKED";
    case PlanningFailure2D::NoIKSolution: return "NO VALID IK SOLUTION";
    case PlanningFailure2D::PathSearchExhausted: return "SEARCH BUDGET EXHAUSTED";
    }
    return "UNKNOWN PLANNING STATUS";
}

void PlannedMotion2D::reset() noexcept {
    path_.clear();
    index_ = 0;
    segmentElapsed_ = 0.0;
    segmentDuration_ = 0.0;
    splineTangents_.clear();
    pathDistances_.clear();
    trajectoryDuration_ = 0.0;
    trajectoryElapsed_ = 0.0;
    hasTarget_ = false;
    failed_ = false;
    failure_ = PlanningFailure2D::None;
}

void PlannedMotion2D::usePlan(PlannedJointPath2D plan, math::Vec2 target) {
    target_ = target;
    hasTarget_ = true;
    failed_ = !plan.found();
    failure_ = plan.failure;
    path_ = std::move(plan.waypoints);
    splineTangents_ = std::move(plan.splineTangents);
    pathDistances_ = std::move(plan.pathDistances);
    trajectoryDuration_ = plan.trajectoryDuration;
    trajectoryElapsed_ = 0.0;
    index_ = path_.size() > 1 ? 1 : 0;
    segmentElapsed_ = 0.0;
    segmentDuration_ = 0.0;
}

void PlannedMotion2D::setTarget(std::span<const Joint2D> joints,
                                std::span<const JointLimits2D> limits,
                                math::Vec2 target, const MotionSettings2D& settings,
                                std::span<const geometry::Obstacle2D> obstacles) {
    usePlan(planJointPath2D(joints, limits, target, settings, obstacles), target);
}

MotionStatus2D PlannedMotion2D::advance(std::span<Joint2D> joints,
                                        std::span<const JointLimits2D> limits,
                                        double dt, const MotionSettings2D& settings,
                                        std::span<const geometry::Obstacle2D> obstacles) {
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
    if (index_ >= path_.size() || path_.size() == 1) {
        status.reached = status.positionError <= settings.positionTolerance;
        return status;
    }

    // New C2 joint-space trajectory, prepared and collision-checked by
    // the planner worker. All joints share the same global time parameter,
    // so their movement cannot cut corners between RRT waypoints.
    if (trajectoryDuration_ > 0.0 &&
        pathDistances_.size() == path_.size() &&
        splineTangents_.size() == path_.size()) {
        const double elapsed = std::min(trajectoryDuration_,
                                        trajectoryElapsed_ + dt);
        const bool atGoal = elapsed >= trajectoryDuration_;
        const double s = pathDistances_.back() * elapsed / trajectoryDuration_;
        const auto upper = std::upper_bound(pathDistances_.begin(),
                                             pathDistances_.end(), s);
        const std::size_t index = std::min(
            static_cast<std::size_t>(std::distance(pathDistances_.begin(), upper) -
                                     (upper == pathDistances_.begin() ? 0 : 1)),
            path_.size() - 2);
        const double h = pathDistances_[index + 1] - pathDistances_[index];
        const double u = std::clamp((s - pathDistances_[index]) / h, 0.0, 1.0);
        const auto q = atGoal ? path_.back() : splinePose(
            path_, splineTangents_, pathDistances_, index, u);

        std::vector<Joint2D> proposed(joints.begin(), joints.end());
        for (std::size_t j = 0; j < joints.size(); ++j) {
            proposed[j].angle = q[j];
            proposed[j].angularVelocity = atGoal ? 0.0 :
                (q[j] - joints[j].angle) / dt;
            const double speed = std::min(settings.maxAngularSpeed,
                                          limits[j].maxAngularSpeed);
            const double acceleration =
                std::min(settings.maxAngularAcceleration,
                         limits[j].maxAngularAcceleration);
            if (std::abs(proposed[j].angularVelocity) > speed + 1e-7 ||
                std::abs(proposed[j].angularVelocity -
                         joints[j].angularVelocity) > acceleration * dt + 1e-7) {
                failed_ = true;
                status.blocked = true;
                status.constrained = true;
                for (auto& joint : joints) joint.angularVelocity = 0.0;
                return status;
            }
        }
        SearchContext context(joints, limits, settings, target_, obstacles);
        if (!context.edge(context.angles(), q)) {
            failed_ = true;
            status.blocked = true;
            status.constrained = true;
            for (auto& joint : joints) joint.angularVelocity = 0.0;
            return status;
        }
        std::copy(proposed.begin(), proposed.end(), joints.begin());
        trajectoryElapsed_ = elapsed;
        status.positionError = (target_ - forwardKinematics(joints).back()).length();
        status.reached = atGoal &&
                         status.positionError <= settings.positionTolerance;
        return status;
    }

    // The planner approves straight segments in JOINT SPACE. Driving each
    // joint independently toward the next waypoint cuts the corner of that
    // approved segment and can collide near folded configurations/targets.
    // Instead, use a single time parameter for all joints and stop at each
    // waypoint. A quintic smoothstep has zero endpoint speed/acceleration.
    const auto& a = path_[index_ - 1];
    const auto& b = path_[index_];
    if (segmentDuration_ == 0.0) {
        segmentDuration_ = 2.0 * dt;
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const double distance = std::abs(b[i] - a[i]);
            if (distance == 0.0) continue;
            const double maxSpeed = std::min(settings.maxAngularSpeed,
                                              limits[i].maxAngularSpeed);
            const double maxAcceleration = std::min(settings.maxAngularAcceleration,
                                                     limits[i].maxAngularAcceleration);
            if (!std::isfinite(maxSpeed) || maxSpeed <= 0.0 ||
                !std::isfinite(maxAcceleration) || maxAcceleration <= 0.0)
                throw std::invalid_argument("Invalid joint motion limits");
            // |s'(u)| <= 1.875 and |s''(u)| <= 5.774 for quintic smoothstep.
            // Factor 8 accounts for finite-difference acceleration at 60 Hz,
            // including the final timestep that settles at zero velocity.
            segmentDuration_ = std::max(segmentDuration_,
                std::max(1.875 * distance / maxSpeed,
                         std::sqrt(8.0 * distance / maxAcceleration)));
        }
    }

    const double nextElapsed = std::min(segmentDuration_, segmentElapsed_ + dt);
    const double u = nextElapsed / segmentDuration_;
    const double progress = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    const bool atWaypoint = nextElapsed >= segmentDuration_;

    std::vector<Joint2D> candidate(joints.begin(), joints.end());
    Angles candidateAngles(joints.size());
    for (std::size_t i = 0; i < joints.size(); ++i) {
        candidate[i].angle = a[i] + (b[i] - a[i]) * progress;
        candidate[i].angularVelocity = atWaypoint ? 0.0 :
            (candidate[i].angle - joints[i].angle) / dt;
        candidateAngles[i] = candidate[i].angle;
    }

    SearchContext context(joints, limits, settings, target_, obstacles);
    if (!context.edge(context.angles(), candidateAngles)) {
        // Fail closed if numeric precision or changed constraints invalidate
        // a path segment. Do not take an unvalidated shortcut.
        failed_ = true;
        status.blocked = true;
        status.constrained = true;
        for (auto& joint : joints) joint.angularVelocity = 0.0;
        return status;
    }

    std::copy(candidate.begin(), candidate.end(), joints.begin());
    segmentElapsed_ = nextElapsed;
    if (atWaypoint) {
        segmentElapsed_ = 0.0;
        segmentDuration_ = 0.0;
        ++index_;
    }

    status.positionError = (target_ - forwardKinematics(joints).back()).length();
    status.reached = index_ >= path_.size() &&
                     status.positionError <= settings.positionTolerance;
    return status;
}

} // namespace robotics::kinematics
