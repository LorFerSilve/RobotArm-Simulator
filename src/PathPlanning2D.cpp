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
        std::vector<math::Vec2> result;
        forwardKinematics(joints, q, result);
        return result;
    }

    [[nodiscard]] bool validJointAngles(const Angles& q) const {
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
        return true;
    }

    [[nodiscard]] bool collisionFreeRobot(
        std::span<const math::Vec2> points) const {
        return !hasSelfCollision(points, settings.linkClearance) &&
               !geometry::hasObstacleCollision(points, obstacles,
                                                settings.obstacleClearance);
    }

    [[nodiscard]] bool validRobot(const Angles& q) const {
        return validJointAngles(q) && collisionFreeRobot(positions(q));
    }

    [[nodiscard]] bool valid(const Angles& q) const {
        if (!validJointAngles(q)) return false;
        const auto points = positions(q);
        return collisionFreeRobot(points) &&
               targetApproachClear(points, target, settings.targetClearance);
    }

    // Distance to the entire arm, including its tool, used only to prove
    // that a portion of the path is too far away to hit the target.
    [[nodiscard]] double nearestLinkDistance(
        std::span<const math::Vec2> points) const {
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
        if (!validJointAngles(a)) return false;

        // One reusable FK and two target-depth buffers per edge instead of
        // copying the robot and allocating geometry/depths at every sample.
        std::vector<math::Vec2> points;
        forwardKinematics(joints, a, points);
        if (!collisionFreeRobot(points)) return false;

        std::vector<double> previous(joints.size());
        std::vector<double> next(joints.size());
        targetLinkPenetrations(points, target, settings.targetClearance, previous);

        double maxDelta = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i)
            maxDelta = std::max(maxDelta, std::abs(a[i] - b[i]));
        const auto steps = std::max(
            1, static_cast<int>(std::ceil(maxDelta / edgeResolution)));
        Angles from = a;
        Angles to(a.size());
        Angles q(a.size());

        for (int step = 1; step <= steps; ++step) {
            if (cancelled()) return false;
            const double alpha = static_cast<double>(step) / steps;
            for (std::size_t i = 0; i < q.size(); ++i)
                to[i] = a[i] + alpha * (b[i] - a[i]);
            if (!validJointAngles(to)) return false;

            // Bound Cartesian movement of all links between consecutive
            // samples; preserve target clearance and scene-obstacle margins.
            int checks = 1;
            if (settings.targetClearance > 0.0) {
                double downstream = 0.0;
                double maximumTravel = 0.0;
                for (std::size_t i = q.size(); i-- > 0;) {
                    downstream += joints[i].length;
                    maximumTravel += downstream * std::abs(to[i] - from[i]);
                }
                if (nearestLinkDistance(points) <=
                    settings.targetClearance + maximumTravel) {
                    const double maxTravelPerCheck =
                        std::max(0.02, settings.targetClearance * 0.4);
                    checks = std::max(1, static_cast<int>(
                        std::ceil(maximumTravel / maxTravelPerCheck)));
                }
            }
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
                if (!validJointAngles(q)) return false;
                forwardKinematics(joints, q, points);
                if (!collisionFreeRobot(points)) return false;

                targetLinkPenetrations(
                    points, target, settings.targetClearance, next);
                for (std::size_t i = 0; i < next.size(); ++i)
                    if (next[i] > previous[i] + 1e-9) return false;
                previous.swap(next);
            }
            // "points" now contains the geometry of "to", ready for the
            // next outer step's swept-target proximity estimate.
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
            // IK iterations are virtual numerical guesses, not executed
            // robot states. Do not reject a feasible goal just because an
            // intermediate *calculation* overlaps a scene obstacle.
            // Final IK solutions and the real RRT path are still fully checked.
            if (!c.validJointAngles(next)) continue;
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

// Prefer a Cartesian straight-line motion whenever its inverse-kinematics
// lift is feasible. The tool is sampled along the straight line, while each
// connecting joint-space edge is checked using the existing full safety
// constraints. This is a candidate, never a shortcut around collision checks.
[[nodiscard]] std::vector<Angles> cartesianGuidedPath(
    const SearchContext& c, const Angles& start, math::Vec2 target) {
    const auto origin = c.positions(start).back();
    const double distance = (target - origin).length();
    if (distance <= c.settings.positionTolerance) return {start};
    const int count = std::clamp(static_cast<int>(std::ceil(distance / 0.18)),
                                 6, 28);
    std::vector<Angles> path{start};
    path.reserve(static_cast<std::size_t>(count + 1));
    Angles current = start;

    for (int step = 1; step <= count; ++step) {
        if (c.cancelled()) return {};
        const auto nextTarget = origin + (target - origin) *
            (static_cast<double>(step) / count);
        Angles next;
        if (c.joints.size() == 2) {
            const double l1 = c.joints[0].length;
            const double l2 = c.joints[1].length;
            const double cosine = (nextTarget.dot(nextTarget) - l1*l1 - l2*l2) /
                                  (2.0*l1*l2);
            if (cosine < -1.0 - 1e-10 || cosine > 1.0 + 1e-10)
                return {};
            const double elbow = (std::signbit(start[1]) ? -1.0 : 1.0) *
                std::acos(std::clamp(cosine, -1.0, 1.0));
            const double shoulder = std::atan2(nextTarget.y, nextTarget.x) -
                std::atan2(l2 * std::sin(elbow), l1 + l2 * std::cos(elbow));
            double best = std::numeric_limits<double>::infinity();
            for (int winding = -4; winding <= 4; ++winding) {
                Angles candidate{shoulder + (2.0*pi)*winding, elbow};
                if (!c.validRobot(candidate) || !c.edge(current, candidate))
                    continue;
                const double motion = distanceSquared(current, candidate);
                if (motion < best) {
                    best = motion;
                    next = std::move(candidate);
                }
            }
            if (next.empty()) return {};
        } else {
            next = current;
            if (!refineIK(c, next, nextTarget) ||
                !c.validRobot(next) || !c.edge(current, next))
                return {};
        }
        if ((c.positions(next).back() - nextTarget).length() >
            c.settings.positionTolerance) return {};
        path.push_back(next);
        current = std::move(next);
    }
    if (!c.valid(path.back())) return {};
    return path;
}

// A target inside the first link's turning radius is a topological barrier:
// the shoulder cannot rotate past the target without striking it, regardless
// of what the elbow does. Only report this cause if *every* permitted IK
// shoulder winding necessarily crosses the target direction.
[[nodiscard]] bool shoulderRouteBlocked(
    const SearchContext& c, const Angles& start, const std::vector<Angles>& goals) {
    if (goals.empty() || c.settings.targetClearance <= 0.0 ||
        !c.valid(start)) return false;
    const double r = c.target.length();
    if (r <= c.settings.targetClearance ||
        r > c.joints[0].length + c.settings.targetClearance)
        return false;
    const double theta = std::atan2(c.target.y, c.target.x);
    for (const auto& goal : goals) {
        const double lower = std::min(start[0], goal[0]);
        const double upper = std::max(start[0], goal[0]);
        const double first = std::ceil((lower - theta) / (2.0*pi));
        const double last = std::floor((upper - theta) / (2.0*pi));
        if (first > last) return false; // this goal has an unblocked winding
    }
    return true;
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

// Approximate Cartesian arc length of one STRAIGHT JOINT-SPACE edge.
// Sampling is bounded by the maximum possible tool travel, so a 360 degree
// shoulder turn is not mistaken for a straight tool movement.
[[nodiscard]] double tipDistanceAlongEdge(
    const SearchContext& c, const Angles& a, const Angles& b) {
    double distalReach = 0.0;
    double travelBound = 0.0;
    for (std::size_t i = a.size(); i-- > 0;) {
        distalReach += c.joints[i].length;
        travelBound += distalReach * std::abs(b[i] - a[i]);
    }
    const int samples = std::clamp(
        static_cast<int>(std::ceil(travelBound / 0.06)), 8, 384);
    math::Vec2 previous = c.positions(a).back();
    double total = 0.0;
    Angles q(a.size());
    for (int step = 1; step <= samples; ++step) {
        if (c.cancelled()) return std::numeric_limits<double>::infinity();
        const double t = static_cast<double>(step) / samples;
        for (std::size_t j = 0; j < a.size(); ++j)
            q[j] = a[j] + t * (b[j] - a[j]);
        const auto point = c.positions(q).back();
        total += (point - previous).length();
        previous = point;
    }
    return total;
}

// Local, objective-aware shortcutting of RRT waypoints. Unlike the old greedy
// farthest-joint-space shortcut, this preserves the route unless it actually
// REDUCES Cartesian travel. All accepted new edges pass the SAME swept-edge
// collision and target-contact checks as RRT and the trajectory follower.
[[nodiscard]] std::vector<Angles> shorten(
    const SearchContext& c, std::vector<Angles> path) {
    constexpr int maxPasses = 5;
    constexpr int maxEdgeChecksPerPass = 96;
    constexpr double improvementThreshold = 0.002; // metres

    for (int pass = 0; pass < maxPasses && path.size() > 2; ++pass) {
        if (c.cancelled()) return {};
        std::vector<double> prefix(path.size(), 0.0);
        for (std::size_t i = 1; i < path.size(); ++i) {
            prefix[i] = prefix[i - 1] +
                tipDistanceAlongEdge(c, path[i - 1], path[i]);
        }

        double bestGain = improvementThreshold;
        std::size_t bestFrom = 0, bestTo = 0;
        int inspected = 0;
        // Larger skips first; assess their actual tool-length saving, not
        // their index span or Euclidean distance between joint vectors.
        for (std::size_t gap = path.size() - 1; gap > 1; --gap) {
            for (std::size_t from = 0; from + gap < path.size(); ++from) {
                if (c.cancelled()) return {};
                const std::size_t to = from + gap;
                const double replacement =
                    tipDistanceAlongEdge(c, path[from], path[to]);
                const double saving = prefix[to] - prefix[from] - replacement;
                if (saving > bestGain && c.edge(path[from], path[to])) {
                    bestGain = saving;
                    bestFrom = from;
                    bestTo = to;
                }
                if (++inspected >= maxEdgeChecksPerPass) break;
            }
            if (inspected >= maxEdgeChecksPerPass) break;
        }
        if (bestTo <= bestFrom + 1) break;
        path.erase(path.begin() + static_cast<std::ptrdiff_t>(bestFrom + 1),
                   path.begin() + static_cast<std::ptrdiff_t>(bestTo));
    }
    return path;
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

// Render-only preview, sampled from the same joint-space geometry that the
// follower will execute. No Cartesian shortcuts: even a full shoulder turn
// must be projected through FK instead of drawing start-to-target directly.
[[nodiscard]] PlannedJointPath2D withEndEffectorTrace(
    const SearchContext& c, PlannedJointPath2D plan) {
    if (!plan.found()) return plan;

    plan.jointTraces.resize(c.joints.size());
    if (plan.waypoints.size() == 1) {
        const auto positions = c.positions(plan.waypoints.front());
        for (std::size_t i = 0; i < c.joints.size(); ++i)
            plan.jointTraces[i].push_back(positions[i + 1]);
        plan.endEffectorTrace.push_back(positions.back());
        return plan;
    }

    const bool spline = plan.trajectoryDuration > 0.0 &&
        plan.splineTangents.size() == plan.waypoints.size() &&
        plan.pathDistances.size() == plan.waypoints.size();

    auto robot = c.joints;
    for (std::size_t segment = 0; segment + 1 < plan.waypoints.size(); ++segment) {
        const auto& from = plan.waypoints[segment];
        const auto& to = plan.waypoints[segment + 1];

        // Upper bound for travel of the tip under simultaneous joint motion,
        // making the rendered trace adapt to long arms and full rotations.
        double downstreamLength = 0.0;
        double tipTravelBound = 0.0;
        for (std::size_t j = from.size(); j-- > 0;) {
            downstreamLength += robot[j].length;
            tipTravelBound += downstreamLength * std::abs(to[j] - from[j]);
        }
        const int samples = std::clamp(
            static_cast<int>(std::ceil(tipTravelBound / 0.075)), 16, 512);
        for (int step = segment == 0 ? 0 : 1; step <= samples; ++step) {
            if (c.cancelled()) {
                plan.waypoints.clear();
                plan.endEffectorTrace.clear();
                plan.jointTraces.clear();
                plan.failure = PlanningFailure2D::Cancelled;
                return plan;
            }
            const double t = static_cast<double>(step) / samples;
            Angles angles = spline ?
                splinePose(plan.waypoints, plan.splineTangents,
                           plan.pathDistances, segment, t) : Angles(from.size());
            if (!spline) {
                for (std::size_t j = 0; j < angles.size(); ++j)
                    angles[j] = from[j] + t * (to[j] - from[j]);
            }
            for (std::size_t j = 0; j < robot.size(); ++j)
                robot[j].angle = angles[j];
            const auto positions = forwardKinematics(robot);
            for (std::size_t j = 0; j < robot.size(); ++j)
                plan.jointTraces[j].push_back(positions[j + 1]);
            plan.endEffectorTrace.push_back(positions.back());
        }
    }
    for (std::size_t i = 1; i < plan.endEffectorTrace.size(); ++i)
        plan.endEffectorDistance +=
            (plan.endEffectorTrace[i] - plan.endEffectorTrace[i - 1]).length();
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
        return withEndEffectorTrace(c, std::move(output));
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
        if (elbow < limits[1].minAngle - 1e-9 ||
            elbow > limits[1].maxAngle + 1e-9 ||
            std::abs(elbow) < settings.jointClearance - 1e-9 ||
            std::abs(elbow) > pi - settings.foldClearance + 1e-9) {
            output.failure = PlanningFailure2D::OutOfReach;
            return output;
        }
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

        // Use Cartesian tip travel rather than the smallest joint-angle
        // change as the selection cost. A safe near-straight IK lift takes
        // priority over the broad arc of one joint-space interpolation.
        PlannedJointPath2D best;
        double bestDistance = std::numeric_limits<double>::infinity();
        const auto consider = [&](std::vector<Angles> path) {
            if (path.empty() || c.cancelled()) return;
            PlannedJointPath2D candidate;
            candidate.waypoints = std::move(path);
            candidate.terminalError =
                (target - c.positions(candidate.waypoints.back()).back()).length();
            candidate = withEndEffectorTrace(
                c, prepareTrajectory(c, std::move(candidate)));
            if (candidate.found() &&
                candidate.endEffectorDistance < bestDistance) {
                bestDistance = candidate.endEffectorDistance;
                best = std::move(candidate);
            }
        };
        consider(cartesianGuidedPath(c, start, target));
        for (const auto& goal : goals) {
            if (c.cancelled()) break;
            if (c.edge(start, goal)) consider({start, goal});
        }
        const double directLowerBound = (target - c.positions(start).back()).length();
        // In a free scene, the existing direct/Cartesian choice is sufficient.
        // With obstacles, a valid direct joint-space edge may still trace a
        // wide Cartesian arc; compare it with a few safe RRT alternatives.
        if (best.found() &&
            (obstacles.empty() || bestDistance <= directLowerBound * 1.03))
            return best;

        constexpr int variantsPerGoal = 2;
        constexpr int maxRrtCandidates = 4;
        int candidates = 0;
        for (const auto& goal : goals) {
            for (int variant = 0; variant < variantsPerGoal; ++variant) {
                if (c.cancelled() || candidates >= maxRrtCandidates ||
                    (best.found() && bestDistance <= directLowerBound * 1.03))
                    break;
                const auto path = connectPath(c, start, goal, rng);
                if (path.empty()) continue;
                ++candidates;
                consider(shorten(c, path));
            }
            if (c.cancelled() || candidates >= maxRrtCandidates ||
                (best.found() && bestDistance <= directLowerBound * 1.03))
                break;
        }
        if (c.cancelled()) {
            output.failure = PlanningFailure2D::Cancelled;
            return output;
        }
        if (best.found()) return best;
        output.failure = c.cancelled() ? PlanningFailure2D::Cancelled :
                         goals.empty() ? PlanningFailure2D::NoIKSolution :
                         shoulderRouteBlocked(c, start, goals) ?
                             PlanningFailure2D::TargetBlockedByJointLimits :
                             PlanningFailure2D::PathSearchExhausted;
        return output;
    }

    // Try a collision-checked Cartesian lift first. It tends to make the
    // tool travel nearly straight and avoids unnecessary RRT detours. If any
    // intermediate IK state or motion edge is unsafe, retain the existing
    // multi-start IK/RRT behavior as a robust fallback.
    auto guided = cartesianGuidedPath(c, start, target);
    if (!guided.empty() && !c.cancelled()) {
        output.waypoints = std::move(guided);
        output.terminalError = (target - c.positions(output.waypoints.back()).back()).length();
        return withEndEffectorTrace(c, prepareTrajectory(c, std::move(output)));
    }

    // Redundant arms may have many collision-free IK endpoints. In an
    // obstructed scene, the first discovered RRT route can be much longer
    // than another valid route to the very same Cartesian target.
    //
    // Compare at most three successful alternatives, and stop searching
    // after a small bounded window beyond the first success. This keeps the
    // asynchronous GUI responsive and avoids unbounded "optimization".
    bool foundGoal = false;
    PlannedJointPath2D best;
    double bestDistance = std::numeric_limits<double>::infinity();
    const double directLowerBound = (target - c.positions(start).back()).length();
    int firstSuccessAttempt = -1;
    int routesFound = 0;
    for (int attempt = 0; attempt < 70; ++attempt) {
        if (c.cancelled()) {
            output.failure = PlanningFailure2D::Cancelled;
            return output;
        }
        if (firstSuccessAttempt >= 0 &&
            (routesFound >= 3 || attempt > firstSuccessAttempt + 10))
            break;
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
        if (!c.validJointAngles(seed) || !refineIK(c, seed, target)) continue;
        foundGoal = true;
        auto path = connectPath(c, start, seed, rng);
        if (path.empty()) continue;
        if (firstSuccessAttempt < 0) firstSuccessAttempt = attempt;
        ++routesFound;

        PlannedJointPath2D candidate;
        candidate.waypoints = shorten(c, std::move(path));
        if (candidate.waypoints.empty()) continue;
        candidate.terminalError =
            (target - c.positions(candidate.waypoints.back()).back()).length();
        candidate = withEndEffectorTrace(
            c, prepareTrajectory(c, std::move(candidate)));
        if (candidate.found() &&
            candidate.endEffectorDistance < bestDistance) {
            bestDistance = candidate.endEffectorDistance;
            best = std::move(candidate);
        }

        if (best.found() &&
            (bestDistance <= directLowerBound * 1.03 ||
             (obstacles.empty() && routesFound >= 1)))
            break;
    }
    if (c.cancelled()) {
        output.failure = PlanningFailure2D::Cancelled;
        return output;
    }
    if (best.found()) return best;
    output.failure = foundGoal ? PlanningFailure2D::PathSearchExhausted :
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
    case PlanningFailure2D::TargetBlockedByJointLimits: return "TARGET BLOCKS SHOULDER ROTATION";
    }
    return "UNKNOWN PLANNING STATUS";
}

void PlannedMotion2D::reset() noexcept {
    path_.clear();
    endEffectorTrace_.clear();
    jointTraces_.clear();
    endEffectorDistance_ = 0.0;
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
    endEffectorTrace_ = std::move(plan.endEffectorTrace);
    jointTraces_ = std::move(plan.jointTraces);
    endEffectorDistance_ = plan.endEffectorDistance;
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
