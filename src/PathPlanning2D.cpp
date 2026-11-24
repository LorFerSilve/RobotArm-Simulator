#include "robotics/kinematics/PathPlanning2D.hpp"

#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>

namespace robotics::kinematics {
namespace {

using Angles = std::vector<double>;
constexpr double pi = std::numbers::pi;
constexpr double fullTurn = 2.0 * pi;
constexpr double edgeResolution = 0.04; // maximum angular displacement per check
constexpr double searchStep = 0.32;

int boundedSampleCount(double estimate, int minimum, int maximum) {
    if (!std::isfinite(estimate)) return maximum;
    return static_cast<int>(std::clamp(std::ceil(estimate),
                                      static_cast<double>(minimum),
                                      static_cast<double>(maximum)));
}

struct SearchContext {
    std::vector<Joint2D> joints;
    std::vector<JointLimits2D> limits;
    std::vector<double> linkRadii;
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
          linkRadii(linkRadii2D(state)), settings(s), target(goal),
          obstacles(scene), cancellation(stop) {}

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

    [[nodiscard]] double targetBound() const noexcept {
        return settings.targetObject ?
            targetObjectBound2D(*settings.targetObject) :
            settings.targetClearance;
    }

    [[nodiscard]] bool targetClear(std::span<const math::Vec2> points) const {
        return settings.targetObject ?
            targetApproachClear(points, target, *settings.targetObject, linkRadii) :
            targetApproachClear(points, target, settings.targetClearance, linkRadii);
    }

    // Each internal joint can move in one connected interval. Even if raw
    // limits include both signs, reaching the other branch would cross the
    // forbidden straight or folded configurations.
    [[nodiscard]] std::pair<double, double> angleRange(std::size_t i) const {
        const auto& limit = limits[i];
        if (i == 0) return {limit.minAngle, limit.maxAngle};
        if (std::signbit(joints[i].angle)) {
            return {std::max(limit.minAngle, -pi + settings.foldClearance),
                    std::min(limit.maxAngle, -settings.jointClearance)};
        }
        return {std::max(limit.minAngle, settings.jointClearance),
                std::min(limit.maxAngle, pi - settings.foldClearance)};
    }

    [[nodiscard]] bool validJointAngles(const Angles& q) const {
        if (q.size() != joints.size()) return false;
        for (std::size_t i = 0; i < q.size(); ++i) {
            const auto [low, high] = angleRange(i);
            if (!std::isfinite(q[i]) || q[i] < low || q[i] > high) return false;
        }
        return true;
    }

    [[nodiscard]] bool collisionFreeRobot(
        std::span<const math::Vec2> points) const {
        return !hasSelfCollision(points, settings.linkClearance, linkRadii) &&
               !geometry::hasObstacleCollision(points, obstacles,
                                                settings.obstacleClearance, linkRadii);
    }

    [[nodiscard]] bool validRobot(const Angles& q) const {
        return validJointAngles(q) && collisionFreeRobot(positions(q));
    }

    [[nodiscard]] bool valid(const Angles& q) const {
        if (!validJointAngles(q)) return false;
        const auto points = positions(q);
        return collisionFreeRobot(points) && targetClear(points);
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
        if (cancelled() || !validJointAngles(a) || !validJointAngles(b)) return false;
        if (settings.targetObject) {
            return sweptArmClear2D(joints, a, b, obstacles, settings.linkClearance,
                                  settings.obstacleClearance, target,
                                  *settings.targetObject, cancellation);
        }
        if (settings.targetClearance == 0.0) {
            return sweptArmClear2D(joints, a, b, obstacles, settings.linkClearance,
                                  settings.obstacleClearance, cancellation);
        }

        // One reusable FK and two target-depth buffers per edge instead of
        // copying the robot and allocating geometry/depths at every sample.
        std::vector<math::Vec2> points;
        forwardKinematics(joints, a, points);
        if (!collisionFreeRobot(points)) return false;

        std::vector<double> previous(joints.size());
        std::vector<double> next(joints.size());
        targetLinkPenetrations(points, target, settings.targetClearance, previous, linkRadii);
        const double targetReach = settings.targetClearance + (linkRadii.empty() ? 0.0 :
            *std::max_element(linkRadii.begin(), linkRadii.end()));

        double maxDelta = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i)
            maxDelta = std::max(maxDelta, std::abs(a[i] - b[i]));
        constexpr int maxLegacySamples = 4096;
        const double requiredSteps = std::ceil(maxDelta / edgeResolution);
        if (!(requiredSteps <= maxLegacySamples)) return false;
        const int steps = std::max(1, static_cast<int>(requiredSteps));
        int remainingSamples = maxLegacySamples;
        Angles from = a;
        Angles to(a.size());
        Angles q(a.size());

        for (int step = 1; step <= steps; ++step) {
            if (cancelled()) return false;
            const double alpha = static_cast<double>(step) / steps;
            for (std::size_t i = 0; i < q.size(); ++i)
                to[i] = std::lerp(a[i], b[i], alpha);
            if (!validJointAngles(to)) return false;

            // Bound Cartesian movement of all links between consecutive
            // samples; preserve target clearance and scene-obstacle margins.
            double requiredChecks = 1.0;
            if (targetBound() > 0.0) {
                double downstream = 0.0;
                double maximumTravel = 0.0;
                for (std::size_t i = q.size(); i-- > 0;) {
                    downstream += joints[i].length;
                    maximumTravel += downstream * std::abs(to[i] - from[i]);
                }
                if (nearestLinkDistance(points) <=
                    targetReach + maximumTravel) {
                    const double maxTravelPerCheck =
                        std::max(0.02, targetBound() * 0.4);
                    requiredChecks = std::max(
                        1.0, std::ceil(maximumTravel / maxTravelPerCheck));
                }
            }
            if (!obstacles.empty()) {
                double downstream = 0.0, maximumTravel = 0.0;
                for (std::size_t i = q.size(); i-- > 0;) {
                    downstream += joints[i].length;
                    maximumTravel += downstream * std::abs(to[i] - from[i]);
                }
                requiredChecks = std::max(requiredChecks, std::ceil(maximumTravel / 0.04));
            }

            // A budget limit must reject the edge, never coarsen its checks.
            if (!(requiredChecks <= remainingSamples)) return false;
            const int checks = static_cast<int>(requiredChecks);
            remainingSamples -= checks;
            for (int sub = 1; sub <= checks; ++sub) {
                if (cancelled()) return false;
                const double fraction = static_cast<double>(sub) / checks;
                for (std::size_t i = 0; i < q.size(); ++i)
                    q[i] = std::lerp(from[i], to[i], fraction);
                if (!validJointAngles(q)) return false;
                forwardKinematics(joints, q, points);
                if (!collisionFreeRobot(points)) return false;

                targetLinkPenetrations(points, target, settings.targetClearance, next, linkRadii);
                for (std::size_t i = 0; i < next.size(); ++i)
                    if (next[i] > previous[i] + 1e-9) return false;
                previous.swap(next);
            }
            // "points" now contains the geometry of "to", ready for the
            // next outer step's swept-target proximity estimate.
            from = to;
        }
        // Sampling alone cannot rule out a narrow collision between checks.
        // Certify the whole linear joint-space edge with a conservative
        // bound on link displacement. Fail closed if certification runs out
        // of subdivisions or the request was cancelled.
        return sweptArmClear2D(
            joints, a, b, obstacles, settings.linkClearance,
            settings.obstacleClearance, cancellation);
    }
};

double distanceSquared(const Angles& a, const Angles& b) {
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    return sum;
}

// Virtual damped Jacobian refinement within the physical joint intervals.
// Return the remaining Cartesian error so the caller can distinguish slow
// convergence from a reached pose that fails the separate collision check.
[[nodiscard]] double refineIK(const SearchContext& c, Angles& q, math::Vec2 target) {
    auto positions = c.positions(q);
    auto error = target - positions.back();
    double residual = error.length();
    Angles direction(q.size()), next(q.size());
    for (int step = 0; step < 180 && residual > c.settings.positionTolerance; ++step) {
        if (c.cancelled()) return std::numeric_limits<double>::infinity();
        auto jacobian = endEffectorJacobian(positions);

        // Freeze saturated joints that the gradient would push outward,
        // both in the normal equations and in the resulting update.
        double xx = 0.0025, xy = 0.0, yy = 0.0025;
        for (std::size_t i = 0; i < q.size(); ++i) {
            const auto [low, high] = c.angleRange(i);
            auto& column = jacobian[i];
            const double gradient = column.dot(error);
            if (low == high || (q[i] <= low && gradient < 0.0) ||
                (q[i] >= high && gradient > 0.0))
                column = {};
            xx += column.x * column.x;
            xy += column.x * column.y;
            yy += column.y * column.y;
        }
        const double determinant = xx * yy - xy * xy;
        if (!std::isfinite(determinant) || determinant <= 1e-15) break;
        const math::Vec2 w{(yy * error.x - xy * error.y) / determinant,
                           (xx * error.y - xy * error.x) / determinant};
        if (!std::isfinite(w.x) || !std::isfinite(w.y)) break;
        for (std::size_t i = 0; i < q.size(); ++i)
            direction[i] = std::clamp(jacobian[i].dot(w), -0.3, 0.3);

        bool improved = false;
        for (double alpha = 1.0; alpha >= 1.0 / 128.0; alpha *= 0.5) {
            for (std::size_t i = 0; i < q.size(); ++i) {
                const auto [low, high] = c.angleRange(i);
                next[i] = std::clamp(q[i] + alpha * direction[i], low, high);
            }
            // IK iterations are virtual numerical guesses, not executed
            // robot states. The goal and the actual motion are checked later.
            if (!c.validJointAngles(next)) break;
            forwardKinematics(c.joints, next, positions);
            const auto nextError = target - positions.back();
            const double nextResidual = nextError.length();
            if (nextResidual < residual) {
                q.swap(next);
                error = nextError;
                residual = nextResidual;
                improved = true;
                break;
            }
        }
        if (!improved) break;
    }
    return residual;
}

struct TwoLinkPose {
    double shoulder;
    double elbow;
};

std::optional<TwoLinkPose> twoLinkPose(const SearchContext& c, math::Vec2 target) {
    const double first = c.joints[0].length;
    const double second = c.joints[1].length;
    const auto [low, high] = c.angleRange(1);
    const bool negative = std::signbit(c.joints[1].angle);
    const double minBend = negative ? -high : low;
    const double maxBend = negative ? -low : high;
    const auto reachAt = [first, second](double bend) {
        return std::hypot(first + second * std::cos(bend), second * std::sin(bend));
    };
    const double inner = reachAt(maxBend);
    const double outer = reachAt(minBend);
    const double radius = target.length();
    if (!std::isfinite(radius) || inner - radius > c.settings.positionTolerance ||
        radius - outer > c.settings.positionTolerance)
        return std::nullopt;

    // A target just beyond the annulus can still be reached within tolerance.
    // Use the exact joint boundary in that case to avoid rounding past it.
    double bend = minBend;
    if (radius <= inner) {
        bend = maxBend;
    } else if (radius < outer) {
        const double scale = std::max(first, second);
        const double a = first / scale, b = second / scale, r = radius / scale;
        const double cosine = (r*r - a*a - b*b) / (2.0*a*b);
        if (!std::isfinite(cosine)) return std::nullopt;
        bend = std::clamp(std::acos(std::clamp(cosine, -1.0, 1.0)),
                          minBend, maxBend);
    }
    const double elbow = negative ? -bend : bend;
    const double shoulder = std::atan2(target.y, target.x) -
        std::atan2(second * std::sin(elbow), first + second * std::cos(elbow));
    return TwoLinkPose{shoulder, elbow};
}

// Prefer a Cartesian straight-line motion whenever its inverse-kinematics
// lift is feasible. The tool is sampled along the straight line, while each
// connecting joint-space edge is checked using the existing full safety
// constraints. This is a candidate, never a shortcut around collision checks.
[[nodiscard]] std::vector<Angles> cartesianGuidedPath(
    const SearchContext& c, const Angles& start, math::Vec2 target) {
    const auto origin = c.positions(start).back();
    const double distance = (target - origin).length();
    if (!std::isfinite(distance)) return {};
    if (distance <= c.settings.positionTolerance)
        return c.valid(start) ? std::vector<Angles>{start} : std::vector<Angles>{};
    const int count = boundedSampleCount(distance / 0.18, 6, 28);
    std::vector<Angles> path{start};
    path.reserve(static_cast<std::size_t>(count + 1));
    Angles current = start;

    for (int step = 1; step <= count; ++step) {
        if (c.cancelled()) return {};
        const auto nextTarget = origin + (target - origin) *
            (static_cast<double>(step) / count);
        Angles next;
        if (c.joints.size() == 2) {
            const auto pose = twoLinkPose(c, nextTarget);
            if (!pose) return {};
            const double turn = std::round((current[0] - pose->shoulder) / fullTurn);
            double best = std::numeric_limits<double>::infinity();
            for (int offset = -3; offset <= 3; ++offset) {
                Angles candidate{pose->shoulder + fullTurn * (turn + offset), pose->elbow};
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
            if (!(refineIK(c, next, nextTarget) <= c.settings.positionTolerance) ||
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
    if (goals.empty() || c.targetBound() <= 0.0 || !c.valid(start) ||
        (c.settings.targetObject &&
         c.settings.targetObject->shape != TargetShape2D::Circle))
        return false;
    const double r = c.target.length();
    const double shoulderClearance = c.targetBound() + c.joints[0].linkRadius;
    if (r <= shoulderClearance ||
        r > c.joints[0].length + shoulderClearance)
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
    if (d <= searchStep) return to;
    const double scale = searchStep / d;
    Angles next(from.size());
    for (std::size_t i = 0; i < from.size(); ++i)
        next[i] = std::lerp(from[i], to[i], scale);
    return next;
}

// Returns the newly added node index, or -1 if blocked.
int extend(std::vector<Node>& tree, const Angles& target, const SearchContext& c,
           bool fromStart) {
    const int index = nearest(tree, target);
    const auto& from = tree[static_cast<std::size_t>(index)].q;
    auto next = steer(from, target);
    if (distanceSquared(from, next) < 1e-12) return -1;
    // Goal branches are traversed in reverse. Only the actual start of a
    // plan may supply existing target contact to the retreat rule.
    const bool safe = fromStart ? c.edge(from, next) :
                                 c.valid(next) && c.edge(next, from);
    if (!safe) return -1;
    tree.push_back({std::move(next), index});
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
    const SearchContext& c, const Angles& start, const Angles& goal, std::mt19937& rng) {
    std::vector<Node> fromStart{{start, -1}};
    std::vector<Node> fromGoal{{goal, -1}};
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int iter = 0; iter < 1800; ++iter) {
        if (c.cancelled()) return {};
        const bool forward = iter % 2 == 0;
        auto& growing = forward ? fromStart : fromGoal;
        auto& matching = forward ? fromGoal : fromStart;
        Angles sample(start.size());
        for (std::size_t i = 0; i < sample.size(); ++i) {
            const auto [low, high] = c.angleRange(i);
            sample[i] = std::lerp(low, high, unit(rng));
        }
        // An alternative search must start with a sampled branch. Starting
        // with goal bias would immediately recreate the already-tested edge.
        if (iter > 0 && iter % 9 == 0)
            sample = forward ? goal : start;
        const int grown = extend(growing, sample, c, forward);
        if (grown < 0) continue;
        const auto& destination = growing[static_cast<std::size_t>(grown)].q;
        for (int k = 0; k < 80; ++k) {
            if (c.cancelled()) return {};
            const int newer = extend(matching, destination, c, !forward);
            if (newer < 0) break;
            const auto& endpoint = matching[static_cast<std::size_t>(newer)].q;
            if (distanceSquared(endpoint, destination) < 1e-10) {
                const auto& bridgeStart = forward ? destination : endpoint;
                const auto& bridgeEnd = forward ? endpoint : destination;
                if (bridgeStart != bridgeEnd && !c.edge(bridgeStart, bridgeEnd))
                    break;
                auto prefix = forward ? trace(growing, grown) : trace(matching, newer);
                auto suffix = forward ? trace(matching, newer) : trace(growing, grown);
                std::reverse(suffix.begin(), suffix.end());
                const auto first = bridgeStart == bridgeEnd ? suffix.begin() + 1 : suffix.begin();
                prefix.insert(prefix.end(), first, suffix.end());
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
    const int samples = boundedSampleCount(travelBound / 0.06, 8, 384);
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
    for (auto& q : plan.waypoints) {
        if (unique.empty() || unique.back() != q)
            unique.push_back(std::move(q));
    }
    plan.waypoints = std::move(unique);
    if (plan.waypoints.size() < 2) return plan;

    const std::size_t count = plan.waypoints.size();
    const std::size_t joints = plan.waypoints.front().size();
    constexpr double validationResolution = 0.025;
    constexpr int maxSplineSamples = 4096;
    plan.pathDistances.assign(count, 0.0);
    for (std::size_t i = 1; i < count; ++i) {
        plan.pathDistances[i] = plan.pathDistances[i - 1] +
            std::sqrt(distanceSquared(plan.waypoints[i - 1], plan.waypoints[i]));
        const double span = plan.pathDistances[i] - plan.pathDistances[i - 1];
        if (!(span > 0.0 && span <= validationResolution * maxSplineSamples)) {
            plan.pathDistances.clear();
            return plan;
        }
    }
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
        const double before = plan.pathDistances[i] - plan.pathDistances[i - 1];
        const double after = plan.pathDistances[i + 1] - plan.pathDistances[i];
        for (std::size_t j = 0; j < joints; ++j) {
            const double tangent = 0.45 * (plan.waypoints[i + 1][j] -
                                           plan.waypoints[i - 1][j]) / (before + after);
            const auto [low, high] = c.angleRange(j);
            const double q = plan.waypoints[i][j];
            // The furthest adjacent Bezier controls are q +/- 2*h*tangent/5.
            // Keeping them inside the same interval bounds the entire curve.
            const double minimum = std::max((q - high) / (0.4 * before),
                                            (low - q) / (0.4 * after));
            const double maximum = std::min((q - low) / (0.4 * before),
                                            (high - q) / (0.4 * after));
            plan.splineTangents[i][j] = std::clamp(tangent, minimum, maximum);
        }
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
                static_cast<int>(std::ceil(h / validationResolution)));
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
        const int samples = boundedSampleCount(tipTravelBound / 0.075, 16, 512);
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

void validateMotionLimits(std::span<const JointLimits2D> limits,
                          const MotionSettings2D& settings) {
    const auto positive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    if (!positive(settings.maxAngularSpeed) ||
        !positive(settings.maxAngularAcceleration) ||
        !positive(settings.maxAngularJerk))
        throw std::invalid_argument("Invalid global motion limits");
    if (!positive(settings.jointClearance) || settings.jointClearance >= pi / 2.0 ||
        !std::isfinite(settings.foldClearance) ||
        settings.foldClearance < settings.jointClearance || settings.foldClearance >= pi / 2.0)
        throw std::invalid_argument("Invalid joint safety margins");
    for (const auto& limit : limits) {
        if (!std::isfinite(limit.minAngle) || !std::isfinite(limit.maxAngle) ||
            limit.minAngle >= limit.maxAngle || !positive(limit.maxAngularSpeed) ||
            !positive(limit.maxAngularAcceleration))
            throw std::invalid_argument("Invalid joint limits");
    }
}

bool motionLimitsAllow(const Joint2D& before, const Joint2D& after,
                       const JointLimits2D& limit,
                       const MotionSettings2D& settings, double dt) {
    const double speed = std::min(settings.maxAngularSpeed, limit.maxAngularSpeed);
    const double acceleration = std::min(settings.maxAngularAcceleration,
                                         limit.maxAngularAcceleration);
    return std::abs(after.angularVelocity) <= speed + 1e-7 &&
           std::abs(after.angularVelocity - before.angularVelocity) <= acceleration * dt + 1e-7;
}

void validatePlan(const PlannedJointPath2D& plan, math::Vec2 target) {
    if (!std::isfinite(target.x) || !std::isfinite(target.y))
        throw std::invalid_argument("Non-finite plan target");
    if (!plan.found()) return;
    const std::size_t jointCount = plan.waypoints.front().size();
    const auto validVector = [jointCount](const Angles& vector) {
        return vector.size() == jointCount &&
            std::all_of(vector.begin(), vector.end(),
                        [](double value) { return std::isfinite(value); });
    };
    if (jointCount < 2 ||
        !std::all_of(plan.waypoints.begin(), plan.waypoints.end(), validVector))
        throw std::invalid_argument("Invalid waypoint dimensions or angles");
    if (plan.trajectoryDuration == 0.0 &&
        plan.splineTangents.empty() && plan.pathDistances.empty())
        return;
    if (plan.waypoints.size() < 2 || !std::isfinite(plan.trajectoryDuration) ||
        plan.trajectoryDuration <= 0.0 ||
        plan.splineTangents.size() != plan.waypoints.size() ||
        plan.pathDistances.size() != plan.waypoints.size() ||
        plan.pathDistances.front() != 0.0 ||
        !std::all_of(plan.splineTangents.begin(), plan.splineTangents.end(), validVector))
        throw std::invalid_argument("Invalid trajectory duration, knots or tangents");
    for (std::size_t i = 1; i < plan.pathDistances.size(); ++i) {
        if (!std::isfinite(plan.pathDistances[i]) ||
            plan.pathDistances[i] <= plan.pathDistances[i - 1])
            throw std::invalid_argument("Trajectory knots must increase strictly");
    }
}

void validate(const SearchContext& c, math::Vec2 target) {
    if (c.joints.size() < 2 || c.joints.size() != c.limits.size() ||
        !std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(c.settings.positionTolerance) || c.settings.positionTolerance <= 0.0 ||
        !std::isfinite(c.settings.targetClearance) ||
        c.settings.targetClearance < 0.0 ||
        !std::isfinite(c.settings.obstacleClearance) ||
        c.settings.obstacleClearance < 0.0 ||
        !std::isfinite(c.settings.linkClearance) || c.settings.linkClearance < 0.0)
        throw std::invalid_argument("Planning requires >=2 joints, matching limits and finite target");
    validateMotionLimits(c.limits, c.settings);
    for (const auto& joint : c.joints) {
        if (!std::isfinite(joint.length) || joint.length <= 0.0 ||
            !std::isfinite(joint.angle) || !std::isfinite(joint.angularVelocity))
            throw std::invalid_argument("Invalid robot geometry or state");
    }
    if (c.settings.targetObject &&
        (!validTargetObject2D(*c.settings.targetObject) ||
         c.settings.targetObject->toolLength >= c.joints.back().length))
        throw std::invalid_argument("Invalid target object or tool contact length");
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
    if (c.cancelled()) { output.failure = PlanningFailure2D::Cancelled; return output; }
    const Angles start = c.angles();
    // An already contacted target needs an escape path, not an immediate failure.
    output.terminalError = (target - c.positions(start).back()).length();
    if (output.terminalError <= settings.positionTolerance && c.valid(start)) {
        output.waypoints = {start};
        return withEndEffectorTrace(c, std::move(output));
    }
    double reach = 0.0;
    for (const auto& joint : joints) reach += joint.length;
    if (!std::isfinite(reach)) throw std::overflow_error("Arm reach overflow");
    if (target.length() > reach + settings.positionTolerance) {
        output.failure = PlanningFailure2D::OutOfReach;
        return output;
    }
    // A tool endpoint inside a solid obstacle cannot be a valid goal.
    for (const auto& obstacle : obstacles) {
        if (geometry::pointHitsObstacle(target, obstacle,
                                        settings.obstacleClearance + joints.back().linkRadius)) {
            output.failure = PlanningFailure2D::GoalInObstacle;
            return output;
        }
    }
    // The base is fixed at the origin; a target containing the base can
    // never be cleared by the first link, regardless of reconfiguration.
    const bool targetCoversBase = settings.targetObject ?
        geometry::pointHitsObstacle({0.0, 0.0},
            targetBody2D(*settings.targetObject, target),
            settings.targetObject->clearance + joints.front().linkRadius) :
        (settings.targetClearance > 0.0 &&
         target.length() < settings.targetClearance + joints.front().linkRadius);
    if (targetCoversBase) {
        output.failure = PlanningFailure2D::GoalInObstacle;
        return output;
    }

    std::mt19937 rng{0x526f626fU};
    const double directLowerBound = output.terminalError;
    PlannedJointPath2D best;
    const auto consider = [&](std::vector<Angles> path) {
        if (path.empty() || c.cancelled()) return false;
        PlannedJointPath2D candidate;
        candidate.waypoints = std::move(path);
        candidate.terminalError =
            (target - c.positions(candidate.waypoints.back()).back()).length();
        if (!(candidate.terminalError <= settings.positionTolerance)) return false;
        candidate = withEndEffectorTrace(c, prepareTrajectory(c, std::move(candidate)));
        if (!candidate.found() || !std::isfinite(candidate.endEffectorDistance))
            return false;
        if (!best.found() || candidate.endEffectorDistance < best.endEffectorDistance)
            best = std::move(candidate);
        return true;
    };
    const auto nearStraight = [&] {
        return best.found() && best.endEffectorDistance <= directLowerBound * 1.03;
    };

    if (joints.size() == 2) {
        // The closed-form IK solution is periodic in the shoulder angle.
        // A short shoulder rotation may sweep an intermediate link across the
        // target, while the equivalent angle +/- 2*pi is collision-free.
        // Preserve the elbow branch (switching branches would require
        // crossing a forbidden straight/folded configuration).
        const auto pose = twoLinkPose(c, target);
        if (!pose) {
            output.failure = PlanningFailure2D::OutOfReach;
            return output;
        }
        const double nearestTurn = std::round((start[0] - pose->shoulder) / fullTurn);

        std::vector<Angles> goals;
        for (int offset = -3; offset <= 3; ++offset) {
            const double angle = pose->shoulder + (nearestTurn + offset) * fullTurn;
            const double shoulder = std::clamp(angle, limits[0].minAngle, limits[0].maxAngle);
            Angles candidate{shoulder, pose->elbow};
            if (c.valid(candidate) &&
                (target - c.positions(candidate).back()).length() <= settings.positionTolerance &&
                std::find(goals.begin(), goals.end(), candidate) == goals.end())
                goals.push_back(std::move(candidate));
        }
        std::sort(goals.begin(), goals.end(),
                  [&](const Angles& a, const Angles& b) {
                      return distanceSquared(start, a) <
                             distanceSquared(start, b);
                  });

        // Use Cartesian tip travel rather than the smallest joint-angle
        // change as the selection cost. A safe near-straight IK lift takes
        // priority over the broad arc of one joint-space interpolation.
        std::vector<std::vector<Angles>> comparedPaths;
        const auto considerDistinct = [&](std::vector<Angles> path) {
            if (path.empty() || c.cancelled() ||
                std::find(comparedPaths.begin(), comparedPaths.end(), path) != comparedPaths.end())
                return false;
            comparedPaths.push_back(path);
            return consider(std::move(path));
        };
        considerDistinct(cartesianGuidedPath(c, start, target));
        for (const auto& goal : goals) {
            if (c.cancelled()) break;
            if (c.edge(start, goal)) considerDistinct({start, goal});
        }
        // In a free scene, the existing direct/Cartesian choice is sufficient.
        // With obstacles, a valid direct joint-space edge may still trace a
        // wide Cartesian arc; compare it with a few safe RRT alternatives.
        if (!c.cancelled() && best.found() &&
            (obstacles.empty() || nearStraight()))
            return best;

        constexpr int variantsPerGoal = 2;
        constexpr int maxRrtCandidates = 4;
        int candidates = 0;
        for (const auto& goal : goals) {
            for (int variant = 0; variant < variantsPerGoal; ++variant) {
                if (c.cancelled() || candidates >= maxRrtCandidates || nearStraight())
                    break;
                auto path = connectPath(c, start, goal, rng);
                if (path.empty()) continue;
                if (considerDistinct(shorten(c, std::move(path)))) ++candidates;
            }
            if (c.cancelled() || candidates >= maxRrtCandidates || nearStraight())
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

    constexpr int maxIKAttempts = 72;
    constexpr int extraIKAttempts = 12;
    constexpr int maxRrtSearches = 16;
    constexpr int maxRoutes = 3;
    constexpr double initialRestartRadius = 0.25;
    constexpr double maxRestartRadius = 1.5;
    constexpr double duplicateGoalDistance = 0.01;

    int routes = consider(cartesianGuidedPath(c, start, target)) ? 1 : 0;
    int attemptLimit = routes > 0 ? extraIKAttempts : maxIKAttempts;
    int rrtSearches = 0;
    bool foundIKPose = false;
    bool foundSafeGoal = false;
    std::vector<Angles> testedGoals;
    Angles anchor = start;
    double anchorError = directLowerBound;
    double restartRadius = initialRestartRadius;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int attempt = 0; attempt < attemptLimit && routes < maxRoutes &&
                          !nearStraight() && !c.cancelled(); ++attempt) {
        // Alternate full-range samples with local restarts. Use the best
        // residual, then the first collision-free IK pose, as the local
        // anchor. Expand the neighborhood when local restarts stall.
        Angles seed = start;
        const bool local = attempt % 2 != 0;
        if (attempt > 0) {
            for (std::size_t i = 0; i < seed.size(); ++i) {
                auto [low, high] = c.angleRange(i);
                if (local) {
                    low = std::max(low, anchor[i] - restartRadius);
                    high = std::min(high, anchor[i] + restartRadius);
                }
                seed[i] = std::lerp(low, high, unit(rng));
            }
        }
        const double residual = refineIK(c, seed, target);
        if (c.cancelled()) break;
        const bool reachedIK = residual <= settings.positionTolerance;
        const bool safeGoal = reachedIK && c.valid(seed);
        if (!foundSafeGoal && (safeGoal || residual < anchorError)) {
            anchor = seed;
            anchorError = residual;
            restartRadius = initialRestartRadius;
        } else if (local) {
            restartRadius = std::min(maxRestartRadius, restartRadius * 1.5);
        }
        foundIKPose = foundIKPose || reachedIK;
        foundSafeGoal = foundSafeGoal || safeGoal;
        if (!safeGoal) continue;

        if (std::any_of(testedGoals.begin(), testedGoals.end(), [&](const Angles& goal) {
                return distanceSquared(goal, seed) <
                       duplicateGoalDistance * duplicateGoalDistance;
            }))
            continue;
        testedGoals.push_back(seed);
        std::vector<Angles> path;
        if (c.edge(start, seed)) {
            path = {start, seed};
        } else if (rrtSearches < maxRrtSearches) {
            ++rrtSearches;
            path = connectPath(c, start, seed, rng);
        }
        if (!consider(shorten(c, std::move(path)))) continue;
        ++routes;
        if (routes == 1)
            attemptLimit = std::min(maxIKAttempts, attempt + 1 + extraIKAttempts);
    }
    if (c.cancelled()) {
        output.failure = PlanningFailure2D::Cancelled;
        return output;
    }
    if (best.found()) return best;
    output.failure = foundSafeGoal ? PlanningFailure2D::PathSearchExhausted :
                     foundIKPose ? PlanningFailure2D::IKGoalsInCollision :
                                   PlanningFailure2D::IKSearchExhausted;
    return output;
}

const char* planningFailureMessage(PlanningFailure2D reason) noexcept {
    switch (reason) {
    case PlanningFailure2D::None: return "READY";
    case PlanningFailure2D::Cancelled: return "PLANNING CANCELLED";
    case PlanningFailure2D::OutOfReach: return "OUT OF REACH";
    case PlanningFailure2D::GoalInObstacle: return "TARGET BLOCKED";
    case PlanningFailure2D::NoIKSolution: return "NO VALID IK SOLUTION";
    case PlanningFailure2D::PathSearchExhausted: return "PATH SEARCH BUDGET EXHAUSTED";
    case PlanningFailure2D::TargetBlockedByJointLimits: return "TARGET BLOCKS SHOULDER ROTATION";
    case PlanningFailure2D::MotionLimitExceeded: return "MOTION LIMIT EXCEEDED";
    case PlanningFailure2D::UnsafeMotionStep: return "UNSAFE MOTION STEP";
    case PlanningFailure2D::IKSearchExhausted: return "IK SEARCH BUDGET EXHAUSTED";
    case PlanningFailure2D::IKGoalsInCollision: return "IK FOUND ONLY COLLIDING POSES";
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
    validatePlan(plan, target);
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
    if (!hasTarget_ || joints.size() < 2 || joints.size() != limits.size() || !std::isfinite(dt) ||
        dt <= 0.0 || dt > 0.1)
        throw std::invalid_argument("Follower requires target, joint limits and valid timestep");
    validateMotionLimits(limits, settings);
    if (!path_.empty()) {
        if (joints.size() != path_.front().size())
            throw std::invalid_argument("Plan joint count does not match the arm");
        if (index_ <= 1 && segmentElapsed_ == 0.0 && trajectoryElapsed_ == 0.0) {
            for (std::size_t i = 0; i < joints.size(); ++i)
                if (joints[i].angle != path_.front()[i])
                    throw std::invalid_argument("Plan start pose does not match the arm");
        }
    }

    MotionStatus2D status;
    status.positionError = (target_ - forwardKinematics(joints).back()).length();
    const auto blockStep = [&](PlanningFailure2D reason) {
        failed_ = true;
        failure_ = reason;
        status.blocked = true;
        status.constrained = true;
        for (auto& joint : joints) joint.angularVelocity = 0.0;
        return status;
    };
    status.blocked = failed_;
    if (failed_) {
        status.constrained = !path_.empty();
        for (auto& joint : joints) joint.angularVelocity = 0.0;
        return status;
    }

    if (path_.empty()) return status;
    const SearchContext context(joints, limits, settings, target_, obstacles);
    if (index_ >= path_.size() || path_.size() == 1) {
        if (!context.valid(context.angles()))
            return blockStep(PlanningFailure2D::UnsafeMotionStep);
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
            if (!motionLimitsAllow(joints[j], proposed[j], limits[j], settings, dt))
                return blockStep(PlanningFailure2D::MotionLimitExceeded);
        }
        if ((atGoal && !context.valid(q)) || !context.edge(context.angles(), q))
            return blockStep(PlanningFailure2D::UnsafeMotionStep);
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
    double requiredDuration = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const double distance = std::abs(b[i] - a[i]);
        const double speed = std::min(settings.maxAngularSpeed, limits[i].maxAngularSpeed);
        const double acceleration = std::min(settings.maxAngularAcceleration,
                                             limits[i].maxAngularAcceleration);
        // The quintic smoothstep peaks at 1.875 in speed and 60 in jerk.
        // Factor 8 covers acceleration estimated from consecutive steps.
        requiredDuration = std::max({
            requiredDuration, 1.875 * distance / speed,
            std::sqrt(8.0 * distance / acceleration),
            std::cbrt(60.0 * distance / settings.maxAngularJerk)
        });
    }
    if (!std::isfinite(requiredDuration))
        return blockStep(PlanningFailure2D::MotionLimitExceeded);
    if (segmentDuration_ == 0.0)
        segmentDuration_ = std::max(requiredDuration, 2.0 * dt);
    else if (requiredDuration > segmentDuration_)
        return blockStep(PlanningFailure2D::MotionLimitExceeded);

    const double nextElapsed = std::min(segmentDuration_, segmentElapsed_ + dt);
    const double u = nextElapsed / segmentDuration_;
    const double progress = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    const bool atWaypoint = nextElapsed >= segmentDuration_;

    std::vector<Joint2D> candidate(joints.begin(), joints.end());
    Angles candidateAngles(joints.size());
    for (std::size_t i = 0; i < joints.size(); ++i) {
        candidate[i].angle = std::lerp(a[i], b[i], progress);
        candidate[i].angularVelocity = atWaypoint ? 0.0 :
            (candidate[i].angle - joints[i].angle) / dt;
        candidateAngles[i] = candidate[i].angle;
        if (!motionLimitsAllow(joints[i], candidate[i], limits[i], settings, dt))
            return blockStep(PlanningFailure2D::MotionLimitExceeded);
    }

    const bool finalPose = atWaypoint && index_ + 1 == path_.size();
    if ((finalPose && !context.valid(candidateAngles)) ||
        !context.edge(context.angles(), candidateAngles))
        return blockStep(PlanningFailure2D::UnsafeMotionStep);

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
