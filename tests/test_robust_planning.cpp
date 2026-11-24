#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using namespace robotics::kinematics;
using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::geometry::hasObstacleCollision;
using robotics::geometry::pointHitsObstacle;
using robotics::math::Vec2;

constexpr double pi = std::numbers::pi;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void checkRedundantDetour() {
    constexpr double pi = std::numbers::pi;
    constexpr std::array start{0.3, -0.7, 0.8}, knownGoal{0.607, -2.203, 1.783};
    constexpr std::array<Joint2D, 3> arm{{
        {1.25, start[0]}, {0.75, start[1]}, {1.1, start[2]}
    }};
    // The raw ranges include many turns and both branches. Only the
    // connected interval containing each starting internal angle is usable.
    constexpr std::array<JointLimits2D, 3> broadLimits{{
        {-1.0, 1.4, 2, 6}, {-1000.0, 1000.0, 2, 6}, {-1000.0, 1000.0, 2, 6}
    }};
    const std::array<Obstacle2D, 1> scene{Circle2D{{1.353, -0.201}, 0.04}};
    MotionSettings2D settings;
    settings.obstacleClearance = 0.04;
    std::vector<Vec2> points;
    forwardKinematics(arm, knownGoal, points);
    const Vec2 target = points.back();
    require(sweptArmClear2D(arm, start, knownGoal, scene, settings.linkClearance,
                            settings.obstacleClearance),
            "The reference goal must have a certified safe path");
    const auto plan = planJointPath2D(arm, broadLimits, target, settings, scene);
    require(plan.found(), "Broad raw limits must not starve the safe redundant IK search");
    require(plan.terminalError <= settings.positionTolerance,
            "The alternate IK pose must reach the same tool position");

    const auto withinBranches = [&](std::span<const double> q) {
        constexpr double roundoff = 1e-10;
        return q.size() == arm.size() &&
               q[0] >= broadLimits[0].minAngle - roundoff &&
               q[0] <= broadLimits[0].maxAngle + roundoff &&
               q[1] >= -pi + settings.foldClearance - roundoff &&
               q[1] <= -settings.jointClearance + roundoff &&
               q[2] >= settings.jointClearance - roundoff &&
               q[2] <= pi - settings.foldClearance + roundoff;
    };
    for (const auto& q : plan.waypoints)
        require(withinBranches(q), "All planned waypoints must stay on the starting branches");

    if (!plan.splineTangents.empty()) {
        for (std::size_t i = 0; i + 1 < plan.waypoints.size(); ++i) {
            const double offset = 0.4 * (plan.pathDistances[i + 1] - plan.pathDistances[i]);
            std::array<double, 3> outgoing{}, incoming{};
            for (std::size_t j = 0; j < arm.size(); ++j) {
                outgoing[j] = plan.waypoints[i][j] + offset * plan.splineTangents[i][j];
                incoming[j] = plan.waypoints[i + 1][j] - offset * plan.splineTangents[i + 1][j];
            }
            require(withinBranches(outgoing) && withinBranches(incoming),
                    "Spline control points must bound the whole curve inside the joint branches");
        }
    }

    // The certified reference edge also provides an independent scale for
    // the tool travel. The selected safe route should be comparably short.
    Vec2 previous = forwardKinematics(arm).back();
    double referenceDistance = 0.0;
    for (int step = 1; step <= 128; ++step) {
        const double t = step / 128.0;
        std::array<double, 3> q{};
        for (std::size_t i = 0; i < q.size(); ++i)
            q[i] = std::lerp(start[i], knownGoal[i], t);
        forwardKinematics(arm, q, points);
        referenceDistance += (points.back() - previous).length();
        previous = points.back();
    }
    require(plan.endEffectorDistance <= referenceDistance + 0.02,
            "The selected detour should retain short Cartesian tool travel");

    auto pose = arm;
    PlannedMotion2D follower;
    follower.usePlan(plan, target);
    for (int frame = 0; frame < 1800; ++frame) {
        const auto status = follower.advance(pose, broadLimits, 1.0 / 60.0, settings, scene);
        const std::array angles{pose[0].angle, pose[1].angle, pose[2].angle};
        require(!status.blocked && withinBranches(angles),
                "The recovered path must execute without a forbidden branch transition");
        forwardKinematics(pose, angles, points);
        require(!hasSelfCollision(points, settings.linkClearance) &&
                !hasObstacleCollision(points, scene, settings.obstacleClearance),
                "The shorter route must preserve every collision constraint");
        if (status.reached) return;
    }
    throw std::runtime_error("The redundant detour must reach the target");
}

void checkOtherBranchIsNotReachable() {
    constexpr std::array<Joint2D, 3> arm{{{1.0, 0.0}, {0.8, 0.7}, {0.6, 0.5}}};
    constexpr std::array<JointLimits2D, 3> limits{{
        {-0.025, 0.025, 2, 6}, {-1.0, 1.0, 2, 6}, {0.48, 0.52, 2, 6}
    }};
    auto otherBranch = arm;
    otherBranch[1].angle = -0.7;
    const auto target = forwardKinematics(otherBranch).back();

    // The shoulder and positive internal angles keep the tool above this
    // target throughout the allowed branch. Flipping q[1] is within the raw
    // joint range but would cross the forbidden straight configuration.
    const auto blocked = planJointPath2D(arm, limits, target);
    require(!blocked.found() && blocked.failure == PlanningFailure2D::IKSearchExhausted,
            "A valid pose on the disconnected branch must not qualify as an IK goal");
    const auto admissible = planJointPath2D(otherBranch, limits, target);
    require(admissible.found(), "The same Cartesian goal is valid on the other branch");
}

void checkSearchDiagnostics() {
    constexpr std::array<Joint2D, 3> arm{{{1.25, -0.3}, {0.75, -0.7}, {1.1, 0.8}}};
    constexpr std::array<JointLimits2D, 3> limits{{
        {-0.8, 0.8, 2, 6}, {-0.701, -0.699, 2, 6}, {0.799, 0.801, 2, 6}
    }};
    auto goal = arm;
    goal[0].angle = 0.35;
    const auto goalPoints = forwardKinematics(goal);
    const auto target = goalPoints.back();
    const auto firstLink = goalPoints[1] * 0.64;
    const std::array<Obstacle2D, 1> goalScene{Circle2D{firstLink, 0.03}};
    const std::array<Obstacle2D, 1> routeScene{Circle2D{{0.8, 0.0}, 0.03}};
    MotionSettings2D settings;
    settings.obstacleClearance = 0.04;
    require(planJointPath2D(arm, limits, target, settings).found(),
            "The test target must be reachable with the narrow joint ranges");
    require(!pointHitsObstacle(target, goalScene.front(), settings.obstacleClearance) &&
            hasObstacleCollision(goalPoints, goalScene, settings.obstacleClearance),
            "The obstacle must block the goal arm pose while keeping the tool point clear");
    const auto colliding = planJointPath2D(arm, limits, target, settings, goalScene);
    require(!colliding.found() && colliding.failure == PlanningFailure2D::IKGoalsInCollision,
            "Reached but colliding IK poses need a distinct failure reason");

    require(!hasObstacleCollision(goalPoints, routeScene, settings.obstacleClearance),
            "The route barrier must leave the reference goal pose clear");
    const auto noRoute = planJointPath2D(arm, limits, target, settings, routeScene);
    require(!noRoute.found() && noRoute.failure == PlanningFailure2D::PathSearchExhausted,
            "A collision-free goal with no certified route must report a path search failure");
}

} // namespace

int main() {
    try {
        checkRedundantDetour();
        checkOtherBranchIsNotReachable();
        checkSearchDiagnostics();
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
