#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <array>
#include <iostream>
#include <numbers>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using namespace robotics::kinematics;
using robotics::geometry::Box2D;
using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::geometry::hasObstacleCollision;
using robotics::geometry::segmentHitsObstacle;
using robotics::math::Vec2;

constexpr double pi = std::numbers::pi;
constexpr double sceneMargin = 1.0 / 32.0;
constexpr double shoulderRadius = 1.0 / 16.0;
constexpr double elbow = -2.125;
constexpr double grazingCenterX = 2.0 + shoulderRadius + 2.0 * sceneMargin;
constexpr double safeGap = 0.00025;
constexpr std::array<JointLimits2D, 2> limits{{{-0.1, 0.1, 2, 6}, {-2.3, -1.7, 2, 6}}};

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::array<Joint2D, 2> twoLinkArm(double shoulder, double secondJoint) {
    return {{{2.0, shoulder, 0.0, shoulderRadius}, {1.0, secondJoint, 0.0, sceneMargin}}};
}

void checkSceneGrazing() {
    // Only at shoulder angle zero does the first link reach x = 2.
    // That pose occurs at t = 1/3, away from every binary subdivision midpoint.
    constexpr std::array start{-1.0 / 32.0, elbow}, end{1.0 / 16.0, elbow};
    constexpr std::array closest{0.0, elbow};
    const auto arm = twoLinkArm(start[0], start[1]);
    const auto radii = linkRadii2D(arm);
    std::vector<Vec2> positions;
    for (const double gap : {safeGap, 1e-13, 0.0, -safeGap}) {
        const Vec2 center{grazingCenterX + gap, 0.0};
        for (const Obstacle2D& obstacle : std::array<Obstacle2D, 2>{
                 Circle2D{center, sceneMargin}, Box2D{center, sceneMargin, 0.125}}) {
            const std::span scene(&obstacle, 1);
            for (const auto& q : {start, end}) {
                forwardKinematics(arm, q, positions);
                require(!hasObstacleCollision(positions, scene, sceneMargin, radii),
                        "Both endpoints of the grazing edge must be clear");
            }
            forwardKinematics(arm, closest, positions);
            require(hasObstacleCollision(positions, scene, sceneMargin, radii) == (gap <= 0.0),
                    "The closest pose must distinguish positive gaps from contact");
            require(sweptArmClear2D(arm, start, end, scene, 0.0, sceneMargin) == (gap == safeGap),
                    "Only the grazing edge with provable clearance may certify");
        }
    }
}

void checkSelfGrazing() {
    // The third link can approach the horizontal first link no closer than
    // 0.25 m: its pivot is 1 m high and the link is 0.75 m long.
    const std::array<Joint2D, 3> arm{{
        {2.0, 0.0, 0.0, 0.125}, {1.0, pi / 2.0}, {0.75, pi, 0.0, 0.0625}
    }};
    const std::array start{0.0, pi / 2.0, pi - 1.0 / 32.0};
    const std::array end{0.0, pi / 2.0, pi + 1.0 / 16.0};
    const auto radii = linkRadii2D(arm);
    std::vector<Vec2> positions;
    for (const double gap : {safeGap, 0.0, -safeGap}) {
        const double clearance = 0.0625 - gap;
        for (const auto& q : {start, end}) {
            forwardKinematics(arm, q, positions);
            require(!hasSelfCollision(positions, clearance, radii),
                    "Both endpoints of the self-grazing edge must be clear");
        }
        require(hasSelfCollision(forwardKinematics(arm), clearance, radii) == (gap <= 0.0),
                "Nonadjacent capsules must count tangency as self-contact");
        require(sweptArmClear2D(arm, start, end, {}, clearance, 0.0) == (gap > 0.0),
                "Swept self-clearance must distinguish a near miss from intermediate contact");
    }
}

void checkTargetShaftGrazing() {
    const std::array<Joint2D, 1> arm{{{2.0, 0.0, 0.0, shoulderRadius}}};
    constexpr std::array start{-1.0 / 32.0}, end{1.0 / 16.0};
    const auto radii = linkRadii2D(arm);
    const auto closest = forwardKinematics(arm);
    std::vector<Vec2> positions;
    TargetObject2D object;
    object.radius = object.halfWidth = sceneMargin;
    object.halfHeight = 0.125;
    object.clearance = sceneMargin;
    object.toolLength = 0.25;
    for (const auto shape : {TargetShape2D::Circle, TargetShape2D::Rectangle}) {
        object.shape = shape;
        for (const double gap : {safeGap, 0.0, -safeGap}) {
            const Vec2 target{grazingCenterX - object.toolLength + gap, 0.0};
            for (const auto& q : {start, end}) {
                forwardKinematics(arm, q, positions);
                require(targetApproachClear(positions, target, object, radii),
                        "Both endpoints must keep the protected shaft clear");
            }
            require(segmentHitsObstacle(closest.front(), closest.back(),
                                        targetBody2D(object, target)),
                    "The tool portion must pass through the selected target");
            require(targetApproachClear(closest, target, object, radii) == (gap > 0.0),
                    "Only the tool may touch either target shape");
            require(sweptArmClear2D(arm, start, end, {}, 0.0, 0.0, target, object) == (gap > 0.0),
                    "A target must allow tool-only motion and reject intermediate shaft contact");
        }
    }
}

void checkNarrowPassage() {
    auto arm = twoLinkArm(-0.0005, -2.0);
    const Vec2 target = forwardKinematics(twoLinkArm(0.0005, -1.85)).back();
    const auto radii = linkRadii2D(arm);
    const auto passage = [](double spareWidthPerSide) {
        const double wallOffset = 0.125 + shoulderRadius + sceneMargin + spareWidthPerSide;
        return std::array<Obstacle2D, 2>{
            Circle2D{{1.125, wallOffset}, 0.125},
            Box2D{{1.0, -wallOffset}, 0.25, 0.125}
        };
    };

    constexpr std::array centeredStart{0.0, -2.0}, centeredEnd{0.0, -1.85};
    const auto centered = twoLinkArm(centeredStart[0], centeredStart[1]);
    const auto closed = passage(0.0);
    require(!sweptArmClear2D(centered, centeredStart, centeredEnd, closed, 0.0, sceneMargin),
            "A passage with exactly enough width for contact must reject motion");

    // At x = 1.125 the circle and the box leave 2 mm of spare width total
    // for the first link, after accounting for its diameter and both margins.
    const auto scene = passage(0.001);
    MotionSettings2D settings;
    settings.obstacleClearance = sceneMargin;
    auto plan = planJointPath2D(arm, limits, target, settings, scene);
    require(plan.found(), "The planner must find the open narrow passage");
    PlannedMotion2D follower;
    follower.usePlan(std::move(plan), target);
    for (int frame = 0; frame < 1000; ++frame) {
        const auto status = follower.advance(arm, limits, 1.0 / 60.0, settings, scene);
        require(!status.blocked, "The follower must not block in the open narrow passage");
        require(!hasObstacleCollision(forwardKinematics(arm), scene, sceneMargin, radii),
                "The arm must stay clear of both sides of the passage");
        if (status.reached) return;
    }
    throw std::runtime_error("The follower must reach its goal through the narrow passage");
}

void checkBlockedRoute() {
    const auto arm = twoLinkArm(-1.0 / 32.0, elbow);
    const Vec2 target = forwardKinematics(twoLinkArm(1.0 / 16.0, elbow)).back();
    const std::array<Obstacle2D, 1> closed{Circle2D{{grazingCenterX, 0.0}, sceneMargin}};
    const std::array<Obstacle2D, 1> open{Circle2D{{grazingCenterX + safeGap, 0.0}, sceneMargin}};
    MotionSettings2D settings;
    settings.obstacleClearance = sceneMargin;
    require(planJointPath2D(arm, limits, target, settings, open).found(),
            "The planner must accept the same route when there is enough clearance");
    // The shoulder limits prevent wrapping around, so any path to this pose
    // must pass through angle zero where the shoulder capsule touches.
    const auto blocked = planJointPath2D(arm, limits, target, settings, closed);
    require(!blocked.found() && blocked.failure == PlanningFailure2D::PathSearchExhausted,
            "The planner must reject the intermediate contact between two clear poses");

    auto pose = twoLinkArm(-1.0 / 2048.0, elbow);
    const auto start = pose;
    const auto endpoint = twoLinkArm(3.0 / 1024.0, elbow);
    const Vec2 shortTarget = forwardKinematics(endpoint).back();
    PlannedJointPath2D shortPlan;
    shortPlan.waypoints = {{pose[0].angle, elbow}, {endpoint[0].angle, elbow}};
    PlannedMotion2D follower;
    follower.usePlan(shortPlan, shortTarget);
    const auto clearStep = follower.advance(pose, limits, 0.08, settings, open);
    require(!clearStep.blocked && pose[0].angle > 0.0 &&
                !hasObstacleCollision(forwardKinematics(pose), closed, sceneMargin, linkRadii2D(pose)),
            "The control step crosses angle zero and ends clear even in the closed scene");

    pose = start;
    follower.usePlan(std::move(shortPlan), shortTarget);
    const auto unsafeStep = follower.advance(pose, limits, 0.08, settings, closed);
    require(unsafeStep.blocked && follower.failureReason() == PlanningFailure2D::UnsafeMotionStep &&
                pose[0].angle == start[0].angle && pose[1].angle == start[1].angle,
            "The follower must reject a grazing collision within a single step");
}

} // namespace

int main() {
    try {
        checkSceneGrazing();
        checkSelfGrazing();
        checkTargetShaftGrazing();
        checkNarrowPassage();
        checkBlockedRoute();
        std::cout << "2D grazing contact and narrow passage regressions passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
