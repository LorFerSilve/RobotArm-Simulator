#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

namespace {

using namespace robotics::kinematics;
using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::geometry::hasObstacleCollision;
using robotics::math::Vec2;

constexpr double pi = std::numbers::pi;
constexpr double dt = 1.0 / 60.0;
constexpr std::array<Joint2D, 2> start{{{2.0, 0.0}, {1.0, -pi / 2.0}}};
constexpr std::array<JointLimits2D, 2> limits{{
    {-pi, pi, 2.0, 6.0}, {-pi + 0.16, -0.16, 2.0, 6.0}
}};

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

Vec2 goalAt(double shoulder) {
    auto pose = start;
    pose.front().angle = shoulder;
    return forwardKinematics(pose).back();
}

PlannedJointPath2D polylineTo(double shoulder) {
    PlannedJointPath2D plan;
    plan.waypoints = {{0.0, -pi / 2.0}, {shoulder, -pi / 2.0}};
    return plan;
}

void checkRetarget(bool useSpline) {
    MotionSettings2D settings;
    settings.maxAngularJerk = 12.0;
    auto arm = start;
    const Vec2 first = goalAt(1.1);
    const Vec2 latest = goalAt(-0.55);
    const auto initialPlan = useSpline ? planJointPath2D(arm, limits, first, settings) :
                                         polylineTo(1.1);
    require(initialPlan.found() && (!useSpline || initialPlan.trajectoryDuration > 0.0),
            "The fixture requires a valid trajectory of the requested type");
    PlannedMotion2D follower;
    follower.usePlan(initialPlan, first);
    std::array<double, 2> previousAcceleration{};
    const auto step = [&] {
        const auto before = arm;
        const auto status = follower.advance(arm, limits, dt, settings);
        require(!status.blocked, "A clear retarget must not cause an emergency stop");
        for (std::size_t j = 0; j < arm.size(); ++j) {
            const double acceleration = (arm[j].angularVelocity - before[j].angularVelocity) / dt;
            require(std::abs(arm[j].angularVelocity) <= limits[j].maxAngularSpeed + 1e-8 &&
                    std::abs(acceleration) <= limits[j].maxAngularAcceleration + 1e-8,
                    "The retarget must preserve the velocity and acceleration limits");
            require(std::abs(acceleration - previousAcceleration[j]) <= settings.maxAngularJerk * dt + 1e-7,
                    "Retargeting must continue velocity and acceleration under the jerk limit");
            previousAcceleration[j] = acceleration;
        }
        return status;
    };
    for (int frame = 0; frame < 240 && arm[0].angularVelocity < 0.15; ++frame)
        (void)step();
    require(arm[0].angularVelocity >= 0.15 && previousAcceleration[0] > 0.1,
            "Retarget must begin with nonzero velocity and acceleration");
    const auto before = arm;
    follower.setTarget(arm, limits, goalAt(-0.25), settings);
    require(follower.stopState() == MotionStop2D::Braking &&
            arm[0].angle == before[0].angle && arm[0].angularVelocity == before[0].angularVelocity,
            "Requesting a new target must preserve the current position and rate");
    (void)step();
    require(arm[0].angle > before[0].angle && arm[0].angularVelocity > 0.0,
            "Braking must start from the active motion, including its direction");
    follower.setTarget(arm, limits, latest, settings);

    bool stopped = false;
    for (int frame = 0; frame < 1800; ++frame) {
        const auto status = step();
        if (follower.stopState() == MotionStop2D::Controlled) {
            require(arm[0].angularVelocity == 0.0 && arm[1].angularVelocity == 0.0,
                    "The replacement plan must start from a controlled rest");
            require((forwardKinematics(arm).back() - first).length() > settings.positionTolerance,
                    "The controlled stop should brake before the abandoned goal");
            stopped = true;
        }
        if (status.reached) {
            require(stopped && (forwardKinematics(arm).back() - latest).length() <=
                                settings.positionTolerance,
                    "Only the latest requested goal should be executed after stopping");
            return;
        }
    }
    throw std::runtime_error("The retarget did not complete");
}

void checkEmergencyStop() {
    MotionSettings2D settings;
    settings.obstacleClearance = 0.0;
    auto arm = start;
    PlannedMotion2D follower;
    follower.usePlan(polylineTo(1.1), goalAt(1.1));
    for (int frame = 0; frame < 40; ++frame)
        require(!follower.advance(arm, limits, dt, settings).blocked,
                "The obstacle fixture must first accelerate");
    require(arm[0].angularVelocity > 0.1, "The fixture must be in motion");

    const auto before = arm;
    auto manual = follower;
    auto manualArm = arm;
    manual.emergencyStop(manualArm);
    require(manual.stopState() == MotionStop2D::Emergency &&
            manualArm[0].angle == before[0].angle && manualArm[0].angularVelocity == 0.0,
            "A manual emergency stop must hold the current pose immediately");
    require(manual.advance(manualArm, limits, dt, settings).blocked &&
            manualArm[0].angle == before[0].angle, "The emergency stop must stay latched");

    follower.requestStop();
    auto clearFollower = follower;
    auto next = arm;
    require(!clearFollower.advance(next, limits, dt, settings).blocked &&
            next[0].angle > arm[0].angle, "The unblocked controlled stop should advance");
    const double angle = (arm[0].angle + next[0].angle) * 0.5;
    const std::array<Obstacle2D, 1> scene{
        Circle2D{{std::cos(angle), std::sin(angle)}, (next[0].angle - arm[0].angle) / 8.0}
    };
    require(!hasObstacleCollision(forwardKinematics(arm), scene, 0.0),
            "The emergency obstacle must leave the current pose clear");
    require(follower.advance(arm, limits, dt, settings, scene).blocked &&
            follower.stopState() == MotionStop2D::Emergency &&
            follower.failureReason() == PlanningFailure2D::UnsafeMotionStep &&
            arm[0].angle == before[0].angle && arm[0].angularVelocity == 0.0,
            "An unsafe braking step must emergency-stop before moving");

    for (int frame = 0; frame < 300 && clearFollower.stopState() == MotionStop2D::Braking; ++frame)
        require(!clearFollower.advance(next, limits, dt, settings).blocked,
                "A clear manual stop must finish without an emergency");
    require(clearFollower.stopState() == MotionStop2D::Controlled &&
            next[0].angularVelocity == 0.0, "A manual controlled stop must end at rest");
    const double stoppedAngle = next[0].angle;
    require(!clearFollower.advance(next, limits, dt, settings).blocked &&
            next[0].angle == stoppedAngle, "The canceled route must stay stopped");
}

void checkCurvedSweep() {
    auto arm = start;
    arm[0].angle = -0.05;
    constexpr double elbow = -pi / 2.0;
    PlannedJointPath2D plan;
    plan.waypoints = {{-0.05, elbow}, {0.0, elbow}, {0.05, elbow}};
    plan.splineTangents = {{0.0, 0.0}, {-0.8, 0.0}, {0.0, 0.0}};
    plan.pathDistances = {0.0, 1.0, 2.0};
    plan.trajectoryDuration = 0.1;
    MotionSettings2D settings;
    settings.obstacleClearance = 0.0;
    settings.maxAngularSpeed = 100.0;
    settings.maxAngularAcceleration = 2000.0;
    const std::array<JointLimits2D, 2> wideLimits{{
        {-pi, pi, 100.0, 2000.0}, {-3.0, -0.15, 100.0, 2000.0}
    }};
    const std::array<Obstacle2D, 1> scene{Circle2D{{std::cos(0.1), std::sin(0.1)}, 0.02}};
    auto midpoint = arm;
    midpoint[0].angle = 0.1; // value at u = 0.5 of the first quintic segment
    require(sweptArmClear2D(arm, plan.waypoints[0], plan.waypoints[1], scene, 0.0, 0.0) &&
            hasObstacleCollision(forwardKinematics(midpoint), scene, 0.0),
            "The straight chord must miss the obstacle while the true arc hits it");

    PlannedMotion2D follower;
    auto clearArm = arm;
    follower.usePlan(plan, goalAt(0.05));
    require(!follower.advance(clearArm, wideLimits, 0.05, settings).blocked &&
            clearArm[0].angle == 0.0, "The rounded path must execute in a clear scene");
    follower.usePlan(plan, goalAt(0.05));
    require(follower.advance(arm, wideLimits, 0.05, settings, scene).blocked &&
            follower.stopState() == MotionStop2D::Emergency &&
            follower.failureReason() == PlanningFailure2D::UnsafeMotionStep &&
            arm[0].angle == -0.05, "A curve between clear frame poses must be checked before moving");
}

} // namespace

int main() {
    try {
        checkRetarget(false);
        checkRetarget(true);
        checkEmergencyStop();
        checkCurvedSweep();
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
