#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace robotics::kinematics;
using robotics::math::Vec2;

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void verifyPath(std::span<const Joint2D> initial,
                const PlannedJointPath2D& path,
                Vec2 target, const MotionSettings2D& settings) {
    for (std::size_t k = 1; k < path.waypoints.size(); ++k) {
        for (int step = 0; step <= 120; ++step) {
            auto pose = std::vector<Joint2D>(initial.begin(), initial.end());
            const double t = static_cast<double>(step) / 120.0;
            for (std::size_t j = 0; j < pose.size(); ++j) {
                pose[j].angle = (1.0 - t) * path.waypoints[k - 1][j] +
                                t * path.waypoints[k][j];
            }
            const auto positions = forwardKinematics(pose);
            require(targetApproachClear(positions, target, settings.targetClearance),
                    "Planned trajectory passes through the target");
            require(!hasSelfCollision(positions, settings.linkClearance),
                    "Planned trajectory self-collides");
            require(pose[1].angle < -settings.jointClearance &&
                    pose[1].angle > -std::numbers::pi + settings.foldClearance,
                    "Planned trajectory violates elbow branch or folding margin");
        }
    }
}

} // namespace

int main() {
    try {
        const double pi = std::numbers::pi;
        MotionSettings2D settings;
        settings.foldClearance = 0.55;
        settings.targetClearance = 0.16;

        const double l1 = 4.0;
        const double l2 = 2.0;
        // Same geometry as the playtest screenshot: initial end effector (4,0),
        // then click (2,3). The initial elbow is bent clockwise.
        const double startElbow = -std::acos(-0.25);
        const double startShoulder = -std::atan2(l2 * std::sin(startElbow),
                                                  l1 + l2 * std::cos(startElbow));
        std::vector<Joint2D> joints{{l1, startShoulder, 0.0},
                                    {l2, startElbow, 0.0}};
        const std::vector<JointLimits2D> limits{
            {-2.0 * pi, 2.0 * pi, 2.0, 6.0},
            {-pi + 0.56, -0.16, 2.0, 6.0}
        };
        const Vec2 target{2.0, 3.0};
        require((forwardKinematics(joints).back() - Vec2{4.0, 0.0}).length() < 1e-10,
                "Initial robot is not at (4,0)");

        const double cosine = (target.dot(target) - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
        const double endElbow = -std::acos(cosine);
        const double shortShoulder = std::atan2(target.y, target.x) -
            std::atan2(l2 * std::sin(endElbow), l1 + l2 * std::cos(endElbow));
        const double longShoulder = shortShoulder - 2.0 * pi;

        // The shortest joint-space interpolation crosses the object,
        // but the equivalent shoulder orientation via -2*pi is safe.
        bool shortUnsafe = false;
        for (int step = 1; step <= 400; ++step) {
            auto pose = joints;
            const double t = static_cast<double>(step) / 400.0;
            pose[0].angle = startShoulder + t * (shortShoulder - startShoulder);
            pose[1].angle = startElbow + t * (endElbow - startElbow);
            if (!targetApproachClear(forwardKinematics(pose), target,
                                     settings.targetClearance)) shortUnsafe = true;
        }
        require(shortUnsafe, "Regression must demonstrate a blocked short route");

        for (int step = 0; step <= 400; ++step) {
            auto pose = joints;
            const double t = static_cast<double>(step) / 400.0;
            pose[0].angle = startShoulder + t * (longShoulder - startShoulder);
            pose[1].angle = startElbow + t * (endElbow - startElbow);
            require(targetApproachClear(forwardKinematics(pose), target,
                                        settings.targetClearance),
                    "Long shoulder rotation must avoid the object");
        }

        const auto plan = planJointPath2D(joints, limits, target, settings);
        require(plan.found(), "Planner failed to find the safe two-link detour");
        require(plan.terminalError <= settings.positionTolerance,
                "Detour does not reach the requested end-effector position");
        require(plan.waypoints.back().at(0) < -3.0,
                "Planner selected a colliding short shoulder rotation");
        verifyPath(joints, plan, target, settings);

        PlannedMotion2D follower;
        follower.setTarget(joints, limits, target, settings);
        require(follower.pathAvailable(), "Follower failed to plan the detour");
        bool reached = false;
        constexpr double dt = 1.0 / 60.0;
        for (int frame = 0; frame < 1800; ++frame) {
            const auto before = joints;
            const auto status = follower.advance(joints, limits, dt, settings);
            require(!status.blocked, "Safe detour was blocked during execution");
            const auto positions = forwardKinematics(joints);
            require(targetApproachClear(positions, target, settings.targetClearance),
                    "Follower touched target with a non-tool arm segment");
            for (std::size_t i = 0; i < joints.size(); ++i) {
                require(joints[i].angle >= limits[i].minAngle - 1e-8 &&
                        joints[i].angle <= limits[i].maxAngle + 1e-8,
                        "Follower violated a joint limit");
                require(std::abs(joints[i].angularVelocity) <=
                        limits[i].maxAngularSpeed + 1e-8,
                        "Follower exceeded maximum angular velocity");
                require(std::abs(joints[i].angularVelocity - before[i].angularVelocity) <=
                        limits[i].maxAngularAcceleration * dt + 1e-8,
                        "Follower exceeded maximum angular acceleration");
            }
            if (status.reached) { reached = true; break; }
        }
        require(reached, "Safe detour follower failed to reach (2,3)");
        std::cout << "Two-joint (4,0) -> (2,3) safe-detour test passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
