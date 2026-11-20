#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::MotionSettings2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::hasSelfCollision;
using robotics::kinematics::planJointPath2D;
using robotics::kinematics::targetApproachClear;
using robotics::kinematics::targetLinkPenetrations;
using robotics::math::Vec2;

namespace {

void require(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}

std::vector<Joint2D> startArm() {
    return {{1.25, 0.3, 0.0}, {0.75, -0.7, 0.0}, {1.1, 0.8, 0.0}};
}

void verifyNoDeeperContact(const std::vector<double>& before,
                           const std::vector<double>& after) {
    require(before.size() == after.size(), "Contact profile size changed");
    for (std::size_t i = 0; i < before.size(); ++i)
        require(after[i] <= before[i] + 1e-8,
                "An arm link made new or deeper contact with the target");
}

void testRecovery(const std::vector<double>& goalAngles,
                  bool startingInContact, bool expectUnsafeStraightLine) {
    auto initial = startArm();
    const std::vector<JointLimits2D> limits{
        {-2.0 * std::numbers::pi, 2.0 * std::numbers::pi, 2.0, 6.0},
        {-std::numbers::pi + 0.16, -0.16, 2.0, 6.0},
        {0.16, std::numbers::pi - 0.16, 2.0, 6.0}
    };
    MotionSettings2D safety{};
    safety.foldClearance = 0.55;
    safety.targetClearance = 0.16;

    auto desiredPose = initial;
    for (std::size_t i = 0; i < desiredPose.size(); ++i)
        desiredPose[i].angle = goalAngles[i];
    const Vec2 target = forwardKinematics(desiredPose).back();
    require(targetApproachClear(forwardKinematics(desiredPose),
                                target, safety.targetClearance),
            "Regression end pose must be target-clear");
    require(!hasSelfCollision(forwardKinematics(desiredPose), safety.linkClearance),
            "Regression end pose must be collision-free");

    const auto initialDepth = targetLinkPenetrations(
        forwardKinematics(initial), target, safety.targetClearance);
    const bool alreadyTouching = std::any_of(initialDepth.begin(), initialDepth.end(),
        [](double depth) { return depth > 1e-8; });
    require(alreadyTouching == startingInContact,
            "Regression setup must have expected initial contact");

    bool straightLineUnsafe = false;
    for (int k = 1; k <= 100; ++k) {
        auto pose = initial;
        const double alpha = static_cast<double>(k) / 100.0;
        for (std::size_t i = 0; i < pose.size(); ++i)
            pose[i].angle += alpha * (goalAngles[i] - pose[i].angle);
        if (!targetApproachClear(forwardKinematics(pose), target,
                                 safety.targetClearance)) {
            straightLineUnsafe = true;
            break;
        }
    }
    if (expectUnsafeStraightLine)
        require(straightLineUnsafe, "Regression must challenge the straight route");

    const auto path = planJointPath2D(initial, limits, target, safety);
    require(path.found(), "Planner must find a valid reconfiguration");
    require(path.terminalError <= safety.positionTolerance,
            "Planned end pose does not reach the target");

    // Geometric path: new contact is forbidden, but an existing overlap
    // is allowed to diminish as the arm safely retreats from a newly set target.
    auto previous = initialDepth;
    for (std::size_t k = 1; k < path.waypoints.size(); ++k) {
        for (int step = 1; step <= 80; ++step) {
            auto pose = initial;
            const double alpha = static_cast<double>(step) / 80.0;
            for (std::size_t i = 0; i < pose.size(); ++i)
                pose[i].angle = (1.0 - alpha) * path.waypoints[k - 1][i] +
                                alpha * path.waypoints[k][i];
            const auto positions = forwardKinematics(pose);
            verifyNoDeeperContact(previous, targetLinkPenetrations(
                positions, target, safety.targetClearance));
            previous = targetLinkPenetrations(
                positions, target, safety.targetClearance);
            require(!hasSelfCollision(positions, safety.linkClearance),
                    "Planned reconfiguration self-collided");
            for (std::size_t i = 1; i < pose.size(); ++i)
                require(std::abs(pose[i].angle) <=
                        std::numbers::pi - safety.foldClearance + 1e-8,
                        "Planned motion folds an elbow too far");
        }
    }
    require(targetApproachClear(forwardKinematics(desiredPose),
                                target, safety.targetClearance),
            "End pose should be clear");

    // The acceleration-limited follower can deviate from straight waypoint
    // edges. Check the actual simulated movement, not just the plan.
    PlannedMotion2D controller;
    controller.setTarget(initial, limits, target, safety);
    require(controller.pathAvailable(), "Follower has no recovery path");
    previous = initialDepth;
    bool reached = false;
    for (int frame = 0; frame < 1800; ++frame) {
        const auto before = initial;
        const auto result = controller.advance(initial, limits, 1.0 / 60.0, safety);
        require(!result.blocked, "Follower stopped on a recoverable target");
        const auto positions = forwardKinematics(initial);
        const auto now = targetLinkPenetrations(
            positions, target, safety.targetClearance);
        verifyNoDeeperContact(previous, now);
        previous = now;
        require(!hasSelfCollision(positions, safety.linkClearance),
                "Follower self-collided");
        for (std::size_t i = 0; i < initial.size(); ++i) {
            require(std::abs(initial[i].angularVelocity) <=
                    limits[i].maxAngularSpeed + 1e-8,
                    "Follower exceeded a joint speed limit");
            require(std::abs(initial[i].angularVelocity -
                             before[i].angularVelocity) <=
                    limits[i].maxAngularAcceleration / 60.0 + 1e-8,
                    "Follower exceeded a joint acceleration limit");
        }
        if (result.reached) { reached = true; break; }
    }
    require(reached, "Follower did not complete a recoverable move");
    require(targetApproachClear(forwardKinematics(initial),
                                target, safety.targetClearance),
            "Follower left the target in contact with an arm link");
}

} // namespace

int main() {
    try {
        // The direct interpolation passes through the target, but a
        // collision-free detour exists.
        testRecovery({-0.8938201259, -0.3751571800, 1.5485608078}, false, true);

        // The new target is already under the current second/third link.
        // The first movement must reduce, rather than increase, contact.
        testRecovery({0.6976318399, -2.4245492828, 1.6957925523}, true, true);

        std::cout << "Target detour and safe retreat tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
