#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

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
using robotics::math::Vec2;

namespace {

void require(bool yes, const char* why) {
    if (!yes) throw std::runtime_error(why);
}

} // namespace

int main() {
    try {
        constexpr double pi = std::numbers::pi;
        constexpr double degrees = pi / 180.0;
        constexpr double dt = 1.0 / 60.0;

        // Five-link preset based on the reported playtest: tip initially
        // near (4, -9), with each link approximately two metres long.
        std::vector<Joint2D> joints{
            {2.0, -57.0 * degrees, 0.0},
            {2.0, -21.0 * degrees, 0.0},
            {2.0, 26.0 * degrees, 0.0},
            {2.0, -35.0 * degrees, 0.0},
            {2.0, 32.0 * degrees, 0.0}
        };
        const std::vector<JointLimits2D> limits{
            {-2.0 * pi, 2.0 * pi, 2.0, 6.0},
            {-pi + 0.56, -0.16, 2.0, 6.0},
            {0.16, pi - 0.56, 2.0, 6.0},
            {-pi + 0.56, -0.16, 2.0, 6.0},
            {0.16, pi - 0.56, 2.0, 6.0}
        };
        MotionSettings2D settings{};
        settings.foldClearance = 0.55;
        settings.targetClearance = 0.16;

        const auto initialTip = forwardKinematics(joints).back();
        require((initialTip - Vec2{4.0, -9.0}).length() < 0.2,
                "Regression preset is not near (4,-9)");

        const Vec2 target{-3.0, -4.0};
        const auto plan = planJointPath2D(joints, limits, target, settings);
        require(plan.found(), "Unable to plan the five-link recovery");
        require(plan.terminalError <= settings.positionTolerance,
                "Five-link planned end pose does not reach target");

        // Verify the approved joint-space line segments.
        for (std::size_t k = 1; k < plan.waypoints.size(); ++k) {
            for (int sample = 0; sample <= 120; ++sample) {
                const double u = static_cast<double>(sample) / 120.0;
                auto pose = joints;
                for (std::size_t i = 0; i < pose.size(); ++i)
                    pose[i].angle = (1.0 - u) * plan.waypoints[k - 1][i] +
                                     u * plan.waypoints[k][i];
                const auto positions = forwardKinematics(pose);
                require(!hasSelfCollision(positions, settings.linkClearance),
                        "Planned five-link route has a self collision");
                require(targetApproachClear(positions, target, settings.targetClearance),
                        "Planned five-link route passes through target");
            }
        }

        PlannedMotion2D motion;
        motion.setTarget(joints, limits, target, settings);
        require(motion.pathAvailable(), "Follower did not receive the planned path");
        bool reached = false;
        for (int frame = 0; frame < 4000; ++frame) {
            const auto previous = joints;
            const auto outcome = motion.advance(joints, limits, dt, settings);
            require(!outcome.blocked, "Five-link path follower became blocked");
            const auto positions = forwardKinematics(joints);
            require(!hasSelfCollision(positions, settings.linkClearance),
                    "Five-link follower caused self collision");
            require(targetApproachClear(positions, target, settings.targetClearance),
                    "Five-link follower hit the target");
            for (std::size_t i = 0; i < joints.size(); ++i) {
                require(joints[i].angle >= limits[i].minAngle - 1e-8 &&
                        joints[i].angle <= limits[i].maxAngle + 1e-8,
                        "Five-link follower crossed a joint limit");
                require(std::abs(joints[i].angularVelocity) <=
                        limits[i].maxAngularSpeed + 1e-8,
                        "Five-link follower exceeded speed limit");
                require(std::abs(joints[i].angularVelocity -
                                 previous[i].angularVelocity) <=
                        limits[i].maxAngularAcceleration * dt + 1e-8,
                        "Five-link follower exceeded acceleration limit");
            }
            if (outcome.reached) { reached = true; break; }
        }
        require(reached, "Five-link robot did not arrive at (-3,-4)");
        require((forwardKinematics(joints).back() - target).length() <=
                settings.positionTolerance, "Five-link final tip position incorrect");

        std::cout << "Five-link coordinated path following passed\n";
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
