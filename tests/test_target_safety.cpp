#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace robotics::kinematics;
using robotics::math::Vec2;

int main() {
    const auto require = [](bool value, const char* label) {
        if (!value) throw std::runtime_error(label);
    };
    try {
        require(!targetApproachClear(
            std::vector<Vec2>{{0, 0}, {2, 0}, {2, 2}}, {1, 0}, 0.16),
            "Do not let the first link hit the target");
        require(!targetApproachClear(
            std::vector<Vec2>{{0, 0}, {0, 2}, {2, 2}}, {1, 2}, 0.16),
            "Do not let the last-link body hit the target far from the tip");
        require(targetApproachClear(
            std::vector<Vec2>{{0, 0}, {0, 2}, {2, 2}}, {2, 2}, 0.16),
            "Allow end-effector arrival");
        require(targetApproachClear(
            std::vector<Vec2>{{0, 0}, {0, 2}, {2, 2}}, {-1, 2}, 0.16),
            "Allow unobstructed poses");

        const double pi = std::numbers::pi;
        std::vector<Joint2D> arm{{1.25, 0.3, 0}, {0.75, -0.7, 0}, {1.1, 0.8, 0}};
        std::vector<JointLimits2D> limits{
            {-2 * pi, 2 * pi, 2, 6}, {-pi + 0.16, -0.16, 2, 6},
            {0.16, pi - 0.16, 2, 6}
        };
        MotionSettings2D options;
        options.foldClearance = 0.55;
        options.targetClearance = 0.16;
        const Vec2 goal{1.6, 1.1};
        const auto plan = planJointPath2D(arm, limits, goal, options);
        require(plan.found(), "Find a path with stricter safety constraints");

        for (std::size_t k = 1; k < plan.waypoints.size(); ++k) {
            for (int sub = 0; sub <= 80; ++sub) {
                auto pose = arm;
                const double alpha = static_cast<double>(sub) / 80.0;
                for (std::size_t i = 0; i < pose.size(); ++i) {
                    pose[i].angle = (1 - alpha) * plan.waypoints[k - 1][i] +
                                    alpha * plan.waypoints[k][i];
                    if (i > 0) {
                        require(std::abs(pose[i].angle) >= options.jointClearance - 1e-8,
                                "Do not straighten an internal joint");
                        require(std::abs(pose[i].angle) <= pi - options.foldClearance + 1e-8,
                                "Do not fold an internal joint too tightly");
                    }
                }
                const auto positions = forwardKinematics(pose);
                require(targetApproachClear(positions, goal, options.targetClearance),
                        "Every segment of the path must keep target clearance");
                require(!hasSelfCollision(positions, options.linkClearance),
                        "Every segment of the path must remain self-collision-free");
            }
        }

        // The actual acceleration-limited follower must be safe as well;
        // collision-free planner waypoints alone are not sufficient.
        auto moving = arm;
        PlannedMotion2D controller;
        controller.setTarget(moving, limits, goal, options);
        require(controller.pathAvailable(), "Follower has a planned route");
        bool reached = false;
        for (int frame = 0; frame < 1200; ++frame) {
            const auto status = controller.advance(moving, limits, 1.0 / 60.0, options);
            const auto state = forwardKinematics(moving);
            require(!status.blocked, "Follower must not cross a prohibited region");
            require(targetApproachClear(state, goal, options.targetClearance),
                    "Follower must keep target clearance every frame");
            require(!hasSelfCollision(state, options.linkClearance),
                    "Follower must stay self-collision-free");
            if (status.reached) { reached = true; break; }
        }
        require(reached, "Follower reaches goal without touching intermediate links");

        const auto positions = forwardKinematics(arm);
        const Vec2 linkTarget = positions[0] + (positions[1] - positions[0]) * 0.5;
        const auto unsafe = planJointPath2D(arm, limits, linkTarget, options);
        require(!unsafe.found(), "Target placed on a link should fail safely");

        auto folded = arm;
        folded[1].angle = -pi + 0.2;
        bool rejected = false;
        try {
            (void)planJointPath2D(folded, limits, goal, options);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Reject unsafe folded starting configurations");
        std::cout << "Target clearance and fold safety passed\n";
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
