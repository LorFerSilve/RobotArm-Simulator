#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"

#include <algorithm>
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
        // Regression: the old implementation exempted the ENTIRE last
        // segment as soon as the tip was within 2*clearance of the target.
        // That let the shaft cut across the object before the tip arrived.
        require(!targetApproachClear(
            std::vector<Vec2>{{0, 0}, {2, 0}, {4, 0}}, {3.7, 0}, 0.16),
            "Last-link shaft must stay out of target near the tip");
        require(targetApproachClear(
            std::vector<Vec2>{{0, 0}, {2, 0}, {4, 0}}, {3.95, 0}, 0.16),
            "A short tool tip may approach the target");
        const auto closeShaft = targetLinkPenetrations(
            std::vector<Vec2>{{0, 0}, {2, 0}, {4, 0}}, {3.7, 0}, 0.16);
        require(closeShaft.back() > 0.0,
                "Target penetration must report last-link shaft contact");

        // The allocation-free query, reusable penetration buffer and legacy
        // vector-returning query must agree, including last-link tool contact.
        const std::vector<Vec2> probe{{0, 0}, {2, 0}, {4, 0}};
        std::vector<double> reusableDepths(2, -1.0);
        const auto* depthStorage = reusableDepths.data();
        for (const auto& target : std::vector<Vec2>{
                {1.0, 0.0}, {3.7, 0.0}, {3.95, 0.0},
                {4.0, 0.0}, {5.0, 5.0}}) {
            for (double clearance : {0.0, 0.16, 0.3}) {
                const auto legacy =
                    targetLinkPenetrations(probe, target, clearance);
                targetLinkPenetrations(probe, target, clearance, reusableDepths);
                require(reusableDepths.data() == depthStorage,
                        "Depth query reuses caller-owned storage");
                require(reusableDepths == legacy,
                        "Buffered and vector target depths agree");
                const bool allClear = std::all_of(
                    legacy.begin(), legacy.end(),
                    [](double depth) { return depth == 0.0; });
                require(targetApproachClear(probe, target, clearance) == allClear,
                        "Allocation-free target test agrees with depths");
            }
        }
        bool rejectedShortDepths = false;
        try {
            std::vector<double> tooShort(1);
            targetLinkPenetrations(probe, {3.7, 0}, 0.16, tooShort);
        } catch (const std::invalid_argument&) {
            rejectedShortDepths = true;
        }
        require(rejectedShortDepths, "Depth query rejects wrong buffer size");


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

        // The fixed base cannot move out of a physical target. This is
        // fundamentally different from contact along a movable arm link,
        // which may be resolved by a safe retreat and reconfiguration.
        const auto impossible = planJointPath2D(arm, limits, {0.0, 0.0}, options);
        require(!impossible.found(), "Target covering the fixed base is impossible");

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
