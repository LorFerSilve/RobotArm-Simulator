#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::MotionSettings2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::hasSelfCollision;
using robotics::kinematics::targetLinkPenetrations;
using robotics::math::Vec2;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double dt = 1.0 / 60.0;

struct TransitionCase {
    const char* name;
    std::vector<Joint2D> initial;
    std::vector<JointLimits2D> limits;
    Vec2 start;
    Vec2 destination;
    MotionSettings2D settings;
    bool required; // true: a known reachable regression, false: exploratory
};

void check(bool ok, const std::string& problem) {
    if (!ok) throw std::runtime_error(problem);
}

// Returns false only when the bounded planner cannot find an exploratory
// path. All executed movement must pass the exact same safety assertions.
bool moveTo(std::vector<Joint2D>& robot,
            const TransitionCase& scenario, Vec2 target, bool required) {
    PlannedMotion2D motion;
    motion.setTarget(robot, scenario.limits, target, scenario.settings);
    if (!motion.pathAvailable()) {
        if (required)
            throw std::runtime_error(std::string(scenario.name) +
                                     ": planner could not find a required path");
        return false;
    }

    auto previousPenetration = targetLinkPenetrations(
        forwardKinematics(robot), target, scenario.settings.targetClearance);

    for (int frame = 0; frame < 4500; ++frame) {
        const auto previous = robot;
        const auto state = motion.advance(robot, scenario.limits, dt,
                                          scenario.settings);
        check(!state.blocked, std::string(scenario.name) + ": follower blocked");
        const auto positions = forwardKinematics(robot);
        check(!hasSelfCollision(positions, scenario.settings.linkClearance),
              std::string(scenario.name) + ": self-collision");

        const auto penetration = targetLinkPenetrations(
            positions, target, scenario.settings.targetClearance);
        for (std::size_t i = 0; i < penetration.size(); ++i) {
            check(penetration[i] <= previousPenetration[i] + 1e-7,
                  std::string(scenario.name) + ": increased target penetration");
        }
        previousPenetration = penetration;

        for (std::size_t i = 0; i < robot.size(); ++i) {
            const auto& bounds = scenario.limits[i];
            check(robot[i].angle >= bounds.minAngle - 1e-8 &&
                  robot[i].angle <= bounds.maxAngle + 1e-8,
                  std::string(scenario.name) + ": joint limit");
            check(std::abs(robot[i].angularVelocity) <=
                  std::min(scenario.settings.maxAngularSpeed,
                           bounds.maxAngularSpeed) + 1e-8,
                  std::string(scenario.name) + ": speed limit");
            check(std::abs(robot[i].angularVelocity -
                           previous[i].angularVelocity) <=
                  std::min(scenario.settings.maxAngularAcceleration,
                           bounds.maxAngularAcceleration) * dt + 1e-7,
                  std::string(scenario.name) + ": acceleration limit");
        }
        if (state.reached) {
            check((positions.back() - target).length() <=
                  scenario.settings.positionTolerance,
                  std::string(scenario.name) + ": inaccurate arrival");
            for (const double depth : penetration)
                check(depth <= 1e-8,
                      std::string(scenario.name) + ": touched target at arrival");
            return true;
        }
    }
    throw std::runtime_error(std::string(scenario.name) +
                             ": trajectory did not finish within 4500 frames");
}

} // namespace

int main() {
    try {
        MotionSettings2D safe{};
        safe.foldClearance = 0.55;
        safe.targetClearance = 0.16;
        const auto angle = pi / 180.0;
        const std::vector<Joint2D> three{
            {1.25, 0.3, 0.0}, {0.75, -0.7, 0.0}, {1.1, 0.8, 0.0}
        };
        const std::vector<JointLimits2D> threeLimits{
            {-2*pi, 2*pi, 2.0, 6.0}, {-pi + .16, -.16, 2.0, 6.0},
            {.16, pi - .16, 2.0, 6.0}
        };
        const std::vector<Joint2D> five{
            {2.0, -57.0*angle, 0.0}, {2.0, -21.0*angle, 0.0},
            {2.0, 26.0*angle, 0.0}, {2.0, -35.0*angle, 0.0},
            {2.0, 32.0*angle, 0.0}
        };
        const std::vector<JointLimits2D> fiveLimits{
            {-2*pi, 2*pi, 2.0, 6.0},
            {-pi+.56, -.16, 2.0, 6.0}, {.16, pi-.56, 2.0, 6.0},
            {-pi+.56, -.16, 2.0, 6.0}, {.16, pi-.56, 2.0, 6.0}
        };
        const std::vector<Joint2D> two{
            {1.0, pi/3.0, 0.0}, {1.0, -2.0*pi/3.0, 0.0}
        };
        const std::vector<JointLimits2D> twoLimits{
            {-2*pi, 2*pi, 2.0, 6.0}, {-pi+.56, -.16, 2.0, 6.0}
        };

        // Extend this table whenever a playtest discovers a new transition.
        // A starting Cartesian point alone is ambiguous for a redundant arm,
        // hence every case also stores its complete initial joint pose.
        const std::vector<TransitionCase> cases{
            {"two-joint around-target (1,0) -> (0,1)",
             two, twoLimits, {1.0, 0.0}, {0.0, 1.0}, safe, true},
            {"three-joint preset -> (1.6,1.1)",
             three, threeLimits, forwardKinematics(three).back(),
             {1.6, 1.1}, {}, true},
            {"three-joint (1.6,1.1) -> (-1,-0.4)",
             three, threeLimits, {1.6, 1.1},
             {-1.0, -0.4}, {}, true},
            {"five-joint preset (~4,-9) -> (-3,-4)",
             five, fiveLimits, forwardKinematics(five).back(),
             {-3.0, -4.0}, safe, true},
            {"five-joint exploratory (2,3) -> (5,4)",
             five, fiveLimits, {2.0, 3.0}, {5.0, 4.0}, safe, false}
        };

        for (const auto& scenario : cases) {
            std::cout << "TRANSITION " << scenario.name << '\n';
            auto robot = scenario.initial;
            const double initialError =
                (forwardKinematics(robot).back() - scenario.start).length();
            if (initialError > 0.005) {
                if (!moveTo(robot, scenario, scenario.start, scenario.required)) {
                    std::cout << "  INCONCLUSIVE: no path to start pose\n";
                    continue;
                }
            }
            if (moveTo(robot, scenario, scenario.destination, scenario.required)) {
                std::cout << "  PASS: reached target safely\n";
            } else {
                std::cout << "  INCONCLUSIVE: bounded planner found no path\n";
            }
        }
        std::cout << "Motion transition matrix completed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
