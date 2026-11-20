#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/RobotConfiguration2D.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace robotics::kinematics;
using robotics::math::Vec2;

bool rejects(const std::vector<JointDefinition2D>& config) {
    try { (void)makeRobot2D(config); }
    catch (const std::invalid_argument&) { return true; }
    return false;
}

int main() {
    int failed = 0;
    auto test = [&](bool success, const char* label) {
        if (!success) { ++failed; std::cerr << "FAILED: " << label << '\n'; }
    };
    const double pi = std::numbers::pi;
    std::vector<JointDefinition2D> config{
        {2.0, pi / 2.0, {-2.0 * pi, 2.0 * pi, 1.0, 3.0}},
        {1.0, -pi / 2.0, {-pi + 0.16, -0.16, 0.5, 2.0}}
    };
    auto robot = makeRobot2D(config);
    test(robot.joints.size() == 2 && robot.limits.size() == 2,
         "robot state and limits created");
    test(robot.joints[0].angularVelocity == 0.0 &&
         robot.joints[0].length == 2.0, "state initialized");
    test((forwardKinematics(robot.joints).back() - Vec2{1.0, 2.0}).length() < 1e-12,
         "correct FK from definition");
    test(rejects({}), "reject empty definition");
    auto changed = config;
    changed[0].length = -1.0;
    test(rejects(changed), "reject negative link length");
    changed = config;
    changed[1].limits.minAngle = -3.0;
    changed[1].limits.maxAngle = 0.5;
    test(rejects(changed), "reject elbow branch crossing zero");
    changed = config;
    changed[1].limits.maxAngularSpeed = 0.0;
    test(rejects(changed), "reject zero speed");
    changed = config;
    changed[1].initialAngle = 0.2;
    test(rejects(changed), "reject initial angle outside bounds");
    changed = config;
    changed[1].limits.minAngle = changed[1].limits.maxAngle;
    test(rejects(changed), "reject reversed limits");
    changed = config;
    changed.push_back({0.7, 0.85, {0.16, pi - 0.16, 1.8, 4.0}});
    test(makeRobot2D(changed).joints.size() == 3, "three-link robot");

    auto extended = changed;
    for (int n = 4; n <= 6; ++n) {
        const bool negative = n % 2 == 0;
        extended.push_back({0.7, negative ? -0.85 : 0.85,
            negative ? JointLimits2D{-pi + 0.16, -0.16, 1.8, 4.0} :
                       JointLimits2D{0.16, pi - 0.16, 1.8, 4.0}});
        test(makeRobot2D(extended).joints.size() == static_cast<std::size_t>(n),
             "extended robot configuration");
    }
    return failed == 0 ? 0 : 1;
}
