#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/RobotConfiguration2D.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace robotics::kinematics;
using robotics::math::Vec2;

int main() {
    int failures = 0;
    const auto check = [&](bool ok, const char* label) {
        if (!ok) { std::cerr << "FAILED: " << label << '\n'; ++failures; }
    };
    constexpr double dt = 1.0 / 60.0;
    const double pi = std::numbers::pi;
    std::vector<JointDefinition2D> definition{
        {2.0, pi / 2.0, {1.2, 1.8, 0.30, 1.0}},
        {1.0, -pi / 2.0, {-2.0, -0.8, 0.15, 0.60}}
    };
    auto robot = makeRobot2D(definition);
    bool limited = false;
    for (int t = 0; t < 500; ++t) {
        const double prev0 = robot.joints[0].angularVelocity;
        const double prev1 = robot.joints[1].angularVelocity;
        const auto outcome = advanceTowardTarget(robot.joints, {-2.0, -1.0}, dt,
                                                 {}, robot.limits);
        limited = limited || outcome.constrained;
        for (std::size_t i = 0; i < robot.joints.size(); ++i) {
            const auto& j = robot.joints[i];
            const auto& limit = robot.limits[i];
            check(j.angle >= limit.minAngle - 1e-8 &&
                  j.angle <= limit.maxAngle + 1e-8, "angle within limits");
            check(std::abs(j.angularVelocity) <= limit.maxAngularSpeed + 1e-8,
                  "individual speed obeyed");
        }
        check(std::abs(robot.joints[0].angularVelocity - prev0) <= 1.0 * dt + 1e-7,
              "shoulder acceleration bound");
        check(std::abs(robot.joints[1].angularVelocity - prev1) <= 0.60 * dt + 1e-7,
              "elbow acceleration bound");
    }
    check(limited, "inaccessible target respects joint limits");
    check((forwardKinematics(robot.joints).back() - Vec2{-2.0, -1.0}).length() > 0.005,
          "inaccessible target not misreported as reached");

    std::vector<JointDefinition2D> three = {
        {1.25, 0.3, {-2.0 * pi, 2.0 * pi, 0.6, 2.0}},
        {0.75, -0.7, {-2.3, -0.2, 0.5, 1.8}},
        {1.1, 0.8, {0.2, 2.3, 0.4, 1.4}}
    };
    auto multi = makeRobot2D(three);
    for (int t = 0; t < 300; ++t) {
        (void)advanceTowardTarget(multi.joints, {1.6, 1.1}, dt, {}, multi.limits);
        for (std::size_t i = 0; i < multi.joints.size(); ++i) {
            const auto& j = multi.joints[i];
            const auto& limit = multi.limits[i];
            check(j.angle >= limit.minAngle - 1e-8 && j.angle <= limit.maxAngle + 1e-8,
                  "3-link limits obeyed");
            check(std::abs(j.angularVelocity) <= limit.maxAngularSpeed + 1e-8,
                  "3-link speeds obeyed");
        }
        check(!hasSelfCollision(forwardKinematics(multi.joints), 0.1),
              "3-link collision free");
    }

    bool rejected = false;
    try {
        (void)advanceTowardTarget(robot.joints, {0.0, 1.0}, dt, {},
                                  std::vector<JointLimits2D>{robot.limits[0]});
    } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "reject mismatched constraint count");

    return failures == 0 ? 0 : 1;
}
