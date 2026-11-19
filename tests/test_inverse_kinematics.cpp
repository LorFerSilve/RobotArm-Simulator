#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/InverseKinematics2D.hpp"
#include "robotics/simulation/Simulation2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::kinematics::computeIKCommand;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::IKSettings2D;
using robotics::kinematics::Joint2D;
using robotics::math::Vec2;
using robotics::simulation::advanceJoints;

bool near(double value, double expected, double eps = 1e-10) {
    return std::isfinite(value) && std::abs(value - expected) < eps;
}

template <typename Exception, typename Function>
bool throws(Function&& fn) {
    try {
        fn();
    } catch (const Exception&) {
        return true;
    }
    return false;
}

// Iterate the same controller and integrator used by the CLI.
bool driveTo(std::vector<Joint2D> joints, Vec2 target,
             const IKSettings2D& options = {}, std::size_t maxIterations = 600) {
    constexpr double dt = 1.0 / 60.0;
    for (std::size_t frame = 0; frame <= maxIterations; ++frame) {
        const auto command = computeIKCommand(joints, target, options);
        if (command.reached) {
            return command.positionError <= options.positionTolerance &&
                   (forwardKinematics(joints).back() - target).length() <= options.positionTolerance;
        }
        if (frame == maxIterations) {
            break;
        }
        for (std::size_t i = 0; i < joints.size(); ++i) {
            joints[i].angularVelocity = command.angularVelocities[i];
        }
        advanceJoints(joints, dt);
    }
    return false;
}

int main() {
    int failures = 0;
    const auto check = [&failures](bool valid, const char* label) {
        if (!valid) {
            std::cerr << "FAILED: " << label << '\n';
            ++failures;
        }
    };

    const std::vector<Joint2D> two{
        {2.0, std::numbers::pi / 2.0, 0.0},
        {1.0, -std::numbers::pi / 2.0, 0.0}
    };
    const auto atTarget = computeIKCommand(two, {1.0, 2.0});
    check(atTarget.reached && atTarget.positionError < 1e-12,
          "already at target");
    check(atTarget.angularVelocities.size() == 2 &&
          atTarget.angularVelocities[0] == 0.0 && atTarget.angularVelocities[1] == 0.0,
          "zero commands at target");

    // Single-link case: at q=0, Jacobian column is (0,1).
    // For unit gain and damping=0.1, omega=1/(1+0.1^2).
    const std::vector<Joint2D> single{{1.0, 0.0, 0.0}};
    IKSettings2D unit{};
    unit.positionGain = 1.0;
    unit.maxAngularSpeed = 100.0;
    const auto cmd = computeIKCommand(single, {1.0, 1.0}, unit);
    check(near(cmd.angularVelocities.at(0), 1.0 / 1.01),
          "single-link damped least squares");

    check(driveTo(two, {1.5, 1.5}), "two-joint convergence");
    check(driveTo(two, {0.5, 2.5}), "second two-joint target");
    const std::vector<Joint2D> three{
        {1.25, 0.3, 0.0}, {0.75, -0.7, 0.0}, {1.1, 0.8, 0.0}
    };
    check(driveTo(three, {1.6, 1.1}), "three-joint convergence");

    // Starting from a singular straight arm is numerically safe even if
    // a particular radial target is not locally movable in first order.
    const std::vector<Joint2D> straight{{2.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const auto singular = computeIKCommand(straight, {2.0, 0.0});
    check(!singular.reached && singular.angularVelocities.size() == 2 &&
          near(singular.angularVelocities[0], 0.0) &&
          near(singular.angularVelocities[1], 0.0), "singular case stable");
    check(driveTo(straight, {2.5, 0.5}), "near-singular convergence");

    // Ensure all joint speed limits are respected while retaining direction.
    IKSettings2D limited{};
    limited.maxAngularSpeed = 0.05;
    const auto slow = computeIKCommand(two, {-1.0, 1.5}, limited);
    double peak = 0.0;
    for (double speed : slow.angularVelocities) {
        peak = std::max(peak, std::abs(speed));
    }
    check(near(peak, limited.maxAngularSpeed), "uniform angular speed limiting");

    check(!driveTo(two, {100.0, 0.0}, {}, 200), "unreachable target not reported as reached");

    check(throws<std::invalid_argument>([] {
        (void)computeIKCommand(std::vector<Joint2D>{}, {0.0, 0.0});
    }), "reject empty joints");
    check(throws<std::invalid_argument>([&] {
        (void)computeIKCommand(two, {std::numeric_limits<double>::quiet_NaN(), 0.0});
    }), "reject invalid target");
    check(throws<std::invalid_argument>([&] {
        IKSettings2D invalid{};
        invalid.damping = 0.0;
        (void)computeIKCommand(two, {0.0, 0.0}, invalid);
    }), "reject zero damping");
    check(throws<std::invalid_argument>([&] {
        IKSettings2D invalid{};
        invalid.maxAngularSpeed = -1.0;
        (void)computeIKCommand(two, {0.0, 0.0}, invalid);
    }), "reject negative speed limit");
    check(throws<std::invalid_argument>([&] {
        auto invalid = two;
        invalid[0].angularVelocity = std::numeric_limits<double>::infinity();
        (void)computeIKCommand(invalid, {0.0, 0.0});
    }), "reject invalid joint state");
    check(throws<std::overflow_error>([&] {
        (void)computeIKCommand(two, {std::numeric_limits<double>::max(), 1e308});
    }), "reject non-finite controller result");

    return failures == 0 ? 0 : 1;
}
