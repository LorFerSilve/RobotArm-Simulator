#include "robotics/simulation/Simulation2D.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::kinematics::Joint2D;
using robotics::simulation::advanceJoints;

bool near(double actual, double expected) {
    return std::abs(actual - expected) < 1e-12;
}

int main() {
    int failures = 0;
    const auto check = [&failures](bool ok, const char* name) {
        if (!ok) {
            std::cerr << "FAILED: " << name << '\n';
            ++failures;
        }
    };

    const double halfPi = std::numbers::pi / 2.0;
    std::vector<Joint2D> joints{{2.0, 0.0, std::numbers::pi},
                                {1.0, halfPi, -halfPi}};
    advanceJoints(joints, 0.5);
    check(near(joints[0].angle, halfPi), "first joint integration");
    check(near(joints[1].angle, halfPi / 2.0), "second joint integration");
    check(near(joints[0].angularVelocity, std::numbers::pi), "velocity unchanged");

    std::vector<Joint2D> stopped{{1.0, 0.25, 0.0}};
    advanceJoints(stopped, 1.0 / 60.0);
    check(near(stopped[0].angle, 0.25), "stationary joint");

    auto rejectsDt = [](double dt) {
        try {
            std::vector<Joint2D> robot{{1.0, 0.0, 0.0}};
            advanceJoints(robot, dt);
            return false;
        } catch (const std::invalid_argument&) {
            return true;
        }
    };
    check(rejectsDt(0.0), "reject zero dt");
    check(rejectsDt(-1.0), "reject negative dt");
    check(rejectsDt(std::numeric_limits<double>::infinity()), "reject infinity dt");
    check(rejectsDt(std::numeric_limits<double>::quiet_NaN()), "reject NaN dt");

    std::vector<Joint2D> invalid{
        {1.0, 0.5, 1.0},
        {1.0, 0.0, std::numeric_limits<double>::infinity()}
    };
    try {
        advanceJoints(invalid, 1.0);
        check(false, "reject invalid velocity");
    } catch (const std::invalid_argument&) {
        check(near(invalid[0].angle, 0.5), "no partial update on invalid velocity");
    }

    std::vector<Joint2D> overflow{
        {1.0, 0.5, 1.0},
        {1.0, 0.0, std::numeric_limits<double>::max()}
    };
    try {
        advanceJoints(overflow, 2.0);
        check(false, "reject overflow");
    } catch (const std::overflow_error&) {
        check(near(overflow[0].angle, 0.5), "no partial update on overflow");
    }

    return failures == 0 ? 0 : 1;
}
