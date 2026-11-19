#include "robotics/kinematics/ForwardKinematics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::kinematics::Joint2D;
using robotics::kinematics::forwardKinematics;

bool near(double a, double b) {
    return std::abs(a - b) < 1e-12;
}

int main() {
    int failures = 0;
    const auto check = [&failures](bool condition, const char* name) {
        if (!condition) {
            std::cerr << "FAILED: " << name << '\n';
            ++failures;
        }
    };

    const std::vector<Joint2D> straight{{2.0, 0.0}, {1.0, 0.0}};
    const auto s = forwardKinematics(straight);
    check(s.size() == 3, "count N+1");
    check(near(s.at(0).x, 0.0) && near(s.at(0).y, 0.0), "base");
    check(near(s.at(1).x, 2.0) && near(s.at(1).y, 0.0), "straight first link");
    check(near(s.at(2).x, 3.0) && near(s.at(2).y, 0.0), "straight end effector");

    const double halfPi = std::numbers::pi / 2.0;
    const std::vector<Joint2D> bent{{2.0, halfPi}, {1.0, -halfPi}};
    const auto b = forwardKinematics(bent);
    check(near(b.at(1).x, 0.0) && near(b.at(1).y, 2.0), "bent first link");
    check(near(b.at(2).x, 1.0) && near(b.at(2).y, 2.0), "bent end effector");

    const std::vector<Joint2D> three{{1.0, halfPi}, {2.0, halfPi}, {1.0, -halfPi}};
    const auto t = forwardKinematics(three);
    check(t.size() == 4, "three-joint count");
    check(near(t.back().x, -2.0) && near(t.back().y, 2.0), "three-joint relative angles");

    check(forwardKinematics(std::vector<Joint2D>{}).size() == 1,
          "empty arm returns origin");

    const auto rejects = [](const Joint2D& joint) {
        try {
            (void)forwardKinematics(std::vector<Joint2D>{joint});
            return false;
        } catch (const std::invalid_argument&) {
            return true;
        }
    };
    check(rejects({-1.0, 0.0}), "negative length");
    check(rejects({0.0, 0.0}), "zero length");
    check(rejects({1.0, std::numeric_limits<double>::quiet_NaN()}), "NaN angle");
    check(rejects({1.0, std::numeric_limits<double>::infinity()}), "infinite angle");

    return failures == 0 ? 0 : 1;
}
