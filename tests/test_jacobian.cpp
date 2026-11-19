#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::kinematics::Joint2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::endEffectorJacobian;
using robotics::kinematics::endEffectorVelocity;
using robotics::kinematics::jacobianDeterminant2D;
using robotics::math::Vec2;

bool near(double a, double b, double tolerance = 1e-10) {
    return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) < tolerance;
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

int main() {
    int failures = 0;
    const auto check = [&failures](bool condition, const char* label) {
        if (!condition) {
            std::cerr << "FAILED: " << label << '\n';
            ++failures;
        }
    };

    // Analytically known two-link arm: tip (1, 2), pivots (0,0), (0,2).
    const std::vector<Joint2D> arm{
        {2.0, std::numbers::pi / 2.0, std::numbers::pi / 6.0},
        {1.0, -std::numbers::pi / 2.0, -std::numbers::pi / 12.0}
    };
    const auto jacobian = endEffectorJacobian(forwardKinematics(arm));
    check(jacobian.size() == 2, "Jacobian column count");
    check(near(jacobian[0].x, -2.0) && near(jacobian[0].y, 1.0),
          "first Jacobian column");
    check(near(jacobian[1].x, 0.0) && near(jacobian[1].y, 1.0),
          "second Jacobian column");
    check(near(jacobianDeterminant2D(jacobian), -2.0), "two-link determinant");

    const auto velocity = endEffectorVelocity(jacobian, arm);
    check(near(velocity.x, -std::numbers::pi / 3.0), "tip x velocity");
    check(near(velocity.y, std::numbers::pi / 12.0), "tip y velocity");

    // A straight 2-link arm has a singular (zero-determinant) Jacobian.
    const std::vector<Joint2D> straight{{2.0, 0.0}, {1.0, 0.0}};
    check(near(jacobianDeterminant2D(endEffectorJacobian(
          forwardKinematics(straight))), 0.0), "straight-arm singularity");

    // Test each column against the finite-difference derivative of FK.
    const std::vector<Joint2D> three{
        {1.25, 0.3, 0.4}, {0.75, -0.7, -0.2}, {1.1, 0.8, 0.6}
    };
    const auto columns = endEffectorJacobian(forwardKinematics(three));
    check(columns.size() == three.size(), "three-link column count");
    constexpr double epsilon = 1e-6;
    for (std::size_t i = 0; i < three.size(); ++i) {
        auto plus = three;
        auto minus = three;
        plus[i].angle += epsilon;
        minus[i].angle -= epsilon;
        const auto p = forwardKinematics(plus).back();
        const auto m = forwardKinematics(minus).back();
        check(near(columns[i].x, (p.x - m.x) / (2.0 * epsilon), 1e-8) &&
              near(columns[i].y, (p.y - m.y) / (2.0 * epsilon), 1e-8),
              "three-link numerical derivative");
    }
    check(throws<std::invalid_argument>([&] {
        (void)jacobianDeterminant2D(columns);
    }), "determinant undefined for 2x3");

    // Empty robot (base position only): 2x0 Jacobian and zero velocity.
    const auto none = endEffectorJacobian(std::vector<Vec2>{{0.0, 0.0}});
    check(none.empty(), "zero-joint Jacobian");
    const auto stopped = endEffectorVelocity(none, std::vector<Joint2D>{});
    check(near(stopped.x, 0.0) && near(stopped.y, 0.0), "zero-joint velocity");

    check(throws<std::invalid_argument>([] {
        (void)endEffectorJacobian(std::vector<Vec2>{});
    }), "reject empty positions");
    check(throws<std::invalid_argument>([] {
        (void)endEffectorJacobian(std::vector<Vec2>{
            {0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}});
    }), "reject non-finite positions");
    check(throws<std::invalid_argument>([&] {
        (void)endEffectorVelocity(jacobian, std::vector<Joint2D>{{1.0, 0.0}});
    }), "reject mismatched number of joints");
    check(throws<std::invalid_argument>([&] {
        auto broken = arm;
        broken[0].angularVelocity = std::numeric_limits<double>::quiet_NaN();
        (void)endEffectorVelocity(jacobian, broken);
    }), "reject non-finite angular velocity");
    check(throws<std::invalid_argument>([] {
        (void)jacobianDeterminant2D(std::vector<Vec2>{{1.0, 2.0}});
    }), "reject nonsquare determinant");

    return failures == 0 ? 0 : 1;
}
