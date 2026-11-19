#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/SmoothMotion2D.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::kinematics::advanceTowardTarget;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::hasSelfCollision;
using robotics::kinematics::Joint2D;
using robotics::kinematics::MotionSettings2D;
using robotics::math::Vec2;

constexpr double dt = 1.0 / 60.0;

bool safeMotion(std::vector<Joint2D>& arm, Vec2 target, int frames) {
    const MotionSettings2D limits{};
    bool reached = false;
    for (int frame = 0; frame < frames; ++frame) {
        const auto before = arm;
        const auto result = advanceTowardTarget(arm, target, dt, limits);
        reached = reached || result.reached;
        for (std::size_t i = 0; i < arm.size(); ++i) {
            if (std::abs(arm[i].angularVelocity) > limits.maxAngularSpeed + 1e-8)
                throw std::runtime_error("Angular speed limit violated");
            if (!result.constrained &&
                std::abs(arm[i].angularVelocity - before[i].angularVelocity) >
                    limits.maxAngularAcceleration * dt + 1e-8)
                throw std::runtime_error("Angular acceleration limit violated");
            if (i > 0) {
                const double a = arm[i].angle;
                const double sign = std::signbit(before[i].angle) ? -1.0 : 1.0;
                if (sign * a < limits.jointClearance - 1e-8 ||
                    sign * a > std::numbers::pi - limits.jointClearance + 1e-8)
                    throw std::runtime_error("Joint switched elbow branch");
            }
        }
        if (hasSelfCollision(forwardKinematics(arm), limits.linkClearance))
            throw std::runtime_error("Links collided");
    }
    return reached;
}

int main() {
    int failures = 0;
    auto check = [&failures](bool ok, const char* what) {
        if (!ok) { std::cerr << "FAILED: " << what << '\n'; ++failures; }
    };
    const auto initial = [] {
        return std::vector<Joint2D>{{2.0, std::numbers::pi / 2.0, 0.0},
                                    {1.0, -std::numbers::pi / 2.0, 0.0}};
    };

    try {
        for (Vec2 target : {Vec2{1.5, 1.5}, {2.0, 0.5}, {-2.0, 0.6}, {0.5, -2.0}}) {
            auto arm = initial();
            check(safeMotion(arm, target, 750), "Two-joint convergence");
            check((forwardKinematics(arm).back() - target).length() < 0.005,
                  "Two-joint final position");
        }
        auto outer = initial();
        check(!safeMotion(outer, {2.0, -3.0}, 900),
              "Unreachable outer target must not report success");
        const auto outerTip = forwardKinematics(outer).back();
        check(std::abs(outer[0].angularVelocity) < 1e-5 &&
              std::abs(outer[1].angularVelocity) < 1e-5,
              "Unreachable target must settle without jitter");
        safeMotion(outer, {2.0, -3.0}, 60);
        check((forwardKinematics(outer).back() - outerTip).length() < 1e-5,
              "Unreachable target must remain stationary");

        auto inner = initial();
        check(!safeMotion(inner, {0.0, 0.0}, 900),
              "Unreachable inner target must not report success");

        auto moving = initial();
        for (int block = 0; block < 80; ++block) {
            const double phase = static_cast<double>(block) * 0.74;
            safeMotion(moving, {3.5 * std::cos(phase), 3.5 * std::sin(phase)}, 20);
        }
        check(true, "Rapid target switching preserved limits and elbow branch");

        std::vector<Joint2D> redundant{{1.25, 0.3, 0.0},
                                       {0.75, -0.7, 0.0},
                                       {1.1, 0.8, 0.0}};
        check(safeMotion(redundant, {1.6, 1.1}, 500), "Three-joint convergence");
        for (int block = 0; block < 70; ++block) {
            const double phase = static_cast<double>(block) * 0.62;
            safeMotion(redundant, {3.5 * std::cos(phase),
                                   3.5 * std::sin(phase)}, 25);
        }
        check(true, "Three-joint collision avoidance on changing targets");

        check(hasSelfCollision(std::vector<Vec2>{{0, 0}, {2, 0}, {2, 2}, {1, -1}},
                               0.1), "Detect crossing between nonadjacent segments");
        check(!hasSelfCollision(std::vector<Vec2>{{0, 0}, {2, 0}, {2, 2}, {3, 2}},
                                0.1), "Do not reject separated segments");
        auto bad = initial();
        bool rejected = false;
        try {
            (void)advanceTowardTarget(bad,
                {std::numeric_limits<double>::quiet_NaN(), 1.0}, dt);
        } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "Reject non-finite target");
        rejected = false;
        try {
            (void)advanceTowardTarget(bad, {1.0, 2.0}, 0.0);
        } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "Reject zero timestep");
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
    return failures == 0 ? 0 : 1;
}
