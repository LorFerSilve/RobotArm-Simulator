#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <stop_token>
#include <vector>

using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::geometry::hasObstacleCollision;
using robotics::kinematics::Joint2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::sweptArmClear2D;
using robotics::math::Vec2;

namespace {
void require(bool ok, const char* description) {
    if (!ok) throw std::runtime_error(description);
}
} // namespace

int main() {
    try {
        constexpr double pi = std::numbers::pi;
        constexpr double sceneClearance = 0.0001;
        const std::vector<Joint2D> arm{
            {2.0, 0.0, 0.0}, {1.0, -pi / 2.0, 0.0}
        };
        const std::vector<double> start{0.0, -pi / 2.0};
        const std::vector<double> end{2.0 * pi, -pi / 2.0};

        // A circle thinner than the old 0.04-radian fixed check spacing:
        // BOTH endpoints and all those fixed samples are collision-free,
        // yet the first link sweeps across the circle at angle 0.017 rad.
        const std::vector<Obstacle2D> thinScene{
            Circle2D{{1.3 * std::cos(0.017), 1.3 * std::sin(0.017)}, 0.0008}
        };
        const auto steps = static_cast<int>(std::ceil(2.0 * pi / 0.04));
        for (int i = 0; i <= steps; ++i) {
            std::vector<double> q{2.0 * pi * i / steps, -pi / 2.0};
            std::vector<Vec2> positions;
            forwardKinematics(arm, q, positions);
            require(!hasObstacleCollision(positions, thinScene, sceneClearance),
                    "Regression fixture should evade legacy uniform sampling");
        }
        require(!sweptArmClear2D(
                    arm, start, end, thinScene, 0.1, sceneClearance),
                "Full-turn sweep must reject a missed narrow collision");

        const auto distant = std::vector<Obstacle2D>{
            Circle2D{{-8.0, 8.0}, 0.4}
        };
        require(sweptArmClear2D(arm, start, end, distant, 0.1, 0.1),
                "A full-turn motion without nearby obstacles must certify");
        require(sweptArmClear2D(arm, start, end, {}, 0.1, 0.1),
                "Two-link motion without static obstacles is self-safe");

        // In four-link arms, a straight joint-space interpolation may
        // self-intersect even if its initial and terminal poses are safe.
        const std::vector<Joint2D> four{
            {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
            {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}
        };
        const std::vector<double> selfStart{
            -0.2801284757719924, 2.421035427174905,
             1.6603937035528296, 2.0099474929725223
        };
        const std::vector<double> selfEnd{
            -0.728984788994322, 1.2291495140475175,
             0.6115497954736806, 1.934674912047273
        };
        require(sweptArmClear2D(
                    four, selfStart, selfStart, {}, 0.1, 0.1),
                "Self-safe starting pose must certify");
        require(sweptArmClear2D(
                    four, selfEnd, selfEnd, {}, 0.1, 0.1),
                "Self-safe final pose must certify");
        require(!sweptArmClear2D(
                    four, selfStart, selfEnd, {}, 0.1, 0.1),
                "Intermediate self-intersection must reject full edge");

        // Checks must be cancellation-aware and must reject bad geometry,
        // without depending on raylib or a real GUI session.
        std::stop_source cancelled;
        cancelled.request_stop();
        require(!sweptArmClear2D(
                    arm, start, end, distant, 0.1, 0.1, cancelled.get_token()),
                "Canceled certification must not publish a safe result");

        bool rejected = false;
        try {
            (void)sweptArmClear2D(
                arm, std::vector<double>{0.0}, end, distant, 0.1, 0.1);
        } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Mismatched joint angles must throw");

        std::cout << "Certified conservative swept collision regression passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
