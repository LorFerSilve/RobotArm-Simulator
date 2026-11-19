#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/simulation/Simulation2D.hpp"

#include <charconv>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>
#include <vector>

int main(int argc, char* argv[]) {
    using robotics::kinematics::Joint2D;
    using robotics::kinematics::forwardKinematics;
    using robotics::simulation::advanceJoints;

    constexpr std::size_t defaultSteps = 10;
    constexpr std::size_t maxSteps = 10000;
    constexpr double dt = 1.0 / 60.0;
    constexpr double radToDeg = 180.0 / std::numbers::pi;
    std::size_t steps = defaultSteps;

    if (argc > 2) {
        std::cerr << "Usage: robot_cli [steps: 0..10000]\n";
        return 2;
    }
    if (argc == 2) {
        const std::string_view argument{argv[1]};
        const auto [end, error] = std::from_chars(
            argument.data(), argument.data() + argument.size(), steps);
        if (error != std::errc{} || end != argument.data() + argument.size() ||
            steps > maxSteps) {
            std::cerr << "Invalid step count. Usage: robot_cli [steps: 0..10000]\n";
            return 2;
        }
    }

    std::vector<Joint2D> joints{
        {2.0, std::numbers::pi / 2.0, std::numbers::pi / 6.0},
        {1.0, -std::numbers::pi / 2.0, -std::numbers::pi / 12.0}
    };

    std::cout << std::fixed << std::setprecision(4);
    for (std::size_t frame = 0; frame <= steps; ++frame) {
        const auto positions = forwardKinematics(joints);
        std::cout << "Frame " << frame << " | t=" << frame * dt << " s\n";
        for (std::size_t i = 0; i < joints.size(); ++i) {
            std::cout << "  Joint " << i + 1
                      << " | angle=" << joints[i].angle * radToDeg << " deg"
                      << " | speed=" << joints[i].angularVelocity * radToDeg << " deg/s"
                      << " | endpoint=(" << positions[i + 1].x << ", "
                      << positions[i + 1].y << ") m\n";
        }
        const auto& tip = positions.back();
        std::cout << "  End effector: (" << tip.x << ", " << tip.y << ") m\n";

        if (frame != steps) {
            advanceJoints(joints, dt);
        }
    }
}
