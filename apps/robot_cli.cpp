#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/InverseKinematics2D.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"
#include "robotics/simulation/Simulation2D.hpp"

#include <charconv>
#include <cstddef>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

constexpr std::size_t maxSteps = 10000;
constexpr double dt = 1.0 / 60.0;
constexpr double radToDeg = 180.0 / std::numbers::pi;

void usage() {
    std::cerr << "Usage:\n"
              << "  robot_cli [steps: 0..10000]              (constant joint speeds)\n"
              << "  robot_cli ik <x> <y> [steps: 0..10000]  (target in metres)\n";
}

bool parseSteps(std::string_view text, std::size_t& steps) {
    const auto [end, error] = std::from_chars(
        text.data(), text.data() + text.size(), steps);
    return error == std::errc{} && end == text.data() + text.size() &&
           steps <= maxSteps;
}

bool parseCoordinate(std::string_view text, double& coordinate) {
    const auto [end, error] = std::from_chars(
        text.data(), text.data() + text.size(), coordinate);
    return error == std::errc{} && end == text.data() + text.size() &&
           std::isfinite(coordinate);
}

} // namespace

int main(int argc, char* argv[]) {
    using robotics::kinematics::Joint2D;
    using robotics::kinematics::computeIKCommand;
    using robotics::kinematics::forwardKinematics;
    using robotics::kinematics::endEffectorJacobian;
    using robotics::kinematics::endEffectorVelocity;
    using robotics::kinematics::jacobianDeterminant2D;
    using robotics::math::Vec2;
    using robotics::simulation::advanceJoints;

    const bool ikMode = argc >= 2 && std::string_view{argv[1]} == "ik";
    std::size_t steps = ikMode ? 300 : 10;
    Vec2 target{};

    if (ikMode) {
        if ((argc != 4 && argc != 5) ||
            !parseCoordinate(argv[2], target.x) ||
            !parseCoordinate(argv[3], target.y) ||
            (argc == 5 && !parseSteps(argv[4], steps))) {
            usage();
            return 2;
        }
    } else if (argc > 2 || (argc == 2 && !parseSteps(argv[1], steps))) {
        usage();
        return 2;
    }

    std::vector<Joint2D> joints{
        {2.0, std::numbers::pi / 2.0, std::numbers::pi / 6.0},
        {1.0, -std::numbers::pi / 2.0, -std::numbers::pi / 12.0}
    };

    std::cout << std::fixed << std::setprecision(4);
    if (ikMode) {
        std::cout << "IK target: (" << target.x << ", " << target.y << ") m\n";
        // Check the obvious out-of-reach case for a base at the origin.
        double maximumReach = 0.0;
        for (const auto& joint : joints) {
            maximumReach += joint.length;
        }
        if (target.length() > maximumReach) {
            std::cout << "Warning: target lies beyond maximum arm reach ("
                      << maximumReach << " m)\n";
        }
    }

    bool reached = false;
    for (std::size_t frame = 0; frame <= steps; ++frame) {
        double positionError = 0.0;
        if (ikMode) {
            const auto command = computeIKCommand(joints, target);
            positionError = command.positionError;
            reached = command.reached;
            for (std::size_t i = 0; i < joints.size(); ++i) {
                joints[i].angularVelocity = command.angularVelocities[i];
            }
        }

        const auto positions = forwardKinematics(joints);
        const auto jacobian = endEffectorJacobian(positions);
        const auto velocity = endEffectorVelocity(jacobian, joints);
        std::cout << "Frame " << frame << " | t=" << static_cast<double>(frame) * dt << " s\n";
        for (std::size_t i = 0; i < joints.size(); ++i) {
            std::cout << "  Joint " << i + 1
                      << " | angle=" << joints[i].angle * radToDeg << " deg"
                      << " | speed=" << joints[i].angularVelocity * radToDeg << " deg/s"
                      << " | endpoint=(" << positions[i + 1].x << ", "
                      << positions[i + 1].y << ") m\n";
        }
        const auto& tip = positions.back();
        std::cout << "  End effector: (" << tip.x << ", " << tip.y << ") m\n";
        std::cout << "  End effector velocity: (" << velocity.x << ", "
                  << velocity.y << ") m/s\n";
        for (std::size_t i = 0; i < jacobian.size(); ++i) {
            std::cout << "  Jacobian column " << i + 1 << ": ("
                      << jacobian[i].x << ", " << jacobian[i].y << ") m/rad\n";
        }
        if (jacobian.size() == 2) {
            std::cout << "  Jacobian determinant: "
                      << jacobianDeterminant2D(jacobian) << " m^2\n";
        }
        if (ikMode) {
            std::cout << "  Target error: " << std::setprecision(6)
                      << positionError << " m\n" << std::setprecision(4);
        }

        if (ikMode && reached) {
            std::cout << "IK result: reached target within tolerance\n";
            break;
        }
        if (frame != steps) {
            advanceJoints(joints, dt);
        }
    }
    if (ikMode && !reached) {
        std::cout << "IK result: target not reached within " << steps
                  << " steps\n";
    }
}
