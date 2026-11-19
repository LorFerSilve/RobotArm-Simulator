#include "robotics/kinematics/ForwardKinematics.hpp"

#include <iomanip>
#include <iostream>
#include <numbers>
#include <vector>

int main() {
    using robotics::kinematics::Joint2D;
    using robotics::kinematics::forwardKinematics;

    const std::vector<Joint2D> joints{
        {2.0, std::numbers::pi / 2.0},
        {1.0, -std::numbers::pi / 2.0}
    };
    const auto positions = forwardKinematics(joints);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Base: (" << positions[0].x << ", "
              << positions[0].y << ") m\n";

    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto& point = positions[i + 1];
        std::cout << "Joint " << i + 1
                  << " | angle=" << joints[i].angle * 180.0 / std::numbers::pi
                  << " deg | endpoint=(" << point.x << ", "
                  << point.y << ") m\n";
    }

    const auto& tip = positions.back();
    std::cout << "End effector: (" << tip.x << ", " << tip.y << ") m\n";
}
