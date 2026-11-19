#include "robotics/kinematics/ForwardKinematics.hpp"

#include <cmath>
#include <stdexcept>

namespace robotics::kinematics {

std::vector<math::Vec2> forwardKinematics(
    std::span<const Joint2D> joints) {
    std::vector<math::Vec2> positions;
    positions.reserve(joints.size() + 1);
    positions.push_back({0.0, 0.0});

    double absoluteAngle = 0.0;

    for (const Joint2D& joint : joints) {
        if (!std::isfinite(joint.length) || joint.length <= 0.0 ||
            !std::isfinite(joint.angle)) {
            throw std::invalid_argument(
                "Joint length must be positive and finite; angle must be finite");
        }

        absoluteAngle += joint.angle;
        if (!std::isfinite(absoluteAngle)) {
            throw std::overflow_error("Accumulated joint angle is not finite");
        }

        const math::Vec2 offset{
            joint.length * std::cos(absoluteAngle),
            joint.length * std::sin(absoluteAngle)
        };
        const math::Vec2 position = positions.back() + offset;

        if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
            throw std::overflow_error("Joint position is not finite");
        }

        positions.push_back(position);
    }

    return positions;
}

} // namespace robotics::kinematics
