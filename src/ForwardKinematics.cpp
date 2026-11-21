#include "robotics/kinematics/ForwardKinematics.hpp"

#include <cmath>
#include <stdexcept>

namespace robotics::kinematics {
namespace {

// One implementation of the relative-angle kinematic chain, shared by the
// public API and the planner's buffer-reusing angle override. No robot copies.
template <typename AngleAt>
void calculatePositions(std::span<const Joint2D> joints, AngleAt angleAt,
                        std::vector<math::Vec2>& positions) {
    positions.clear();
    positions.reserve(joints.size() + 1);
    positions.push_back({0.0, 0.0});

    double absoluteAngle = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const double angle = angleAt(i);
        if (!std::isfinite(joints[i].length) || joints[i].length <= 0.0 ||
            !std::isfinite(angle)) {
            throw std::invalid_argument(
                "Joint length must be positive and finite; angle must be finite");
        }

        absoluteAngle += angle;
        if (!std::isfinite(absoluteAngle)) {
            throw std::overflow_error("Accumulated joint angle is not finite");
        }

        const math::Vec2 offset{
            joints[i].length * std::cos(absoluteAngle),
            joints[i].length * std::sin(absoluteAngle)
        };
        const math::Vec2 position = positions.back() + offset;

        if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
            throw std::overflow_error("Joint position is not finite");
        }

        positions.push_back(position);
    }
}

} // namespace

std::vector<math::Vec2> forwardKinematics(
    std::span<const Joint2D> joints) {
    std::vector<math::Vec2> positions;
    calculatePositions(joints, [&](std::size_t i) { return joints[i].angle; },
                       positions);
    return positions;
}

void forwardKinematics(std::span<const Joint2D> joints,
                       std::span<const double> angles,
                       std::vector<math::Vec2>& positions) {
    if (angles.size() != joints.size())
        throw std::invalid_argument("Joint-angle count must match joint count");
    calculatePositions(joints, [&](std::size_t i) { return angles[i]; },
                       positions);
}

} // namespace robotics::kinematics
