#include "robotics/kinematics/Jacobian2D.hpp"

#include <cmath>
#include <stdexcept>

namespace robotics::kinematics {

namespace {

bool finite(math::Vec2 value) {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

} // namespace

std::vector<math::Vec2> endEffectorJacobian(
    std::span<const math::Vec2> positions) {
    if (positions.empty()) {
        throw std::invalid_argument("Forward-kinematics positions cannot be empty");
    }

    const auto tip = positions.back();
    if (!finite(tip)) {
        throw std::invalid_argument("Non-finite end-effector position");
    }

    std::vector<math::Vec2> columns;
    columns.reserve(positions.size() - 1);

    // All downstream links rotate about joint i at positions[i].
    // d(x, y)/d(theta_i) = (-(tip.y - joint.y), tip.x - joint.x).
    for (std::size_t i = 0; i + 1 < positions.size(); ++i) {
        const auto pivot = positions[i];
        if (!finite(pivot)) {
            throw std::invalid_argument("Non-finite joint position");
        }

        const math::Vec2 column{-(tip.y - pivot.y), tip.x - pivot.x};
        if (!finite(column)) {
            throw std::overflow_error("Jacobian component overflow");
        }
        columns.push_back(column);
    }

    return columns;
}

math::Vec2 endEffectorVelocity(
    std::span<const math::Vec2> jacobianColumns,
    std::span<const Joint2D> joints) {
    if (jacobianColumns.size() != joints.size()) {
        throw std::invalid_argument("Jacobian and joint counts must match");
    }

    math::Vec2 velocity{};
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (!finite(jacobianColumns[i]) ||
            !std::isfinite(joints[i].angularVelocity)) {
            throw std::invalid_argument("Non-finite Jacobian or joint angular velocity");
        }
        const auto next = velocity + jacobianColumns[i] * joints[i].angularVelocity;
        if (!finite(next)) {
            throw std::overflow_error("End-effector velocity overflow");
        }
        velocity = next;
    }
    return velocity;
}

double jacobianDeterminant2D(
    std::span<const math::Vec2> jacobianColumns) {
    if (jacobianColumns.size() != 2) {
        throw std::invalid_argument("Determinant requires a 2 x 2 Jacobian");
    }
    const auto a = jacobianColumns[0];
    const auto b = jacobianColumns[1];
    if (!finite(a) || !finite(b)) {
        throw std::invalid_argument("Non-finite Jacobian column");
    }

    const double determinant = a.x * b.y - b.x * a.y;
    if (!std::isfinite(determinant)) {
        throw std::overflow_error("Jacobian determinant overflow");
    }
    return determinant;
}

} // namespace robotics::kinematics
