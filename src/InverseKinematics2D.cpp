#include "robotics/kinematics/InverseKinematics2D.hpp"

#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotics::kinematics {

namespace {

bool finite(math::Vec2 vector) {
    return std::isfinite(vector.x) && std::isfinite(vector.y);
}

} // namespace

IKCommand2D computeIKCommand(std::span<const Joint2D> joints,
                             math::Vec2 target,
                             const IKSettings2D& settings) {
    if (joints.empty() || !finite(target) ||
        !std::isfinite(settings.positionGain) || settings.positionGain <= 0.0 ||
        !std::isfinite(settings.damping) || settings.damping <= 0.0 ||
        !std::isfinite(settings.maxAngularSpeed) || settings.maxAngularSpeed <= 0.0 ||
        !std::isfinite(settings.positionTolerance) || settings.positionTolerance <= 0.0) {
        throw std::invalid_argument("Invalid IK target, settings, or empty robot arm");
    }

    for (const auto& joint : joints) {
        if (!std::isfinite(joint.angularVelocity)) {
            throw std::invalid_argument("Non-finite joint angular velocity");
        }
    }

    // Use the shared FK + Jacobian implementation, so all configurations
    // use precisely the same coordinate and angle conventions.
    const auto positions = forwardKinematics(joints);
    const math::Vec2 error = target - positions.back();
    if (!finite(error)) {
        throw std::overflow_error("IK positional error overflow");
    }

    const double distance = error.length();
    if (!std::isfinite(distance)) {
        throw std::overflow_error("IK positional error magnitude overflow");
    }

    IKCommand2D command{std::vector<double>(joints.size(), 0.0),
                        distance,
                        distance <= settings.positionTolerance};
    if (command.reached) {
        return command;
    }

    const auto columns = endEffectorJacobian(positions);

    // Damped least squares for the 2 x N translational Jacobian:
    // q_dot = J^T (J J^T + lambda^2 I)^-1 * (gain * positionError).
    // Only a 2 x 2 system is inverted, regardless of the joint count.
    double xx = settings.damping * settings.damping;
    double xy = 0.0;
    double yy = xx;
    for (const auto& column : columns) {
        xx += column.x * column.x;
        xy += column.x * column.y;
        yy += column.y * column.y;
    }

    const double determinant = xx * yy - xy * xy;
    const math::Vec2 desiredVelocity = error * settings.positionGain;
    if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yy) ||
        !std::isfinite(determinant) || determinant <= 0.0 ||
        !finite(desiredVelocity)) {
        throw std::overflow_error("IK normal equations are numerically invalid");
    }

    // Solve (JJ^T + lambda^2 I) w = desiredVelocity.
    const math::Vec2 w{(yy * desiredVelocity.x - xy * desiredVelocity.y) / determinant,
                       (xx * desiredVelocity.y - xy * desiredVelocity.x) / determinant};
    if (!finite(w)) {
        throw std::overflow_error("IK normal equation solution overflow");
    }

    double peakSpeed = 0.0;
    for (std::size_t i = 0; i < columns.size(); ++i) {
        const double speed = columns[i].dot(w);
        if (!std::isfinite(speed)) {
            throw std::overflow_error("IK angular velocity overflow");
        }
        command.angularVelocities[i] = speed;
        peakSpeed = std::max(peakSpeed, std::abs(speed));
    }

    // Uniform scaling preserves the direction of the joint velocity vector.
    if (peakSpeed > settings.maxAngularSpeed) {
        const double scale = settings.maxAngularSpeed / peakSpeed;
        for (double& speed : command.angularVelocities) {
            speed *= scale;
        }
    }

    return command;
}

} // namespace robotics::kinematics
