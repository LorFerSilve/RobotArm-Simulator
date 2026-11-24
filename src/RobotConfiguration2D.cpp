#include "robotics/kinematics/RobotConfiguration2D.hpp"

#include "robotics/geometry/LinkRadii2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace robotics::kinematics {

Robot2D makeRobot2D(std::span<const JointDefinition2D> definition,
                    const MotionSettings2D& settings) {
    if (definition.size() < 2 || !std::isfinite(settings.jointClearance) ||
        settings.jointClearance <= 0.0 ||
        settings.jointClearance >= std::numbers::pi / 2.0 ||
        !std::isfinite(settings.foldClearance) ||
        settings.foldClearance < settings.jointClearance ||
        settings.foldClearance >= std::numbers::pi / 2.0)
        throw std::invalid_argument("Robot needs at least two joints and valid clearance");

    Robot2D result;
    result.joints.reserve(definition.size());
    result.limits.reserve(definition.size());

    for (std::size_t i = 0; i < definition.size(); ++i) {
        const auto& part = definition[i];
        const auto& bounds = part.limits;
        if (!std::isfinite(part.length) || part.length <= 0.0 ||
            !geometry::validLinkRadius(part.linkRadius) ||
            !std::isfinite(part.initialAngle) ||
            !std::isfinite(bounds.minAngle) || !std::isfinite(bounds.maxAngle) ||
            bounds.minAngle >= bounds.maxAngle ||
            !std::isfinite(bounds.maxAngularSpeed) || bounds.maxAngularSpeed <= 0.0 ||
            !std::isfinite(bounds.maxAngularAcceleration) ||
            bounds.maxAngularAcceleration <= 0.0 ||
            part.initialAngle < bounds.minAngle || part.initialAngle > bounds.maxAngle)
            throw std::invalid_argument("Invalid link geometry, initial angle or joint limits");

        // Stay within the selected elbow branch; self-folding would allow
        // adjacent links to overlap and invalidate the current controller.
        if (i > 0) {
            const double m = settings.jointClearance;
            const bool positive = bounds.minAngle >= m &&
                                  bounds.maxAngle <= std::numbers::pi - settings.foldClearance;
            const bool negative = bounds.minAngle >= -std::numbers::pi + settings.foldClearance &&
                                  bounds.maxAngle <= -m;
            if (!positive && !negative)
                throw std::invalid_argument("Internal joint limits must stay on one safe elbow branch");
        }
        result.joints.push_back({part.length, part.initialAngle, 0.0, part.linkRadius});
        result.limits.push_back(bounds);
    }

    if (hasSelfCollision(forwardKinematics(result.joints), settings.linkClearance,
                         linkRadii2D(result.joints)))
        throw std::invalid_argument("Initial robot configuration has intersecting links");
    return result;
}

} // namespace robotics::kinematics
