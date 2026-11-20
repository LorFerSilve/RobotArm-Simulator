#pragma once

#include "robotics/kinematics/SmoothMotion2D.hpp"

#include <span>
#include <vector>

namespace robotics::kinematics {

// Fixed robot geometry and settings, separate from its changing joint state.
struct JointDefinition2D {
    double length;       // metres
    double initialAngle; // radians, relative to preceding link
    JointLimits2D limits;
};

struct Robot2D {
    std::vector<Joint2D> joints;
    std::vector<JointLimits2D> limits;
};

// Validates angles, motion limits, clearances and initial self-collisions.
// Throws std::invalid_argument for an invalid robot definition.
[[nodiscard]] Robot2D makeRobot2D(
    std::span<const JointDefinition2D> definition,
    const MotionSettings2D& settings = {});

} // namespace robotics::kinematics
