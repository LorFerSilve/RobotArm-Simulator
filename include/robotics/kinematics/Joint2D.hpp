#pragma once

namespace robotics::kinematics {

struct Joint2D {
    double length;           // Length of the link in metres.
    double angle;            // Relative rotation in radians.
    double angularVelocity{}; // Angular velocity in radians per second.
};

} // namespace robotics::kinematics
