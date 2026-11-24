#pragma once

namespace robotics::kinematics {

struct Joint2D {
    double length;           // Length of the link in metres.
    double angle;            // Relative rotation in radians.
    double angularVelocity{}; // Angular velocity in radians per second.
    double linkRadius{};      // Physical capsule radius in metres (0 = centreline).
};

} // namespace robotics::kinematics
