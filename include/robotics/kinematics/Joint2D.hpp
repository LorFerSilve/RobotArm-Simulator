#pragma once

namespace robotics::kinematics {

struct Joint2D {
    double length; // Length of the link in metres.
    double angle;  // Rotation relative to the previous link, in radians.
};

} // namespace robotics::kinematics
