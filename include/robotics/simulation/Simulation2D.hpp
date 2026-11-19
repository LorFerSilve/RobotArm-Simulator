#pragma once

#include "robotics/kinematics/Joint2D.hpp"

#include <span>

namespace robotics::simulation {

// Advance all joint angles by one fixed timestep (seconds).
// Angular velocities are constant during this step; no dynamics are simulated.
// Throws without modifying any joint if the inputs or resulting angles are invalid.
void advanceJoints(std::span<kinematics::Joint2D> joints, double dt);

} // namespace robotics::simulation
