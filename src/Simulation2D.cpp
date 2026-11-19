#include "robotics/simulation/Simulation2D.hpp"

#include <cmath>
#include <stdexcept>

namespace robotics::simulation {

void advanceJoints(std::span<kinematics::Joint2D> joints, double dt) {
    if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::invalid_argument("Timestep must be positive and finite");
    }

    // Validate the entire step first, avoiding partially updated robot state.
    for (const auto& joint : joints) {
        if (!std::isfinite(joint.length) || joint.length <= 0.0 ||
            !std::isfinite(joint.angle) || !std::isfinite(joint.angularVelocity)) {
            throw std::invalid_argument("Invalid joint parameters");
        }
        if (!std::isfinite(joint.angle + joint.angularVelocity * dt)) {
            throw std::overflow_error("Joint angle overflow during simulation step");
        }
    }

    for (auto& joint : joints) {
        joint.angle += joint.angularVelocity * dt;
    }
}

} // namespace robotics::simulation
