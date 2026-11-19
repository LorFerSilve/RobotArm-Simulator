#pragma once

#include "robotics/kinematics/Joint2D.hpp"
#include "robotics/math/Vec2.hpp"

#include <numbers>
#include <span>
#include <vector>

namespace robotics::kinematics {

struct IKSettings2D {
    double positionGain{4.0};              // Proportional Cartesian control, 1/s.
    double damping{0.1};                   // DLS regularization, metres/radian.
    double maxAngularSpeed{std::numbers::pi}; // Joint speed limit, radians/s.
    double positionTolerance{0.005};       // Goal tolerance, metres.
};

struct IKCommand2D {
    std::vector<double> angularVelocities; // Radians/s, one command per joint.
    double positionError{};                // Metres.
    bool reached{};
};

// Calculate a damped least-squares inverse-kinematics velocity command.
// Does not change the robot state. Use Simulation2D::advanceJoints to
// integrate the returned angular velocities at a fixed timestep.
//
// The solution is local: it does not guarantee convergence from every
// configuration, particularly when the arm is exactly singular.
// Throws std::invalid_argument for malformed input and std::overflow_error
// if finite input produces non-finite intermediate computations.
[[nodiscard]] IKCommand2D computeIKCommand(
    std::span<const Joint2D> joints,
    math::Vec2 target,
    const IKSettings2D& settings = {});

} // namespace robotics::kinematics
