#pragma once

#include "robotics/kinematics/Joint2D.hpp"
#include "robotics/math/Vec2.hpp"

#include <span>
#include <vector>

namespace robotics::kinematics {

// Returns N + 1 points: the origin followed by every link endpoint.
// The base is fixed at (0, 0); joint angles are relative, in radians.
// Throws std::invalid_argument for invalid joint parameters and
// std::overflow_error if an intermediate result is not finite.
[[nodiscard]] std::vector<math::Vec2> forwardKinematics(
    std::span<const Joint2D> joints);

} // namespace robotics::kinematics
