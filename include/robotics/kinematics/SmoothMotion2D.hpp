#pragma once

#include "robotics/kinematics/Joint2D.hpp"
#include "robotics/math/Vec2.hpp"

#include <span>
#include <vector>

namespace robotics::kinematics {

struct MotionSettings2D {
    double maxAngularSpeed{2.0};         // rad/s
    double maxAngularAcceleration{6.0}; // rad/s^2
    double jointClearance{0.14};        // radians from straight/folded singular poses
    double linkClearance{0.10};         // metres between nonadjacent links
    double foldClearance{0.14};        // minimum angle away from a complete fold
    double targetClearance{0.0};       // optional target-object radius, metres
    double obstacleClearance{0.10};    // robot half-width + obstacle safety margin, m
    double positionTolerance{0.005};  // metres
    double maxAngularJerk{120.0};       // rad/s^3, global spline limit
};

// Per-joint constraints. The first joint may rotate through zero; for each
// remaining joint, min/max must stay within a single non-collinear branch.
struct JointLimits2D {
    double minAngle;               // radians
    double maxAngle;               // radians
    double maxAngularSpeed;        // rad/s
    double maxAngularAcceleration; // rad/s^2
};

struct MotionStatus2D {
    double positionError{}; // distance to the originally requested target, m
    bool reached{};
    bool blocked{};  // cannot take a safe step toward target
    bool constrained{}; // collision/joint constraints reduced the commanded step
    bool outOfReach{};
};

// One collision-checked, acceleration-limited fixed timestep. Mutates the arm.
// Two links use analytic IK with a preserved elbow branch; three or more use
// the existing Jacobian DLS solver. No dynamic forces or obstacle models.
// Optional limits are checked per joint. Empty limits preserve legacy behaviour.
[[nodiscard]] MotionStatus2D advanceTowardTarget(
    std::span<Joint2D> joints, math::Vec2 target, double dt,
    const MotionSettings2D& settings = {},
    std::span<const JointLimits2D> limits = {});

// Per-link penetration depth into the target keep-out zone (metres).
// Zero means that link does not penetrate the protected zone. For the last
// link, the tool-tip allowance is accounted for here.
[[nodiscard]] std::vector<double> targetLinkPenetrations(
    std::span<const math::Vec2> positions, math::Vec2 target, double clearance);

// The target represents a small physical object. Links except the tool-end
// must keep clearance; the final link can approach within the tool-tip zone.
// Clearance zero disables the target-object check for legacy simulations.
[[nodiscard]] bool targetApproachClear(std::span<const math::Vec2> positions,
                                       math::Vec2 target, double clearance);

[[nodiscard]] bool hasSelfCollision(std::span<const math::Vec2> positions,
                                    double clearance);

} // namespace robotics::kinematics
