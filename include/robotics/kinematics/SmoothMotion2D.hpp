#pragma once

#include "robotics/kinematics/Joint2D.hpp"
#include "robotics/kinematics/TargetObject2D.hpp"
#include "robotics/math/Vec2.hpp"

#include <optional>
#include <span>
#include <vector>

namespace robotics::kinematics {

struct MotionSettings2D {
    double maxAngularSpeed{2.0};         // rad/s
    double maxAngularAcceleration{6.0}; // rad/s^2
    double jointClearance{0.14};        // radians from straight/folded singular poses
    double linkClearance{0.10};         // gap between nonadjacent link surfaces, m
    double foldClearance{0.14};        // minimum angle away from a complete fold
    double targetClearance{0.0};       // optional target-object radius, metres
    double obstacleClearance{0.10};    // gap from link surfaces to scene obstacles, m
    double positionTolerance{0.005};  // metres
    double maxAngularJerk{120.0};       // rad/s^3, global planned-motion limit
    // Explicit target geometry and physical terminal tool section. When
    // omitted, existing targetClearance behavior is preserved for legacy APIs.
    std::optional<TargetObject2D> targetObject{};
};

// Per-joint constraints. The shoulder may rotate through zero. Planning
// restricts the other joints to the interval of their starting branch.
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
// the existing Jacobian DLS solver. Explicit targets use swept shaft checks.
// Scene obstacles and dynamic forces are outside this controller's scope.
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

// Reuse caller-owned per-link storage in the planner's inner collision loop.
// The buffer must have positions.size() - 1 entries.
void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, double clearance,
                            std::span<double> depths);
void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, double clearance,
                            std::span<double> depths, std::span<const double> linkRadii);

// The target represents a small physical object. Links except the tool-end
// must keep clearance; the final link can approach within the tool-tip zone.
// Clearance zero disables the target-object check for legacy simulations.
[[nodiscard]] bool targetApproachClear(std::span<const math::Vec2> positions,
                                       math::Vec2 target, double clearance);
[[nodiscard]] bool targetApproachClear(std::span<const math::Vec2> positions,
                                       math::Vec2 target, double clearance,
                                       std::span<const double> linkRadii);


// Explicit target-object contact geometry, independent from the legacy
// point/radius overloads above. The last link has a specified distal tool
// length; only that portion may contact the target body. The entire
// protected shaft is checked, including tangency at the clearance boundary.
void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, const TargetObject2D& object,
                            std::span<double> depths);
void targetLinkPenetrations(std::span<const math::Vec2> positions,
                            math::Vec2 target, const TargetObject2D& object,
                            std::span<double> depths, std::span<const double> linkRadii);
[[nodiscard]] std::vector<double> targetLinkPenetrations(
    std::span<const math::Vec2> positions, math::Vec2 target,
    const TargetObject2D& object);
[[nodiscard]] bool targetApproachClear(
    std::span<const math::Vec2> positions, math::Vec2 target,
    const TargetObject2D& object);
[[nodiscard]] bool targetApproachClear(
    std::span<const math::Vec2> positions, math::Vec2 target,
    const TargetObject2D& object, std::span<const double> linkRadii);

[[nodiscard]] bool hasSelfCollision(std::span<const math::Vec2> positions,
                                    double clearance);
// Nonadjacent capsules collide if their combined radii and the gap reach
// the centreline distance. Adjacent links share a pivot and use angle limits.
[[nodiscard]] bool hasSelfCollision(std::span<const math::Vec2> positions,
                                    double clearance, std::span<const double> linkRadii);

// Validate radii and extract them once for a series of collision queries.
// Returns an empty vector for a centreline arm.
[[nodiscard]] std::vector<double> linkRadii2D(std::span<const Joint2D> joints);

} // namespace robotics::kinematics
