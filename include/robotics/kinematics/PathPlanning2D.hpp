#pragma once

#include "robotics/kinematics/SmoothMotion2D.hpp"
#include "robotics/geometry/Obstacles2D.hpp"

#include <cstddef>
#include <span>
#include <stop_token>
#include <vector>

namespace robotics::kinematics {

enum class PlanningFailure2D {
    None,
    Cancelled,
    OutOfReach,
    GoalInObstacle,
    NoIKSolution,
    PathSearchExhausted,
    TargetBlockedByJointLimits
};

[[nodiscard]] const char* planningFailureMessage(PlanningFailure2D reason) noexcept;

struct PlannedJointPath2D {
    // Includes the initial configuration and the terminal IK configuration.
    // Empty if a collision-free solution was not found within the search budget.
    std::vector<std::vector<double>> waypoints;
    double terminalError{};
    // Total Cartesian distance travelled by the end effector along this plan.
    // Computed from the same sampled trace shown by the GUI.
    double endEffectorDistance{};
    // Pre-validated C2 spline geometry, generated in the planning worker.
    // Empty for already-reached targets, otherwise size matches waypoints.
    std::vector<std::vector<double>> splineTangents;
    std::vector<double> pathDistances;
    double trajectoryDuration{};
    // End-effector positions sampled from the *actual* selected joint-space
    // trajectory (spline or verified polyline). Computed on the planner worker.
    std::vector<math::Vec2> endEffectorTrace;
    // Intermediate joint traces sampled at the same times as the tool trace.
    std::vector<std::vector<math::Vec2>> jointTraces;
    PlanningFailure2D failure{PlanningFailure2D::None};

    [[nodiscard]] bool found() const noexcept { return !waypoints.empty(); }
};

// Search in joint configuration space, not just end-effector space.
// Deterministic, bounded multi-start IK followed by bidirectional RRT-Connect.
// There is no guarantee that a valid path will be found within the budget.
[[nodiscard]] PlannedJointPath2D planJointPath2D(
    std::span<const Joint2D> joints,
    std::span<const JointLimits2D> limits,
    math::Vec2 target,
    const MotionSettings2D& settings = {},
    std::span<const geometry::Obstacle2D> obstacles = {},
    std::stop_token cancellation = {});

// Stateful path follower: execute planned joint-space waypoints with bounded
// joint velocity and acceleration, checking collisions before each step.
class PlannedMotion2D {
public:
    void reset() noexcept;
    // Install an already computed path without repeating IK/RRT on the UI thread.
    void usePlan(PlannedJointPath2D plan, math::Vec2 target);
    void setTarget(std::span<const Joint2D> joints,
                   std::span<const JointLimits2D> limits,
                   math::Vec2 target,
                   const MotionSettings2D& settings = {},
                   std::span<const geometry::Obstacle2D> obstacles = {});
    [[nodiscard]] MotionStatus2D advance(std::span<Joint2D> joints,
                                          std::span<const JointLimits2D> limits,
                                          double dt,
                                          const MotionSettings2D& settings = {},
                                          std::span<const geometry::Obstacle2D> obstacles = {});
    [[nodiscard]] bool pathAvailable() const noexcept { return !path_.empty(); }
    // Read-only drawing data; reset/new plan clears or replaces it.
    [[nodiscard]] double endEffectorDistance() const noexcept {
        return endEffectorDistance_;
    }
    [[nodiscard]] std::span<const math::Vec2> endEffectorTrace() const noexcept {
        return endEffectorTrace_;
    }
    [[nodiscard]] const std::vector<std::vector<math::Vec2>>&
    jointTraces() const noexcept { return jointTraces_; }
    [[nodiscard]] std::span<const double> finalJointAngles() const noexcept {
        return path_.empty() ? std::span<const double>{} :
                              std::span<const double>{path_.back()};
    }
    [[nodiscard]] bool searchFailed() const noexcept { return failed_; }
    [[nodiscard]] PlanningFailure2D failureReason() const noexcept { return failure_; }
    [[nodiscard]] bool hasTarget() const noexcept { return hasTarget_; }
    [[nodiscard]] math::Vec2 target() const noexcept { return target_; }

private:
    std::vector<std::vector<double>> path_;
    std::vector<math::Vec2> endEffectorTrace_;
    std::vector<std::vector<math::Vec2>> jointTraces_;
    double endEffectorDistance_{};
    std::size_t index_{};
    double segmentElapsed_{};
    double segmentDuration_{};
    std::vector<std::vector<double>> splineTangents_;
    std::vector<double> pathDistances_;
    double trajectoryDuration_{};
    double trajectoryElapsed_{};
    math::Vec2 target_{};
    bool hasTarget_{};
    bool failed_{};
    PlanningFailure2D failure_{PlanningFailure2D::None};
};

} // namespace robotics::kinematics
