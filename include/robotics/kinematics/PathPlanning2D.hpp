#pragma once

#include "robotics/kinematics/SmoothMotion2D.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace robotics::kinematics {

struct PlannedJointPath2D {
    // Includes the initial configuration and the terminal IK configuration.
    // Empty if a collision-free solution was not found within the search budget.
    std::vector<std::vector<double>> waypoints;
    double terminalError{};

    [[nodiscard]] bool found() const noexcept { return !waypoints.empty(); }
};

// Search in joint configuration space, not just end-effector space.
// Deterministic, bounded multi-start IK followed by bidirectional RRT-Connect.
// There is no guarantee that a valid path will be found within the budget.
[[nodiscard]] PlannedJointPath2D planJointPath2D(
    std::span<const Joint2D> joints,
    std::span<const JointLimits2D> limits,
    math::Vec2 target,
    const MotionSettings2D& settings = {});

// Stateful path follower: execute planned joint-space waypoints with bounded
// joint velocity and acceleration, checking collisions before each step.
class PlannedMotion2D {
public:
    void reset() noexcept;
    void setTarget(std::span<const Joint2D> joints,
                   std::span<const JointLimits2D> limits,
                   math::Vec2 target,
                   const MotionSettings2D& settings = {});
    [[nodiscard]] MotionStatus2D advance(std::span<Joint2D> joints,
                                          std::span<const JointLimits2D> limits,
                                          double dt,
                                          const MotionSettings2D& settings = {});
    [[nodiscard]] bool pathAvailable() const noexcept { return !path_.empty(); }
    [[nodiscard]] bool searchFailed() const noexcept { return failed_; }
    [[nodiscard]] bool hasTarget() const noexcept { return hasTarget_; }
    [[nodiscard]] math::Vec2 target() const noexcept { return target_; }

private:
    std::vector<std::vector<double>> path_;
    std::size_t index_{};
    math::Vec2 target_{};
    bool hasTarget_{};
    bool failed_{};
};

} // namespace robotics::kinematics
