#pragma once

#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/Joint2D.hpp"
#include "robotics/kinematics/TargetObject2D.hpp"
#include "robotics/math/Vec2.hpp"

#include <cstddef>
#include <deque>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace robotics::diagnostics {

enum class PlannerState2D {
    Idle, Planning, Moving, Paused, Reached, Blocked, NoPath
};

[[nodiscard]] const char* plannerStateName(PlannerState2D state) noexcept;

struct TelemetryFrame2D {
    double time{};
    std::vector<kinematics::Joint2D> joints;
    math::Vec2 target{};
    std::vector<geometry::Obstacle2D> obstacles;
    std::vector<double> angularAccelerations; // radians / second^2
    double cartesianError{};                  // metres
    double jacobianCondition{};               // +infinity = singular
    PlannerState2D plannerState{PlannerState2D::Idle};
    std::optional<kinematics::TargetObject2D> targetObject;
};

// Bounded, deterministic diagnostic recording. No raylib dependencies.
// Saving/reloading snapshots preserves the exact joint and scene geometry
// for visualization/replay without rerunning the nondeterministic planner.
class TelemetryRecorder2D {
public:
    explicit TelemetryRecorder2D(std::size_t capacity = 3600);

    void record(double time,
                std::span<const kinematics::Joint2D> joints,
                math::Vec2 target,
                PlannerState2D state,
                std::span<const geometry::Obstacle2D> obstacles = {});
    void record(double time,
                std::span<const kinematics::Joint2D> joints,
                math::Vec2 target,
                PlannerState2D state,
                std::span<const geometry::Obstacle2D> obstacles,
                const std::optional<kinematics::TargetObject2D>& targetObject);

    void clear() noexcept { frames_.clear(); }
    [[nodiscard]] bool empty() const noexcept { return frames_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return frames_.size(); }
    [[nodiscard]] const std::deque<TelemetryFrame2D>& frames() const noexcept {
        return frames_;
    }
    [[nodiscard]] const TelemetryFrame2D& at(std::size_t i) const {
        return frames_.at(i);
    }

    // Simple versioned text format; paths are provided by the caller.
    void save(const std::string& path) const;
    // Atomic replacement: an invalid file leaves the previous recording intact.
    void load(const std::string& path);

private:
    std::size_t capacity_;
    std::deque<TelemetryFrame2D> frames_;
};

} // namespace robotics::diagnostics
