#pragma once

#include "robotics/kinematics/PathPlanning2D.hpp"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace robotics::kinematics {

struct AsyncPlanResult2D {
    std::uint64_t requestId{};
    math::Vec2 target{};
    PlannedJointPath2D plan;
    std::string error; // invalid configuration or unexpected solver exception
};

// One worker, one pending request. New inputs supersede prior searches;
// the worker observes a stop token inside IK and RRT. All scene data is
// copied before leaving the GUI thread. No graphics API is used here.
class AsyncPlanner2D {
public:
    AsyncPlanner2D();
    ~AsyncPlanner2D();

    AsyncPlanner2D(const AsyncPlanner2D&) = delete;
    AsyncPlanner2D& operator=(const AsyncPlanner2D&) = delete;

    [[nodiscard]] std::uint64_t submit(
        std::span<const Joint2D> joints,
        std::span<const JointLimits2D> limits,
        math::Vec2 target,
        const MotionSettings2D& settings,
        std::span<const geometry::Obstacle2D> obstacles = {});

    void cancel();
    [[nodiscard]] bool busy() const;
    [[nodiscard]] std::optional<AsyncPlanResult2D> takeResult();

private:
    struct Request {
        std::uint64_t id;
        std::vector<Joint2D> joints;
        std::vector<JointLimits2D> limits;
        math::Vec2 target;
        MotionSettings2D settings;
        std::vector<geometry::Obstacle2D> obstacles;
        std::stop_token stop;
    };

    void run();

    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::optional<Request> pending_;
    std::optional<AsyncPlanResult2D> completed_;
    std::stop_source cancelCurrent_;
    std::uint64_t requestId_{};
    bool busy_{};
    bool closing_{};
    // The worker can enter run() as soon as its constructor starts it.
    std::jthread worker_;
};

} // namespace robotics::kinematics
