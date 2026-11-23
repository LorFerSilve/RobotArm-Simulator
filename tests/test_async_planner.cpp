#include "robotics/kinematics/AsyncPlanner2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::kinematics::AsyncPlanner2D;
using robotics::kinematics::AsyncPlanResult2D;
using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::PlanningFailure2D;
using robotics::kinematics::planJointPath2D;

void require(bool ok, const char* reason) {
    if (!ok) throw std::runtime_error(reason);
}

std::optional<AsyncPlanResult2D> waitForResult(AsyncPlanner2D& planner) {
    const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (std::chrono::steady_clock::now() < timeout) {
        if (auto result = planner.takeResult()) return result;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return std::nullopt;
}

int main() {
    try {
        constexpr double pi = std::numbers::pi;
        const std::vector<Joint2D> arm{
            {2.0, pi / 2.0, 0.0}, {1.0, -pi / 2.0, 0.0}
        };
        const std::vector<JointLimits2D> limits{
            {-2*pi, 2*pi, 2.0, 6.0},
            {-pi + 0.56, -0.16, 2.0, 6.0}
        };

        std::stop_source cancelled;
        cancelled.request_stop();
        const auto stopped = planJointPath2D(
            arm, limits, {1.5, 1.5}, {}, {}, cancelled.get_token());
        require(!stopped.found() && stopped.failure == PlanningFailure2D::Cancelled,
                "Cancelled planner must not publish a path");

        const auto unreachable = planJointPath2D(arm, limits, {10.0, 0.0});
        require(unreachable.failure == PlanningFailure2D::OutOfReach,
                "Out-of-reach target must have a specific diagnosis");

        const std::vector<Obstacle2D> obstacle{Circle2D{{1.5, 1.5}, 0.25}};
        const auto blocked = planJointPath2D(arm, limits, {1.5, 1.5}, {}, obstacle);
        require(blocked.failure == PlanningFailure2D::GoalInObstacle,
                "Solid target obstruction must have a specific diagnosis");

        const auto currentTip = robotics::kinematics::forwardKinematics(arm).back();
        for (int attempt = 0; attempt < 32; ++attempt) {
            AsyncPlanner2D fresh;
            require(!fresh.busy() && !fresh.takeResult(),
                    "A new worker must start with an empty queue");
            const auto firstId = fresh.submit(arm, limits, currentTip, {});
            const auto first = waitForResult(fresh);
            require(first && first->requestId == firstId &&
                        first->error.empty() && first->plan.found(),
                    "A newly started worker must process its first request");
            // Destruction must also join a worker with a pending or active job.
            (void)fresh.submit(arm, limits, {1.5, 1.5}, {});
        }

        AsyncPlanner2D background;
        const auto obsolete = background.submit(arm, limits, {1.5, 1.5}, {});
        const auto current = background.submit(arm, limits, {1.0, 2.0}, {});
        require(current > obsolete, "Request IDs should be monotonically increasing");

        const auto result = waitForResult(background);
        require(result.has_value(), "Background planning did not return a result");
        require(result->requestId == current,
                "Late result of a superseded request was published");
        require(result->error.empty() && result->plan.found(),
                "Latest background request should produce a path");
        require((result->target - robotics::math::Vec2{1.0, 2.0}).length() < 1e-9,
                "Latest target was not preserved");
        require(!background.busy(), "Planner busy flag not cleared");

        (void)background.submit(arm, limits, {1.5, 1.5}, {});
        background.cancel();
        require(!background.busy() && !background.takeResult().has_value(),
                "Cancelling must clear stale state");
        std::cout << "Asynchronous planning, cancellation and diagnostics passed\n";
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
