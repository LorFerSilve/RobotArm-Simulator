#include "robotics/kinematics/AsyncPlanner2D.hpp"

#include <exception>
#include <utility>

namespace robotics::kinematics {

AsyncPlanner2D::AsyncPlanner2D() : worker_([this] { run(); }) {}

AsyncPlanner2D::~AsyncPlanner2D() {
    {
        std::lock_guard lock(mutex_);
        closing_ = true;
        cancelCurrent_.request_stop();
        pending_.reset();
    }
    ready_.notify_one();
    worker_.join();
}

std::uint64_t AsyncPlanner2D::submit(
    std::span<const Joint2D> joints, std::span<const JointLimits2D> limits,
    math::Vec2 target, const MotionSettings2D& settings,
    std::span<const geometry::Obstacle2D> obstacles) {
    std::uint64_t id;
    {
        std::lock_guard lock(mutex_);
        cancelCurrent_.request_stop();
        cancelCurrent_ = std::stop_source{};
        id = ++requestId_;
        pending_ = Request{
            id, {joints.begin(), joints.end()}, {limits.begin(), limits.end()},
            target, settings, {obstacles.begin(), obstacles.end()},
            cancelCurrent_.get_token()
        };
        completed_.reset();
        busy_ = true;
    }
    ready_.notify_one();
    return id;
}

void AsyncPlanner2D::cancel() {
    {
        std::lock_guard lock(mutex_);
        cancelCurrent_.request_stop();
        ++requestId_;
        pending_.reset();
        completed_.reset();
        busy_ = false;
    }
    ready_.notify_one();
}

bool AsyncPlanner2D::busy() const {
    std::lock_guard lock(mutex_);
    return busy_;
}

std::optional<AsyncPlanResult2D> AsyncPlanner2D::takeResult() {
    std::lock_guard lock(mutex_);
    auto result = std::move(completed_);
    completed_.reset();
    return result;
}

void AsyncPlanner2D::run() {
    for (;;) {
        Request request{};
        {
            std::unique_lock lock(mutex_);
            ready_.wait(lock, [this] { return closing_ || pending_.has_value(); });
            if (closing_) return;
            request = std::move(*pending_);
            pending_.reset();
        }

        AsyncPlanResult2D result;
        result.requestId = request.id;
        result.target = request.target;
        try {
            result.plan = planJointPath2D(
                request.joints, request.limits, request.target,
                request.settings, request.obstacles, request.stop);
        } catch (const std::exception& ex) {
            result.error = ex.what();
        } catch (...) {
            result.error = "Unexpected error during path planning";
        }

        {
            std::lock_guard lock(mutex_);
            // An obsolete job cannot publish a plan for a newer scene/target.
            if (!closing_ && request.id == requestId_ &&
                !request.stop.stop_requested()) {
                completed_ = std::move(result);
                busy_ = false;
            }
        }
    }
}

} // namespace robotics::kinematics
