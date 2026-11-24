#include "robotics/diagnostics/Telemetry2D.hpp"

#include "robotics/geometry/LinkRadii2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robotics::diagnostics {
namespace {
bool validState(PlannerState2D state) noexcept {
    return state >= PlannerState2D::Idle && state <= PlannerState2D::NoPath;
}
} // namespace

const char* plannerStateName(PlannerState2D state) noexcept {
    switch (state) {
    case PlannerState2D::Idle: return "IDLE";
    case PlannerState2D::Planning: return "PLANNING";
    case PlannerState2D::Moving: return "MOVING";
    case PlannerState2D::Paused: return "PAUSED";
    case PlannerState2D::Reached: return "REACHED";
    case PlannerState2D::Blocked: return "BLOCKED";
    case PlannerState2D::NoPath: return "NO PATH";
    }
    return "UNKNOWN";
}

TelemetryRecorder2D::TelemetryRecorder2D(std::size_t capacity)
    : capacity_(capacity) {
    if (capacity == 0 || capacity > 100000)
        throw std::invalid_argument("Telemetry capacity must be 1..100000");
}

void TelemetryRecorder2D::record(
    double time, std::span<const kinematics::Joint2D> joints,
    math::Vec2 target, PlannerState2D state,
    std::span<const geometry::Obstacle2D> obstacles) {
    record(time, joints, target, state, obstacles, std::nullopt);
}

void TelemetryRecorder2D::record(
    double time, std::span<const kinematics::Joint2D> joints,
    math::Vec2 target, PlannerState2D state,
    std::span<const geometry::Obstacle2D> obstacles,
    const std::optional<kinematics::TargetObject2D>& targetObject) {
    if (!std::isfinite(time) || !std::isfinite(target.x) ||
        !std::isfinite(target.y) || !validState(state) ||
        joints.size() < 2 ||
        (!frames_.empty() && time <= frames_.back().time))
        throw std::invalid_argument("Invalid time, target or planner state");

    if (targetObject && (!kinematics::validTargetObject2D(*targetObject) ||
                         targetObject->toolLength >= joints.back().length))
        throw std::invalid_argument("Invalid recorded target geometry");
    for (const auto& obstacle : obstacles)
        if (!geometry::validObstacle(obstacle))
            throw std::invalid_argument("Invalid recorded obstacle");

    // FK validates link lengths and angles.
    for (const auto& joint : joints) {
        if (!std::isfinite(joint.angularVelocity))
            throw std::invalid_argument("Non-finite recorded joint velocity");
        if (!geometry::validLinkRadius(joint.linkRadius))
            throw std::invalid_argument("Invalid recorded link radius");
    }

    const auto points = kinematics::forwardKinematics(joints);
    const auto columns = kinematics::endEffectorJacobian(points);

    TelemetryFrame2D frame;
    frame.time = time;
    frame.target = target;
    frame.targetObject = targetObject;
    frame.plannerState = state;
    frame.joints.assign(joints.begin(), joints.end());
    frame.obstacles.assign(obstacles.begin(), obstacles.end());
    frame.cartesianError = (target - points.back()).length();
    frame.jacobianCondition = kinematics::jacobianConditionNumber2D(columns);
    if (!std::isfinite(frame.cartesianError))
        throw std::overflow_error("Cartesian error overflow");
    frame.angularAccelerations.assign(joints.size(), 0.0);

    if (!frames_.empty() && frames_.back().joints.size() == joints.size()) {
        const auto& previous = frames_.back();
        const double dt = time - previous.time;
        for (std::size_t i = 0; i < joints.size(); ++i) {
            frame.angularAccelerations[i] =
                (joints[i].angularVelocity -
                 previous.joints[i].angularVelocity) / dt;
            if (!std::isfinite(frame.angularAccelerations[i]))
                throw std::overflow_error("Angular acceleration overflow");
        }
    }

    if (frames_.size() == capacity_) frames_.pop_front();
    frames_.push_back(std::move(frame));
}

void TelemetryRecorder2D::save(const std::string& path) const {
    std::ofstream out(path, std::ios::trunc);
    if (!out) throw std::runtime_error("Could not open telemetry file for writing");
    out << std::setprecision(17) << "RARM2D 4 " << frames_.size() << '\n';
    for (const auto& frame : frames_) {
        out << frame.time << ' ' << frame.target.x << ' ' << frame.target.y
            << ' ' << static_cast<int>(frame.plannerState)
            << ' ' << frame.joints.size() << ' ' << frame.obstacles.size() << '\n';
        for (std::size_t i = 0; i < frame.joints.size(); ++i) {
            const auto& joint = frame.joints[i];
            out << joint.length << ' ' << joint.angle << ' '
                << joint.angularVelocity << ' '
                << frame.angularAccelerations[i] << ' ' << joint.linkRadius << '\n';
        }
        for (const auto& obstacle : frame.obstacles) {
            if (const auto* circle = std::get_if<geometry::Circle2D>(&obstacle)) {
                out << "C " << circle->center.x << ' ' << circle->center.y
                    << ' ' << circle->radius << '\n';
            } else {
                const auto& box = std::get<geometry::Box2D>(obstacle);
                out << "B " << box.center.x << ' ' << box.center.y
                    << ' ' << box.halfWidth << ' ' << box.halfHeight << '\n';
            }
        }
        if (const auto& object = frame.targetObject) {
            const char kind = object->shape == kinematics::TargetShape2D::Circle ? 'C' : 'B';
            out << kind << ' ' << object->radius << ' ' << object->halfWidth << ' '
                << object->halfHeight << ' ' << object->clearance << ' '
                << object->toolLength << '\n';
        } else {
            out << "N\n";
        }
    }
    if (!out) throw std::runtime_error("Could not write telemetry file");
}

void TelemetryRecorder2D::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Could not open telemetry file");

    std::string magic;
    int version = 0;
    std::size_t count = 0;
    if (!(in >> magic >> version >> count) ||
        magic != "RARM2D" || version < 2 || version > 4 || count > capacity_)
        throw std::runtime_error("Unsupported or oversized telemetry file");

    TelemetryRecorder2D parsed(capacity_);
    for (std::size_t frameIndex = 0; frameIndex < count; ++frameIndex) {
        double time = 0.0;
        math::Vec2 target{};
        int state = 0;
        std::size_t jointCount = 0, obstacleCount = 0;
        if (!(in >> time >> target.x >> target.y >>
              state >> jointCount >> obstacleCount) ||
            state < 0 || state > static_cast<int>(PlannerState2D::NoPath) ||
            jointCount < 2 || jointCount > 64 || obstacleCount > 64)
            throw std::runtime_error("Malformed telemetry frame");

        std::vector<kinematics::Joint2D> joints(jointCount);
        std::vector<double> accelerations(jointCount);
        for (std::size_t i = 0; i < jointCount; ++i) {
            auto& joint = joints[i];
            if (!(in >> joint.length >> joint.angle >>
                  joint.angularVelocity >> accelerations[i]) ||
                !std::isfinite(accelerations[i]))
                throw std::runtime_error("Malformed telemetry joint");
            if (version >= 4 && !(in >> joint.linkRadius))
                throw std::runtime_error("Missing telemetry link radius");
        }

        std::vector<geometry::Obstacle2D> obstacles;
        obstacles.reserve(obstacleCount);
        for (std::size_t i = 0; i < obstacleCount; ++i) {
            char kind = 0;
            math::Vec2 center{};
            if (!(in >> kind >> center.x >> center.y))
                throw std::runtime_error("Malformed telemetry obstacle");
            if (kind == 'C') {
                double radius = 0.0;
                if (!(in >> radius))
                    throw std::runtime_error("Malformed telemetry circle");
                obstacles.emplace_back(geometry::Circle2D{center, radius});
            } else if (kind == 'B') {
                double hw = 0.0, hh = 0.0;
                if (!(in >> hw >> hh))
                    throw std::runtime_error("Malformed telemetry box");
                obstacles.emplace_back(geometry::Box2D{center, hw, hh});
            } else {
                throw std::runtime_error("Unknown telemetry obstacle type");
            }
        }
        std::optional<kinematics::TargetObject2D> targetObject;
        if (version >= 3) {
            char kind = 0;
            if (!(in >> kind) || (kind != 'N' && kind != 'C' && kind != 'B'))
                throw std::runtime_error("Malformed telemetry target");
            if (kind != 'N') {
                auto& object = targetObject.emplace();
                object.shape = kind == 'C' ? kinematics::TargetShape2D::Circle :
                                             kinematics::TargetShape2D::Rectangle;
                if (!(in >> object.radius >> object.halfWidth >> object.halfHeight
                         >> object.clearance >> object.toolLength))
                    throw std::runtime_error("Malformed telemetry target geometry");
            }
        }
        parsed.record(time, joints, target, static_cast<PlannerState2D>(state),
                      obstacles, targetObject);
        parsed.frames_.back().angularAccelerations =
            std::move(accelerations);
    }
    // A trailing broken record must not turn a partially corrupt file into
    // a successful replay. Plain trailing whitespace is valid.
    in >> std::ws;
    if (!in.eof())
        throw std::runtime_error("Trailing data in telemetry file");

    frames_.swap(parsed.frames_);
}

} // namespace robotics::diagnostics
