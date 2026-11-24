#include "robotics/diagnostics/Telemetry2D.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace robotics::diagnostics;
using robotics::geometry::Box2D;
using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::kinematics::Joint2D;
using robotics::kinematics::TargetObject2D;
using robotics::kinematics::TargetShape2D;
using robotics::kinematics::jacobianConditionNumber2D;
using robotics::math::Vec2;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    try {
        const std::vector<Vec2> identity{{1, 0}, {0, 1}};
        require(std::abs(jacobianConditionNumber2D(identity) - 1.0) < 1e-12,
                "Orthogonal equal-norm columns have condition 1");
        require(std::abs(jacobianConditionNumber2D(
            std::vector<Vec2>{{1, 0}, {0, 1}, {1, 0}}) - std::sqrt(2.0)) < 1e-12,
                "Condition number works for non-square Jacobians");
        require(std::isinf(jacobianConditionNumber2D(
            std::vector<Vec2>{{1, 0}, {2, 0}})),
            "Collinear columns must be singular");
        require(std::isinf(jacobianConditionNumber2D(std::vector<Vec2>{})),
                "Empty Jacobian is singular");
        bool rejectedNaN = false;
        try {
            (void)jacobianConditionNumber2D(
                std::vector<Vec2>{{std::numeric_limits<double>::quiet_NaN(), 0}});
        } catch (const std::invalid_argument&) {
            rejectedNaN = true;
        }
        require(rejectedNaN, "Condition number rejects NaN");

        std::vector<Joint2D> joints{
            {2.0, std::numbers::pi/2, 0.0},
            {1.0, -std::numbers::pi/2, 0.0}
        };
        const std::vector<Obstacle2D> scene{
            Circle2D{{3.0, 0.0}, 0.4},
            Box2D{{-2.0, 1.5}, 0.6, 0.2}
        };
        TelemetryRecorder2D recording(3);
        TargetObject2D circleTarget;
        circleTarget.radius = 0.22;
        circleTarget.clearance = 0.04;
        TargetObject2D rectangleTarget = circleTarget;
        rectangleTarget.shape = TargetShape2D::Rectangle;
        rectangleTarget.halfWidth = 0.31;
        rectangleTarget.halfHeight = 0.19;
        rectangleTarget.toolLength = 0.48;
        recording.record(0.0, joints, {1.0, 2.0},
                         PlannerState2D::Idle, scene);
        require(recording.size() == 1 &&
                recording.at(0).cartesianError < 1e-12,
                "Recorder must derive Cartesian error from actual FK");
        require(recording.at(0).angularAccelerations[0] == 0.0,
                "First sample has no acceleration reference");
        joints[0].angularVelocity = 0.2;
        joints[0].linkRadius = 0.13;
        joints[1].linkRadius = 0.02;
        recording.record(0.1, joints, {1.1, 2.1},
                         PlannerState2D::Moving, scene, circleTarget);
        require(std::abs(recording.at(1).angularAccelerations[0] - 2.0) < 1e-12,
                "Angular acceleration is measured from successive velocities");
        require(recording.at(1).plannerState == PlannerState2D::Moving,
                "Planner status must be recorded");

        joints[0].angle += 0.1;
        joints[0].linkRadius = 0.22;
        recording.record(0.2, joints, {1.3, 2.2},
                         PlannerState2D::Paused, scene, rectangleTarget);
        recording.record(0.3, joints, {1.5, 2.1},
                         PlannerState2D::Reached, scene);
        require(recording.size() == 3 &&
                std::abs(recording.at(0).time - 0.1) < 1e-12,
                "Bounded recorder evicts oldest entries");

        const char* filename = "robotarm-telemetry-test.rarm";
        const char* badFile = "robotarm-telemetry-invalid.rarm";
        recording.save(filename);
        TelemetryRecorder2D loaded(3);
        loaded.load(filename);
        require(loaded.size() == 3, "Loaded recording must preserve frame count");
        for (std::size_t i = 0; i < loaded.size(); ++i) {
            const auto& a = recording.at(i);
            const auto& b = loaded.at(i);
            require(a.time == b.time && a.target.x == b.target.x &&
                    a.target.y == b.target.y &&
                    a.plannerState == b.plannerState &&
                    a.angularAccelerations == b.angularAccelerations &&
                    a.joints.size() == b.joints.size() &&
                    a.obstacles.size() == b.obstacles.size(),
                    "Replay must preserve timing, targets, states and scene counts");
            require(a.targetObject.has_value() == b.targetObject.has_value(),
                    "Replay must preserve explicit target presence per frame");
            if (a.targetObject) {
                const auto& original = *a.targetObject;
                const auto& restored = *b.targetObject;
                require(original.shape == restored.shape &&
                        original.radius == restored.radius &&
                        original.halfWidth == restored.halfWidth &&
                        original.halfHeight == restored.halfHeight &&
                        original.clearance == restored.clearance &&
                        original.toolLength == restored.toolLength,
                        "Replay must preserve the target geometry and tool section");
            }
            for (std::size_t j = 0; j < a.joints.size(); ++j)
                require(a.joints[j].length == b.joints[j].length &&
                        a.joints[j].angle == b.joints[j].angle &&
                        a.joints[j].angularVelocity == b.joints[j].angularVelocity &&
                        a.joints[j].linkRadius == b.joints[j].linkRadius,
                        "Replay must preserve exact joint states");
        }
        require(std::get<Circle2D>(loaded.at(0).obstacles[0]).radius == 0.4,
                "Circle obstacle survives replay");
        require(std::get<Box2D>(loaded.at(0).obstacles[1]).halfWidth == 0.6,
                "Rectangle obstacle survives replay");

        {
            std::ofstream legacy(badFile);
            legacy << "RARM2D 2 1\n0 0 0 1 2 0\n1 0 0 0\n2 1 0 0\n";
        }
        TelemetryRecorder2D legacy;
        legacy.load(badFile);
        require(legacy.size() == 1 && !legacy.at(0).targetObject &&
                    legacy.at(0).joints[0].linkRadius == 0.0,
                "Version 2 remains readable with centreline links");
        {
            std::ofstream previous(badFile);
            previous << "RARM2D 3 1\n0 0 0 1 2 0\n1 0 0 0\n2 1 0 0\n"
                        "B 0.2 0.1 0.15 0.02 0.3\n";
        }
        legacy.load(badFile);
        require(legacy.at(0).targetObject &&
                    legacy.at(0).targetObject->shape == TargetShape2D::Rectangle &&
                    legacy.at(0).joints[1].linkRadius == 0.0,
                "Version 3 keeps its target geometry and centreline links");
        {
            std::ofstream invalid(badFile);
            invalid << "RARM2D 3 1\n0 0 0 1 2 0\n"
                       "1 0 0 0\n2 1 0 0\nB 0.2 0.1 0.15 0.02 2.1\n";
        }
        bool rejected = false;
        try { loaded.load(badFile); }
        catch (const std::exception&) { rejected = true; }
        require(rejected && loaded.size() == 3,
                "Corrupt load must leave existing recording unchanged");

        {
            std::ofstream invalid(badFile);
            invalid << "RARM2D 4 1\n0 0 0 1 2 0\n"
                       "1 0 0 0 -0.1\n2 1 0 0 0.2\nN\n";
        }
        rejected = false;
        try { loaded.load(badFile); }
        catch (const std::exception&) { rejected = true; }
        require(rejected && loaded.size() == 3,
                "Invalid radii must leave the previous recording intact");

        rejected = false;
        try {
            loaded.record(0.3, joints, {1.0, 1.0}, PlannerState2D::Moving);
        } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Non-increasing timestamps rejected");

        std::remove(filename);
        std::remove(badFile);
        std::cout << "Diagnostic history, Jacobian condition and file replay passed\n";
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
