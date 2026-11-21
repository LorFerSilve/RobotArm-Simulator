#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/TargetObject2D.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::geometry::Box2D;
using robotics::geometry::Circle2D;
using robotics::geometry::segmentSignedDistanceToObstacle;
using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::MotionSettings2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::TargetObject2D;
using robotics::kinematics::TargetShape2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::planJointPath2D;
using robotics::kinematics::targetApproachClear;
using robotics::kinematics::targetBody2D;
using robotics::kinematics::targetLinkPenetrations;
using robotics::kinematics::validTargetObject2D;
using robotics::math::Vec2;

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
} // namespace

int main() {
    try {
        const std::vector<Vec2> links{{0, 0}, {2, 0}, {4, 0}};
        TargetObject2D circle{};
        require(validTargetObject2D(circle), "Default target geometry valid");
        require(std::holds_alternative<Circle2D>(
                    targetBody2D(circle, {3.95, 0})),
                "Default target body is a circle");

        require(targetApproachClear(links, {3.95, 0}, circle),
                "Explicit terminal tool may contact a circular object");
        require(!targetApproachClear(links, {3.8, 0}, circle),
                "Final link's shaft must not contact the circular target");
        require(!targetApproachClear(links, {1.0, 0}, circle),
                "First link is never exempt from target contact");

        const auto shaftCollision =
            targetLinkPenetrations(links, Vec2{3.8, 0}, circle);
        require(shaftCollision.size() == 2 &&
                shaftCollision.back() > 0.0 &&
                shaftCollision.front() == 0.0,
                "Circular object reports penetration on the correct shaft");
        std::vector<double> reusedDepths(2);
        targetLinkPenetrations(links, {3.8, 0}, circle, reusedDepths);
        require(reusedDepths == shaftCollision,
                "New object uses same buffered-depth API");

        TargetObject2D rectangle = circle;
        rectangle.shape = TargetShape2D::Rectangle;
        rectangle.halfWidth = 0.10;
        rectangle.halfHeight = 0.08;
        require(std::holds_alternative<Box2D>(
                    targetBody2D(rectangle, {3.95, 0})),
                "Rectangle target body has explicit width and height");
        require(targetApproachClear(links, {3.95, 0}, rectangle),
                "Terminal tool may enter the rectangle contact zone");
        require(!targetApproachClear(links, {3.78, 0}, rectangle),
                "Final shaft must not penetrate rectangle");
        require(!targetApproachClear(links, {1.0, 0}, rectangle),
                "First link cannot pierce rectangle");

        // The signed distance is negative even when a segment crosses the
        // entire box with BOTH endpoints outside it. This makes retreat
        // profiling meaningful instead of a binary penetration flag.
        const Box2D box{{0, 0}, 0.4, 0.25};
        const double deep =
            segmentSignedDistanceToObstacle({-2, 0}, {2, 0}, box);
        const double shallow =
            segmentSignedDistanceToObstacle({-2, 0.20}, {2, 0.20}, box);
        const double clear =
            segmentSignedDistanceToObstacle({-2, 0.8}, {2, 0.8}, box);
        require(deep < -0.20 && shallow < 0.0 &&
                deep < shallow && clear > 0.0,
                "Signed rectangle distance distinguishes deep/shallow/clear");
        require(std::abs(
                segmentSignedDistanceToObstacle({-2, 0}, {2, 0},
                                               Circle2D{{0, 0}, 0.4}) + 0.4) <
                1e-12, "Circle signed depth is geometric");

        // Additional object/shaft clearance really increases exclusion.
        TargetObject2D expanded = circle;
        expanded.clearance = 0.12;
        require(!targetApproachClear(links, {3.95, 0}, expanded),
                "Extra shaft clearance must not be silently ignored");

        bool threw = false;
        try {
            TargetObject2D invalid = circle;
            invalid.toolLength = 2.0;
            (void)targetApproachClear(links, {3.95, 0}, invalid);
        } catch (const std::invalid_argument&) { threw = true; }
        require(threw, "Tool cannot span all of final arm link");

        TargetObject2D invalid = circle;
        invalid.radius = std::numeric_limits<double>::quiet_NaN();
        require(!validTargetObject2D(invalid),
                "Invalid target dimensions rejected");

        // End-to-end: explicit target geometry is passed from the settings
        // through path planning AND every executed follower step.
        constexpr double pi = std::numbers::pi;
        const std::vector<Joint2D> initial{
            {2.0, pi/2, 0.0}, {1.0, -pi/2, 0.0}
        };
        const std::vector<JointLimits2D> limits{
            {-2*pi, 2*pi, 2.0, 6.0},
            {-pi + 0.56, -0.16, 2.0, 6.0}
        };
        MotionSettings2D settings{};
        settings.targetClearance = 0.0; // proves explicit model overrides legacy
        settings.targetObject = circle;
        const Vec2 target{1.5, 1.5};
        const auto path = planJointPath2D(initial, limits, target, settings);
        require(path.found(), "Planner must accept a safe circular target pose");
        PlannedMotion2D follower;
        follower.usePlan(path, target);
        auto moving = initial;
        bool arrived = false;
        for (int i = 0; i < 1500; ++i) {
            const auto status =
                follower.advance(moving, limits, 1.0 / 60.0, settings);
            require(!status.blocked,
                    "Explicit target contact must be safe during execution");
            if (status.reached) { arrived = true; break; }
        }
        require(arrived, "Explicit circular target should be reached");

        settings.targetObject = rectangle;
        const auto rectanglePlan =
            planJointPath2D(initial, limits, target, settings);
        require(rectanglePlan.found(),
                "Planner must accept a safe rectangular target pose");

        // A target containing the fixed base cannot be cleared; never
        // pretend that the terminal tool exemption applies to other links.
        const auto invalidGoal =
            planJointPath2D(initial, limits, {0.0, 0.0}, settings);
        require(!invalidGoal.found(),
                "Object covering fixed base must not be a reachable target");

        std::cout << "Explicit circle/rectangle target and tool contact passed\n";
    } catch (const std::exception& ex) {
        std::cerr << "FAILED: " << ex.what() << '\n';
        return 1;
    }
}
