#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"
#include "robotics/kinematics/TargetObject2D.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

using robotics::geometry::Box2D;
using robotics::geometry::Circle2D;
using robotics::geometry::segmentSignedDistanceToObstacle;
using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::MotionSettings2D;
using robotics::kinematics::PlannedJointPath2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::TargetObject2D;
using robotics::kinematics::TargetShape2D;
using robotics::kinematics::advanceTowardTarget;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::planJointPath2D;
using robotics::kinematics::sweptArmClear2D;
using robotics::kinematics::targetApproachClear;
using robotics::kinematics::targetBody2D;
using robotics::kinematics::targetLinkPenetrations;
using robotics::kinematics::validTargetObject2D;
using robotics::math::Vec2;

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void followTarget(std::vector<Joint2D> joints,
                  const std::vector<JointLimits2D>& limits, Vec2 target,
                  const MotionSettings2D& settings) {
    const auto& object = *settings.targetObject;
    PlannedMotion2D follower;
    follower.setTarget(joints, limits, target, settings);
    require(follower.pathAvailable(), "Planner must find a safe explicit-target route");
    auto previous = targetLinkPenetrations(forwardKinematics(joints), target, object);
    for (int frame = 0; frame < 1800; ++frame) {
        const auto status = follower.advance(joints, limits, 1.0 / 60.0, settings);
        require(!status.blocked, "Certified target approach must remain executable");
        const auto positions = forwardKinematics(joints);
        const auto depth = targetLinkPenetrations(positions, target, object);
        for (std::size_t i = 0; i < depth.size(); ++i)
            require(depth[i] <= previous[i] + 1e-9,
                    "Execution created or deepened shaft contact");
        previous = depth;
        if (status.reached) {
            require(targetApproachClear(positions, target, object),
                    "Only the tool may be in contact on arrival");
            return;
        }
    }
    throw std::runtime_error("Follower did not reach the explicit target");
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
        followTarget(initial, limits, target, settings);
        settings.targetObject = rectangle;
        followTarget(initial, limits, target, settings);

        auto touching = initial;
        touching[0].angle = 0.0;
        const Vec2 touchingTarget{1.5, -0.065};
        const std::vector<double> contactPose{0.0, -pi/2};
        const std::vector<double> retreatPose{0.30, -pi/2};
        const std::vector<double> deeperThenClear{-0.30, -pi/2};
        for (const auto& object : {circle, rectangle}) {
            require(!targetApproachClear(forwardKinematics(touching),
                                         touchingTarget, object),
                    "The retreat fixture must start in contact");
            require(sweptArmClear2D(touching, contactPose, retreatPose, {}, 0.1, 0.1,
                                     touchingTarget, object),
                    object.shape == TargetShape2D::Circle ?
                        "Monotonically separating circle contact must certify" :
                        "Monotonically separating rectangle contact must certify");
            require(!sweptArmClear2D(touching, contactPose, deeperThenClear, {}, 0.1, 0.1,
                                      touchingTarget, object),
                    "An edge must not deepen initial contact before clearing");
            require(!sweptArmClear2D(touching, retreatPose, contactPose, {}, 0.1, 0.1,
                                      touchingTarget, object),
                    "A clear shaft must not re-enter the target");
            settings.targetObject = object;
            followTarget(touching, limits, touchingTarget, settings);
            const Vec2 centeredTarget{1.5, 0.0};
            require(sweptArmClear2D(touching, contactPose, retreatPose, {}, 0.1, 0.1,
                                     centeredTarget, object),
                    "A shaft starting through the target centre must be able to retreat");
            followTarget(touching, limits, centeredTarget, settings);
        }

        // A target containing the fixed base cannot be cleared; never
        // pretend that the terminal tool exemption applies to other links.
        const auto invalidGoal =
            planJointPath2D(initial, limits, {0.0, 0.0}, settings);
        require(!invalidGoal.found(),
                "Object covering fixed base must not be a reachable target");

        // The first link must cross q[0] = 0 on any path to this target
        // within the given shoulder limits. There it penetrates the circle
        // by 1 micrometre, in a window narrower than the old sampling grid.
        settings.targetObject = circle;
        const Vec2 grazingTarget{2.159999, 0.0};
        auto grazingStart = initial;
        grazingStart[0].angle = -0.017123;
        grazingStart[1].angle = -1.8;
        auto crossing = grazingStart;
        crossing[0].angle = 0.0;
        require(targetApproachClear(forwardKinematics(grazingStart),
                                    grazingTarget, circle),
                "Grazing regression starts with clear links");
        require(!targetApproachClear(forwardKinematics(crossing),
                                     grazingTarget, circle),
                "The analytic crossing pose must penetrate the target");
        auto grazingLimits = limits;
        grazingLimits[0].minAngle = -0.1;
        grazingLimits[0].maxAngle = 0.8;
        require(!planJointPath2D(grazingStart, grazingLimits,
                                 grazingTarget, settings).found(),
                "Planner must reject a target collision between samples");

        // An installed polyline must pass the same check on every step.
        grazingStart[0].angle = -0.0008;
        PlannedJointPath2D unchecked;
        unchecked.waypoints = {{-0.0008, -1.8}, {0.0032, -1.8}};
        PlannedMotion2D follower;
        follower.usePlan(std::move(unchecked), grazingTarget);
        const auto step = follower.advance(grazingStart, grazingLimits, 0.06, settings);
        require(step.blocked && grazingStart[0].angle == -0.0008,
                "Follower must not cross the target between two clear poses");

        TargetObject2D tangent = circle;
        tangent.radius = 0.125;
        require(!targetApproachClear(links, {1.0, 0.125}, tangent),
                "Tangency of an upstream link is forbidden contact");

        // The direct controller uses the explicit target even when the
        // legacy radius is zero. Its first step would deepen existing contact.
        auto directMotion = touching;
        const Vec2 directTarget{1.0, 0.08};
        const auto refused = advanceTowardTarget(directMotion, directTarget,
                                                 1.0 / 60.0, settings, limits);
        require(refused.blocked && directMotion[0].angle == touching[0].angle,
                "Direct controller must not deepen explicit target contact");

        std::cout << "Explicit circle/rectangle target and tool contact passed\n";
    } catch (const std::exception& ex) {
        std::cerr << "FAILED: " << ex.what() << '\n';
        return 1;
    }
}
