#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/RobotConfiguration2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using namespace robotics::kinematics;
using robotics::geometry::Box2D;
using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::geometry::hasObstacleCollision;
using robotics::math::Vec2;

constexpr double pi = std::numbers::pi;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void checkSceneAndSelf() {
    const std::vector<Vec2> points{{0, 0}, {2, 0}, {2, -1}};
    const std::vector<Obstacle2D> shapes{
        Circle2D{{1, 0.1875}, 0.03125}, Box2D{{1, 0.1875}, 0.125, 0.03125}
    };
    const std::vector<double> thickLast{0.0625, 0.125};
    const std::vector<double> thickFirst{0.125, 0.0625};
    for (const auto& shape : shapes) {
        const std::span<const Obstacle2D> scene(&shape, 1);
        require(!hasObstacleCollision(points, scene, 0.03125, thickLast),
                "Thickness on a distant link must not affect another link");
        require(hasObstacleCollision(points, scene, 0.03125, thickFirst),
                "A link's radius and scene margin together must count at tangency");
    }

    const std::vector<Vec2> folded{{0, 0}, {2, 0}, {2, 0.25}, {0, 0.25}};
    const std::vector<double> safeRadii{0.0625, 0.5, 0.0625};
    const std::vector<double> touchingRadii{0.0625, 0.5, 0.125};
    require(!hasSelfCollision(folded, 0.0625, safeRadii),
            "Adjacent links share a pivot and are excluded from the capsule test");
    require(hasSelfCollision(folded, 0.0625, touchingRadii),
            "The radii of both nonadjacent links must contribute to clearance");

    MotionSettings2D settings;
    settings.linkClearance = 0.0625;
    std::vector<JointDefinition2D> definition{
        {2, 0, {-pi, pi, 2, 6}, 0.0625},
        {0.25, pi/2, {0.16, 2.8, 2, 6}, 0.5},
        {2, pi/2, {0.16, 2.8, 2, 6}, 0.0625}
    };
    const auto safe = makeRobot2D(definition, settings);
    require(safe.joints[1].linkRadius == 0.5, "Configuration must carry physical radii");
    const std::vector<double> angles{0, pi/2, pi/2};
    auto thickArm = safe.joints;
    thickArm[2].linkRadius = 0.1875;
    require(!sweptArmClear2D(thickArm, angles, angles, {}, 0.0625, 0.0),
            "Swept self-collision must account for both radii");
    for (const double invalid : {0.1875, -0.1, std::numeric_limits<double>::quiet_NaN()}) {
        definition[2].linkRadius = invalid;
        bool rejected = false;
        try { (void)makeRobot2D(definition, settings); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Configuration must reject colliding or invalid radii");
    }

    std::vector<Joint2D> single{{2, 0}};
    const std::vector<double> start{-0.4}, end{0.4};
    const std::vector<Obstacle2D> narrow{Circle2D{{2.125, 0}, 0.03125}};
    require(sweptArmClear2D(single, start, end, narrow, 0.0, 0.0),
            "The centreline route must be clear");
    single[0].linkRadius = 0.125;
    require(sweptArmClear2D(single, start, start, narrow, 0.0, 0.0) &&
            sweptArmClear2D(single, end, end, narrow, 0.0, 0.0),
            "The wide link must stay clear at both endpoints");
    require(!sweptArmClear2D(single, start, end, narrow, 0.0, 0.0),
            "Swept obstacle checks must catch intermediate capsule contact");
}

void checkTargetShaft() {
    const std::vector<Vec2> points{{0, 0}, {2, 0}, {2, -1}};
    const Vec2 tip = points.back();
    const std::vector<double> toolClear{0.0, 0.0625};
    const std::vector<double> shaftHits{0.0, 0.125};
    TargetObject2D object;
    object.radius = object.halfWidth = object.halfHeight = 0.125;
    object.clearance = 0.03125;
    object.toolLength = 0.25;
    const std::vector<JointLimits2D> limits{{-1, 1, 2, 6}, {-2.5, -0.16, 2, 6}};
    for (const auto shape : {TargetShape2D::Circle, TargetShape2D::Rectangle}) {
        object.shape = shape;
        require(targetApproachClear(points, tip, object, toolClear),
                "The terminal tool may touch an explicit target");
        require(!targetApproachClear(points, tip, object, shaftHits),
                "The protected shaft capsule must stay outside both target shapes");
        std::vector<double> depth(2);
        targetLinkPenetrations(points, tip, object, depth, shaftHits);
        require(depth[0] == 0.0 && std::abs(depth[1] - 0.03125) < 1e-12,
                "Penetration and the clearance predicate must use the same radius");

        MotionSettings2D settings;
        settings.targetObject = object;
        std::vector<Joint2D> reaching{{2, 0}, {1, -pi/2, 0, toolClear[1]}};
        require(planJointPath2D(reaching, limits, tip, settings).found(),
                "A collision-free tool pose must qualify as a goal");
        reaching.back().linkRadius = shaftHits[1];
        require(!planJointPath2D(reaching, limits, tip, settings).found(),
                "Goal validation must reject contact by the wider shaft");
    }
    require(!hasObstacleCollision(points, {}, 0.0, toolClear),
            "Empty scenes must remain collision-free");
    const std::vector<Obstacle2D> solidToolObstacle{Circle2D{tip, 0.01}};
    require(hasObstacleCollision(points, solidToolObstacle, 0.0, toolClear),
            "The tool exemption applies only to the selected target");

    const std::vector<Joint2D> arm{{2, 0, 0, 0.125}, {1, -pi/2}};
    const std::vector<double> contact{0, -pi/2}, clear{0.25, -pi/2};
    object.shape = TargetShape2D::Circle;
    object.radius = 0.03125;
    object.clearance = 0.0;
    const Vec2 touched{1.5, -0.125};
    require(sweptArmClear2D(arm, contact, clear, {}, 0.0, 0.0, touched, object),
            "An overlap caused by link thickness must support certified retreat");
    require(!sweptArmClear2D(arm, clear, contact, {}, 0.0, 0.0, touched, object),
            "A clear capsule cannot enter the target");
}

bool reachesTarget(std::vector<Joint2D> arm, std::span<const JointLimits2D> limits,
                   const PlannedJointPath2D& plan, Vec2 target,
                   const MotionSettings2D& settings, std::span<const Obstacle2D> scene) {
    PlannedMotion2D follower;
    follower.usePlan(plan, target);
    const auto radii = linkRadii2D(arm);
    for (int step = 0; step < 1000; ++step) {
        const auto result = follower.advance(arm, limits, 1.0 / 60.0, settings, scene);
        require(!hasObstacleCollision(forwardKinematics(arm), scene,
                                      settings.obstacleClearance, radii),
                "The running follower must not execute a radius-induced collision");
        if (result.reached || result.blocked) return result.reached;
    }
    throw std::runtime_error("Follower must reach its goal or stop safely");
}

void checkPlanningAndExecution() {
    std::vector<Joint2D> arm{{2, 0, 0, 0.0625}, {1, -pi/2, 0, 0.03125}};
    const std::vector<JointLimits2D> limits{{-1, 1, 2, 6}, {-2.5, -0.16, 2, 6}};
    auto goal = arm;
    goal[0].angle = 0.4;
    const auto end = forwardKinematics(goal);
    const Vec2 target = end.back();
    const std::vector<Obstacle2D> scene{
        Circle2D{target + (target - end[1]) * 0.125, 0.03125}
    };
    MotionSettings2D settings;
    settings.obstacleClearance = 0.03125;
    const auto plan = planJointPath2D(arm, limits, target, settings, scene);
    require(plan.found(), "The original link radii must admit the scene target");
    require(reachesTarget(arm, limits, plan, target, settings, scene),
            "The follower must execute the route with the original radii");

    arm.back().linkRadius = 0.09375;
    const auto blocked = planJointPath2D(arm, limits, target, settings, scene);
    require(blocked.failure == PlanningFailure2D::GoalInObstacle && !blocked.found(),
            "Planning must reject a goal blocked by the terminal link radius");

    require(!reachesTarget(arm, limits, plan, target, settings, scene),
            "The follower must stop before the goal after a geometry change");
}

} // namespace

int main() {
    try {
        checkSceneAndSelf();
        checkTargetShaft();
        checkPlanningAndExecution();
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
