#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/SweptCollision2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::geometry::hasObstacleCollision;
using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::MotionSettings2D;
using robotics::kinematics::PlannedJointPath2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::planJointPath2D;
using robotics::kinematics::sweptArmClear2D;
using robotics::math::Vec2;

namespace {
constexpr double pi = std::numbers::pi;

void require(bool condition, const char* why) {
    if (!condition) throw std::runtime_error(why);
}

double traceLength(const PlannedJointPath2D& plan) {
    double length = 0.0;
    for (std::size_t i = 1; i < plan.endEffectorTrace.size(); ++i)
        length += (plan.endEffectorTrace[i] -
                   plan.endEffectorTrace[i - 1]).length();
    return length;
}

void followScene(std::vector<Joint2D> joints,
                 const std::vector<JointLimits2D>& limits,
                 Vec2 target,
                 const std::vector<Obstacle2D>& obstacles,
                 const PlannedJointPath2D& plan,
                 const MotionSettings2D& settings = {}) {
    PlannedMotion2D motion;
    motion.usePlan(plan, target);
    bool reached = false;
    constexpr double dt = 1.0 / 60.0;
    for (int frame = 0; frame < 3000; ++frame) {
        const auto old = joints;
        const auto status = motion.advance(joints, limits, dt, settings, obstacles);
        require(!status.blocked, "Optimized scene trajectory blocked");
        require(!hasObstacleCollision(forwardKinematics(joints), obstacles,
                                      settings.obstacleClearance),
                "Optimized scene trajectory touched an obstacle");
        for (std::size_t i = 0; i < joints.size(); ++i) {
            require(joints[i].angle >= limits[i].minAngle - 1e-8 &&
                    joints[i].angle <= limits[i].maxAngle + 1e-8,
                    "Optimized scene trajectory crossed a joint limit");
            require(std::abs(joints[i].angularVelocity) <=
                    limits[i].maxAngularSpeed + 1e-8,
                    "Optimized scene trajectory exceeded a speed limit");
            require(std::abs(joints[i].angularVelocity -
                             old[i].angularVelocity) <=
                    limits[i].maxAngularAcceleration * dt + 1e-7,
                    "Optimized scene trajectory exceeded an acceleration limit");
        }
        if (status.reached) {
            reached = true;
            break;
        }
    }
    require(reached, "Optimized scene trajectory failed to reach target");
    require((forwardKinematics(joints).back() - target).length() <= 0.005,
            "Optimized scene trajectory ended at the wrong point");
}

} // namespace

int main() {
    try {
        // The short shoulder arc hits the circular obstacle, but the
        // equivalent long-way rotation is physically admissible.
        // This exact fixture was already used for obstacle-detour safety.
        std::vector<Joint2D> arm{
            {2.0, 0.3, 0.0}, {1.0, -pi/2.0, 0.0}
        };
        const std::vector<JointLimits2D> limits{
            {-2*pi, 2*pi, 2.0, 6.0},
            {-pi+0.56, -0.16, 2.0, 6.0}
        };
        const auto goalArm = std::vector<Joint2D>{
            {2.0, -0.2, 0.0}, {1.0, -pi/2.0, 0.0}
        };
        const Vec2 target = forwardKinematics(goalArm).back();
        const std::vector<Obstacle2D> scene{
            Circle2D{{1.5 * std::cos(0.05), 1.5 * std::sin(0.05)}, 0.12}
        };

        // A known-safe comparison route keeps the elbow fixed and rotates
        // the shoulder in the opposite direction. The optimized planner
        // should not select a longer candidate than this admissible baseline.
        double baseline = 0.0;
        auto reference = arm;
        Vec2 previous = forwardKinematics(reference).back();
        for (int frame = 1; frame <= 1200; ++frame) {
            const double t = static_cast<double>(frame) / 1200.0;
            reference[0].angle = 0.3 + t * ((2*pi - 0.2) - 0.3);
            const auto points = forwardKinematics(reference);
            require(!hasObstacleCollision(points, scene, 0.10),
                    "Reference opposite-direction shoulder route must be safe");
            baseline += (points.back() - previous).length();
            previous = points.back();
        }

        const auto optimized = planJointPath2D(arm, limits, target, {}, scene);
        require(optimized.found(), "Scene optimizer must find the safe detour");
        require(optimized.endEffectorTrace.size() > 2,
                "Scene optimizer must produce the actual end-effector trace");
        require(std::abs(optimized.endEffectorDistance -
                         traceLength(optimized)) < 1e-8,
                "Reported tip distance must match the rendered trajectory");
        require(optimized.endEffectorDistance >=
                (target - forwardKinematics(arm).back()).length() - 1e-5,
                "Tip travel must respect the Cartesian straight-line lower bound");
        require(optimized.endEffectorDistance <= baseline * 1.02,
                "Optimizer selected a detour longer than a known safe alternative");
        followScene(arm, limits, target, scene, optimized);

        // The straight tool path through this circle is obstructed. A
        // redundant three-joint arm must find a collision-free alternative
        // without cutting through the obstacle to save travel distance.
        std::vector<Joint2D> redundant{
            {1.25, 0.3, 0.0}, {0.75, -0.7, 0.0}, {1.1, 0.8, 0.0}
        };
        const std::vector<JointLimits2D> bounds{
            {-2*pi, 2*pi, 2.0, 6.0},
            {-pi+0.16, -0.16, 2.0, 6.0},
            {0.16, pi-0.16, 2.0, 6.0}
        };
        const Vec2 blockedStraightTarget{1.6, 1.1};
        const std::vector<Obstacle2D> anotherScene{
            Circle2D{{2.0, 0.9}, 0.12}
        };
        const auto alternative = planJointPath2D(
            redundant, bounds, blockedStraightTarget, {}, anotherScene);
        require(alternative.found(),
                "Redundant arm must retain a safe obstacle detour");
        require(std::isfinite(alternative.endEffectorDistance) &&
                alternative.endEffectorDistance > 0.0,
                "Optimized redundant-arm path must have finite tool length");
        require(std::abs(alternative.endEffectorDistance -
                         traceLength(alternative)) < 1e-8,
                "Redundant-arm travel metric must match the preview");
        followScene(redundant, bounds, blockedStraightTarget, anotherScene,
                    alternative);

        // The direct edge is safe, but the obstacle blocks the straight
        // Cartesian lift. The shoulder limits allow only one IK winding.
        // Repeating that edge as an RRT alternative misses a shorter path.
        const std::vector<double> from{0.0789975233, -0.2339312208};
        const std::vector<double> to{0.2695424112, -2.2420678704};
        const std::vector<Joint2D> singleWinding{{2.0, from[0]}, {1.0, from[1]}};
        const std::vector<JointLimits2D> oneWindingLimits{
            {from[0] - 0.26, to[0] + 0.26, 2, 6}, {-2.65, -0.16, 2, 6}
        };
        const std::vector<Obstacle2D> shortScene{
            Circle2D{{2.2406629905, 0.2802591308}, 0.0527970114}
        };
        MotionSettings2D narrow;
        narrow.obstacleClearance = 0.015;
        require(sweptArmClear2D(singleWinding, from, to, shortScene,
                                 narrow.linkClearance, narrow.obstacleClearance),
                "The direct edge must be continuously scene-clear");
        std::vector<Vec2> pose;
        forwardKinematics(singleWinding, from, pose);
        Vec2 prior = pose.back();
        double directTravel = 0.0;
        for (int i = 1; i <= 300; ++i) {
            const double t = i / 300.0;
            const std::vector<double> q{
                std::lerp(from[0], to[0], t), std::lerp(from[1], to[1], t)
            };
            forwardKinematics(singleWinding, q, pose);
            directTravel += (pose.back() - prior).length();
            prior = pose.back();
        }
        const auto nearby = planJointPath2D(singleWinding, oneWindingLimits,
                                             prior, narrow, shortScene);
        require(nearby.waypoints.size() > 2 &&
                    nearby.endEffectorDistance < directTravel - 0.02,
                "An RRT alternative must improve on the safe direct edge");
        followScene(singleWinding, oneWindingLimits, prior, shortScene, nearby, narrow);

        std::cout << "Two-link safe reference tip travel: " << baseline
                  << " m; selected optimized travel: "
                  << optimized.endEffectorDistance << " m\n";
        std::cout << "Three-link obstacle travel: "
                  << alternative.endEffectorDistance << " m\n";
        std::cout << "Cartesian obstacle path optimization passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
