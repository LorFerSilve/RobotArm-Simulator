#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::MotionSettings2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::hasSelfCollision;
using robotics::kinematics::planJointPath2D;
using robotics::math::Vec2;

void require(bool yes, const char* what) {
    if (!yes) throw std::runtime_error(what);
}

void run(std::vector<Joint2D> joints, std::vector<JointLimits2D> limits,
         Vec2 target, MotionSettings2D settings = {}) {
    const auto plan = planJointPath2D(joints, limits, target, settings);
    require(plan.found(), "Expected a collision-free path");
    require(plan.waypoints.size() == plan.splineTangents.size(),
            "Every waypoint must have a spline tangent");
    if (plan.waypoints.size() > 1) {
        require(plan.pathDistances.size() == plan.waypoints.size(),
                "Spline arc-length knots must match waypoints");
        require(plan.trajectoryDuration > 0.0,
                "Planner must supply a time-parameterized trajectory");
        require(plan.splineTangents.front() ==
                std::vector<double>(joints.size(), 0.0),
                "Initial joint velocity must be zero");
        require(plan.splineTangents.back() ==
                std::vector<double>(joints.size(), 0.0),
                "Final joint velocity must be zero");
    }

    PlannedMotion2D follower;
    follower.usePlan(plan, target);
    const double dt = 1.0 / 60.0;
    bool arrived = false;
    for (int frame = 0; frame < 2500; ++frame) {
        const auto old = joints;
        const auto result = follower.advance(joints, limits, dt, settings);
        require(!result.blocked, "Spline follower blocked unexpectedly");
        require(!hasSelfCollision(forwardKinematics(joints),
                                  settings.linkClearance),
                "Spline follower self-collision");
        for (std::size_t j = 0; j < joints.size(); ++j) {
            require(joints[j].angle >= limits[j].minAngle - 1e-8 &&
                    joints[j].angle <= limits[j].maxAngle + 1e-8,
                    "Spline follower violates joint range");
            require(std::abs(joints[j].angularVelocity) <=
                    std::min(settings.maxAngularSpeed,
                             limits[j].maxAngularSpeed) + 1e-8,
                    "Spline follower exceeds joint speed");
            require(std::abs(joints[j].angularVelocity -
                             old[j].angularVelocity) <=
                    std::min(settings.maxAngularAcceleration,
                             limits[j].maxAngularAcceleration) * dt + 1e-7,
                    "Spline follower exceeds joint acceleration");
        }
        if (result.reached) { arrived = true; break; }
    }
    require(arrived, "Smooth trajectory did not reach target");
    require((forwardKinematics(joints).back() - target).length() <=
            settings.positionTolerance, "Wrong end-effector final point");
    for (const auto& j : joints)
        require(std::abs(j.angularVelocity) < 1e-8, "Robot must stop at final target");
}

int main() {
    try {
        constexpr double pi = std::numbers::pi;
        run({{2.0, pi/2, 0.0}, {1.0, -pi/2, 0.0}},
            {{-2*pi, 2*pi, 2.0, 6.0},
             {-pi + 0.56, -0.16, 2.0, 6.0}}, {1.5, 1.5});

        std::vector<Joint2D> three{
            {1.25, 0.3, 0.0}, {0.75, -0.7, 0.0}, {1.1, 0.8, 0.0}
        };
        std::vector<JointLimits2D> threeBounds{
            {-2*pi, 2*pi, 2.0, 6.0},
            {-pi+0.16, -0.16, 2.0, 6.0},
            {0.16, pi-0.16, 2.0, 6.0}
        };
        run(three, threeBounds, {1.6, 1.1});
        run(three, threeBounds, {-1.0, -0.4});

        MotionSettings2D safe{};
        safe.foldClearance = 0.55;
        safe.targetClearance = 0.16;
        run({{2.0, -57*pi/180, 0.0}, {2.0, -21*pi/180, 0.0},
             {2.0, 26*pi/180, 0.0}, {2.0, -35*pi/180, 0.0},
             {2.0, 32*pi/180, 0.0}},
            {{-2*pi, 2*pi, 2.0, 6.0},
             {-pi+.56, -.16, 2.0, 6.0},
             {.16, pi-.56, 2.0, 6.0},
             {-pi+.56, -.16, 2.0, 6.0},
             {.16, pi-.56, 2.0, 6.0}},
            {-3.0, -4.0}, safe);
        std::cout << "C2 trajectory safety, limits and arrivals passed\n";
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
