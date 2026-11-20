#include "robotics/geometry/Obstacles2D.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using robotics::geometry::Box2D;
using robotics::geometry::Circle2D;
using robotics::geometry::Obstacle2D;
using robotics::geometry::hasObstacleCollision;
using robotics::geometry::pointHitsObstacle;
using robotics::geometry::segmentHitsObstacle;
using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::planJointPath2D;
using robotics::math::Vec2;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    try {
        const Obstacle2D circle = Circle2D{{1.0, 1.0}, 0.35};
        const Obstacle2D box = Box2D{{2.0, -1.0}, 0.5, 0.25};
        require(pointHitsObstacle({1.1, 1.1}, circle), "Circle point inside");
        require(!pointHitsObstacle({2.0, 1.0}, circle), "Circle point outside");
        require(segmentHitsObstacle({0.0, 1.0}, {2.0, 1.0}, circle),
                "Circle crossing must collide");
        require(!segmentHitsObstacle({0.0, 2.0}, {2.0, 2.0}, circle),
                "Separated circle segment");
        require(!segmentHitsObstacle({0.0, 1.45}, {2.0, 1.45}, circle, 0.09),
                "Clearance just smaller than gap must not collide");
        require(segmentHitsObstacle({0.0, 1.45}, {2.0, 1.45}, circle, 0.11),
                "Circle clearance expands the collision radius");
        require(segmentHitsObstacle({1.0, -1.0}, {3.0, -1.0}, box),
                "Segment intersects rectangle");
        require(segmentHitsObstacle({1.0, -1.25}, {3.0, -1.25}, box),
                "Tangency counts as collision");
        require(!segmentHitsObstacle({1.0, -1.6}, {3.0, -1.6}, box, 0.3),
                "Rectangle with separated capsule");
        require(segmentHitsObstacle({1.0, -1.6}, {3.0, -1.6}, box, 0.36),
                "Rectangle with capsule clearance");
        require(!segmentHitsObstacle({0.0, 0.0}, {0.0, 1.0}, box),
                "Distant rectangle");
        require(pointHitsObstacle({2.3, -1.1}, box),
                "Point inside box");
        require(pointHitsObstacle({2.65, -1.0}, box, 0.2),
                "Rounded rectangle clearance");
        require(!pointHitsObstacle({2.7, -1.3}, box, 0.2),
                "Corner distance is Euclidean, not square expansion");

        require(hasObstacleCollision(std::vector<Vec2>{{0, 0}, {2, 0}, {2, 2}},
                                     std::vector<Obstacle2D>{Circle2D{{1.0, 0.2}, 0.1}},
                                     0.1),
                "Obstacle collides with the first link");
        require(hasObstacleCollision(std::vector<Vec2>{{0, 0}, {2, 0}, {2, 2}},
                                     std::vector<Obstacle2D>{Box2D{{2.0, 1.0}, 0.2, 0.2}},
                                     0.0),
                "Obstacle collides with the final link");
        require(!hasObstacleCollision(std::vector<Vec2>{{0, 0}, {2, 0}, {2, 2}},
                                      std::vector<Obstacle2D>{Circle2D{{5.0, 5.0}, 0.3}},
                                      0.1),
                "Far scene is safe");
        for (const Obstacle2D invalid : {
                Obstacle2D{Circle2D{{0.0, 0.0}, -1.0}},
                Obstacle2D{Box2D{{0.0, 0.0}, 0.0, 1.0}}}) {
            require(!robotics::geometry::validObstacle(invalid),
                    "Invalid obstacle must be rejected");
            bool threw = false;
            try { (void)pointHitsObstacle({1, 1}, invalid); }
            catch (const std::invalid_argument&) { threw = true; }
            require(threw, "Invalid obstacle query must throw");
        }

        const double pi = std::numbers::pi;
        std::vector<Joint2D> joints{{2.0, pi / 2.0, 0.0},
                                    {1.0, -pi / 2.0, 0.0}};
        const std::vector<JointLimits2D> limits{
            {-2.0*pi, 2.0*pi, 2.0, 6.0},
            {-pi+0.56, -0.16, 2.0, 6.0}
        };
        const Vec2 target{1.5, 1.5};
        const auto baseline = planJointPath2D(joints, limits, target);
        require(baseline.found(), "Known two-joint path should exist");

        // The desired end effector cannot enter a solid scene obstacle.
        const std::vector<Obstacle2D> blockedTarget{
            Circle2D{target, 0.25}
        };
        const auto blocked = planJointPath2D(joints, limits, target, {}, blockedTarget);
        require(!blocked.found(), "Obstacle covering the destination rejects goal");

        // A scene that begins with a link intersecting an obstacle is invalid.
        const std::vector<Obstacle2D> blockedStart{
            Box2D{{0.0, 1.0}, 0.2, 0.2}
        };
        bool threw = false;
        try { (void)planJointPath2D(joints, limits, target, {}, blockedStart); }
        catch (const std::invalid_argument&) { threw = true; }
        require(threw, "Obstacle overlapping initial arm must be detected");

        // A distant obstacle may not change a verified ordinary trajectory.
        const std::vector<Obstacle2D> clearScene{
            Circle2D{{-8.0, 8.0}, 0.4},
            Box2D{{7.0, 7.0}, 0.6, 0.6}
        };
        auto planned = planJointPath2D(joints, limits, target, {}, clearScene);
        require(planned.found(), "Distant obstacles must not block planning");
        PlannedMotion2D follower;
        follower.setTarget(joints, limits, target, {}, clearScene);
        require(follower.pathAvailable(), "Scene-aware follower should find safe plan");
        bool arrived = false;
        for (int frame = 0; frame < 1000; ++frame) {
            const auto result = follower.advance(joints, limits, 1.0 / 60.0,
                                                 {}, clearScene);
            require(!result.blocked, "Scene-aware follower blocked unexpectedly");
            require(!hasObstacleCollision(forwardKinematics(joints), clearScene, 0.10),
                    "Follower crossed an obstacle");
            if (result.reached) { arrived = true; break; }
        }
        require(arrived, "Scene-aware follower did not reach the destination");

        std::cout << "Obstacle geometry, planning and execution passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
