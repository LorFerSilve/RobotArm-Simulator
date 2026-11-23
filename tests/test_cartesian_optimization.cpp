#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace robotics::kinematics;
using robotics::math::Vec2;

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

double directTipDistance(const std::vector<Joint2D>& start,
                         double targetShoulder, double targetElbow) {
    auto robot = start;
    Vec2 previous = forwardKinematics(robot).back();
    double distance = 0.0;
    for (int i = 1; i <= 600; ++i) {
        const double fraction = static_cast<double>(i) / 600.0;
        robot[0].angle = start[0].angle +
                          fraction * (targetShoulder - start[0].angle);
        robot[1].angle = start[1].angle +
                          fraction * (targetElbow - start[1].angle);
        const auto position = forwardKinematics(robot).back();
        distance += (position - previous).length();
        previous = position;
    }
    return distance;
}

} // namespace

int main() {
    try {
        constexpr double pi = std::numbers::pi;

        // This safe 2-link move would traditionally draw a broad arc even
        // though a nearly straight Cartesian path with the same elbow branch
        // and joint limits exists.
        std::vector<Joint2D> arm{
            {1.0, 2.113870212716014, 0.0},
            {1.0, -0.35123607589057926, 0.0}
        };
        const std::vector<JointLimits2D> limits{
            {-2*pi, 2*pi, 2.0, 6.0},
            {-pi+0.56, -0.16, 2.0, 6.0}
        };
        const double goalShoulder = 0.9220734636133903;
        const double goalElbow = -1.7304200413746085;
        auto goalRobot = arm;
        goalRobot[0].angle = goalShoulder;
        goalRobot[1].angle = goalElbow;
        const Vec2 target = forwardKinematics(goalRobot).back();
        const Vec2 start = forwardKinematics(arm).back();
        const double straightDistance = (target - start).length();
        const double directDistance =
            directTipDistance(arm, goalShoulder, goalElbow);

        MotionSettings2D settings;
        settings.foldClearance = 0.55;
        settings.targetClearance = 0.16;

        require(directDistance > straightDistance * 1.15,
                "Test fixture does not have a significantly curved direct route");

        const auto optimized = planJointPath2D(arm, limits, target, settings);
        require(optimized.found(), "Planner must find a Cartesian-short route");
        require(optimized.endEffectorTrace.size() > 10,
                "Optimized path must have a drawable tip trace");
        require((optimized.endEffectorTrace.front()-start).length() < 1e-8,
                "Trace must start at the actual initial tool position");
        require((optimized.endEffectorTrace.back()-target).length() <
                settings.positionTolerance,
                "Trace must end at the requested Cartesian point");
        require(optimized.endEffectorDistance >= straightDistance - 1e-6,
                "Tip travel cannot be less than the Euclidean lower bound");
        require(optimized.endEffectorDistance < directDistance * 0.92,
                "Cartesian-guided planner should beat the old broad arc");
        require(optimized.endEffectorDistance < straightDistance * 1.10,
                "Feasible Cartesian guide should be close to a straight line");

        PlannedMotion2D motion;
        motion.usePlan(optimized, target);
        require(std::abs(motion.endEffectorDistance() -
                         optimized.endEffectorDistance) < 1e-9,
                "Follower must retain the measured tip travel distance");
        bool arrived = false;
        for (int frame = 0; frame < 2400; ++frame) {
            const auto status = motion.advance(arm, limits, 1.0/60.0, settings);
            require(!status.blocked, "Optimized path must execute safely");
            if (status.reached) { arrived = true; break; }
        }
        require(arrived, "Optimized trajectory must reach the target");

        // Screenshot regression: 1+1 metre links, q=(-164.85,-93.18) deg.
        // The target lies INSIDE the first link radius. Every permitted
        // negative-elbow IK winding crosses the target direction because
        // reaching it on the other side requires shoulder < -360 degrees.
        const std::vector<Joint2D> screenshot{
            {1.0, -164.85*pi/180.0, 0.0},
            {1.0, -93.18*pi/180.0, 0.0}
        };
        const Vec2 screenshotTarget{-0.36, -0.80};
        const auto blocked = planJointPath2D(
            screenshot, limits, screenshotTarget, settings);
        require(!blocked.found(), "Screenshot scenario must fail closed");
        require(blocked.failure == PlanningFailure2D::TargetBlockedByJointLimits,
                "Screenshot should report the shoulder-target barrier, not an RRT budget issue");

        // The same IK point is not a safe transition merely because the
        // endpoint is reachable. Never remove collision checks to force it.
        auto noFold = screenshot;
        noFold[0].angle = -410.2445182768835 * pi/180.0;
        noFold[1].angle = -127.96645408214137 * pi/180.0;
        require((forwardKinematics(noFold).back() - screenshotTarget).length() < 1e-8,
                "Extended-turn alternative must be geometrically valid");
        require(noFold[0].angle < limits.front().minAngle,
                "Alternative must actually violate the original shoulder limit");

        // Also detect the inner radius imposed by a tight folding limit.
        std::vector<Joint2D> unequal{
            {2.0, pi/2.0, 0.0}, {1.0, -pi/2.0, 0.0}
        };
        const auto tooClose = planJointPath2D(
            unequal, limits, {0.1, 0.5}, settings);
        require(!tooClose.found() &&
                tooClose.failure == PlanningFailure2D::OutOfReach,
                "Inner-workspace exclusion should be diagnosed as unreachable");

        const std::vector<Joint2D> bounded{{1.0, 0.5}, {1.0, -1.0}};
        const std::vector<JointLimits2D> boundedLimits{
            {-1.0, 1.0, 2.0, 6.0}, {-2.4, -0.5, 2.0, 6.0}
        };
        const double outer = 2.0 * std::cos(0.25);
        const Vec2 nearBoundary{outer + 0.002, 0.0};
        const auto withinTolerance = planJointPath2D(bounded, boundedLimits, nearBoundary);
        require(withinTolerance.found() && withinTolerance.terminalError <= 0.005 &&
                    withinTolerance.waypoints.back()[1] <= -0.5,
                "A goal within tolerance must be reachable at the legal elbow boundary");
        const auto outside = planJointPath2D(bounded, boundedLimits, {outer + 0.006, 0.0});
        require(!outside.found() && outside.failure == PlanningFailure2D::OutOfReach,
                "Outside workspace and tolerance must still be rejected");

        std::cout << "Cartesian distance optimization and target barrier tests passed\n";
    } catch (const std::exception& ex) {
        std::cerr << "FAILED: " << ex.what() << '\n';
        return 1;
    }
}
