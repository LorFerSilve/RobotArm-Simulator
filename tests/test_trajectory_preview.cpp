#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace robotics::kinematics;
using robotics::math::Vec2;

namespace {

void require(bool okay, const char* why) {
    if (!okay) throw std::runtime_error(why);
}

double pointSegmentDistance(Vec2 point, Vec2 a, Vec2 b) {
    const Vec2 delta = b - a;
    const double squared = delta.dot(delta);
    const double t = squared > 0.0 ?
        std::clamp((point - a).dot(delta) / squared, 0.0, 1.0) : 0.0;
    return (point - (a + delta * t)).length();
}

double distanceToTrace(Vec2 point, std::span<const Vec2> trace) {
    double closest = 1e100;
    for (std::size_t i = 1; i < trace.size(); ++i)
        closest = std::min(closest,
                           pointSegmentDistance(point, trace[i - 1], trace[i]));
    return closest;
}

} // namespace

int main() {
    try {
        constexpr double pi = std::numbers::pi;
        const double elbow = -std::acos(-0.25);
        const double shoulder = -std::atan2(2.0 * std::sin(elbow),
                                            4.0 + 2.0 * std::cos(elbow));
        std::vector<Joint2D> robot{{4.0, shoulder, 0.0}, {2.0, elbow, 0.0}};
        const std::vector<JointLimits2D> limits{
            {-2.0 * pi, 2.0 * pi, 2.0, 6.0},
            {-pi + 0.56, -0.16, 2.0, 6.0}
        };
        MotionSettings2D settings;
        settings.foldClearance = 0.55;
        settings.targetClearance = 0.16;
        const Vec2 target{2.0, 3.0};

        const auto start = forwardKinematics(robot).back();
        require((start - Vec2{4.0, 0.0}).length() < 1e-8,
                "Regression start must be (4, 0)");

        const auto path = planJointPath2D(robot, limits, target, settings);
        require(path.found(), "Known long-way route should be found");
        require(path.endEffectorTrace.size() > 25,
                "The worker must produce a sufficiently detailed preview");
        require((path.endEffectorTrace.front() - start).length() < 1e-8,
                "Preview must begin at the original tool position");
        require((path.endEffectorTrace.back() - target).length() <=
                settings.positionTolerance,
                "Preview must terminate at the IK target");

        // If the drawing were merely a straight Cartesian line it would miss
        // the loop made by the long-way shoulder rotation.
        double biggestDeviation = 0.0;
        for (const auto& point : path.endEffectorTrace)
            biggestDeviation = std::max(
                biggestDeviation, pointSegmentDistance(point, start, target));
        require(biggestDeviation > 1.0,
                "End-effector preview cannot be a straight-line shortcut");

        PlannedMotion2D follower;
        follower.usePlan(path, target);
        require(follower.endEffectorTrace().size() ==
                path.endEffectorTrace.size(),
                "Installing a plan must retain its exact preview");
        bool arrived = false;
        for (int frame = 0; frame < 2200; ++frame) {
            const auto state = follower.advance(robot, limits, 1.0 / 60.0, settings);
            require(!state.blocked, "Follower blocked while executing preview");
            require(distanceToTrace(forwardKinematics(robot).back(),
                                    follower.endEffectorTrace()) < 0.08,
                    "Rendered end-effector path differs from executed motion");
            if (state.reached) { arrived = true; break; }
        }
        require(arrived, "Robot failed to finish previewed long-way trajectory");

        follower.reset();
        require(follower.endEffectorTrace().empty(),
                "Reset must remove the old trajectory silhouette");
        require(!follower.pathAvailable(), "Reset must also invalidate the path");

        const auto impossible = planJointPath2D(robot, limits, {50.0, 50.0},
                                                settings);
        require(!impossible.found() && impossible.endEffectorTrace.empty(),
                "An invalid target must not show a fictitious path");
        follower.usePlan(impossible, {50.0, 50.0});
        require(follower.endEffectorTrace().empty(),
                "Failed plan must not display the previous trace");

        // Already-reached goals are allowed to have a one-point trace.
        const auto current = forwardKinematics(robot).back();
        const auto alreadyThere = planJointPath2D(robot, limits, current, settings);
        require(alreadyThere.found() &&
                alreadyThere.endEffectorTrace.size() == 1,
                "An already-reached goal has no movement to preview");
        std::cout << "End-effector trajectory preview passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
