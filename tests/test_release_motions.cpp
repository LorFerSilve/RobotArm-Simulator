#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/RobotConfiguration2D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using robotics::kinematics::Joint2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::MotionSettings2D;
using robotics::kinematics::PlannedMotion2D;
using robotics::kinematics::forwardKinematics;
using robotics::kinematics::hasSelfCollision;
using robotics::kinematics::makeRobot2D;
using robotics::kinematics::JointDefinition2D;
using robotics::kinematics::targetLinkPenetrations;
using robotics::kinematics::targetApproachClear;
using robotics::math::Vec2;

constexpr double pi = std::numbers::pi;
constexpr double dt = 1.0 / 60.0;
constexpr int maxFrames = 2400;

struct MotionCase {
    std::string name;
    std::vector<Vec2> startPositions; // base (0,0), then all joint endpoints
    Vec2 target;
};

using CaseGroup = std::vector<MotionCase>;

void check(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}

MotionSettings2D safetySettings() {
    MotionSettings2D settings;
    settings.foldClearance = 0.55;
    settings.targetClearance = 0.16;
    return settings;
}

std::vector<JointLimits2D> limitsFor(const std::vector<Joint2D>& joints) {
    std::vector<JointLimits2D> limits;
    limits.reserve(joints.size());
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i == 0) {
            limits.push_back({-2.0 * pi, 2.0 * pi, 2.0, 6.0});
        } else if (joints[i].angle < 0.0) {
            limits.push_back({-pi + 0.56, -0.16, 2.0, 6.0});
        } else {
            limits.push_back({0.16, pi - 0.56, 2.0, 6.0});
        }
    }
    return limits;
}

// The case stores *positions*, not derived lengths or angles. This adapter
// reconstructs the exact relative revolute-joint model used by the engine.
std::vector<Joint2D> jointsFromPositions(const std::vector<Vec2>& positions) {
    check(positions.size() >= 3, "A testcase needs the base and >=2 link endpoints");
    check(positions.front().length() <= 1e-10, "The simulator base must be (0,0)");
    std::vector<Joint2D> result;
    result.reserve(positions.size() - 1);
    double previousAbsolute = 0.0;
    for (std::size_t i = 1; i < positions.size(); ++i) {
        const auto delta = positions[i] - positions[i - 1];
        const double length = delta.length(); // Euclidean distance formula
        check(std::isfinite(length) && length > 0.0, "Invalid link length");
        const double absolute = std::atan2(delta.y, delta.x);
        const double relative = std::remainder(absolute - previousAbsolute,
                                               2.0 * pi);
        result.push_back({length, relative, 0.0});
        previousAbsolute = absolute;
    }
    const auto reconstructed = forwardKinematics(result);
    for (std::size_t i = 0; i < positions.size(); ++i)
        check((reconstructed[i] - positions[i]).length() < 1e-8,
              "Positions -> relative angles -> FK did not round-trip");
    return result;
}

// Independent geometric reference for creating test fixtures. The test runner
// still reads ONLY the resulting list of positions and the target.
std::vector<Vec2> positionsFromAngles(const std::vector<double>& lengths,
                                      const std::vector<double>& relativeAngles) {
    check(lengths.size() == relativeAngles.size(), "Geometry size mismatch");
    std::vector<Vec2> positions{{0.0, 0.0}};
    positions.reserve(lengths.size() + 1);
    double absolute = 0.0;
    for (std::size_t i = 0; i < lengths.size(); ++i) {
        absolute += relativeAngles[i];
        const Vec2 offset{lengths[i] * std::cos(absolute),
                          lengths[i] * std::sin(absolute)};
        positions.push_back(positions.back() + offset);
    }
    return positions;
}

std::string describe(const MotionCase& test) {
    std::ostringstream out;
    out << std::setprecision(12) << test.name << "\nstartPositions = {";
    for (const auto& p : test.startPositions)
        out << " (" << p.x << ", " << p.y << ")";
    out << " }\ntarget = (" << test.target.x << ", " << test.target.y << ")";
    return out.str();
}

// A generated "must reach" target needs a witness: at least one valid,
// collision-free trajectory to a joint pose with the target at its tool.
bool witnessIsSafe(const std::vector<double>& lengths,
                   const std::vector<double>& beginAngles,
                   const std::vector<double>& endAngles,
                   const Vec2& target, const MotionSettings2D& settings) {
    std::vector<double> q(beginAngles.size());
    for (int sample = 0; sample <= 80; ++sample) {
        const double t = static_cast<double>(sample) / 80.0;
        for (std::size_t j = 0; j < q.size(); ++j)
            q[j] = beginAngles[j] + t * (endAngles[j] - beginAngles[j]);
        const auto positions = positionsFromAngles(lengths, q);
        if (hasSelfCollision(positions, settings.linkClearance) ||
            !targetApproachClear(positions, target, settings.targetClearance))
            return false;
    }
    return true;
}

CaseGroup buildGroup(int n, int desiredCount, const MotionSettings2D& settings) {
    std::mt19937 rng{static_cast<std::mt19937::result_type>(0x7a31b2c0U + n)};
    std::uniform_real_distribution<double> lengthDistribution(0.8, 1.7);
    std::uniform_real_distribution<double> shoulderDistribution(-1.4, 1.4);
    std::uniform_real_distribution<double> bendDistribution(0.36, 0.89);
    std::uniform_real_distribution<double> rotationDistribution(0.20, 0.51);

    CaseGroup cases;
    cases.reserve(static_cast<std::size_t>(desiredCount));
    for (int attempt = 0;
         cases.size() < static_cast<std::size_t>(desiredCount) &&
         attempt < desiredCount * 80; ++attempt) {
        std::vector<double> lengths(static_cast<std::size_t>(n));
        std::vector<double> angles(static_cast<std::size_t>(n));
        for (int j = 0; j < n; ++j) {
            lengths[static_cast<std::size_t>(j)] = lengthDistribution(rng);
            if (j == 0) angles[0] = shoulderDistribution(rng);
            else angles[static_cast<std::size_t>(j)] =
                ((j + attempt) % 2 == 0 ? 1.0 : -1.0) *
                bendDistribution(rng);
        }
        const auto beginPositions = positionsFromAngles(lengths, angles);
        if (hasSelfCollision(beginPositions, settings.linkClearance)) continue;
        const auto reconstruction = jointsFromPositions(beginPositions);
        const auto bounds = limitsFor(reconstruction);
        std::vector<JointDefinition2D> definition;
        for (std::size_t j = 0; j < reconstruction.size(); ++j)
            definition.push_back({reconstruction[j].length,
                                  reconstruction[j].angle, bounds[j]});
        try {
            (void)makeRobot2D(definition, settings);
        } catch (const std::invalid_argument&) {
            continue;
        }

        auto endAngles = angles;
        // Mix shoulder-only rotation with cases that require coordinated
        // adjustments of proximal, distal, and multiple internal joints.
        // These are more informative than hundreds of rigidly rotated poses.
        const int mode = attempt % 5;
        endAngles[0] += (attempt % 2 ? 1.0 : -1.0) *
                        rotationDistribution(rng);
        for (int j = 1; j < n; ++j) {
            const bool change =
                (mode == 1 && j == 1) ||
                (mode == 2 && j == n - 1) ||
                (mode == 3 && j % 2 == 0) ||
                (mode == 4 && j % 2 == 1);
            if (change)
                endAngles[static_cast<std::size_t>(j)] +=
                    (j % 2 == 0 ? 1.0 : -1.0) *
                    (0.04 + 0.14 * (static_cast<double>((attempt + j) % 7) / 6.0));
        }
        const Vec2 target = positionsFromAngles(lengths, endAngles).back();
        if (!witnessIsSafe(lengths, angles, endAngles, target, settings)) continue;
        if ((target - beginPositions.back()).length() <= 0.1) continue;

        MotionCase test;
        test.name = "n=" + std::to_string(n) + " / case " +
                    std::to_string(cases.size() + 1);
        test.startPositions = beginPositions;
        test.target = target;
        cases.push_back(std::move(test));
    }
    check(cases.size() == static_cast<std::size_t>(desiredCount),
          "Could not create the required number of independently verified cases");
    return cases;
}

void runCase(const MotionCase& scenario, const MotionSettings2D& settings) {
    try {
        auto joints = jointsFromPositions(scenario.startPositions);
        const auto limits = limitsFor(joints);
        PlannedMotion2D follower;
        follower.setTarget(joints, limits, scenario.target, settings);
        check(follower.pathAvailable(), "Planner failed on a certified-reachable goal");

        auto previousPenetration = targetLinkPenetrations(
            scenario.startPositions, scenario.target, settings.targetClearance);
        for (int frame = 0; frame < maxFrames; ++frame) {
            const auto previous = joints;
            const auto status = follower.advance(joints, limits, dt, settings);
            check(!status.blocked, "Follower blocked");

            const auto points = forwardKinematics(joints);
            check(!hasSelfCollision(points, settings.linkClearance),
                  "Arm self-collision");

            const auto penetration = targetLinkPenetrations(
                points, scenario.target, settings.targetClearance);
            for (std::size_t i = 0; i < penetration.size(); ++i)
                check(penetration[i] <= previousPenetration[i] + 1e-7,
                      "New or deeper contact with target");
            previousPenetration = penetration;

            for (std::size_t i = 0; i < joints.size(); ++i) {
                const auto& limitsI = limits[i];
                check(joints[i].angle >= limitsI.minAngle - 1e-8 &&
                      joints[i].angle <= limitsI.maxAngle + 1e-8,
                      "Joint rotation exceeded configured limits");
                check(std::abs(joints[i].angularVelocity) <=
                      std::min(settings.maxAngularSpeed,
                               limitsI.maxAngularSpeed) + 1e-8,
                      "Joint speed exceeded limit");
                check(std::abs(joints[i].angularVelocity -
                               previous[i].angularVelocity) <=
                      std::min(settings.maxAngularAcceleration,
                               limitsI.maxAngularAcceleration) * dt + 1e-7,
                      "Joint acceleration exceeded limit");
            }
            if (status.reached) {
                check((points.back() - scenario.target).length() <=
                      settings.positionTolerance, "End effector missed target");
                return;
            }
        }
        throw std::runtime_error("Follower did not reach goal within frame budget");
    } catch (const std::exception& e) {
        throw std::runtime_error(describe(scenario) + "\nFAIL: " + e.what());
    }
}

void checkUnreachable(int n, const MotionCase& fixture,
                      const MotionSettings2D& settings) {
    auto joints = jointsFromPositions(fixture.startPositions);
    const auto limits = limitsFor(joints);
    double reach = 0.0;
    for (const auto& joint : joints) reach += joint.length;
    PlannedMotion2D follower;
    const Vec2 outside{reach + 15.0, 0.0};
    follower.setTarget(joints, limits, outside, settings);
    check(!follower.pathAvailable() && follower.searchFailed(),
          "n=" + std::to_string(n) + ": unreachable target must not be accepted");
}

} // namespace

int main(int argc, char** argv) {
    try {
        // 20 per group by default = 120 certified-reachable motions.
        // Explicit stress mode runs 100 per group = 600 motions.
        bool stress = false;
        bool list = false;
        for (int i = 1; i < argc; ++i) {
            const std::string argument(argv[i]);
            if (argument == "--stress") stress = true;
            else if (argument == "--list") list = true;
            else throw std::invalid_argument(
                "Usage: test_release_motions [--stress] [--list]");
        }
        const int perGroup = stress ? 100 : 20;
        const auto settings = safetySettings();

        std::array<CaseGroup, 6> groups;
        for (int n = 2; n <= 7; ++n)
            groups[static_cast<std::size_t>(n - 2)] =
                buildGroup(n, perGroup, settings);

        // Inspect or archive the entire Cartesian fixture corpus without
        // executing the trajectories; each line preserves all joint points.
        if (list) {
            for (int n = 2; n <= 7; ++n) {
                std::cout << "GROUP " << n << " joints\n";
                for (const auto& test : groups[static_cast<std::size_t>(n - 2)])
                    std::cout << describe(test) << "\n";
            }
            return 0;
        }

        int successful = 0;
        for (int n = 2; n <= 7; ++n) {
            const auto& group = groups[static_cast<std::size_t>(n - 2)];
            std::cout << n << " joints: " << group.size()
                      << " certified reachable transitions\n";
            for (const auto& test : group) {
                runCase(test, settings);
                ++successful;
            }
            checkUnreachable(n, group.front(), settings);
            std::cout << "  PASS " << group.size()
                      << " motions + 1 unreachable-goal check\n";
        }
        std::cout << "PASS: " << successful
                  << " complete motion simulations across 6 joint-count groups\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
