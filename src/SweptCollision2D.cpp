#include "robotics/kinematics/SweptCollision2D.hpp"

#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/SmoothMotion2D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace robotics::kinematics {

bool sweptArmClear2D(
    std::span<const Joint2D> joints,
    std::span<const double> startAngles,
    std::span<const double> endAngles,
    std::span<const geometry::Obstacle2D> obstacles,
    double selfClearance,
    double obstacleClearance,
    std::stop_token cancellation) {
    if (joints.empty() || joints.size() != startAngles.size() ||
        joints.size() != endAngles.size() ||
        !std::isfinite(selfClearance) || selfClearance < 0.0 ||
        !std::isfinite(obstacleClearance) || obstacleClearance < 0.0)
        throw std::invalid_argument("Invalid swept collision parameters");

    double cumulativeAngularRate = 0.0;
    double maximumPointSpeed = 0.0;
    double reach = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (!std::isfinite(joints[i].length) || joints[i].length <= 0.0 ||
            !std::isfinite(startAngles[i]) || !std::isfinite(endAngles[i]))
            throw std::invalid_argument("Invalid swept joint geometry");
        reach += joints[i].length;
        cumulativeAngularRate += std::abs(endAngles[i] - startAngles[i]);
        maximumPointSpeed += joints[i].length * cumulativeAngularRate;
    }
    if (!std::isfinite(maximumPointSpeed))
        throw std::overflow_error("Swept movement bound overflow");
    for (const auto& obstacle : obstacles)
        if (!geometry::validObstacle(obstacle))
            throw std::invalid_argument("Invalid swept obstacle");

    if (cancellation.stop_requested()) return false;
    // With two links, there are no nonadjacent links to collide.
    if (obstacles.empty() && joints.size() <= 2) return true;

    constexpr int maxDepth = 24;
    std::vector<double> angles(joints.size());
    std::vector<math::Vec2> positions;
    positions.reserve(joints.size() + 1);

    // A simple Lipschitz bound: for any t in [lo,hi], any link endpoint
    // moves at most (hi-lo)/2 * maximumPointSpeed relative to the midpoint.
    // An entire link segment stays within the same displacement radius
    // because each point is a convex combination of its two endpoints.
    //
    // Therefore if every midpoint segment is farther than that radius
    // from scene obstacles, and nonadjacent links are separated by more
    // than twice the radius, ALL intervening poses are collision-free.
    // Small numerical guard errs on the side of rejecting near-contact.
    const double guard = 1e-12 * std::max(1.0, reach);

    const auto certify = [&](auto&& self, double lo, double hi, int depth) -> bool {
        if (cancellation.stop_requested()) return false;
        const double mid = lo + (hi - lo) * 0.5;
        for (std::size_t i = 0; i < angles.size(); ++i)
            angles[i] = startAngles[i] +
                        mid * (endAngles[i] - startAngles[i]);
        forwardKinematics(joints, angles, positions);

        // Midpoint collision means the entire edge is not safe.
        if (!obstacles.empty() &&
            geometry::hasObstacleCollision(
                positions, obstacles, obstacleClearance))
            return false;
        if (joints.size() >= 3 &&
            hasSelfCollision(positions, selfClearance))
            return false;

        const double displacement =
            (hi - lo) * 0.5 * maximumPointSpeed + guard;
        const bool sceneCertified = obstacles.empty() ||
            !geometry::hasObstacleCollision(
                positions, obstacles, obstacleClearance + displacement);
        const bool selfCertified = joints.size() <= 2 ||
            !hasSelfCollision(positions, selfClearance + 2.0 * displacement);
        if (sceneCertified && selfCertified) return true;

        // No maximum-interval assumption: if we cannot prove safety,
        // conservatively reject instead of allowing a missed collision.
        if (depth == maxDepth) return false;
        return self(self, lo, mid, depth + 1) &&
               self(self, mid, hi, depth + 1);
    };

    return certify(certify, 0.0, 1.0, 0);
}

} // namespace robotics::kinematics
