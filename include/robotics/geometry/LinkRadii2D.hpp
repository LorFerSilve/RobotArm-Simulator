#pragma once

#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>

namespace robotics::geometry {

[[nodiscard]] inline bool validLinkRadius(double radius) noexcept {
    return std::isfinite(radius) && radius >= 0.0;
}

// An empty span denotes the legacy centreline model. Otherwise it must
// contain one physical radius per link, in base-to-tip order.
inline void validateLinkRadii(std::span<const double> radii, std::size_t linkCount) {
    if (!radii.empty() && radii.size() != linkCount)
        throw std::invalid_argument("Link radii must match the number of links");
    for (const double radius : radii)
        if (!validLinkRadius(radius))
            throw std::invalid_argument("Link radii must be finite and nonnegative");
}

[[nodiscard]] inline double linkRadiusAt(std::span<const double> radii,
                                         std::size_t link) noexcept {
    return radii.empty() ? 0.0 : radii[link];
}

} // namespace robotics::geometry
