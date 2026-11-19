#pragma once

#include <cmath>

namespace robotics::math {

struct Vec2 {
    double x{};
    double y{};

    [[nodiscard]] constexpr Vec2 operator+(const Vec2& other) const noexcept {
        return {x + other.x, y + other.y};
    }

    [[nodiscard]] constexpr Vec2 operator-(const Vec2& other) const noexcept {
        return {x - other.x, y - other.y};
    }

    [[nodiscard]] constexpr Vec2 operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar};
    }

    [[nodiscard]] constexpr double dot(const Vec2& other) const noexcept {
        return x * other.x + y * other.y;
    }

    [[nodiscard]] double length() const {
        return std::hypot(x, y);
    }
};

} // namespace robotics::math
