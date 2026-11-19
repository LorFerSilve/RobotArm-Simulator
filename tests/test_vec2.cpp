#include "robotics/math/Vec2.hpp"

#include <cmath>
#include <iostream>

using robotics::math::Vec2;

bool near(double a, double b) {
    return std::abs(a - b) < 1e-12;
}

int main() {
    constexpr Vec2 a{3.0, 4.0};
    constexpr Vec2 b{1.0, -2.0};
    constexpr Vec2 sum = a + b;
    constexpr Vec2 difference = a - b;
    constexpr Vec2 scaled = a * 2.0;
    static_assert(sum.x == 4.0 && sum.y == 2.0);
    static_assert(a.dot(b) == -5.0);

    int failures = 0;
    const auto check = [&failures](bool condition, const char* name) {
        if (!condition) {
            std::cerr << "FAILED: " << name << '\n';
            ++failures;
        }
    };

    check(near(sum.x, 4.0) && near(sum.y, 2.0), "addition");
    check(near(difference.x, 2.0) && near(difference.y, 6.0), "subtraction");
    check(near(scaled.x, 6.0) && near(scaled.y, 8.0), "scalar multiplication");
    check(near(a.dot(b), -5.0), "dot product");
    check(near(a.length(), 5.0), "length");

    return failures == 0 ? 0 : 1;
}
