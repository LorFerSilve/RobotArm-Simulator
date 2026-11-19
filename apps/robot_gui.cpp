#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/SmoothMotion2D.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace {

using robotics::kinematics::Joint2D;
using robotics::math::Vec2;

constexpr int initialWidth = 1200;
constexpr int initialHeight = 760;
constexpr int sidebarWidth = 320;
constexpr int headerHeight = 62;
constexpr double fixedDt = 1.0 / 60.0;
constexpr double radToDeg = 180.0 / std::numbers::pi;

constexpr Color background{15, 21, 33, 255};
constexpr Color canvasColor{19, 28, 44, 255};
constexpr Color sidebarColor{25, 35, 52, 255};
constexpr Color muted{153, 173, 194, 255};
constexpr Color light{234, 242, 250, 255};
constexpr Color gridColor{39, 53, 72, 255};
constexpr Color blue{80, 167, 245, 255};
constexpr Color orange{248, 181, 88, 255};
constexpr Color green{80, 217, 166, 255};
constexpr Color red{246, 120, 122, 255};

std::vector<Joint2D> makeArm(int jointCount) {
    if (jointCount == 3) {
        return {
            {1.25, 0.3, 0.0},
            {0.75, -0.7, 0.0},
            {1.10, 0.8, 0.0}
        };
    }
    return {
        {2.0, std::numbers::pi / 2.0, 0.0},
        {1.0, -std::numbers::pi / 2.0, 0.0}
    };
}

struct Viewport {
    float centerX{};
    float centerY{};
    float pixelsPerMetre{};

    [[nodiscard]] Vector2 toScreen(Vec2 point) const {
        return {centerX + static_cast<float>(point.x) * pixelsPerMetre,
                centerY - static_cast<float>(point.y) * pixelsPerMetre};
    }

    [[nodiscard]] Vec2 toWorld(Vector2 point) const {
        return {(point.x - centerX) / pixelsPerMetre,
                (centerY - point.y) / pixelsPerMetre};
    }
};

Viewport makeViewport(int canvasWidth, int height, double maximumReach, float zoom) {
    constexpr float margin = 45.0F;
    const float usableWidth = static_cast<float>(canvasWidth) - 2.0F * margin;
    const float usableHeight = static_cast<float>(height - headerHeight) - 2.0F * margin;
    const float radius = static_cast<float>(maximumReach + 0.3);
    const float scale = std::min(usableWidth, usableHeight) / (2.0F * radius);
    return {static_cast<float>(canvasWidth) / 2.0F,
            (static_cast<float>(headerHeight + height) / 2.0F),
            scale * zoom};
}

void drawGrid(const Viewport& view, int canvasWidth, int height) {
    BeginScissorMode(0, headerHeight, canvasWidth, height - headerHeight);
    const double xRadius = static_cast<double>(canvasWidth) / (2.0 * view.pixelsPerMetre);
    const double yRadius = static_cast<double>(height - headerHeight) / (2.0 * view.pixelsPerMetre);
    constexpr double spacing = 0.5;

    for (int i = -static_cast<int>(std::ceil(xRadius / spacing));
         i <= static_cast<int>(std::ceil(xRadius / spacing)); ++i) {
        const auto x = view.toScreen({i * spacing, 0.0}).x;
        DrawLineV({x, static_cast<float>(headerHeight)}, {x, static_cast<float>(height)},
                  i == 0 ? Color{72, 95, 122, 255} : gridColor);
    }
    for (int i = -static_cast<int>(std::ceil(yRadius / spacing));
         i <= static_cast<int>(std::ceil(yRadius / spacing)); ++i) {
        const auto y = view.toScreen({0.0, i * spacing}).y;
        DrawLineV({0.0F, y}, {static_cast<float>(canvasWidth), y},
                  i == 0 ? Color{72, 95, 122, 255} : gridColor);
    }

    const auto origin = view.toScreen({0.0, 0.0});
    DrawText("0, 0", static_cast<int>(origin.x + 9.0F),
             static_cast<int>(origin.y + 8.0F), 13, muted);
    EndScissorMode();
}

void drawArm(const Viewport& view, const std::vector<Vec2>& positions,
             Vec2 target, bool reachable) {
    const auto destination = view.toScreen(target);
    const Color targetColor = reachable ? green : red;
    DrawCircleLines(static_cast<int>(destination.x), static_cast<int>(destination.y),
                    13.0F, targetColor);
    DrawLineV({destination.x - 19.0F, destination.y},
              {destination.x + 19.0F, destination.y}, targetColor);
    DrawLineV({destination.x, destination.y - 19.0F},
              {destination.x, destination.y + 19.0F}, targetColor);

    const Color linkColors[] = {blue, orange, green};
    for (std::size_t i = 1; i < positions.size(); ++i) {
        DrawLineEx(view.toScreen(positions[i - 1]), view.toScreen(positions[i]),
                   10.0F, linkColors[(i - 1) % 3]);
    }
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto p = view.toScreen(positions[i]);
        DrawCircleV(p, i + 1 == positions.size() ? 10.0F : 8.0F, light);
        DrawCircleV(p, i + 1 == positions.size() ? 5.0F : 4.0F,
                    i + 1 == positions.size() ? green : background);
    }
}

void drawSidebar(int screenWidth, int height, const std::vector<Joint2D>& joints,
                 Vec2 target, Vec2 endEffector, Vec2 tipVelocity,
                 double error, double elapsed, bool paused,
                 bool reached, bool stalled, bool constrained, bool unreachable) {
    const int x = screenWidth - sidebarWidth;
    DrawRectangle(x, headerHeight, sidebarWidth, height - headerHeight, sidebarColor);
    DrawLine(x, headerHeight, x, height, gridColor);
    const int left = x + 22;
    int y = headerHeight + 26;
    DrawText("SIMULATION", left, y, 19, light);
    y += 36;

    const char* status = paused ? "PAUSED" : reached ? "TARGET REACHED" :
        unreachable ? "UNREACHABLE" :
        stalled ? "BLOCKED" : constrained ? "AVOIDING CONTACT" : "MOVING";
    const Color statusColor = paused ? orange : reached ? green :
        (stalled || unreachable) ? red : constrained ? orange : blue;
    DrawText(status, left, y, 16, statusColor);
    y += 33;
    DrawText(TextFormat("Time: %.2f s", elapsed), left, y, 16, muted);
    y += 34;
    DrawLine(x + 20, y, screenWidth - 20, y, gridColor);
    y += 20;

    DrawText("CARTESIAN SPACE", left, y, 17, light);
    y += 31;
    DrawText(TextFormat("Target:  (%.2f, %.2f) m", target.x, target.y), left, y, 15, muted);
    y += 27;
    DrawText(TextFormat("Tip:     (%.2f, %.2f) m", endEffector.x, endEffector.y), left, y, 15, muted);
    y += 27;
    DrawText(TextFormat("Error:   %.4f m", error), left, y, 15, reached ? green : light);
    y += 27;
    DrawText(TextFormat("Speed:   %.3f m/s", tipVelocity.length()), left, y, 15, muted);
    y += 34;
    DrawLine(x + 20, y, screenWidth - 20, y, gridColor);
    y += 20;

    DrawText("JOINT STATE", left, y, 17, light);
    y += 32;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        DrawText(TextFormat("Joint %i", static_cast<int>(i + 1)), left, y, 15,
                 i == 0 ? blue : i == 1 ? orange : green);
        y += 25;
        DrawText(TextFormat("Angle: %7.2f deg", joints[i].angle * radToDeg),
                 left + 9, y, 15, muted);
        y += 22;
        DrawText(TextFormat("Rate:  %7.2f deg/s", joints[i].angularVelocity * radToDeg),
                 left + 9, y, 15, muted);
        y += 32;
    }


}

} // namespace

int main() {
    using robotics::kinematics::advanceTowardTarget;
    using robotics::kinematics::MotionStatus2D;
    using robotics::kinematics::MotionSettings2D;
    using robotics::kinematics::forwardKinematics;
    using robotics::kinematics::endEffectorJacobian;
    using robotics::kinematics::endEffectorVelocity;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(initialWidth, initialHeight, "RobotArm Simulator - 2D");
    SetWindowMinSize(980, 720);
    SetTargetFPS(60);

    int jointCount = 2;
    auto joints = makeArm(jointCount);
    Vec2 target{1.5, 1.5};
    double simulatedTime = 0.0;
    double accumulator = 0.0;
    bool paused = false;
    float zoom = 1.0F;
    MotionStatus2D motionState{};
    const MotionSettings2D motionSettings{};

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
            accumulator = 0.0;
            if (paused) {
                for (auto& joint : joints) joint.angularVelocity = 0.0;
            }
        }
        const bool reset = IsKeyPressed(KEY_R);
        const bool selectTwo = IsKeyPressed(KEY_TWO);
        const bool selectThree = IsKeyPressed(KEY_THREE);
        if (reset || selectTwo || selectThree) {
            if (selectTwo) jointCount = 2;
            if (selectThree) jointCount = 3;
            joints = makeArm(jointCount);
            target = {1.5, 1.5};
            simulatedTime = 0.0;
            accumulator = 0.0;
            paused = false;
            motionState = {};
        }

        const int width = GetScreenWidth();
        const int height = GetScreenHeight();
        const int canvasWidth = width - sidebarWidth;
        double reach = 0.0;
        for (const auto& joint : joints) reach += joint.length;
        const Vector2 mouse = GetMousePosition();
        const bool mouseInCanvas = mouse.x >= 0.0F &&
            mouse.x < static_cast<float>(canvasWidth) &&
            mouse.y >= static_cast<float>(headerHeight) &&
            mouse.y < static_cast<float>(height);
        if (mouseInCanvas) {
            zoom = std::clamp(zoom + GetMouseWheelMove() * 0.12F, 0.55F, 2.5F);
        }
        const auto viewport = makeViewport(canvasWidth, height, reach, zoom);
        if (mouseInCanvas && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            target = viewport.toWorld(mouse);
        }

        if (!paused) {
            // Fixed-step control, independent of rendering refresh rate.
            accumulator += std::min(static_cast<double>(GetFrameTime()), 0.1);
            for (int steps = 0; accumulator >= fixedDt && steps < 6; ++steps) {
                motionState = advanceTowardTarget(joints, target, fixedDt, motionSettings);
                simulatedTime += fixedDt;
                accumulator -= fixedDt;
            }
        }

        const auto positions = forwardKinematics(joints);
        const auto jacobian = endEffectorJacobian(positions);
        const auto velocity = endEffectorVelocity(jacobian, joints);
        const double error = (target - positions.back()).length();
        const bool reached = error <= 0.005;
        const bool stalled = !reached && motionState.blocked;
        const double radius = target.length();
        bool unreachable = radius > reach;
        if (joints.size() == 2) {
            const double margin = motionSettings.jointClearance;
            const double innerRadius = std::hypot(joints[0].length + joints[1].length *
                std::cos(std::numbers::pi - margin), joints[1].length *
                std::sin(std::numbers::pi - margin));
            const double outerRadius = std::hypot(joints[0].length + joints[1].length *
                std::cos(margin), joints[1].length * std::sin(margin));
            unreachable = radius < innerRadius || radius > outerRadius;
        }

        BeginDrawing();
        ClearBackground(background);
        DrawRectangle(0, headerHeight, canvasWidth, height - headerHeight, canvasColor);
        drawGrid(viewport, canvasWidth, height);
        BeginScissorMode(0, headerHeight, canvasWidth, height - headerHeight);
        drawArm(viewport, positions, target, !unreachable);
        EndScissorMode();
        DrawText("CLICK/DRAG Target   SPACE Pause   R Reset   2/3 Joints   WHEEL Zoom",
                 20, height - 28, 15, muted);
        DrawRectangle(0, 0, width, headerHeight, background);
        DrawText("ROBOTARM", 23, 16, 25, light);
        DrawText("2D Kinematics Laboratory", 197, 22, 17, muted);
        drawSidebar(width, height, joints, target, positions.back(), velocity,
                    error, simulatedTime, paused, reached, stalled,
                    motionState.constrained, unreachable);
        EndDrawing();
    }

    CloseWindow();
}
