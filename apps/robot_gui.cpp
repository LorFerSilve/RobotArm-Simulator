#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/SmoothMotion2D.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/RobotConfiguration2D.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"

#include <raylib.h>

#include <algorithm>
#include <array>
#include <string>
#include <stdexcept>
#include <cmath>
#include <numbers>
#include <vector>

namespace {

using robotics::kinematics::Joint2D;
using robotics::math::Vec2;

constexpr int initialWidth = 1260;
constexpr int initialHeight = 760;
constexpr int sidebarWidth = 350;
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

using robotics::kinematics::JointDefinition2D;
using robotics::kinematics::JointLimits2D;
using robotics::kinematics::Robot2D;
using robotics::kinematics::makeRobot2D;

constexpr double degToRad = std::numbers::pi / 180.0;
constexpr std::size_t maxGuiJoints = 6;

JointDefinition2D makeNewLink(bool positiveBranch) {
    const double margin = 0.16;
    const double foldMargin = 0.56; // GUI physical folding clearance
    const double angle = positiveBranch ? 0.85 : -0.85;
    return {0.7, angle,
            {positiveBranch ? margin : -std::numbers::pi + foldMargin,
             positiveBranch ? std::numbers::pi - foldMargin : -margin,
             2.0, 6.0}};
}

std::vector<JointDefinition2D> makePreset(int count) {
    const JointDefinition2D base{2.0, std::numbers::pi / 2.0,
        {-2.0 * std::numbers::pi, 2.0 * std::numbers::pi, 2.0, 6.0}};
    const JointDefinition2D elbow{1.0, -std::numbers::pi / 2.0,
        {-std::numbers::pi + 0.56, -0.16, 2.0, 6.0}};
    if (count == 2) return {base, elbow};
    if (count == 3) {
        return {
            {1.25, 0.3, base.limits},
            {0.75, -0.7, elbow.limits},
            {1.1, 0.8, {0.16, std::numbers::pi - 0.56, 2.0, 6.0}}
        };
    }
    // Four/five-link presets: an unfolded, collision-free arm with 2 m
    // links. The five-link tip starts near (4, -9), useful for regression
    // tests of recovery toward (-3, -4).
    constexpr double radians = std::numbers::pi / 180.0;
    const JointLimits2D positive{0.16, std::numbers::pi - 0.56, 2.0, 6.0};
    std::vector<JointDefinition2D> result{
        {2.0, -57.0 * radians, base.limits},
        {2.0, -21.0 * radians, elbow.limits},
        {2.0, 26.0 * radians, positive},
        {2.0, -35.0 * radians, elbow.limits}
    };
    if (count == 5) result.push_back({2.0, 32.0 * radians, positive});
    return result;
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

bool button(int x, int y, int w, int h, const char* label,
            Vector2 mouse, bool pressed, bool enabled = true) {
    const Rectangle rect{static_cast<float>(x), static_cast<float>(y),
                         static_cast<float>(w), static_cast<float>(h)};
    const bool hover = enabled && CheckCollisionPointRec(mouse, rect);
    DrawRectangleRec(rect, !enabled ? gridColor : hover ? blue : Color{47, 65, 86, 255});
    const int font = 15;
    DrawText(label, x + (w - MeasureText(label, font)) / 2,
             y + (h - font) / 2, font, enabled ? light : muted);
    return enabled && hover && pressed;
}

// Editor changes are kept in a draft. Apply creates a validated new robot
// atomically, so an invalid intermediate edit never damages a live simulation.
void drawConfigurator(int x, int height, std::vector<JointDefinition2D>& draft,
                      std::size_t& selected, bool& applyRequested,
                      std::string& feedback, Vector2 mouse, bool click) {
    const int left = x + 20;
    int y = headerHeight + 84;
    DrawText("ROBOT CONFIGURATION", left, y, 19, light);
    y += 41;
    if (button(left, y, 127, 30, "+ ADD JOINT", mouse, click,
               draft.size() < maxGuiJoints)) {
        const bool positive = std::signbit(draft.back().initialAngle);
        draft.push_back(makeNewLink(positive));
        selected = draft.size() - 1;
        feedback = "Changes pending: press APPLY";
    }
    if (button(left + 142, y, 163, 30, "- REMOVE JOINT", mouse, click,
               draft.size() > 2)) {
        draft.pop_back();
        selected = std::min(selected, draft.size() - 1);
        feedback = "Changes pending: press APPLY";
    }
    y += 43;
    if (button(left, y, 40, 30, "<", mouse, click, selected > 0)) --selected;
    DrawText(TextFormat("Joint %d of %d", static_cast<int>(selected + 1),
                        static_cast<int>(draft.size())), left + 70, y + 6, 17, light);
    if (button(left + 265, y, 40, 30, ">", mouse, click,
               selected + 1 < draft.size())) ++selected;
    y += 48;

    auto& item = draft[selected];
    const std::array<const char*, 6> labels{
        "Length (m)", "Initial angle (deg)", "Minimum angle (deg)",
        "Maximum angle (deg)", "Max speed (deg/s)", "Max accel (deg/s^2)"};
    const std::array<double, 6> values{
        item.length, item.initialAngle * radToDeg,
        item.limits.minAngle * radToDeg, item.limits.maxAngle * radToDeg,
        item.limits.maxAngularSpeed * radToDeg,
        item.limits.maxAngularAcceleration * radToDeg};
    const std::array<double, 6> steps{0.1, 5.0, 5.0, 5.0, 10.0, 20.0};

    for (std::size_t i = 0; i < labels.size(); ++i) {
        DrawText(labels[i], left, y, 14, muted);
        DrawText(TextFormat(i == 0 ? "%.2f" : "%.0f", values[i]),
                 left + 162, y, 16, light);
        const bool minus = button(left + 233, y - 5, 32, 27, "-", mouse, click);
        const bool plus = button(left + 273, y - 5, 32, 27, "+", mouse, click);
        if (minus || plus) {
            const double change = steps[i] * (plus ? 1.0 : -1.0);
            switch (i) {
            case 0: item.length = std::clamp(item.length + change, 0.1, 5.0); break;
            case 1: item.initialAngle += change * degToRad; break;
            case 2: item.limits.minAngle += change * degToRad; break;
            case 3: item.limits.maxAngle += change * degToRad; break;
            case 4: item.limits.maxAngularSpeed =
                std::clamp(item.limits.maxAngularSpeed + change * degToRad, 10.0 * degToRad, 6.0); break;
            case 5: item.limits.maxAngularAcceleration =
                std::clamp(item.limits.maxAngularAcceleration + change * degToRad, 20.0 * degToRad, 20.0); break;
            }
            feedback = "Changes pending: press APPLY";
        }
        y += 45;
    }
    y += 10;
    if (button(left, y, 305, 35, "APPLY & RESET ROBOT", mouse, click))
        applyRequested = true;
    y += 46;
    DrawText("Invalid edits are rejected on APPLY.", left, y, 13, muted);
    y += 23;
    if (!feedback.empty()) {
        const std::string line = feedback.substr(0, 41);
        DrawText(line.c_str(), left, y, 13, feedback.rfind("Error:", 0) == 0 ? red : green);
        if (feedback.size() > 41 && y + 36 < height)
            DrawText(feedback.substr(41, 41).c_str(), left, y + 17, 13, red);
    }
}

void drawSidebar(int screenWidth, int height, const std::vector<Joint2D>& joints,
                 Vec2 target, Vec2 endEffector, Vec2 tipVelocity,
                 double error, double elapsed, bool paused,
                 bool reached, bool stalled, bool constrained, bool unreachable,
                 bool noPath) {
    const int x = screenWidth - sidebarWidth;
    DrawRectangle(x, headerHeight + 64, sidebarWidth, height - headerHeight - 64, sidebarColor);
    DrawLine(x, headerHeight, x, height, gridColor);
    const int left = x + 22;
    int y = headerHeight + 83;
    DrawText("SIMULATION", left, y, 19, light);
    y += 36;

    const char* status = paused ? "PAUSED" : reached ? "TARGET REACHED" :
        noPath ? "NO SAFE PATH FOUND" : unreachable ? "UNREACHABLE" :
        stalled ? "BLOCKED" : constrained ? "AVOIDING CONTACT" : "MOVING";
    const Color statusColor = paused ? orange : reached ? green :
        (stalled || unreachable || noPath) ? red : constrained ? orange : blue;
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
    using robotics::kinematics::PlannedMotion2D;
    using robotics::kinematics::MotionSettings2D;
    using robotics::kinematics::forwardKinematics;
    using robotics::kinematics::endEffectorJacobian;
    using robotics::kinematics::endEffectorVelocity;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(initialWidth, initialHeight, "RobotArm Simulator - 2D");
    SetWindowMinSize(980, 720);
    SetTargetFPS(60);

    int jointCount = 2;
    const robotics::kinematics::MotionSettings2D motionSettings = [] {
        robotics::kinematics::MotionSettings2D settings{};
        settings.foldClearance = 0.55; // about 31.5 degrees away from a full fold
        settings.targetClearance = 0.16; // target body/arm clearance, metres
        return settings;
    }();
    auto appliedDefinition = makePreset(jointCount);
    auto draft = appliedDefinition;
    Robot2D robot = makeRobot2D(appliedDefinition, motionSettings);
    auto& joints = robot.joints;
    std::size_t selectedJoint = 0;
    bool configTab = false;
    std::string feedback;
    Vec2 target{1.5, 1.5};
    double simulatedTime = 0.0;
    double accumulator = 0.0;
    bool paused = false;
    float zoom = 1.0F;
    MotionStatus2D motionState{};
    PlannedMotion2D plannedMotion;
    bool pendingPlan = true;
    bool draggingTarget = false;

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
        const bool selectFour = IsKeyPressed(KEY_FOUR);
        const bool selectFive = IsKeyPressed(KEY_FIVE);
        if (reset || selectTwo || selectThree || selectFour || selectFive) {
            if (selectTwo || selectThree || selectFour || selectFive) {
                jointCount = selectFive ? 5 : selectFour ? 4 : selectThree ? 3 : 2;
                appliedDefinition = makePreset(jointCount);
                draft = appliedDefinition;
                selectedJoint = 0;
                feedback.clear();
            }
            robot = makeRobot2D(appliedDefinition, motionSettings);
            target = jointCount >= 4 ? forwardKinematics(joints).back() :
                                          Vec2{1.5, 1.5};
            simulatedTime = 0.0;
            accumulator = 0.0;
            paused = false;
            motionState = {};
            plannedMotion.reset();
            pendingPlan = true;
            draggingTarget = false;
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
            draggingTarget = true;
        }
        if (draggingTarget && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            draggingTarget = false;
            pendingPlan = true;
        }

        if (!paused) {
            // Plan a safe joint-space route once the target is committed.
            // Retain immediate analytic motion while dragging a 2-link arm.
            accumulator += std::min(static_cast<double>(GetFrameTime()), 0.1);
            for (int steps = 0; accumulator >= fixedDt && steps < 6; ++steps) {
                if (draggingTarget) {
                    if (joints.size() == 2) {
                        motionState = advanceTowardTarget(joints, target, fixedDt,
                                                          motionSettings, robot.limits);
                    } else {
                        // Wait for the committed target; do not follow an old path.
                        for (auto& joint : joints) joint.angularVelocity = 0.0;
                    }
                } else {
                    if (pendingPlan) {
                        plannedMotion.setTarget(joints, robot.limits, target, motionSettings);
                        pendingPlan = false;
                    }
                    motionState = plannedMotion.advance(joints, robot.limits,
                                                         fixedDt, motionSettings);
                }
                simulatedTime += fixedDt;
                accumulator -= fixedDt;
            }
        }

        const auto positions = forwardKinematics(joints);
        const auto jacobian = endEffectorJacobian(positions);
        const auto velocity = endEffectorVelocity(jacobian, joints);
        const double error = (target - positions.back()).length();
        const bool reached = !draggingTarget && motionState.reached &&
                             error <= motionSettings.positionTolerance;
        const bool stalled = !reached && motionState.blocked;
        const double radius = target.length();
        bool unreachable = radius > reach;
        if (joints.size() == 2) {
            const double margin = motionSettings.jointClearance;
            const double innerRadius = std::hypot(joints[0].length + joints[1].length *
                std::cos(std::numbers::pi - motionSettings.foldClearance), joints[1].length *
                std::sin(std::numbers::pi - motionSettings.foldClearance));
            const double outerRadius = std::hypot(joints[0].length + joints[1].length *
                std::cos(margin), joints[1].length * std::sin(margin));
            unreachable = radius < innerRadius || radius > outerRadius;
        }

        BeginDrawing();
        ClearBackground(background);
        DrawRectangle(0, headerHeight, canvasWidth, height - headerHeight, canvasColor);
        drawGrid(viewport, canvasWidth, height);
        BeginScissorMode(0, headerHeight, canvasWidth, height - headerHeight);
        drawArm(viewport, positions, target,
                !unreachable && !(!draggingTarget && plannedMotion.searchFailed()));
        EndScissorMode();
        DrawText("CLICK/DRAG Target   SPACE Pause   R Reset   2-5 Presets   WHEEL Zoom",
                 20, height - 28, 15, muted);
        DrawRectangle(0, 0, width, headerHeight, background);
        DrawText("ROBOTARM", 23, 16, 25, light);
        DrawText("2D Kinematics Laboratory", 197, 22, 17, muted);
        const int panelX = width - sidebarWidth;
        DrawRectangle(panelX, headerHeight, sidebarWidth,
                      height - headerHeight, sidebarColor);
        const bool click = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
        const Vector2 cursor = GetMousePosition();
        if (button(panelX + 20, headerHeight + 14, 145, 35, "TELEMETRY",
                   cursor, click)) configTab = false;
        if (button(panelX + 182, headerHeight + 14, 148, 35, "CONFIGURE",
                   cursor, click)) configTab = true;
        if (configTab) {
            bool apply = false;
            drawConfigurator(panelX, height, draft, selectedJoint,
                             apply, feedback, cursor, click);
            if (apply) {
                try {
                    auto next = makeRobot2D(draft, motionSettings);
                    robot = std::move(next);
                    appliedDefinition = draft;
                    jointCount = static_cast<int>(draft.size());
                    target = forwardKinematics(joints).back();
                    simulatedTime = 0.0;
                    accumulator = 0.0;
                    paused = true;
                    motionState = {};
                    plannedMotion.reset();
                    pendingPlan = true;
                    draggingTarget = false;
                    feedback = "Applied. SPACE to move.";
                } catch (const std::invalid_argument& ex) {
                    feedback = std::string("Error: ") + ex.what();
                }
            }
        } else {
            drawSidebar(width, height, joints, target, positions.back(), velocity,
                        error, simulatedTime, paused, reached, stalled,
                        motionState.constrained, unreachable,
                        !draggingTarget && plannedMotion.searchFailed());
        }
        EndDrawing();
    }

    CloseWindow();
}
