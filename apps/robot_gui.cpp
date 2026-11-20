#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/SmoothMotion2D.hpp"
#include "robotics/kinematics/PathPlanning2D.hpp"
#include "robotics/kinematics/AsyncPlanner2D.hpp"
#include "robotics/kinematics/RobotConfiguration2D.hpp"
#include "robotics/kinematics/Jacobian2D.hpp"
#include "robotics/geometry/Obstacles2D.hpp"

#include <raylib.h>

#include <algorithm>
#include <array>
#include <string>
#include <stdexcept>
#include <cmath>
#include <numbers>
#include <vector>
#include <span>
#include <type_traits>
#include <variant>

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

// A subdued end-effector trajectory silhouette. Points come from FK samples
// of the planner's verified spline (or its fallback joint-space polyline),
// not from a naive straight line between the tool and its Cartesian goal.
void drawEndEffectorTrace(const Viewport& view, std::span<const Vec2> trace) {
    if (trace.size() < 2) return;
    constexpr Color pathColor{104, 198, 255, 156};
    constexpr Color markerColor{104, 198, 255, 205};
    for (std::size_t i = 1; i < trace.size(); ++i) {
        DrawLineEx(view.toScreen(trace[i - 1]), view.toScreen(trace[i]),
                   2.0F, pathColor);
    }
    // Small markers establish direction without obscuring joints/obstacles.
    for (int marker = 1; marker <= 3; ++marker) {
        const std::size_t index = (trace.size() - 1) *
                                  static_cast<std::size_t>(marker) / 4;
        DrawCircleV(view.toScreen(trace[index]), 3.2F, markerColor);
    }
    DrawText("PLANNED TIP PATH", 20, headerHeight + 15, 13, markerColor);
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


using robotics::geometry::Obstacle2D;
using robotics::geometry::Circle2D;
using robotics::geometry::Box2D;
namespace geom = robotics::geometry;

enum class CanvasTool { Target, PlaceCircle, PlaceBox, MoveObstacle };

const char* toolName(CanvasTool tool) {
    switch (tool) {
    case CanvasTool::Target: return "TARGET";
    case CanvasTool::PlaceCircle: return "PLACE CIRCLE";
    case CanvasTool::PlaceBox: return "PLACE BOX";
    case CanvasTool::MoveObstacle: return "MOVE / SELECT";
    }
    return "TARGET";
}

bool shapeSafe(const Obstacle2D& obstacle, const std::vector<Vec2>& positions,
               double margin) {
    return geom::validObstacle(obstacle) &&
        !geom::hasObstacleCollision(positions,
                                   std::span<const Obstacle2D>(&obstacle, 1),
                                   margin);
}

void moveObstacle(Obstacle2D& obstacle, Vec2 position) {
    std::visit([&](auto& shape) { shape.center = position; }, obstacle);
}

void drawScene(const Viewport& view, const std::vector<Obstacle2D>& obstacles,
               int selected) {
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
        const bool highlighted = static_cast<int>(i) == selected;
        const Color fill = highlighted ? Color{248, 181, 88, 100} :
                                          Color{246, 120, 122, 80};
        const Color edge = highlighted ? orange : red;
        std::visit([&](const auto& shape) {
            using Shape = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<Shape, Circle2D>) {
                const auto center = view.toScreen(shape.center);
                const float r = static_cast<float>(shape.radius) * view.pixelsPerMetre;
                DrawCircleV(center, r, fill);
                DrawCircleLines(static_cast<int>(center.x),
                                static_cast<int>(center.y), r, edge);
            } else {
                const auto leftTop = view.toScreen(
                    {shape.center.x - shape.halfWidth,
                     shape.center.y + shape.halfHeight});
                const Rectangle rect{leftTop.x, leftTop.y,
                    static_cast<float>(2.0 * shape.halfWidth) * view.pixelsPerMetre,
                    static_cast<float>(2.0 * shape.halfHeight) * view.pixelsPerMetre};
                DrawRectangleRec(rect, fill);
                DrawRectangleLinesEx(rect, highlighted ? 3.0F : 2.0F, edge);
            }
        }, obstacles[i]);
    }
}

bool drawObstaclePanel(int x, int height, CanvasTool& tool,
                       std::vector<Obstacle2D>& obstacles, int& selected,
                       std::string& message, const std::vector<Vec2>& positions,
                       double clearance, Vector2 mouse, bool click) {
    const int left = x + 20;
    int y = headerHeight + 83;
    bool changed = false;
    DrawText("SCENE OBSTACLES", left, y, 19, light);
    y += 39;
    if (button(left, y, 146, 32, "TARGET  [T]", mouse, click))
        tool = CanvasTool::Target;
    if (button(left + 159, y, 146, 32, "MOVE  [V]", mouse, click))
        tool = CanvasTool::MoveObstacle;
    y += 42;
    if (button(left, y, 146, 32, "+ CIRCLE  [C]", mouse, click,
               obstacles.size() < 16))
        tool = CanvasTool::PlaceCircle;
    if (button(left + 159, y, 146, 32, "+ BOX  [B]", mouse, click,
               obstacles.size() < 16))
        tool = CanvasTool::PlaceBox;
    y += 45;
    DrawText(TextFormat("Tool: %s", toolName(tool)), left, y, 15, green);
    y += 26;
    DrawText("Left-click in canvas to place/select.", left, y, 13, muted);
    y += 21;
    DrawText("Drag selected shape to move it.", left, y, 13, muted);
    y += 21;
    DrawText("Changes pause and invalidate the path.", left, y, 13, muted);
    y += 31;
    DrawLine(left, y, left + 305, y, gridColor);
    y += 25;

    DrawText(TextFormat("Obstacles: %d / 16",
                        static_cast<int>(obstacles.size())), left, y, 17, light);
    y += 37;
    if (selected >= 0 && static_cast<std::size_t>(selected) < obstacles.size()) {
        const auto& obstacle = obstacles[static_cast<std::size_t>(selected)];
        const bool circle = std::holds_alternative<Circle2D>(obstacle);
        const Vec2 center = std::visit([](const auto& shape) {
            return shape.center;
        }, obstacle);
        DrawText(TextFormat("Selected #%d : %s", selected + 1,
                            circle ? "CIRCLE" : "RECTANGLE"), left, y, 16, orange);
        y += 29;
        DrawText(TextFormat("Center: (%.2f, %.2f) m", center.x, center.y),
                 left, y, 15, muted);
        y += 31;

        const int rows = circle ? 1 : 2;
        for (int axis = 0; axis < rows; ++axis) {
            const double size = circle ?
                std::get<Circle2D>(obstacle).radius :
                axis == 0 ? std::get<Box2D>(obstacle).halfWidth * 2.0 :
                            std::get<Box2D>(obstacle).halfHeight * 2.0;
            DrawText(circle ? "Radius" : axis == 0 ? "Width" : "Height",
                     left, y + 4, 15, muted);
            DrawText(TextFormat("%.2f m", size), left + 135, y + 4, 15, light);
            const bool minus = button(left + 234, y, 30, 28, "-", mouse, click);
            const bool plus = button(left + 274, y, 30, 28, "+", mouse, click);
            if (minus || plus) {
                auto updated = obstacle;
                const double change = (plus ? 0.1 : -0.1);
                if (circle) {
                    auto& shape = std::get<Circle2D>(updated);
                    shape.radius = std::clamp(shape.radius + change, 0.1, 5.0);
                } else {
                    auto& shape = std::get<Box2D>(updated);
                    if (axis == 0) shape.halfWidth =
                        std::clamp(shape.halfWidth + change * 0.5, 0.1, 5.0);
                    else shape.halfHeight =
                        std::clamp(shape.halfHeight + change * 0.5, 0.1, 5.0);
                }
                if (shapeSafe(updated, positions, clearance)) {
                    obstacles[static_cast<std::size_t>(selected)] = updated;
                    changed = true;
                    message = "Obstacle size updated.";
                } else message = "Blocked: obstacle would hit the arm.";
            }
            y += 42;
        }
        y += 9;
        if (button(left, y, 305, 34, "DELETE SELECTED  [DEL]",
                   mouse, click)) {
            obstacles.erase(obstacles.begin() + selected);
            selected = -1;
            changed = true;
            message = "Obstacle removed.";
        }
    } else {
        DrawText("No obstacle selected.", left, y, 15, muted);
    }
    y = std::max(y + 44, height - 97);
    if (!message.empty())
        DrawText(message.c_str(), left, y, 13,
                 message.find("Blocked") != std::string::npos ? red : green);
    return changed;
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
                 bool noPath, bool planning, const char* failureReason,
                 double routeDistance, double straightDistance) {
    const int x = screenWidth - sidebarWidth;
    DrawRectangle(x, headerHeight + 64, sidebarWidth, height - headerHeight - 64, sidebarColor);
    DrawLine(x, headerHeight, x, height, gridColor);
    const int left = x + 22;
    int y = headerHeight + 83;
    DrawText("SIMULATION", left, y, 19, light);
    y += 36;

    const char* status = planning ? "PLANNING..." : paused ? "PAUSED" : reached ? "TARGET REACHED" :
        noPath ? "NO SAFE PATH FOUND" : unreachable ? "UNREACHABLE" :
        stalled ? "BLOCKED" : constrained ? "AVOIDING CONTACT" : "MOVING";
    const Color statusColor = planning ? blue : paused ? orange : reached ? green :
        (stalled || unreachable || noPath) ? red : constrained ? orange : blue;
    DrawText(status, left, y, 16, statusColor);
    y += 33;
    if (noPath && !planning) {
        DrawText(failureReason, left, y, 13, red);
        y += 25;
    }
    DrawText(TextFormat("Time: %.2f s", elapsed), left, y, 16, muted);
    y += 26;
    if (routeDistance > 0.0) {
        DrawText(TextFormat("Tip travel: %.2f m (direct %.2f m)",
                            routeDistance, straightDistance), left, y, 14, blue);
        y += 26;
    } else {
        y += 8;
    }
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
    using robotics::kinematics::AsyncPlanner2D;
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
    int activeTab = 0; // 0 telemetry, 1 robot configuration, 2 obstacles
    std::string feedback;
    std::string obstacleFeedback;
    std::vector<Obstacle2D> obstacles;
    int selectedObstacle = -1;
    CanvasTool canvasTool = CanvasTool::Target;
    bool draggingObstacle = false;
    Vec2 target{1.5, 1.5};
    double simulatedTime = 0.0;
    double accumulator = 0.0;
    bool paused = false;
    float zoom = 1.0F;
    MotionStatus2D motionState{};
    PlannedMotion2D plannedMotion;
    AsyncPlanner2D asyncPlanner;
    std::string planningError;
    bool pendingPlan = true;
    bool draggingTarget = false;

    auto sceneChanged = [&] {
        // A new or moved obstacle invalidates every previous path.
        // Safely stop before following an obsolete collision model.
        paused = true;
        accumulator = 0.0;
        asyncPlanner.cancel();
        plannedMotion.reset();
        pendingPlan = true;
        planningError.clear();
        motionState = {};
        for (auto& joint : joints) joint.angularVelocity = 0.0;
    };

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
            const bool changePreset = selectTwo || selectThree || selectFour || selectFive;
            const int nextCount = changePreset ?
                (selectFive ? 5 : selectFour ? 4 : selectThree ? 3 : 2) : jointCount;
            auto nextDefinition = changePreset ? makePreset(nextCount) : appliedDefinition;
            auto nextRobot = makeRobot2D(nextDefinition, motionSettings);
            if (geom::hasObstacleCollision(forwardKinematics(nextRobot.joints),
                                            obstacles, motionSettings.obstacleClearance)) {
                obstacleFeedback = "Blocked: preset intersects obstacle.";
                sceneChanged();
            } else {
                if (changePreset) {
                    jointCount = nextCount;
                    appliedDefinition = std::move(nextDefinition);
                    draft = appliedDefinition;
                    selectedJoint = 0;
                    feedback.clear();
                }
                robot = std::move(nextRobot);
                target = jointCount >= 4 ? forwardKinematics(joints).back() :
                                              Vec2{1.5, 1.5};
                simulatedTime = 0.0;
                accumulator = 0.0;
                paused = false;
                motionState = {};
                asyncPlanner.cancel();
                plannedMotion.reset();
                pendingPlan = true;
                planningError.clear();
                draggingTarget = false;
            }
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
        if (IsKeyPressed(KEY_T)) canvasTool = CanvasTool::Target;
        if (IsKeyPressed(KEY_C)) { canvasTool = CanvasTool::PlaceCircle; activeTab = 2; }
        if (IsKeyPressed(KEY_B)) { canvasTool = CanvasTool::PlaceBox; activeTab = 2; }
        if (IsKeyPressed(KEY_V)) { canvasTool = CanvasTool::MoveObstacle; activeTab = 2; }

        bool obstacleEdited = false;
        if (IsKeyPressed(KEY_DELETE) && selectedObstacle >= 0 &&
            static_cast<std::size_t>(selectedObstacle) < obstacles.size()) {
            obstacles.erase(obstacles.begin() + selectedObstacle);
            selectedObstacle = -1;
            draggingObstacle = false;
            obstacleEdited = true;
            obstacleFeedback = "Obstacle removed.";
        }
        const Vec2 worldMouse = viewport.toWorld(mouse);
        if (mouseInCanvas && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (canvasTool == CanvasTool::PlaceCircle ||
                canvasTool == CanvasTool::PlaceBox) {
                if (obstacles.size() >= 16) {
                    obstacleFeedback = "Blocked: maximum of 16 obstacles.";
                } else {
                    const Obstacle2D created = canvasTool == CanvasTool::PlaceCircle ?
                        Obstacle2D{Circle2D{worldMouse, 0.4}} :
                        Obstacle2D{Box2D{worldMouse, 0.6, 0.35}};
                    if (shapeSafe(created, forwardKinematics(joints),
                                  motionSettings.obstacleClearance)) {
                        obstacles.push_back(created);
                        selectedObstacle = static_cast<int>(obstacles.size()) - 1;
                        obstacleEdited = true;
                        obstacleFeedback = "Obstacle added.";
                        canvasTool = CanvasTool::MoveObstacle;
                    } else obstacleFeedback = "Blocked: obstacle would hit the arm.";
                }
            } else if (canvasTool == CanvasTool::MoveObstacle) {
                selectedObstacle = -1;
                const double selectMargin = 9.0 / viewport.pixelsPerMetre;
                for (std::size_t i = obstacles.size(); i-- > 0;) {
                    if (geom::pointHitsObstacle(worldMouse, obstacles[i], selectMargin)) {
                        selectedObstacle = static_cast<int>(i);
                        break;
                    }
                }
                draggingObstacle = selectedObstacle >= 0;
            }
        }
        if (draggingObstacle && mouseInCanvas &&
            IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
            selectedObstacle >= 0 &&
            static_cast<std::size_t>(selectedObstacle) < obstacles.size()) {
            auto candidate = obstacles[static_cast<std::size_t>(selectedObstacle)];
            moveObstacle(candidate, worldMouse);
            if (shapeSafe(candidate, forwardKinematics(joints),
                          motionSettings.obstacleClearance)) {
                obstacles[static_cast<std::size_t>(selectedObstacle)] = candidate;
                obstacleEdited = true;
                obstacleFeedback = "Obstacle moved.";
            } else obstacleFeedback = "Blocked: obstacle would hit the arm.";
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) draggingObstacle = false;

        if (canvasTool == CanvasTool::Target && mouseInCanvas &&
            IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (!draggingTarget) {
                asyncPlanner.cancel();
                plannedMotion.reset();
                planningError.clear();
                for (auto& joint : joints) joint.angularVelocity = 0.0;
            }
            target = worldMouse;
            draggingTarget = true;
        }
        if (draggingTarget && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            draggingTarget = false;
            pendingPlan = true;
        }
        if (obstacleEdited) sceneChanged();

        if (!paused) {
            // Plan a safe joint-space route once the target is committed.
            // Retain immediate analytic motion while dragging a 2-link arm.
            accumulator += std::min(static_cast<double>(GetFrameTime()), 0.1);
            for (int steps = 0; accumulator >= fixedDt && steps < 6; ++steps) {
                if (draggingTarget) {
                    // Never execute a path toward a target that is still moving.
                    motionState = {};
                } else {
                    if (pendingPlan) {
                        plannedMotion.reset();
                        planningError.clear();
                        (void)asyncPlanner.submit(joints, robot.limits, target,
                                                  motionSettings, obstacles);
                        pendingPlan = false;
                    }
                    if (auto result = asyncPlanner.takeResult()) {
                        if (!result->error.empty()) {
                            planningError = result->error;
                            plannedMotion.reset();
                        } else {
                            plannedMotion.usePlan(std::move(result->plan), result->target);
                        }
                    }
                    if (plannedMotion.hasTarget())
                        motionState = plannedMotion.advance(
                            joints, robot.limits, fixedDt, motionSettings, obstacles);
                    else
                        motionState = {};
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
        const bool planning = asyncPlanner.busy();
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
        drawScene(viewport, obstacles, selectedObstacle);
        if (!draggingTarget && !reached && !planning &&
            plannedMotion.pathAvailable())
            drawEndEffectorTrace(viewport, plannedMotion.endEffectorTrace());
        drawArm(viewport, positions, target,
                !unreachable && !(plannedMotion.searchFailed() || !planningError.empty()));
        EndScissorMode();
        DrawText("T Target   C Circle   B Box   V Move   DEL Remove   SPACE Pause   2-5 Presets",
                 20, height - 28, 15, muted);
        DrawRectangle(0, 0, width, headerHeight, background);
        DrawText("ROBOTARM", 23, 16, 25, light);
        DrawText("2D Kinematics Laboratory", 197, 22, 17, muted);
        const int panelX = width - sidebarWidth;
        DrawRectangle(panelX, headerHeight, sidebarWidth,
                      height - headerHeight, sidebarColor);
        const bool click = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
        const Vector2 cursor = GetMousePosition();
        if (button(panelX + 14, headerHeight + 14, 100, 35, "TELEMETRY",
                   cursor, click)) activeTab = 0;
        if (button(panelX + 125, headerHeight + 14, 100, 35, "CONFIGURE",
                   cursor, click)) activeTab = 1;
        if (button(panelX + 236, headerHeight + 14, 100, 35, "OBSTACLES",
                   cursor, click)) activeTab = 2;
        if (activeTab == 1) {
            bool apply = false;
            drawConfigurator(panelX, height, draft, selectedJoint,
                             apply, feedback, cursor, click);
            if (apply) {
                try {
                    auto next = makeRobot2D(draft, motionSettings);
                    if (geom::hasObstacleCollision(forwardKinematics(next.joints),
                                                   obstacles,
                                                   motionSettings.obstacleClearance))
                        throw std::invalid_argument("Robot intersects an obstacle");
                    robot = std::move(next);
                    appliedDefinition = draft;
                    jointCount = static_cast<int>(draft.size());
                    target = forwardKinematics(joints).back();
                    simulatedTime = 0.0;
                    accumulator = 0.0;
                    paused = true;
                    motionState = {};
                    asyncPlanner.cancel();
                    plannedMotion.reset();
                    planningError.clear();
                    pendingPlan = true;
                    draggingTarget = false;
                    feedback = "Applied. SPACE to move.";
                } catch (const std::invalid_argument& ex) {
                    feedback = std::string("Error: ") + ex.what();
                }
            }
        } else if (activeTab == 2) {
            const bool changed = drawObstaclePanel(
                panelX, height, canvasTool, obstacles, selectedObstacle,
                obstacleFeedback, positions, motionSettings.obstacleClearance,
                cursor, click);
            if (changed) sceneChanged();
        } else {
            drawSidebar(width, height, joints, target, positions.back(), velocity,
                        error, simulatedTime, paused, reached, stalled,
                        motionState.constrained, unreachable,
                        plannedMotion.searchFailed() || !planningError.empty(),
                        planning,
                        !planningError.empty() ? planningError.c_str() :
                        robotics::kinematics::planningFailureMessage(
                            plannedMotion.failureReason()),
                        plannedMotion.endEffectorDistance(),
                        plannedMotion.endEffectorTrace().empty() ? 0.0 :
                            (target - plannedMotion.endEffectorTrace().front()).length());
        }
        EndDrawing();
    }

    CloseWindow();
}
