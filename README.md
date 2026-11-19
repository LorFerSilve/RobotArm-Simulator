# RobotArm-Simulator

A modular C++20 robotic arm simulator featuring custom kinematics, Jacobian-based motion control, and a real-time 2D visualization.

## Current functionality

- Planar forward kinematics for any number of revolute joints.
- Fixed-timestep joint simulation using joint velocities.
- Analytic 2 x N Jacobian, Cartesian end-effector velocity, and 2-joint determinant.
- Custom damped least-squares inverse kinematics, velocity limiting, and target tolerance.
- Collision-aware GUI motion: analytic two-link IK with a stable elbow branch, angular speed/acceleration caps, joint clearance, and nonadjacent link collision checks for three or more links.
- CLI frame telemetry and five automatic tests using CTest.
- Interactive 2D GUI, using raylib **only** for display and input: target selection by mouse, 2/3-joint presets, real-time telemetry, pause/reset, grid and zoom.

The GUI and CLI use the **same** C++20 robotics engine. The GUI simulates **kinematics only**, without forces, torques, or 3D dynamics. Its motion controller now prevents specified link overlaps but does not model full rigid-body physics.

## Build the CLI and run tests

Requires CMake 3.20+ and a C++20 compiler:

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

On Windows with Visual Studio:

```powershell
.\build\Debug\robot_cli.exe ik 1.5 1.5 300
```

## Build and launch the interactive GUI

The GUI additionally requires Git and internet access **on the first configuration** to fetch [raylib 6.2](https://github.com/raysan5/raylib/releases/tag/6.2). This external library is downloaded into CMake's ignored build directory and does **not** replace the custom math/IK code. A default CLI/test build does not download raylib.

From a PowerShell terminal at the repository root:

```powershell
cmake -S . -B build -DROBOTARM_BUILD_GUI=ON
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
.\build\Debug\robot_gui.exe
```

The GUI uses 60 Hz fixed-step simulation and a 60 FPS rendering target. Rendering and simulation advance separately. Two joints use closed-form IK and an acceleration-limited joint-space servo to maintain the original elbow configuration. Three joints use the existing damped-least-squares IK with safety-constrained steps.

### GUI controls

| Input | Function |
| --- | --- |
| Left click/drag within the canvas | Set the end-effector target (metres) |
| `Space` | Pause/resume |
| `R` | Reset to initial pose and target |
| `2` | Two-joint robot preset |
| `3` | Three-joint robot preset |
| Mouse wheel over canvas | Zoom |

The right panel shows target and tip coordinates, Cartesian speed, goal error, simulation time, per-joint angles and angular velocities.

**Limitations:** The two-link robot stays on its original elbow branch and avoids collinear, overlapping configurations. Targets outside the resulting annular workspace are marked **UNREACHABLE** and approached only to the nearest permitted radius; the robot then settles instead of oscillating. For 3+ links, collision checks prohibit nonadjacent segments from intersecting, but a movement may stop at a constrained local configuration: full global collision-free path planning, environment obstacles, joint torques, and physical dynamics remain out of scope. Hard safety constraints can cause abrupt stopping in 3-joint mode.

## Math and IK conventions

- Angles are stored in radians (relative to the previous link), speeds in rad/s; GUI/CLI display degrees and degrees/s.
- The base is fixed at `(0, 0)` metres. Forward kinematics returns N+1 positions: the base and every link endpoint.
- The end-effector Jacobian has two rows and N columns. For two joints only, its determinant can be evaluated.
- Inverse kinematics uses `qdot = J^T (J J^T + lambda^2 I)^-1 * positionGain * (target - tip)`, followed by a uniform angular velocity limit.
- The timestep is `1/60` simulated second; no dynamics or hardware control is implemented.

## Motion safety in the GUI

`advanceTowardTarget` (in `SmoothMotion2D`) is an independent fixed-step
controller. For two joints it calculates a closed-form target pose, keeps the
initial elbow sign, and applies bounded speed, acceleration and braking.
For 3+ joints it uses the existing Jacobian solver, checks joint clearance
and nonadjacent link separation, and shortens unsafe proposed steps.

Defaults: 2 rad/s maximum angular speed, 6 rad/s² maximum angular
acceleration, 0.14 rad elbow margin, 0.10 m nonadjacent-link clearance.
These are **kinematic simulation constraints**, not a certified industrial
collision model. Reaching every feasible goal is not guaranteed.
