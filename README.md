# RobotArm-Simulator

A modular C++20 robotic arm simulator featuring custom kinematics, Jacobian-based motion control, and a real-time 2D visualization.

## Current functionality

- Planar forward kinematics for any number of revolute joints.
- Fixed-timestep joint simulation using joint velocities.
- Analytic 2 x N Jacobian, Cartesian end-effector velocity, and 2-joint determinant.
- Custom damped least-squares inverse kinematics with velocity limiting and target tolerance.
- CLI frame telemetry and five automatic tests using CTest.
- Interactive 2D GUI, using raylib **only** for display and input: target selection by mouse, 2/3-joint presets, real-time telemetry, pause/reset, grid and zoom.

The GUI and CLI use the **same** C++20 robotics engine. The GUI currently simulates kinematics, not physical dynamics (no forces, torques or collisions). 3D is not implemented yet.

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

The GUI uses 60 Hz fixed-step simulation and a 60 FPS rendering target. Rendering and simulation advance separately. The simulation uses proportional damped-least-squares IK to track the mouse-selected target.

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

**Limitations:** DLS is a local numerical method. Exactly singular poses can stall, targets can be unreachable, and there are not yet joint-angle constraints or obstacle handling. A red target lies beyond the arm's **outer** reach; a target inside that radius is not necessarily reachable by every arm configuration.

## Math and IK conventions

- Angles are stored in radians (relative to the previous link), speeds in rad/s; GUI/CLI display degrees and degrees/s.
- The base is fixed at `(0, 0)` metres. Forward kinematics returns N+1 positions: the base and every link endpoint.
- The end-effector Jacobian has two rows and N columns. For two joints only, its determinant can be evaluated.
- Inverse kinematics uses `qdot = J^T (J J^T + lambda^2 I)^-1 * positionGain * (target - tip)`, followed by a uniform angular velocity limit.
- The timestep is `1/60` simulated second; no dynamics or hardware control is implemented.
