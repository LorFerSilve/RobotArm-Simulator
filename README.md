# RobotArm-Simulator

A modular C++20 robotic arm simulator featuring custom kinematics, Jacobian-based motion control, and a real-time 2D visualization.

## Current functionality

- Planar forward kinematics for any number of revolute joints.
- Fixed-timestep joint simulation using joint velocities.
- Analytic 2 x N Jacobian, Cartesian end-effector velocity, and 2-joint determinant.
- Custom damped least-squares inverse kinematics, velocity limiting, and target tolerance.
- Collision-aware GUI motion: analytic two-link IK with a stable elbow branch, angular speed/acceleration caps, joint clearance, nonadjacent-link collision checks, and extra target-object/folding safeguards.
- CLI frame telemetry and ten automatic tests using CTest.
- Interactive 2D GUI, using raylib **only** for display and input: target selection by mouse, real-time telemetry, pause/reset, grid and zoom.
- **Configurable robot arm (Phase 1):** edit 2–6 joints, each link's length and initial relative angle, per-joint rotation range, maximum speed, and maximum acceleration in the GUI.

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

The GUI uses 60 Hz fixed-step simulation and a 60 FPS rendering target. Rendering and simulation advance separately. Two joints use closed-form IK and an acceleration-limited joint-space servo to maintain the original elbow configuration. Three to six joints now use a bounded, goal-aware configuration-space path planner instead of repeatedly applying a local DLS velocity command. The path planner uses multi-start constrained IK to find a safe goal configuration, then direct joint interpolation or a bidirectional RRT-Connect search, followed by an acceleration-limited joint-space follower.

### GUI controls

| Input | Function |
| --- | --- |
| Left click/drag within the canvas | Set the end-effector target (metres); 3+ joint motion is planned on mouse release |
| `Space` | Pause/resume |
| `R` | Reset to initial pose and target |
| `2` | Two-joint robot preset |
| `3` | Three-joint robot preset |
| Mouse wheel over canvas | Zoom |
| `CONFIGURE` tab | Edit a draft robot configuration |
| `+ ADD JOINT` / `- REMOVE JOINT` | Change the link count (2–6) |
| `APPLY & RESET ROBOT` | Validate and activate new geometry and joint limits, paused |
| `TELEMETRY` tab | Return to live joint measurements |

The right panel shows target and tip coordinates, Cartesian speed, goal error, simulation time, per-joint angles and angular velocities.

**Limitations:** The two-link robot stays on its original elbow branch and avoids collinear, overlapping configurations. Targets outside the resulting annular workspace are marked **UNREACHABLE** and approached only to the nearest permitted radius; the robot then settles instead of oscillating. For 3+ links, the new path planner searches for an alternative collision-free joint configuration and a sequence of safe joint-space waypoints. This is a **bounded, sampling-based search**, not a complete solver: if it cannot find a route within its budget, the GUI displays **NO SAFE PATH FOUND** and does not move through the blocked configuration. The existing controller is still available in the core library; physical obstacle modeling, joint torques and dynamics are not implemented.

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

## Configurable robot arm — Phase 1

The right-hand **CONFIGURE** tab contains a simple, dependency-free editor
built with raylib controls. Select a joint with the arrow buttons; use +/- to
adjust its **length (metres), initial angle (degrees), minimum/maximum angle
(degrees), maximum angular speed (degrees/s)**, and **maximum angular
acceleration (degrees/s²)**. Add or remove joints (minimum 2, maximum 6 in
the current GUI). Changes are drafts until **APPLY & RESET ROBOT** is clicked.

Applying a valid configuration resets the arm to its configured initial
angles, resets simulated time, and pauses the simulation. Press **Space** to
resume; press **R** to reset to the *last applied* configuration. The original
`2` and `3` shortcuts still load their respective presets. Configurations
are held in memory for this session, not yet saved to a file.

The geometry and initial limits are represented by
`JointDefinition2D`; `Robot2D` contains the **mutable runtime state**
plus the independent per-joint limits. `makeRobot2D` validates the settings
before activating them. In addition to the existing geometric safeguards,
the motion controller now enforces each joint's configured range, speed,
and acceleration caps (up to the engine's global caps).

Internal joints must stay within one safe positive or negative elbow branch,
avoiding fully folded and straight configurations. Consequently, some
angle ranges are intentionally rejected. Valid configurations are not
guaranteed to reach every requested target, and collision avoidance for
three or more joints remains a local controller, not a global path planner.

## Recovery planning for redundant arms (3+ joints)

The GUI now separates **finding a goal configuration**, **checking safe paths**,
and **following a path**. The reusable C++20 module
`PathPlanning2D.hpp/.cpp` searches for an alternate inverse-kinematics solution
using deterministic multi-start damped-Jacobian refinement. It tries direct
configuration-space interpolation first and falls back to a bounded
bidirectional RRT-Connect search if that path crosses a self-collision or
violates the configured joint limits. Edges are collision-checked at a maximum
angular interpolation step of 0.04 radians; path segments are then shortened
when a direct shortcut remains safe.

The GUI follows the resulting joint waypoints with per-joint acceleration and
speed limits. To avoid running an expensive search every rendering frame, a
new target is committed when the user **releases the mouse button**. The
2-joint arm keeps the previously tested smooth analytic controller.

A planner timeout or infeasible IK configuration is reported explicitly as
`NO SAFE PATH FOUND`, never mislabeled as a successfully reached goal.
This does **not** establish that the goal is mathematically unreachable.

**Outstanding motion-planning work:** interactive planning can still cause a
short UI pause because it currently runs on the rendering thread; future work
should move bounded planning to a worker thread and support interruption when
new targets are chosen. More advanced path smoothing/continuous collision
checking, joint-branch switching, and planning around user-added obstacles
are still open. See `docs/ROADMAP.md`.

## Playtest safety improvement: target objects and tightly folded arms

The GUI now treats the clicked target as a **physical keep-out area** rather
than a purely mathematical point. It checks a target radius of 0.16 m
against all upstream arm links. The last link is allowed into a short
tool-tip zone of twice that radius so that the end effector can arrive.
This is an intentionally simplified contact model, not a rigid-body
simulation with detailed gripper geometry or object collision shapes.
If the target is already underneath an intermediate link, a new 3+ joint
plan fails safely instead of trying to push the arm through the object.

The GUI also uses a tighter **fold margin of 0.55 rad** (about 31.5 degrees)
from the fully folded, overlapping pose. Its joint presets and configurator
ranges reflect this restriction. Custom configurations that would fold too
tightly are rejected on Apply. The two-joint controller checks its target
approach before accepting each simulation step; the multi-joint planner
checks the entire planned path, plus its actual per-frame movement.

The reusable engine retains its previous opt-in defaults
(`foldClearance=jointClearance=0.14`, `targetClearance=0`) to avoid
breaking existing CLI scenarios. Configure the new margins using
`MotionSettings2D`. A larger clearance reduces the reachable workspace.
In particular, **NO SAFE PATH FOUND** can mean that a goal is excluded by
the target/arm safety envelopes, or that the bounded search did not find a
valid path. The GUI does not yet guarantee a detour for every feasible goal.
