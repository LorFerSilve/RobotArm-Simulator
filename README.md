# RobotArm-Simulator

A modular C++20 robotic arm simulator featuring custom kinematics, Jacobian-based motion control, and a real-time 2D visualization.

## Current functionality

- Planar forward kinematics for any number of revolute joints.
- Fixed-timestep joint simulation using joint velocities.
- Analytic 2 x N Jacobian, Cartesian end-effector velocity, and 2-joint determinant.
- Custom damped least-squares inverse kinematics, velocity limiting, and target tolerance.
- Collision-aware GUI motion: analytic two-link IK with a stable elbow branch, angular speed/acceleration caps, joint clearance, nonadjacent-link collision checks, and extra target-object/folding safeguards.
- CLI frame telemetry and fourteen automatic tests using CTest.
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

The GUI uses 60 Hz fixed-step simulation and a 60 FPS rendering target. Rendering and simulation advance separately. Two joints now use closed-form IK **with equivalent full-turn shoulder configurations** and collision-checked joint-space paths; a smooth, acceleration-limited servo follows the chosen route. Three to six joints use a bounded, goal-aware configuration-space path planner instead of repeatedly applying a local DLS velocity command. The path planner uses multi-start constrained IK to find a safe goal configuration, then direct joint interpolation or a bidirectional RRT-Connect search, followed by an acceleration-limited joint-space follower.

### GUI controls

| Input | Function |
| --- | --- |
| Left click/drag within the canvas | Set the end-effector target (metres); safe joint-space motion is planned on mouse release for all arms |
| `Space` | Pause/resume |
| `R` | Reset to initial pose and target |
| `2` | Two-joint robot preset |
| `3` | Three-joint robot preset |
| `4` | Four-joint robot preset (2 m per link) |
| `5` | Five-joint robot preset (2 m per link, starting tip near `(4,-9)`) |
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

The GUI follows the resulting joint waypoints using a **single coordinated quintic interpolation parameter** per segment, bounded by each joint's velocity and acceleration limits. To avoid running an expensive search every rendering frame, a
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
If the target is already underneath an intermediate link, the planner
now looks for a **safe retreat** that reduces the existing contact before
continuing toward a collision-free final pose. It does not drive any link
deeper into the newly selected target.

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

## Target-aware recovery planning (playtest fix)

The configuration-space planner now distinguishes two different operations:

1. **Finding inverse-kinematics solutions** is a mathematical search, not a
   physical trajectory. Constrained IK can explore intermediate joint poses
   that would overlap the target object, but a *final* candidate is only
   accepted if all arm-link and target clearances are satisfied.
2. **Executing the joint-space path** must remain safe. Every interpolated
   path edge and every follower step checks collision. If the target was
   selected beneath an already occupied arm segment, the only permitted
   escape motion is one that never increases that segment's target
   penetration and never creates contact with another segment.

This removes an overly restrictive check that discarded feasible targets
while searching for a goal pose. It also allows the robot to **move away
first, then approach the target from a safe direction**, when the bounded
planner can find such a path. A target overlapping the immovable base
cannot be cleared and is rejected.

The new regression tests cover both a collision-prone straight-line route
and the safe retreat from a target initially under arm segments. As before,
a bounded RRT search can still fail to find some feasible paths, so
`NO SAFE PATH FOUND` never proves that the position is impossible.

## Two-joint detours (playtest regression)

A two-joint arm may reach a Cartesian target safely even when the **shortest
shoulder rotation** would sweep the first link through that target. The
two-joint planner now enumerates equivalent analytical IK poses whose
shoulder angles differ by whole turns (`2*pi`), retaining the configured
elbow branch and individual joint limits. It checks direct joint-space
transitions first, then tries the existing RRT-Connect fallback if necessary.

For example, from an arm with 4 m and 2 m links at end-effector position
`(4, 0)`, the target `(2, 3)` lies in the swept area of the short shoulder
rotation. With a shoulder range of `[-2*pi, 2*pi]`, moving around in the
opposite direction can instead reach the target safely. This case is
verified in `test_two_joint_detour.cpp`, including the executed motion,
target clearance, speed and acceleration limits.

A route involving an extra revolution is only accepted if the configured
shoulder rotation range actually permits it. The planner does not silently
cross joint limits, reverse the elbow branch, or relax target collision
constraints. Target planning runs when the mouse button is released.

## Five-joint waypoint follower and test presets

**Playtest issue:** a planned route was geometrically valid but the robot
sometimes stopped near a target when its separate per-joint speed controllers
made the executed movement deviate from the checked straight-line segment
in joint space. The `PlannedMotion2D` follower now uses a shared **quintic
smoothstep** progress parameter for all joints along each approved segment.
The segment's duration is chosen to respect the strictest joint speed and
acceleration limit; each segment starts and finishes at rest. This removes
the independent joint-space corner cutting that caused unexpected contact
and blocked states. The planner and target/arm collision checks remain active.

The GUI also supports preset hotkeys **2**, **3**, **4**, and **5**.
Presets 4 and 5 have 2 m links and alternating safe elbow branches.
Preset 5 starts with its end effector near `(4,-9)`; try selecting target
`(-3,-4)` to reproduce the reported five-joint playtest.
The initial target for presets 4/5 is set to the current tip, so the arm
starts at rest until you click a new target. Press **R** to reset the
currently selected configuration.

The `FiveJointRecovery2D` CTest regression checks the planned path,
the actual time-stepped follower, target/link separation, self-collisions,
joint ranges, speed limits and acceleration limits.

**Known limitation:** planned paths stop at intermediate waypoints, rather
than using continuous corner-blending. New goals chosen while a robot is
still moving are not yet handled by a full velocity-continuous replanner.
See `docs/ROADMAP.md`.

## Permanent movement regression suite

The table in [docs/MOTION_SCENARIOS.md](docs/MOTION_SCENARIOS.md) collects
reproducible **(start position, destination position)** transitions,
together with joint poses, robot geometry and constraints. The automated
`MotionScenarioMatrix` test runs the actual controller frame by frame, not
merely the IK solver. The table includes the reported multi-joint
`(~4,-9) -> (-3,-4)` scenario and additional pairs such as
`(1,0) -> (0,1)` and `(2,3) -> (5,4)`.

Each known-reachable case must arrive without self-collisions, target
penetration, joint-limit violations or excess joint acceleration/speed.
Exploratory cases may report that the bounded planner did not find a route,
but may never execute an unsafe route. New playtest failures should be
added as explicit fixtures before being considered fixed.

Target protection no longer exempts the entire final link near the goal.
Only a short end-effector tool zone may approach the target object; the
remaining last-link shaft is tested for contact. Joint-space edges also
use more detailed target checks when their possible Cartesian displacement
could skip over the small target safety zone.

The [GitHub Actions CI workflow](.github/workflows/ci.yml) is configured
to build and run the suite on every push and pull request on Linux and
Windows (the latter also compiles the raylib GUI). Local visual playtesting
is still needed for GUI behavior and complex trajectories.

## Release acceptance tests (2–7 joints)

The new `ReleaseMotionMatrix` (120 motions) and
`ReleaseMotionStress` (600 motions) run on the **actual path planner and
frame-by-frame follower**. Every case contains the positions of the base
and all joints at the start plus the required target. Link lengths and
relative angles are recovered from these positions, so arrival is judged
**only by end-effector position**, never by a predetermined final posture.
All cases are deterministic and independently verified reachable before
being accepted into the mandatory set. All unsafe motion or failure to
reach a required target fails the test.

See [the six-group scenario specification](docs/MOTION_SCENARIOS.md).
Seven joints are covered by the engine tests even though the current GUI
editor stops at six.

## Scene obstacles — Phase 2

The simulator now has an **OBSTACLES** sidebar tab. Obstacles are stored
in world metres as either circles or axis-aligned rectangles, independently
of raylib, via `include/robotics/geometry/Obstacles2D.hpp`.

Controls:

| Shortcut / control | Action |
| --- | --- |
| **OBSTACLES** tab | Open the obstacle editor |
| **C** | Place a circle (left-click canvas; default radius 0.4 m) |
| **B** | Place a rectangle (left-click; default width 1.2 m, height 0.7 m) |
| **V** | Select or drag an existing obstacle with the left mouse button |
| **T** | Return to target-selection mode |
| **Delete** | Remove the selected obstacle |
| **+ / -** in obstacle editor | Adjust the selected radius, width or height |

Up to 16 obstacles can be edited in a scene. A new obstacle, or a moved or
resized obstacle, cannot intersect the robot's *current* pose. Scene editing
pauses the simulator and invalidates the old plan: press **Space** to resume.
A target inside an obstacle is rejected as `NO SAFE PATH FOUND`.

The planner uses the existing bounded IK + RRT-Connect implementation to
look for a collision-free route around obstacles; it can still fail to
find a valid path within its budget. Obstacle checks cover **every link**,
including the tool segment; there is no target-style tool-tip exception
for solid scene obstacles. Clearance around the link centerline is 0.10 m
(approximately 0.05 m physical half-width + 0.05 m safety clearance).
The collision functions support circular and rectangular solids, with
Euclidean clearances (not only axis-aligned bounding boxes).

Changing the geometry immediately invalidates the old path; the follower
also rechecks new scene geometry at every movement step. The path sampling
is fine-grained but is **not yet a mathematically certified continuous
swept-volume detector**. For guaranteed safety with moving obstacles,
background replanning and continuous collision detection remain future work.

The new `Obstacles2D` CTest checks geometry, clearances, impossible targets,
invalid starting configurations, the planner and the actual follower.
It also verifies a known reachable **long-way obstacle detour** where the
shorter shoulder rotation crosses a circle.
The existing 120-case and 600-case motion matrix tests still run in CI.
