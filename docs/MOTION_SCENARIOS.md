# Motion regression scenarios

This is the permanent acceptance suite for movement-related changes.

Each entry describes a *start → destination* pair and the robot fixture.
A Cartesian start point alone is insufficient to reproduce a robot pose:
each fixture also specifies link lengths, initial joint angles and limits.
The executable table is \`tests/test_motion_scenarios.cpp\`.

| Fixture | Start (m) | Destination (m) | Expectation |
| --- | --- | --- | --- |
| 2 joints, 1+1 m, clockwise elbow | (1, 0) | (0, 1) | Reachable; detour around target |
| 3 joints, original preset | (2.898, 0.506) | (1.6, 1.1) | Reachable |
| 3 joints, same robot | (1.6, 1.1) | (-1, -0.4) | Reachable, after reaching start |
| 5 joints, 2 m links | (3.988, -8.845) | (-3, -4) | Reachable; known playtest |
| 5 joints, 2 m links | (2, 3) | (5, 4) | Exploratory until reachability is confirmed |

An exploratory case must **still obey all collision and motion limits** if
a plan is found, but \`NO SAFE PATH FOUND\` is recorded as *inconclusive*
rather than proof that the goal is impossible. Do not silently change an
exploratory case to a required success without first demonstrating a valid
route for its specific joint configuration.

## What every executed transition checks

1. The planner finds a path for cases known to be reachable.
2. The end effector reaches the requested destination within tolerance.
3. No nonadjacent arm segments collide.
4. A link may not enter or increase its penetration of the target keep-out zone.
   If the target was selected on a link, permitted retreat must decrease
   contact; the last segment has only a **short tool-tip exception**.
5. Joint angles, angular speeds and angular accelerations remain inside
   their configured limits at every simulated frame.
6. The follower reports an explicit failure, never falsely reports arrival.

Run after every change:

\`\`\`powershell
cmake -S . -B build -DROBOTARM_BUILD_GUI=ON
cmake --build build
ctest --test-dir build --output-on-failure
\`\`\`

With a Visual Studio multi-configuration generator use
\`cmake --build build --config Debug\` and \`ctest --test-dir build -C Debug
--output-on-failure\`. The default GUI executable is
\`build/robot_gui.exe\` for Ninja or \`build/Debug/robot_gui.exe\`
for Visual Studio Debug.

The CI workflow also builds and executes the suite on Linux and Windows
after each push/pull request. For graphical changes, launch the GUI locally
and visually reproduce the corresponding scenario.

## Adding a reported playtest bug

Record the link count, each link's length and joint angle at the *initial*
frame, rotation limits, the start/end coordinates and whether the planner
said \`BLOCKED\`, \`NO SAFE PATH FOUND\`, or merely never reached the goal.
Screenshots alone rarely establish the full joint configuration.

Add a named row to \`test_motion_scenarios.cpp\` or a focused test file.
A regression is considered fixed only when its **executed, frame-by-frame
movement** passes, not just when a static path was found.

**Pending exact reproduction:** the additional multi-joint screenshot from
the November 20 playtest has not yet supplied its complete numeric joint
state. The general target-shaft and swept-path defects are covered here;
the exact screenshot should become a separate required case once those
values are recorded from the telemetry panel.

## Release-grade 6-group motion matrix

The dedicated executable `tests/test_release_motions.cpp` has exactly **six
collections**, one each for **2, 3, 4, 5, 6 and 7 revolute joints**. Joint
counts 2–6 are currently configurable in the GUI; 7 joints is an additional
headless engine scalability group.

A testcase is represented as:

```cpp
struct MotionCase {
    std::string name;
    std::vector<Vec2> startPositions;  // base + all link endpoints
    Vec2 target;                        // required end-effector position
};
```

Thus a robot with N joints stores N+1 positions: `P0=(0,0)` (fixed
base), `P1`, …, `PN` (initial end effector). For every pair of
consecutive points, the test derives the link length:

```text
length[i] = sqrt((x[i+1]-x[i])^2 + (y[i+1]-y[i])^2)
```

It also derives each *relative* joint angle using `atan2` and the
previous link orientation. This matches the engine's kinematic convention.
The full initial pose is verified through an independent
positions → joint angles → forward kinematics round-trip.

**The target is only a Cartesian point.** Joint angles at arrival are
not specified or compared; any safe final configuration is accepted.

To cover more than a handful of manually authored poses, each group
contains deterministic, seeded pseudo-random fixture generation. The
generator materializes a stored `startPositions` list and a target for
every case. Before admitting a *required* case, it also independently
checks a known collision-free reference trajectory to prove reachability.
The actual planner is **not** allowed to use that reference trajectory;
it must discover and execute a valid route itself.

| Mode | Cases per joint count | Total required motions |
| --- | ---: | ---: |
| Normal `ReleaseMotionMatrix` | 20 | 120 |
| Extended `ReleaseMotionStress` | 100 | 600 |

An additional unreachable-target rejection check runs for each group
in each mode. The stress suite is labeled `stress;release` and is
included in the standard `ctest` run. CI runs all tests for every push
and pull request; a failing motion blocks the corresponding status check.

Every simulated frame checks end-effector arrival (≤5 mm), forbidden
target contact, self-collision, relative joint limits, angular velocity
and angular acceleration. If a case fails, the test prints the **full
initial position list and target**, so it can be copied into a permanent
named regression fixture. The random seeds and enumeration order are
stable. The generated cases are intentionally modest, verified feasible
rotations of a multijoint arm; add harder named adversarial cases as
playtests uncover new failure modes.

Run the full suite locally:

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

To run only the larger stress suite:

```powershell
ctest --test-dir build -R ReleaseMotionStress --output-on-failure
```

For Visual Studio, add `--config Debug` to the build and `-C Debug`
to `ctest`. The 7-joint case is an **engine-only** test, not a new GUI
preset.
