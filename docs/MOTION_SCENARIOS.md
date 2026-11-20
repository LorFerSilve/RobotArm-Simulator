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
