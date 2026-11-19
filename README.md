# RobotArm-Simulator

A modular C++20 robotic arm simulator featuring custom forward/inverse kinematics, Jacobian-based motion control, and 2D/3D visualization.

## Current functionality

- 2D planar forward kinematics for any number of links.
- Fixed-timestep joint-angle simulation with constant angular velocities.
- Analytic 2 x N end-effector Jacobian and linear velocity from joint speeds.
- 2 x 2 Jacobian determinant for two-joint singularity analysis.
- CLI output with frame time, joint angles/speeds, link endpoints, end-effector position and velocity, and Jacobian columns.
- Automated tests using CTest.

Inverse kinematics, Jacobian-based motion control, and a GUI are planned but not implemented yet.

## Build and test

Requires CMake 3.20+ and a C++20 compiler.

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

On Windows with the Visual Studio generator, run:

```powershell
.\build\Debug\robot_cli.exe 10
```

The optional argument is the number of simulation steps (0–10000, default 10).
Each step advances **1/60 simulated second**. Frame 0 is also printed, so 10
steps produce 11 frame snapshots. The CLI runs as fast as it can; it does not
sleep to emulate real-time frame pacing.

All joint angles and angular velocities are stored internally in radians
(and radians per second). The CLI prints degrees (and degrees per second).

## Jacobian convention

`endEffectorJacobian(positions)` expects N+1 coordinates returned by
`forwardKinematics(joints)` (base and link endpoints). It returns N `Vec2`
columns representing the 2xN translational Jacobian in metres per radian.
`endEffectorVelocity(columns, joints)` multiplies these columns by joint
angular velocities to obtain the Cartesian end-effector velocity (m/s).
The determinant exists only when the Jacobian is 2x2 (exactly two joints).
