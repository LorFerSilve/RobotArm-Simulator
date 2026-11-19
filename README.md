# RobotArm-Simulator

A modular C++20 robotic arm simulator featuring custom kinematics, Jacobian-based motion control, and future 2D/3D visualization.

## Current functionality

- 2D planar forward kinematics for any number of revolute links.
- Fixed-timestep joint-angle simulation with constant velocities per step.
- Analytic 2 x N end-effector Jacobian, linear velocity, and determinant for two-joint configurations.
- **Inverse kinematics (IK):** damped least-squares velocity controller that steers the arm toward a Cartesian target.
- Joint speed limiting, goal-tolerance detection, and validation of numerical inputs.
- CLI frame telemetry and automated tests via CTest.

A GUI, 3D kinematics, and physical dynamics are not implemented yet.

## Build and test

Requires CMake 3.20+ and a C++20 compiler.

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

On Windows with the Visual Studio generator, run:

```powershell
# Original constant-speed simulation, default 10 steps
.\build\Debug\robot_cli.exe 10

# Drive toward a target position (x=1.5m, y=1.5m), maximum 300 steps
.\build\Debug\robot_cli.exe ik 1.5 1.5 300

# Different target, default 300 steps
.\build\Debug\robot_cli.exe ik -1.5 1.3
```

Frames advance by **1/60 simulated second** (including frame 0 means a maximum
of N+1 displayed snapshots). The simulation runs as quickly as the CPU allows,
without wall-clock delays. IK mode exits early once the position error falls
below 0.005 m (5 mm), or reports when the target was not reached within the
requested number of steps. The allowed range is 0–10000 steps.

All joint angles and joint speeds are stored in radians and radians/s;
the CLI prints degrees and degrees/s. Joint positions are in metres.

## Inverse kinematics

The solver computes a Cartesian error `e = target - endEffectorPosition`
and the corresponding desired velocity `v = positionGain * e`.
It solves the 2 x 2 system for a planar translational Jacobian `J`:

```
qdot = J^T (J J^T + damping^2 I)^-1 v
```

The computed joint velocities are uniformly scaled when necessary to satisfy
`maxAngularSpeed`. The existing `advanceJoints` simulator integrates those
velocities using the fixed timestep. The solver does **not** use Eigen or
another linear-algebra package.

`computeIKCommand` returns a velocity command without modifying the arm; the
caller is responsible for applying it. Default settings are proportional gain
`4 1/s`, damping `0.1 m/rad`, joint speed limit `pi rad/s`, and a position
tolerance of `0.005 m`.

Damped least squares avoids a direct inverse of the potentially singular
Jacobian. It keeps the velocity calculation well-defined for ordinary
singular configurations, but **does not guarantee global convergence**.
For example, an exactly straight two-link arm targeting a point directly
inward along its axis has no first-order solution and can stall.
An unreachable or stalled target is reported as *not reached*, never as a
successful IK movement. Joint limits, collision avoidance, and alternative
configurations are not implemented yet.

## Jacobian convention

`endEffectorJacobian(positions)` expects N+1 coordinates returned by
`forwardKinematics(joints)` (base and link endpoints). It returns N `Vec2`
columns representing the 2 x N translational Jacobian in metres per radian.
`endEffectorVelocity(columns, joints)` multiplies these columns by joint
angular velocities to obtain Cartesian end-effector velocity (m/s).
The determinant exists only when the Jacobian is 2 x 2 (exactly two joints).
