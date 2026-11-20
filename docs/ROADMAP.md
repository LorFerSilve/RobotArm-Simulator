# Development roadmap and known limitations

This file tracks milestones and playtest problems that should not be lost when
the simulator grows. See the root README for build and usage instructions.

## Completed

- [x] 2D forward kinematics, Jacobian, inverse kinematics and fixed-step simulation.
- [x] Interactive raylib 2D GUI.
- [x] Two-joint smooth motion and self-folding protection.
- [x] **Phase 1:** configure 2–6 links, initial poses, joint ranges, speed and acceleration.
- [x] **Playtest recovery (first implementation):** 3+ joint motion uses goal-aware,
      multi-start IK plus a bounded joint-space path planner. A failed search is
      explicitly reported instead of repeatedly applying an invalid local step.
- [x] Regression coverage for constrained, collision-free 3-, 4- and 6-joint paths.

## Outstanding work, in intended order

### Phase 2 — Scene obstacles and geometry

- [ ] Add circles and axis-aligned rectangles to the GUI, including edit/move/delete.
- [ ] Define a shared 2D obstacle representation outside rendering code.
- [ ] Detect link-to-obstacle collisions, respecting arm thickness and clearance.
- [ ] Validate start/goal configurations against scene obstacles.

### Phase 3 — Motion planning and smooth trajectories

- [ ] Integrate obstacle checks into every configuration-space path edge.
- [ ] Improve the planner's search coverage and report *why* planning failed:
      no IK goal, exhausted sample budget, collision, or limited workspace.
- [ ] Move planning off the GUI rendering thread and support cancellation when
      the target/robot/obstacles change.
- [ ] Replace stop-and-go waypoint following with continuous, time-parameterized
      jerk-aware trajectories while preserving collision clearance.
- [ ] Support controlled reconfiguration when feasible (including elbow-branch
      transitions in a future joint/geometry model).
- [ ] Add replanning and safe stopping around moving obstacles.

### Phase 4 — Diagnostics and analysis

- [ ] Graph angle, speed, acceleration, Cartesian error, Jacobian condition
      and planner state over time.
- [ ] Visualize robot paths, candidate end poses, and collision envelopes.
- [ ] Allow reproducible recording/replay of configurations and scenarios.

### Phase 5 — 3D robot kinematics

- [ ] Implement Vec3, transformations and spatial forward kinematics.
- [ ] Spatial Jacobians, orientation targets and 3D IK.
- [ ] Extend collision detection and motion planning to 3D.

## Recent playtest safety fixes and remaining constraints

- [x] **Intermediate links passing through the target point:** introduce a
      target-object keep-out radius and check each planned configuration,
      interpolated path edge, and executed step. Model the final tool-tip
      region separately so the end effector can still reach the goal.
- [x] **Arms folding unnaturally tightly:** use a larger folding margin
      in the GUI, reject configurations violating that margin, and constrain
      planner paths to the same safe angular region.
- [ ] **Target-object geometry:** replace the current target-circle/tool-tip
      approximation with user-configurable object sizes, tool geometry,
      continuous swept-volume collision checks, and approach direction.
- [x] **Target-under-arm recovery (initial implementation):** multi-start IK
      can search through virtual configurations without requiring its internal
      iterations to be target-clear; a real planned route may safely retreat
      from contact that existed when the target was selected.
- [ ] **Further safety-aware replanning:** handle difficult joint-branch or
      clearance constraints where the bounded search still cannot find a
      feasible detour. Do not solve this by relaxing collision constraints.

## Playtest issues to keep visible

0. **Premature failure when an arm could avoid the target:** previously, every
   IK *iteration* was required to stay target-clear, even though these virtual
   calculations are never executed. Also, selecting a target already under a
   link failed immediately. Fixed by separating goal search from safe path
   checks and allowing a contact-reducing retreat; continue testing 3–6 joints.

1. **Redundant-arm apparent freeze**: the earlier 3+ joint controller repeatedly
   halved a DLS velocity at joint limits/self-collisions, sometimes resulting in
   effectively zero movement with a nonzero Cartesian error. Mitigated by the
   goal-aware planner. Keep testing after adding constraints/obstacles.
2. **Reachable target requiring reconfiguration**: locally descending in
   end-effector error cannot always find a safe route through joint space.
   Mitigated by multi-start constrained IK plus an RRT-Connect fallback.
   A bounded sampling algorithm is **not complete** and can still return
   `NO SAFE PATH FOUND` for a reachable target. This result must not be
   presented as proof of geometric impossibility.
3. **Planning pauses GUI rendering**: current search runs synchronously when
   releasing the mouse; move it to a cancelable worker in Phase 3.
4. **Path corners may stop briefly**: the follower intentionally settles at
   intermediate waypoints to avoid unsafe corner cutting. Smooth continuous
   paths will require trajectory generation in Phase 3.
5. **Internal elbow-branch restriction**: internal joint ranges currently
   exclude the straight/folded poses; some physically possible transitions
   would require a more detailed geometric contact model.
6. **Windows build layout**: Ninja commonly writes `build/robot_gui.exe`,
   while a Visual Studio multi-config generator writes
   `build/Debug/robot_gui.exe`. This is expected, not a simulator bug.

## Acceptance tests for future changes

- Preserve the existing two-joint smooth movement and elbow constraints.
- Reach feasible multi-joint targets after changing from other safe poses.
- Never accept a path with self-intersection or joint-limit violations.
- Keep intermediate links outside the target keep-out zone on all sampled
  path and follower states, and keep internal joints outside tight folds.
- Respect configured per-joint speed and acceleration bounds while following.
- Stop and show an explicit failure when planning exhausts its search budget.
- Re-run all CTest targets and validate the GUI on Windows/Ninja and/or MSVC.
