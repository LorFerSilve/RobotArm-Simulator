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
- [x] **Two-joint safe detours:** enumerate periodic closed-form IK solutions,
      check opposite-direction shoulder rotations, and plan joint-space
      paths for both two-joint and redundant arms.
- [x] **Five-joint follower correction:** use one time parameter per
      collision-checked joint-space segment rather than separate joint
      controllers that cut corners; add a 5-joint (4,-9) to (-3,-4)
      playtest regression.
- [x] **Test-friendly presets:** add keyboard shortcuts 4/5 to the
      existing 2/3 presets.

## Outstanding work, in intended order

### Phase 2 — Scene obstacles and geometry

- [x] Add circles and axis-aligned rectangles to the GUI, including edit/move/delete.
- [x] Define a shared 2D obstacle representation outside rendering code.
- [x] Detect link-to-obstacle collisions, respecting arm thickness and clearance.
- [x] Validate start/goal configurations against scene obstacles.

### Phase 3 — Motion planning and smooth trajectories

The first scene-aware planning integration is already included in Phase 2.
A bounded RRT planner can attempt to route around fixed obstacles; global
completeness and real-time replanning are not yet guaranteed.

- [x] Integrate obstacle checks into every configuration-space path edge and follower step.
- [x] Report specific planning failures: no IK solution, exhausted sample
      budget, target inside an obstacle, cancellation, or out-of-reach goal.
- [x] Permit virtual numerical IK iterations through scene obstacles
      while validating final IK poses and all physical trajectories.
- [x] Try straight Cartesian IK waypoint lifts for tip-distance-efficient
      motion; compare valid two-joint candidates by sampled end-effector
      travel distance, with safe IK/RRT fallback.
- [x] Identify two-joint target/shoulder winding barriers and elbow-imposed
      inner workspace limitations without falsely blaming the search budget.
- [ ] Add adaptive RRT budgets and objective-aware multi-start search for
      constrained environments where Cartesian lifting fails.
- [x] Apply bounded Cartesian-distance shortcutting to RRT paths, accepting
      only collision-checked transitions that reduce end-effector travel.
- [x] In obstructed scenes, compare several safe RRT routes / IK endpoints
      and choose the lowest-cost fully prepared trajectory.
- [ ] Explore globally stronger cost-aware planning (e.g. asymptotically
      optimal RRT* or informed sampling). The current method is a bounded
      local improvement, not a guarantee of a global optimum.
- [x] Run planning on a C++20 worker; cancel superseded jobs when targets,
      robot geometry, or obstacles change. Prevent stale results from applying.
- [x] Prepare C2 quintic Hermite trajectories with a shared time clock;
      round intermediate corners only when each new curve is collision-free.
      Respect per-joint speed/acceleration and a global jerk bound.
- [ ] Improve curvature/jerk optimization and remove avoidable stops at
      narrow corners that are currently conservatively rejected.
- [ ] Preserve joint velocity/acceleration continuity across target changes
      while the previous plan is still executing.
- [ ] Support controlled reconfiguration when feasible (including elbow-branch
      transitions in a future joint/geometry model).
- [ ] Add replanning and safe stopping around moving obstacles.

### Phase 4 — Diagnostics and analysis

- [x] Graph angle, speed, acceleration, Cartesian error, Jacobian condition
      and planner state over time in the ANALYZE tab. The history is bounded
      to the most recent 3,600 simulation frames (~60 s at 60 Hz).
- [x] Display a sampled end-effector path silhouette from the actual
      planned spline or collision-checked joint-space polyline.
- [x] Visualize other joint paths, the predicted final pose, and approximate
      link/target collision envelopes. These overlays are diagnostic only,
      not mathematically certified continuous collision detection.
- [x] Support reproducible recording/replay of joint positions, velocities,
      target geometry, tool contact length and static obstacles. The viewer
      replays stored frames and never recomputes a planner solution.
- [ ] Consider optional full-session/unbounded capture for long experiments
      if the current rolling 60-second recording proves insufficient.

### New 2D hardening roadmap — before Phase 5

The 3D phase remains deliberately postponed. Complete these 2D
milestones before generalizing spatial representations.

#### 2D-A — Geometry and collision safety

- [x] Add conservative interval certification for **continuous self- and
      scene-obstacle safety** along linear joint-space edges. Use a bound
      on possible link displacement instead of assuming fixed-step
      collision sampling is sufficient. Reject uncertifiable intervals.
- [x] Integrate the common swept-link check in the planner's edge
      validator (also used by follower and spline checks).
- [x] Regress a narrow obstacle missed by uniform angular sampling and
      a four-link self-contact occurring between safe endpoints.
- [x] Add an explicit solid target model (circle or axis-aligned
      rectangle), configurable shaft clearance and physical terminal
      tool-contact length. Integrate the geometry with planning/follower
      safety and the SCENE editor. Keep old APIs available for compatibility.
- [x] Certify continuous target-object approach and retreat along linear
      joint-space edges, with separate shaft and tool-contact geometry.
- [ ] Support custom link radii where needed, with one consistent
      clearance convention in planning, execution and visualization.
- [ ] Add adversarial checks near tangencies and narrow passages; keep
      rejection conservative if a safe path cannot be certified.

#### 2D-B — Robust IK and planning

- [ ] Improve goal search in difficult redundant configurations with
      adaptive bounded sampling and more informative failure causes.
- [ ] Explore only physically valid joint-branch transitions.
- [ ] Retain collision-aware shortest-tool-path heuristics without
      replacing safety with a distance objective.

#### 2D-C — Motion controller and trajectory quality

- [ ] Keep velocity/acceleration continuous across retargeting and
      distinguish controlled stop from emergency stop.
- [ ] Refine spline corner treatment while preserving verified clearance.

#### 2D-D — End-effector pose and manipulation

- [ ] Support orientation targets (x, y, theta) for robots with
      sufficient degrees of freedom.
- [ ] Model a tool contact frame and basic, meaningful approach tasks.

#### 2D-E — Validation and architecture hardening

- [ ] Capture full replayable **planning scenarios**, including all joint
      limits, motion settings and scene state (beyond GUI-frame replay).
- [ ] Add deterministic safety/property regressions for new features.
- [ ] Split existing planner/GUI modules only if the next change
      meaningfully benefits from clearer responsibilities.

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
- [x] **Target-object geometry, first implementation:** circle/rectangle
      bodies with adjustable size and shaft margin; explicitly sized terminal
      tool contact section (instead of an implicit proportion of target size).
- [x] **Target-object continuous safety on linear edges:** protect the
      shaft against circles and rectangles, including retreat from existing
      contact and rejection of re-entry. The legacy radius uses sampled checks.
- [ ] **Approach direction:** define the desired tool-facing orientation
      when orientation/pose targets are added in 2D-D.
- [x] **Target-under-arm recovery (initial implementation):** multi-start IK
      can search through virtual configurations without requiring its internal
      iterations to be target-clear; a real planned route may safely retreat
      from contact that existed when the target was selected.
- [ ] **Further safety-aware replanning:** handle difficult joint-branch or
      clearance constraints where the bounded search still cannot find a
      feasible detour. Do not solve this by relaxing collision constraints.

## Permanent motion regression policy

- [x] Add `tests/test_motion_scenarios.cpp` with explicit start/destination
      pairs and reproducible joint configurations.
- [x] Add `docs/MOTION_SCENARIOS.md` describing the scenario fixtures and
      how to extend them.
- [x] Add Linux/Windows GitHub Actions build-and-CTest checks.
- [x] Prevent an entire last link from being exempted when the tip nears a
      target; retain only a short end-effector tool allowance.
- [x] Increase target sampling in planned joint-space edges where a large
      Cartesian sweep could cross the target between angular samples.
- [ ] Record the **exact joint angles** from the latest multi-joint
      screenshot and promote that case to a required regression.
- [x] Add conservative continuous interval certification for **static
      scene obstacles and nonadjacent-link contact** on linear joint-space
      edges (2D-A). It rejects intervals it cannot certify.
- [x] Apply the linear certificate to explicit targets and test both
      certified retreat and a grazing collision between clear samples.
- [ ] Support arbitrary curved swept-volume trajectories and moving
      obstacles before making real-world collision-safety claims.

## Playtest issues to keep visible

0a. **Two-joint target blocks shortest shoulder rotation**: a direct
    move from end-effector `(4,0)` to `(2,3)` with 4 m and 2 m links
    rotates the first link through the target. The earlier 2-joint controller
    stopped as `BLOCKED` despite a valid longer route. Now the planner
    checks equivalent shoulder poses offset by full turns and can choose
    the opposite rotation direction. This requires compatible shoulder
    joint limits and remains a bounded, collision-checked search.

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
3. **Planning pauses GUI rendering**: resolved with an asynchronous C++20
   worker and stop-token cancellation on new targets/scene changes.
4. **Path corners may stop briefly**: collision-validated quintic spline
   rounding has been added. If the rounded corner is not provably safe
   under the existing sampling checks, the joint-space tangent becomes zero
   and the robot deliberately pauses at that waypoint.
5. **Internal elbow-branch restriction**: internal joint ranges currently
   exclude the straight/folded poses; some physically possible transitions
   would require a more detailed geometric contact model.
6. **Windows build layout**: Ninja commonly writes `build/robot_gui.exe`,
   while a Visual Studio multi-config generator writes
   `build/Debug/robot_gui.exe`. This is expected, not a simulator bug.

## Engine performance and maintainability audit

- [x] Review FK, collision, planner, asynchronous boundary, tests and GUI
      structure; record measured-versus-hypothesized opportunities in
      [ENGINE_AUDIT.md](ENGINE_AUDIT.md).
- [x] Remove temporary robot copies from planner FK queries; share the same
      FK math and reuse a positions buffer in the swept-edge loop.
- [x] Stop repeated FK work and target-depth vector allocation in sampled
      planner safety checks, preserving exact safety conditions.
- [x] Avoid repeated obstacle-shape validation inside multi-link collision
      batch queries.
- [ ] Collect apples-to-apples Release-mode p50/p95 planning timings and
      allocation counts before any major data-structure redesign.
- [ ] Consider separate trajectory implementation and GUI editor modules
      when their responsibilities next need substantial changes.

## Acceptance tests for future changes

- Preserve the existing two-joint smooth movement and elbow constraints.
- Reach feasible multi-joint targets after changing from other safe poses.
- Never accept a path with self-intersection or joint-limit violations.
- Keep intermediate links outside the target keep-out zone on all sampled
  path and follower states, and keep internal joints outside tight folds.
- Respect configured per-joint speed and acceleration bounds while following.
- Stop and show an explicit failure when planning exhausts its search budget.
- Re-run all CTest targets and validate the GUI on Windows/Ninja and/or MSVC.
