# Engine audit — performance and maintainability

**Scope:** C++20 core, motion planning, forward kinematics, geometry,
asynchronous planning, GUI boundary, build/tests. Reviewed against the
current 2–7 joint engine; GUI currently exposes 2–6 joints.

## Applied, behavior-preserving optimizations

| Component | Previous hot path | Improvement |
| --- | --- | --- |
| Forward kinematics | Every planner pose query copied a full `Joint2D` vector, replaced angles, then created positions | Shared FK kernel with direct angle override; no temporary robot copy, and reusable point buffer in collision edges |
| Planner validation | `valid()` calculated the same joint positions twice | Compute positions once and check self-contact, obstacles and target on the same geometry |
| Edge collision checking | The planner reallocated per-link target penetration vectors and recomputed FK for each sampled pose | Reuse one geometry buffer and two penetration buffers throughout each tested edge; swap depth buffers instead of copying |
| Target approach predicate | `targetApproachClear()` allocated a penetration vector even when the result was just true/false | Evaluate the common per-link depth calculation directly, without allocation, and stop on the first contact |
| Scene obstacle checks | The same obstacle shape and segment endpoints were revalidated for each arm link | Keep validation at the API boundary; perform prevalidated geometry operations inside the batch loop |

**Safety contract is unchanged.** Edges are still checked at the same
angular and Cartesian subdivisions. Joint limits, target-contact retreat,
self-collision, obstacle clearance and cancellation remain in force.
The original FK and penetration-vector functions are retained for callers.

These are algorithmic allocation/recomputation reductions, **not measured
speedup claims**. Run controlled Release benchmarks before claiming a
percentage improvement. CTest is a correctness gate, not a microbenchmark.

## Architecture observations

- **Core/GUI separation is already useful:** `robotics_core` has no
  raylib dependency, and the asynchronous worker copies scene inputs and
  publishes a completed plan. Keep these boundaries.
- **`PathPlanning2D.cpp` is a cohesion hotspot (~53 KB):** goal selection,
  RRT, path scoring, collision-aware spline preparation, trace generation and
  motion execution live together. When the next feature changes trajectory
  generation, extract its *existing* related functions into a dedicated
  implementation unit. Do not add an interface/factory hierarchy just to
  split a source file.
- **`robot_gui.cpp` (~38 KB) owns rendering, UI events and simulator state:**
  during the next substantial GUI feature, extract the existing obstacle
  editor/drawing functions into a small GUI-only file. Avoid premature
  application frameworks.
- **RRT tree nodes store separate angle vectors and use linear nearest-node
  search.** With only 2–7 joints and bounded budgets this may be adequate.
  Profile search time and allocation counts before introducing custom
  allocators, spatial indices or a new dependency.
- **Trajectory and preview caches are moved between the worker and GUI:**
  preserve move semantics. Avoid copying a full plan per frame.
- **Tests:** deterministic regression fixtures and 600 motion cases provide
  valuable coverage. However, discretely sampled collision-free witnesses
  and path checks do **not** constitute mathematical proof of continuous
  swept-volume safety. Retain the continuous-collision-detection roadmap
  item and extend adversarial cases as they are found.

## SOLID and GRASP interpretation

| Principle | Observation | Recommended action |
| --- | --- | --- |
| SRP / High Cohesion | The large planner file mixes IK/RRT, scoring, spline preparation, follower execution and trace rendering data | Separate the existing trajectory code **when a new trajectory feature actually requires it**; do not introduce interfaces with one implementation |
| Information Expert | Scene geometry owns circle/box collision calculations and FK owns relative-angle chain calculations | Preserve these shared source-of-truth operations; the new optimized overloads delegate to the same core math |
| Low Coupling / DIP | `robotics_core` is independent of raylib; the GUI consumes its APIs | Keep the dependency direction unchanged |
| GRASP Controller | `AsyncPlanner2D` owns request scheduling/cancellation and publishes immutable completed plans | Keep threading outside the IK/RRT math and do not create extra manager wrappers |
| OCP | Obstacles currently use a simple `std::variant<Circle2D, Box2D>` | This closed set is appropriate today; add polymorphism only if runtime-defined obstacle types become a concrete requirement |
| LSP / ISP | There is no meaningful inheritance-based service hierarchy | No action needed. Adding interfaces solely to satisfy an acronym would increase complexity |

## Next evidence-led improvements

1. Add optional Release-mode performance benchmarks for FK, collision checks,
   candidate evaluation and end-to-end path latency (p50/p95) across 2–7
   joints and 0/4/16 obstacles. Compare before/after under the same toolchain.
2. Add a separate AddressSanitizer/UndefinedBehaviorSanitizer CI run if the
   current runners and dependencies support it reliably.
3. Profile RRT nearest-node search and trace/spline generation before
   changing the data structures. A flat `std::vector` may still beat a
   more complex design at these sizes.
4. Extract a trajectory module only when further trajectory features make
   the current file harder to maintain; prefer existing project conventions
   and the standard library.

## Regression gate

Before publishing changes to core geometry or motion logic, run:

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

For Ninja, omit `--config Debug` and `-C Debug`. CI also runs its Linux
core build and Windows GUI build. Forward-kinematics equivalence tests,
target-depth buffer equivalence, obstacle-batch equivalence and all previous
motion/scenario tests must pass.
