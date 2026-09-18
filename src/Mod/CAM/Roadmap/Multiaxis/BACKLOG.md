# Ordered multiaxis implementation backlog

Parent: [Implementation plan](README.md). Validation IDs and gates: [Validation matrix](VALIDATION.md).

**Initial state:** all implementation work packages are NOT STARTED. This file is an issue-ready backlog, not a report of implemented features, passed tests, created GitHub issues, or assigned deadlines. Work package IDs remain stable when a package is later split into smaller commits or PRs. The listed dependencies are minimum prerequisites, not permission to bypass a milestone safety gate.

Each completed package must record the actual implementation commit, test command, test result, independently reviewed evidence, remaining limitations, and source baseline. A screenshot alone does not establish motion correctness.

## A. Baseline and kinematic correctness

### MA-001 — Reproducible build and baseline tests

**Depends on:** none. **Gate:** M0. **Suggested branch:** `cam/ma-001-baseline`.

- [ ] Record build environment, dependency versions, source SHA, exact build/test commands, and known baseline failures.
- [ ] Run existing CAM operation, machine, rotation, postprocessor, persistence, and rotary-surface tests.
- [ ] Establish independent small-math tests plus FreeCAD-loaded integration tests; verify their actual discovery/registration.
- [ ] Record controller version and obtain a reference LinuxCNC simulation environment; do not configure real hardware by assumption.

**Done:** a second run can reproduce the build and report. Validation ENV-01–ENV-03.

### MA-002 — Frame, representation, and output decision records

**Depends on:** MA-001. **Gate:** M0. **Suggested branch:** `cam/ma-002-contracts`.

- [ ] Define physical reference topology, transforms, units, tool reference, setup/workplane semantics, and compensation ownership.
- [ ] Compare typed pose storage with annotation-based storage and document compatibility, persistence, and memory costs.
- [ ] Reconcile upstream shared-workplane PR #32722, topology fix PR #32575, simulator issue #29758, and existing ADRs.
- [ ] Define supported initial modes and failure behavior. Record human review without implying upstream approval.

**Done:** no unresolved frame/sign/unit ambiguity in the first supported pipeline. Validation ENV-04, KIN-01.

### MA-003 — Fix single-axis reachability validation

**Depends on:** MA-001. **Gate:** M1. **Suggested branch:** `cam/ma-003-single-axis-reachability`.

- [ ] Reproduce the A-only `(0.866, 0, 0.5)` false-success case from the audited source.
- [ ] Replace the incorrect success expectation; add a reachable YZ-plane normal and both head/table conventions.
- [ ] Validate the achieved full 3D direction, not just the projected angle or the solver's own residual.
- [ ] Reject zero/NaN normals and impossible targets predictably; keep valid existing indexing behavior.

**Primary files:** `Path/Base/Generator/rotation.py`, `CAMTests/TestPathRotationGenerator.py`.

**Done:** the new negative case fails on the baseline, passes with the fix, and independent physical checks cover positive cases. Validation KIN-02–KIN-04. This is the preferred first small code PR.

### MA-004 — Validate and version machine configuration

**Depends on:** MA-002. **Gate:** M1. **Suggested branch:** `cam/ma-004-machine-contract`.

- [ ] Validate graph structure, parent frames, branches, finite vectors/limits, axis sign/zero conventions, and units.
- [ ] Define continuous versus bounded rotary semantics and uncertainty/calibration fields.
- [ ] Add dynamic-limit/capability fields only where needed; use explicit unknown values instead of invented limits.
- [ ] Round-trip machine files; preserve or explicitly migrate legacy behavior.

**Primary files:** `Machine/models/machine.py`, `Machine/models/validate.py`, machine editor, machine tests.

**Done:** invalid/ambiguous machines cannot enter the new realized-motion pipeline. Validation KIN-01, KIN-05, DATA-04.

### MA-005 — Full relative forward kinematics

**Depends on:** MA-003, MA-004. **Gate:** M1. **Suggested branch:** `cam/ma-005-forward-kinematics`.

- [ ] Implement local-frame transforms for both workpiece and tool branches.
- [ ] Include joint origins, signs, zero offsets, tool length, work offsets, and nonintersecting axes.
- [ ] Verify both A-carries-C and C-carries-A reference fixtures independently.
- [ ] Keep document-free math tests and add adapters to FreeCAD data types.

**Done:** independently computed physical point/direction fixtures agree; the implementation does not merely agree with its own inverse. Validation KIN-06–KIN-10.

### MA-006 — Reference-machine inverse pose solver

**Depends on:** MA-005. **Gate:** M1. **Suggested branch:** `cam/ma-006-inverse-kinematics`.

- [ ] Generate candidate rotary configurations and corresponding linear positions for the reference topology.
- [ ] Enforce full-pose reachability, linear/rotary limits, and explicit unsupported-topology errors.
- [ ] Replace inconsistent hierarchy/composition shortcuts in the new path.
- [ ] Separate candidate generation from sequence selection; expose diagnostics and physical residuals.

**Done:** valid poses pass independent round trips, invalid poses are rejected, and legacy API behavior is deliberately adapted. Validation KIN-03–KIN-11.

## B. Dependable indexed machining

### MA-007 — Shared workplane/setup integration

**Depends on:** MA-002, MA-006. **Gate:** M2. **Suggested branch:** `cam/ma-007-indexed-frames`.

- [ ] Integrate or adapt to reviewed upstream workplane changes instead of adding a competing abstraction.
- [ ] Transform model, stock, selection, depths, bounds, starts, and caches into the intended operation frame.
- [ ] Keep source document geometry unchanged and avoid cascading recomputes.
- [ ] Add explicit mode separation so later simultaneous operations cannot be transformed twice.

**Primary files:** `Path/Op/Base.py`, job/workplane code, affected operation adapters.

**Done:** frame/reference meaning is consistent through recompute and save/load. Validation IDX-01–IDX-03, DATA-01.

### MA-008 — Indexed operation and topology compatibility

**Depends on:** MA-007. **Gate:** M2. **Suggested branch:** `cam/ma-008-indexed-coverage`.

- [ ] Reconcile PR #32575 and preserve solids/faces while handling edge-only arc recovery.
- [ ] Audit Profile, Pocket/PocketShape, Drilling, facing, Adaptive, Engrave/Deburr, and surfacing individually.
- [ ] Test supported dressups; reject unsupported transformations and cross-orientation REST machining.
- [ ] Document an operation/tool/workplane capability table.

**Done:** representative indexed fixtures pass without topology loss or stale geometry. Validation IDX-01–IDX-05, DATA-02–DATA-03.

### MA-009 — Job-level safe indexing transitions

**Depends on:** MA-005, MA-007. **Gate:** M2. **Suggested branch:** `cam/ma-009-index-transitions`.

- [ ] Add semantic index requests and realize them after final operation ordering and tool selection.
- [ ] Plan withdrawal, verified indexing region, rotary movement, clamps, and approach using actual state.
- [ ] Check swept stock/fixture/holder/machine clearance conservatively; require explicit initial state.
- [ ] Protect transition barriers from G0 collapsing, array/reorder changes, and postprocessing rewrites.

**Done:** unsafe transitions fail export; reorder, tool-change, and Z-up transitions are verified. Validation IDX-06–IDX-09, COL-01–COL-02, POST-07.

### MA-010 — First indexed LinuxCNC output contract

**Depends on:** MA-008, MA-009. **Gate:** M2. **Suggested branch:** `cam/ma-010-indexed-linuxcnc`.

- [ ] Implement exact coordinate, tool-length, work-offset, mode, wrap, and clamp semantics.
- [ ] Ensure controller-side and CAM-side compensation cannot both apply the same transform.
- [ ] Preserve only valid arcs/cycles; linearize or expand unsupported cases with error bounds.
- [ ] Verify posted indexed programs against a controller/reference interpreter.

**Primary files:** machine-based `Path/Post/Processor.py`, `PostList.py`, LinuxCNC post, post tests.

**Done:** the M2 validation gate is recorded for the reference machine. Validation POST-01–POST-03, POST-06–POST-09, VERIFY-01.

## C. Simultaneous motion infrastructure

### MA-011 — Versioned tool-pose path and legacy adapters

**Depends on:** MA-002, MA-005. **Gate:** M3. **Suggested branch:** `cam/ma-011-pose-path`.

- [ ] Implement the chosen authoritative representation with frames, tip/axis conventions, motion intent, and physical units.
- [ ] Add backward-compatible XYZ conversion and reject unsupported orientation-losing conversions.
- [ ] Implement save/load, precision, versioning, undo/copy, cache invalidation, and old-document migration.
- [ ] Update bindings/build/install/test registration as needed; do not silently reuse ABC or IJK with new meaning.

**Done:** synthetic poses persist and round-trip without semantic loss. Validation DATA-01–DATA-05, POST-01.

### MA-012 — Tool-axis preview and capability-aware consumers

**Depends on:** MA-011, MA-007. **Gate:** M3. **Suggested branch:** `cam/ma-012-pose-preview`.

- [ ] Render tool orientation, reference point, and holder in explicit workpiece/machine views.
- [ ] Adapt or bypass fixed-convention PathSegmentWalker behavior for new pose paths.
- [ ] Define dressup/copy/array/transform support; fail visibly when orientation cannot be preserved.
- [ ] Invalidate display caches when any realization input changes.

**Done:** preview is correct for known poses and clearly distinguished from collision verification. Validation DATA-02–DATA-03, SIM-01–SIM-02.

### MA-013 — Continuous IK branch and winding planner

**Depends on:** MA-006, MA-011. **Gate:** M3. **Suggested branch:** `cam/ma-013-continuous-ik`.

- [ ] Carry unwrapped state and choose candidate sequences using documented deterministic costs.
- [ ] Respect physical limits and target-controller wrap direction semantics.
- [ ] Detect singular/ill-conditioned regions; preserve free-angle continuity or stop explicitly.
- [ ] Permit an unwind only as a separate verified transition, never hidden in a cut.

**Done:** angular seams, singularities, and limit cases pass the independent motion checks. Validation MOT-01–MOT-04.

### MA-014 — Interpolation-aware adaptive realization

**Depends on:** MA-013. **Gate:** M3. **Suggested branch:** `cam/ma-014-motion-tolerance`.

- [ ] Reconstruct the controller's between-block motion through physical kinematics.
- [ ] Refine by tool-position/orientation error with a bounded strategy, not a single midpoint-only heuristic.
- [ ] Detect between-endpoint linear/rotary limit violations and large rotary excursions.
- [ ] Reject uncertainty or sampling limits that prevent the configured error guarantee.

**Done:** intended and realized paths agree within documented budgets on nonlinear fixtures. Validation MOT-05–MOT-07.

### MA-015 — Physical timing and inverse-time output

**Depends on:** MA-010, MA-014. **Gate:** M3. **Suggested branch:** `cam/ma-015-feed-realization`.

- [ ] Add velocity and acceleration/look-ahead feasibility with explicit physical units.
- [ ] Handle pure orientation movement, zero-length blocks, feed-mode changes, and finite duration.
- [ ] Emit supported G93 timing with F on every feed block; keep repeated F values.
- [ ] Audit metric/imperial conversion, modal optimization, G94 restoration, and rotary-only preservation.

**Done:** timed final blocks retain their physical meaning through the complete post pipeline. Validation MOT-08–MOT-09, POST-04–POST-05, POST-07.

## D. First simultaneous operation and verification

### MA-016 — Restricted ball-nose multiaxis finishing

**Depends on:** MA-011–MA-015. **Gate:** M4. **Suggested branch:** `cam/ma-016-ballnose-finish`.

- [ ] Add a selected-surface raster or iso-parametric strategy with lead/lag, side tilt, allowance, boundaries, and tolerance.
- [ ] Compute contact, normals, ball center, and tip reference consistently under tilt.
- [ ] Handle normal orientation, degeneracies, surface/curve seams, and trimmed boundaries.
- [ ] Keep generated paths simulation-only until collision and final-program verification gates pass.

**Done:** reference surfaces pass contact and motion tests without asserting unsupported generality. Validation CUT-01–CUT-06.

### MA-017 — Swept collision checks and bounded tilt adjustment

**Depends on:** MA-009, MA-014, MA-016. **Gate:** M4. **Suggested branch:** `cam/ma-017-collision`.

- [ ] Separate intended cutter engagement from gouging and shank/holder/spindle/fixture/machine collisions.
- [ ] Use accelerated conservative swept checks with geometry and calibration uncertainty allowances.
- [ ] Reject collisions first; then add bounded orientation adjustment and safe relinking.
- [ ] Revalidate kinematics, contact, interpolation, and timing after adjustments; return useful failure reports.

**Done:** endpoint-safe/mid-motion-collision fixtures are rejected or safely replanned. Validation COL-01–COL-06.

### MA-018 — Independent posted-program verification

**Depends on:** MA-010, MA-015, MA-017. **Gate:** M4. **Suggested branch:** `cam/ma-018-post-verification`.

- [ ] Interpret actual emitted blocks using the target's modes, offsets, tool length, feed, and rotary semantics.
- [ ] Compare realized poses with the intended path using an independent oracle.
- [ ] Re-run collision and permitted-cut checks on reconstructed motion.
- [ ] Add fault-injection tests for double compensation, wrong offsets/signs, and lost feed/rotary words.

**Done:** deliberately corrupted posts fail even when the original pose path was valid. Validation VERIFY-01–VERIFY-05.

### MA-019 — Machine-aware playback and diagnostics

**Depends on:** MA-012, MA-017, MA-018. **Gate:** M4. **Suggested branch:** `cam/ma-019-machine-playback`.

- [ ] Animate head/table chains, stock, fixtures, and tool in their correct frames.
- [ ] Display limit, collision, conditioning, and feed problems linked to source operation/final block.
- [ ] Define a simulator adapter boundary rather than merely adding ABC fields to the existing parser.
- [ ] Clearly label preview-only, sampled, uncertain, and verified states.

**Done:** reference-controller playback agrees and existing three-axis simulation is not regressed. Validation SIM-01–SIM-04.

## E. Completion, commissioning, and expansion

### MA-020 — Multidirectional stock and REST verification

**Depends on:** MA-017, MA-018. **Gate:** M5. **Suggested branch:** `cam/ma-020-stock-verification`.

- [ ] Benchmark suitable stock representations for multidirectional removal and undercuts.
- [ ] Implement swept cutting removal in workpiece coordinates with resolution/error bounds.
- [ ] Add allowance/gouge reporting and order-sensitive remaining-stock invalidation.
- [ ] Enable cross-orientation REST only when independently validated.

**Done:** known-volume reference solids and undercuts have bounded verification results. Validation STOCK-01–STOCK-04.

### MA-021 — Measured machine commissioning

**Depends on:** MA-010, MA-018, MA-019. **Gate:** M5. **Suggested branch:** `cam/ma-021-commissioning-docs`.

- [ ] Confirm real coordinated-axis capability and controller contract; measure pivots, signs, offsets, and tool lengths.
- [ ] Record uncertainty, travel limits, machine/fixture geometry, and required startup state.
- [ ] Complete off-machine, spindle-off no-stock, and conservative coupon stages under appropriate operating procedures.
- [ ] Record actual measured errors; do not infer certification from simulation or pass claims from this plan.

**Done:** commissioning evidence supports exactly the advertised machine/configuration scope. Validation HW-01–HW-03.

### MA-022 — Performance, packaging, documentation, and release gates

**Depends on:** MA-018–MA-021. **Gate:** M5. **Suggested branch:** `cam/ma-022-release-hardening`.

- [ ] Profile long pose paths, memory, save/load, cancellation, and deterministic realization.
- [ ] Check native dependency portability/licensing and FreeCAD module/test installation.
- [ ] Add translated UI help, supported-capability tables, example jobs, recovery guidance, and conservative defaults.
- [ ] Publish actual evidence for each release gate; all unsupported cases must fail clearly.

**Done:** a reproducible scoped release, not an unrestricted five-axis claim. Validation PERF-01–PERF-03, DATA-05, RELEASE-01.

### MA-023 — Additional machines and machining strategies

**Depends on:** MA-022. **Gate:** M6. **Suggested branch family:** `cam/ma-023-*`.

- [ ] Add B/C, reversed hierarchy, head-head, and mixed machines behind separate independent fixtures.
- [ ] Add additional surface patterns and undercut access, then swarf and roughing as separate verified strategies.
- [ ] Consider a general numerical solver only with topology-appropriate reachability and conditioning tests.
- [ ] Extend controller modes one contract at a time; no generic claim based on axis letters.

**Done:** each new capability has its own evidence and supported-scope documentation. Validation RELEASE-01 plus extended KIN/CUT/POST suites.

### MA-024 — Upstream contribution and maintenance

**Depends on:** starts with MA-002 and continues across all gates; final reconciliation after MA-022.

- [ ] Submit small, independently reviewable correctness/test changes before broad architecture changes.
- [ ] Coordinate workplane, pose representation, output, and simulation with upstream maintainers.
- [ ] Keep human-authored upstream discussions and comply with current contribution/AI policies.
- [ ] Record upstream disposition, remove superseded fork implementations, and keep migration tests current.

**Done:** the fork's remaining delta is intentional, documented, and maintainable; no upstream acceptance is presumed.

## Suggested first coding session

Complete MA-001, then begin MA-003 with an independent negative test. The following is a proposed regression assertion to add inside the existing rotation test class, not code installed or executed by this planning commit:

```python
def test_single_a_rejects_unreachable_normal(self):
    machine = self._create_single_axis_machine()
    normal = FreeCAD.Vector(0.866, 0.0, 0.5).normalize()
    result = orientation.solve_orientation(machine, normal)
    self.assertFalse(result.success)
```

Also test a reachable direction such as normalized `(0, 1, 1)` and verify the achieved direction through a separately constructed X-axis rotation. Review all current single-axis expectations before patching; do not make the tests pass by weakening the physical constraint.

## Work-package completion record

Copy this record into a completed package or linked implementation report:

```text
Work package:
Implementation commit:
Upstream baseline:
Machine/controller/configuration:
Test commands and environment:
Automated test results:
Independent reference evidence:
Hardware evidence (or explicitly not run):
Known limitations and rejected combinations:
Human review:
Upstream PR/disposition (or none):
```
