# CAM multiaxis implementation plan

**Project:** reliable indexed 3+2 machining, followed by simultaneous five-axis machining in FreeCAD CAM.

**Repository:** `ntrp/FreeCAD`  
**Development branch:** `cam-multiaxis`  
**Prepared:** 2026-09-18  
**Source baseline:** [`691a041981b7972d3170ceadb96163019712d2b0`](https://github.com/FreeCAD/FreeCAD/commit/691a041981b7972d3170ceadb96163019712d2b0), the upstream `main` revision checked during preparation.

This is a fork-local engineering proposal, not an approved upstream roadmap or a claim that the proposed functionality works. The initial project commit adds documentation only. No FreeCAD build, runtime regression suite, LinuxCNC execution, or hardware trial was performed while preparing it. Source findings below come from static review and must be reproduced against the implementation baseline before patches are submitted. AI-assisted planning: GPT-6 Astra Pro. Human review and domain validation remain required.

## Navigation

- [Ordered implementation backlog](BACKLOG.md): 24 proposed work packages, dependencies, and acceptance criteria.
- [Validation matrix and release gates](VALIDATION.md): independent kinematic checks, operation/post integration, collision checks, and commissioning evidence.
- [Existing upstream CAM roadmap](../README.md): this proposal does not supersede it.

## 1. Repository and scope decisions

CAM is integrated into the FreeCAD source tree under `src/Mod/CAM`; it is not a standalone Python plugin that can be forked independently without its C++ modules, geometry kernel, application objects, GUI, and build integration. Retain the full FreeCAD fork and initially keep changes inside CAM, its tests, and necessary build/test registration.

The existing public `ntrp/FreeCAD` fork is reused. Its pre-existing `master` branch and repository settings are preserved. The new `cam-multiaxis` branch starts from the audited upstream revision, not from the old default branch. No upstream pull request is opened by this planning change. Backlog IDs are document identifiers, not GitHub issues or remotely created milestones.

### Initial product target

1. Dependable indexed machining for one explicitly specified table-table trunnion.
2. One simultaneous ball-nose surface-finishing strategy with controlled lead/lag and side tilt.
3. One verified LinuxCNC output contract and a matching simulated machine.
4. Explicit rejection of unsupported configurations, geometry, output modes, and transformations.
5. Progressive expansion to other machine topologies, collision-aware orientation adjustment, more strategies, and multidirectional stock verification.

**Reference machine proposal, not a statement about existing hardware:** XYZ linear axes, an outer A cradle, and an inner C table carried by that cradle. Specify physical joint frames and signs explicitly; the letters alone do not define the machine. LinuxCNC is the initial validation target because its documented `xyzac-trt`/`xyzbc-trt` examples provide useful independent references [S12]. Confirm the actual trunnion and controller before hardware output. Do not assume that an existing desktop CNC controller supports five coordinated axes or TCP merely because a rotary attachment is possible.

General impeller machining, unrestricted swarf cutting, automatic five-axis roughing, and arbitrary-machine support are not MVP requirements. They remain later work, not features obtained automatically from the first finishing operation.

## 2. Source baseline: reuse versus new work

Paths in this section are relative to `src/Mod/CAM/`.

| Component | Observed baseline | Project action |
| --- | --- | --- |
| Indexed operations | `Path/Op/Base.py` has `Workplane`, `_setup_workplane_transform()`, transformed model/stock proxies, and rotary positioning. Indexed support was merged in PR #30106 [S1, S2]. | Harden and reuse; do not rewrite every pocket/profile/drilling strategy. |
| Machine configuration | `Machine/models/machine.py` defines axis roles, hierarchy, origins, limits, maximum velocities, wrapping policies, and TCP/DWO flags [S3]. | Clarify semantics, validate data, and connect it to actual physical transforms. A stored capability is not an implemented output mode. |
| Orientation solving | `Path/Base/Generator/rotation.py` solves indexed orientations. Its generic fallback is unfinished and singularity status is a TODO [S4]. | Fix correctness before adding continuous pose planning. |
| Four-axis surfacing | `Path/Op/RotarySurface.py` and `rotary_*` generators implement dedicated continuous rotary surfacing [S6]. | Reuse geometry/pattern infrastructure where applicable; preserve its regressions. |
| Surface generation | `surface_dropcutter.py` returns XYZ samples with calculated Z values [S7]. | Reuse tessellation/patterns where appropriate; add explicit orientation and cutter-contact handling. |
| Command representation | `App/Command.*` carries numeric parameters and annotations. Existing placement helpers are not a general machine-kinematic solver [S8]. | Keep low-level command compatibility; introduce explicit pose semantics through a reviewed design. |
| Preview | `App/PathSegmentWalker.cpp` uses a fixed ABC rotation convention and a common rotation center [S9]. | Make preview frame-aware; it is not an independent physical-machine verification engine. |
| Simulation | Issue #29758 remains open; the inspected OpenGL parser/motion model lacks full rotary/machine-pose support [S10]. | Develop machine playback and verification explicitly. |
| Postprocessing | Machine-based `Path/Post/Processor.py` and `scripts/linuxcnc_post.py` provide the extension point [S11]. | Add an explicit output contract, state handling, and final-program validation. |

### Specific findings to reproduce first

**F1: single-axis reachability.** The existing A-only test requests normalized `(0.866, 0, 0.5)` and expects success. Rotation around X preserves the nonzero X component, so this direction cannot be aligned with Z by that axis alone. The solver's single-axis validation checks a projected angle rather than the achieved three-dimensional direction [S4, S5]. Replace the invalid expectation and validate accepted solutions with independent forward orientation.

**F2: pivot translations.** The operation setup obtains its geometry matrix from `geom_rotation.toMatrix()`. That path does not apply joint-origin translations [S2, S3]. This may be valid under restricted frame/controller assumptions, but it is not proof of general offset-aware kinematics. For a pivot `c`, rotation of a point is `c + R(p - c)`, not generally `R p`.

**F3: physical hierarchy.** `compute_rotation_matrix()` has a table-table special case that orders azimuth before tilt independently of the configured parent order. Mixed head/table validation checks decomposition consistency rather than a full independently derived relative pose [S4]. Reproduce with physical hierarchy fixtures before refactoring. `R_A R_C` and `R_C R_A` are not generally interchangeable.

**F4: continuity and singularity.** The indexed solver's angle candidates, wrapped-delta costs, and limits are useful, but do not constitute continuous path planning. In particular, a shortest wrapped delta must not be treated as feasible travel on a bounded non-wrapping axis [S4].

**F5: indexing safety.** `Base.py` emits rotary positioning before the operation motions; its final clearance move does not establish a swept-volume-checked inter-operation index transition [S2]. Verify the whole posted job before assigning a safety defect to any one hook; controller-specific preambles or machine assumptions may change behavior.

**F6: transformed topology.** `_transform_shape_with_arc_fix()` can return a compound of edges after arc recovery. PR #32575 is an open fix proposal for the effect on face/solid operations [S13]. Add tests and reconcile with upstream rather than independently copying an unreviewed fix.

Do not reintroduce a supposed missing return-to-zero fix: the inspected Z-up setup already emits explicit zero rotary positions on machines with rotaries [S2]. Test that behavior instead.

## 3. Architecture decisions to settle before coding

### 3.1 Explicit frame and unit contract

Use unambiguous transforms: `T_A_B` maps coordinates expressed in frame B into frame A. Define machine base M, workpiece/setup W, tool attachment H, and tool reference T, plus a local frame for each physical joint.

The relative tool pose is:

```text
T_W_T = inverse(T_M_W) * T_M_T
```

Document active/passive conventions, multiplication order, handedness, local joint axes, encoder zero/sign offsets, work offsets, tool length, and the exact meaning of the tool reference point. Both head and table chains participate in the relative transform. Do not flatten independent branches into one rotary list or sort by axis letter.

**Proposed new math-layer units:** millimetres, seconds, radians, mm/s, and rad/s. Convert explicitly at legacy and controller boundaries. Existing CAM code and the legacy ADR must be examined together: an internal command that resembles G-code does not imply that its F value is already in mm/min. Add dimensional conversion tests rather than relying on naming. Never convert rotary angles as lengths, or inverse-time feed as a length-per-time quantity [S14].

A full workplane placement specifies origin and in-plane orientation for geometry and pattern construction. It must not accidentally impose a third controllable rotary degree of freedom on an axisymmetric five-axis tool. Solve the tool-axis constraint, then map the operation frame consistently.

### 3.2 Machine-independent pose path

Propose an explicit `ToolPosePath` whose samples contain:

```text
ToolPose:
    reference_position_mm
    axis_tip_to_shank_unit_vector
    frame_id
    tool_reference_kind
    motion_kind       # cutting, link, retract, index, approach
    feed_intent       # physical requirement, not a preformatted G-code F
    position_tolerance_mm
    orientation_tolerance_rad
    source_operation_and_segment
```

Position plus a unit axis captures the five relevant pose degrees of freedom for an axisymmetric tool. Use a quaternion only where a full orientation is useful; spindle roll is not automatically an additional commanded machining axis. Define normalization and zero/NaN input rejection.

The pose path should remain upstream of machine-specific XYZABC output. Do not use ABC as universal orientation, and do not overload I/J/K, which already carry arc-center semantics. Retain `Path.Command` compatibility for existing operations and output.

Choose storage after a focused design spike: typed pose data with a derived legacy display/output cache versus versioned command annotations with strict consumer contracts. Review ADR-002 and ADR-007 first [S14]. Do not introduce a second contradictory authoritative path. Define save/restore, copy, undo, migration, precision, unsupported older-version behavior, and cache invalidation. Avoid millions of heavyweight Python objects as the long-term format without measurements.

### 3.3 Functional boundaries

Suggested names below are proposals, not existing APIs:

```text
Machine/Kinematics/          physical transforms, validation, FK/IK candidates
Path/Base/Motion/            pose path, constraints, trajectory realization
Path/Base/Generator/         geometry and machining strategy generation
Path/Post/                  controller-specific lowering and formatting
Path/Main/Gui/ + PathSimulator/    preview, playback, verification integration
```

Conceptual interfaces:

```text
forward_relative_pose(machine, joints, setup, tool)
inverse_pose_candidates(machine, pose, setup, tool)
plan_pose_sequence(machine, poses, initial_state, constraints)
realize_controller_motion(plan, controller_contract)
verify_posted_program(program, machine, setup, tools, geometry)
```

Keep the first math implementation testable without a FreeCAD document or GUI. Use Python/NumPy for initial integration where practical; move measured hotspots to C++ or an appropriately licensed native dependency only when profiling justifies it. Reuse one production transform engine across planner and renderer, but validate it with independent reference mathematics and controller examples.

### 3.4 Output ownership

Select one fully specified first implementation mode, then add the other behind separate tests:

| Mode | XYZ meaning | Compensation owner |
| --- | --- | --- |
| Controller-kinematic/TCP | Tool reference in the agreed workpiece frame, plus selected rotary coordinates. | Controller applies the configured physical compensation. CAM still checks machine feasibility. |
| CAM-compensated | Already realized machine-axis coordinates. | CAM realizes offsets/rotations. Controller must not apply the same transform again. |

For the first target, use the controller-kinematic mode only after matching LinuxCNC's exact topology, signs, offsets, and tool-length behavior. A TCP boolean or generic G43 command is not sufficient. Do not rotate existing indexed geometry and then ask the controller to compensate that same rotation again. DWO, tilted workplanes, and TCP are separate capabilities with separately tested activation/cancellation sequences [S3, S11, S12].

## 4. Step-by-step implementation

### Step 1 — Establish the development and evidence baseline

**Backlog:** MA-001.

Clone the new development branch, configure a reproducible build using the current repository's documented tooling, and record exact compiler/dependency/build/test commands. Keep a Linux environment for headless tests and the reference LinuxCNC configuration; use the normal development workstation for GUI integration. Record FreeCAD, OCCT, OpenCAMLib, Python, NumPy, Qt, and compiler versions actually used.

Run existing CAM tests before changes, especially rotation, machine configuration, indexed operations, rotary surfacing, postprocessing, and persistence. Register all new tests and installed modules in the appropriate CAM test runners/CMake lists. Save logs and separate baseline failures from new regressions. Do not claim the current source passes merely because tests exist.

**Exit:** a reproducible build and an explicit baseline report, with source SHA and tool versions.

### Step 2 — Reconcile upstream work and record design decisions

**Backlog:** MA-002.

Review PR #32722, which currently proposes shared job-owned workplanes as full placements; it was open when this plan was prepared. Review PR #32575 for transformed topology and issue #29758 for simulation [S10, S13]. Recheck status before each overlapping patch. Integrate reviewed upstream changes when suitable; never assume a pending branch is already part of the baseline.

Write fork-local decision records for frames/units, machine topology, pose representation, output ownership, collision scope, and backward compatibility. Treat these as proposals requiring review, not approved upstream ADRs. Consult current contribution and AI policies before proposing upstream work [S15].

**Exit:** decisions and unresolved questions documented; no duplicate workplane abstraction or hidden compensation assumption.

### Step 3 — Add failing physical correctness tests

**Backlog:** MA-003.

Reproduce F1 with the A-only unreachable vector; add reachable A-only directions, head-only and table-only cases, zero-length normals, reversed signs, and degenerate inputs. Create independent reference calculations using explicit matrices, not the solver's own helper or `error_norm`. Keep a minimal regression fix separable from the later architecture change.

**Exit:** the reported defect has a demonstrable failing test, then a minimal correction with unchanged valid behavior.

### Step 4 — Implement validated physical machine transforms

**Backlog:** MA-004, MA-005, MA-006.

Validate the machine graph, parent references, normalized joint vectors, finite limits, coordinate units, and frame conventions. Implement both tool-side and workpiece-side chains with translations, calibrated axis signs/zeros, work offsets, tool attachment, and tool length. Add safe migration/defaults for legacy machine files; reject ambiguous unsupported layouts instead of guessing a root or silently ignoring branches.

Implement inverse pose candidates for the reference A/C topology. Re-evaluate candidates through independent forward checks. Preserve hierarchy; replace the azimuth-sort shortcut where the new contract is used. Keep other topologies explicitly experimental or unsupported until their acceptance fixtures pass. Do not silently change legacy machine semantics without a migration/compatibility decision.

**Exit:** the reference machine passes offset, hierarchy, reachability, and tool-length round trips, including nonzero and nonintersecting pivot fixtures.

### Step 5 — Complete indexed workplane integration

**Backlog:** MA-007, MA-008.

Adapt existing operations to the agreed workplane/setup contract. Reuse transformed geometry, but audit direct `obj.Base`, `self.model`, `self.stock`, and job-shape access paths. Ensure depths, bounds, entry/exit paths, stock references, selection, and cached meshes use the correct frame. Check shape topology, faces, solids, arcs, and naming stability after transformation. Avoid writes to source document placements that trigger cascading recomputes.

Test Profile, Pocket/PocketShape, Drilling, facing, Adaptive, Engrave/Deburr, and supported surfacing individually. Publish an explicit capability matrix. Start with no remaining-stock reuse across orientations; either implement correct stock-frame transformations or reject that combination.

**Exit:** indexed operations reproduce known geometry in multiple workplanes and save/reload without stale or double-transformed results.

### Step 6 — Plan safe indexing at job level

**Backlog:** MA-009.

Represent indexing as a semantic event. At final job sequencing, use the actual previous pose, active tool, work offset, and machine state to plan withdrawal, travel to a verified indexing region, clamp/unclamp where applicable, rotation, and approach. Account for stock, physical workholding, holder, spindle, and machine-component swept volumes, not just the operation's Z-clearance plane.

No safe initial state means a required setup/entry contract or an export error, not an assumed origin. Changing operation order or tool must regenerate transitions. Preserve safety barriers through rapid collapsing and other optimizations. Unknown G0 interpolation must not be treated as a straight collision-free move; use documented target semantics or controlled verified moves.

**Exit:** reordered mixed-workplane jobs have verified transitions, including return to Z-up, restart boundaries, tool changes, and impossible-transition rejection.

### Step 7 — Deliver a verified 3+2 output mode

**Backlog:** MA-010.

Extend the machine-based postprocessor, not an isolated legacy post. Implement one controller contract with explicit modes, rotary limits/wrap behavior, work offsets, tool length, clamps, and startup/shutdown state. Preserve arcs only in supported fixed planes and output modes; otherwise linearize within tolerance. Expand unsupported tilted canned cycles rather than emitting plausible but incorrect code.

Compare final posted output with an independent interpreter/reference controller. No continuous cutting is enabled yet.

**Exit / M2:** a documented reference-machine indexed workflow passes the relevant validation gate. Hardware commissioning remains conditional on measured machine configuration and safety checks.

### Step 8 — Add persistent tool-pose infrastructure and preview

**Backlog:** MA-011, MA-012.

Implement the selected representation, typed frame/reference metadata, unit adapters, and legacy XYZ adapter. Mark simultaneous operations so the existing indexed `Base.py` hook cannot also rotate their geometry or prepend index commands incorrectly. Preserve ordinary three-axis behavior.

Add tool-axis glyphs, tool/holder geometry, and explicit workpiece-frame versus machine-frame views. Do not present the old fixed-ABC/common-center walker as accurate playback for arbitrary machines. Version saved data and invalidate generated caches on geometry, workplane, machine calibration, tool, or tolerance changes. Incompatible dressups must reject the operation explicitly rather than silently drop orientation.

**Exit:** synthetic pose paths round-trip through save/load and display correctly; unsupported transforms fail predictably.

### Step 9 — Implement continuous solution selection

**Backlog:** MA-013.

Generate IK candidates at samples and select a continuous sequence rather than invoking the indexed solver independently at every point. Carry actual unwrapped joint positions. Respect bounded versus continuous axes and controller wrapping policies. Start with candidate-graph/dynamic-programming selection using travel, limit margin, conditioning, and collision costs; preserve deterministic behavior and explain failures.

Detect orientation singularities with the relevant reduced Jacobian or another justified criterion. Near singularities, hold an unconstrained angle continuously, replan orientation within a user-approved envelope, or stop with a diagnostic. Never silently flip the table or insert an unwind inside a cut. Any unwind is a separate verified noncutting transition.

**Exit:** seam, singularity, branch-switch, and winding test paths have continuous feasible motion or a clear refusal.

### Step 10 — Respect actual interpolation and dynamics

**Backlog:** MA-014, MA-015.

Simulate each target controller block's interpolation semantics through forward kinematics. Refine segments when realized tool position/orientation deviates from the intended pose path. Check between endpoints, and do not certify a segment from one midpoint alone. Include limits on sampling error, rotary excursion, and collision uncertainty; stop if the requested bound cannot be established within resource limits.

Use physical feed intent and per-axis velocity limits to establish segment-time lower bounds, then apply acceleration/look-ahead constraints consistent with the controller. The first lower bound can be `dt >= max(ds_tip / v_tip, max_j(abs(dq_j) / v_j_max))`, with compatible units. It is not a complete acceleration or cutting-load model. Handle pure orientation moves explicitly; zero tip distance does not mean zero duration.

For a supported LinuxCNC G93 mode, emit `F = 60 / dt_seconds` and retain an F on every G1/G2/G3 block, including numerically repeated values. G93 is an output choice, not a universal requirement for five-axis cutting. Do not scale inverse-time F for imperial units. Audit the complete optimization pipeline and mode changes [S16].

**Exit:** final blocks meet the configured geometric error budget and feasible timing constraints; no rotary-only motion or required feed word is optimized away.

### Step 11 — Build one restricted simultaneous finishing operation

**Backlog:** MA-016.

Implement `MultiAxisSurfaceFinish` or the upstream-agreed equivalent for an axisymmetric ball-nose tool on selected accessible surfaces. Reuse existing mesh and path-pattern infrastructure where it fits. Provide lead/lag, side tilt, stepover, surface allowance, tolerance, and explicit cutting boundaries. Start with a simple raster or iso-parametric strategy; declare unsupported seams/trimmed-surface cases until tested.

Construct contact points/normals and tool orientations, then calculate the tool reference correctly. For contact q, outward normal n, ball radius r, and unit tool axis a from tip to shank: ball center `c = q + r*n`; nominal tip `p = c - r*a`. Validate the actual cutting hemisphere and adjacent surfaces. Do not append rotary angles to an unmodified existing XYZ tip path.

Resolve normal consistency, curve/surface seams, lead/lag continuity, boundary trimming, and degenerate patches. Separate contact generation from orientation and machine realization. Initial output is simulation-only until the collision and verification gate passes.

**Exit:** a plane, tilted plane, convex curved surface, and a trimmed surface fixture produce bounded, continuous cutter-contact paths without claimed general five-axis coverage.

### Step 12 — Add collision checks and collision-aware orientation

**Backlog:** MA-017.

Represent cutter cutting regions, noncutting shank, holder, spindle, physical workholding, stock, and moving machine components separately. Distinguish intended material removal from gouging or noncutting collisions. Use mesh/analytic broad-phase acceleration and a conservative continuous or adaptively bounded narrow phase. Include tessellation, interpolation, and calibration uncertainty in clearance tests.

First reject invalid motion. Then add bounded tilt adjustment and relinking with a stated search limit; an unsuccessful search must not become a claim that no feasible machining strategy exists. Revalidate geometry and kinematics after every orientation change.

**Exit:** endpoint-safe/mid-motion-unsafe examples are detected; no collision status is marked verified merely because sparse samples passed.

### Step 13 — Verify the posted program and add machine playback

**Backlog:** MA-018, MA-019.

Parse the emitted program in the controller context, including work offsets, modes, tool length, units, and rotary semantics. Reconstruct the actual relative tool path and compare it with the intended path and permitted cutting region. Use independent mathematics or a controller oracle to catch errors shared by generation and preview. A generic G-code parser without the target's compensation semantics is insufficient.

Add machine-aware playback with correct head/table component transforms, axis-limit overlays, and collision location linked to operation and final block. Reuse production transforms for UI consistency, but retain independent numerical tests. Keep the old three-axis simulator behavior isolated and explicit until the backend is genuinely multiaxis-capable.

**Exit / M4:** one reference machine has a verified simultaneous finishing workflow with representative test parts, final-program verification, and usable diagnostics.

### Step 14 — Implement multidirectional stock verification

**Backlog:** MA-020.

Evaluate voxel/SDF, multi-directional dexel, and mesh approaches against accuracy, memory, undercuts, boolean robustness, and integration effort. Reuse an existing backend only after demonstrating the necessary pose/sweep semantics. A single-direction height representation cannot establish general undercut removal.

Add persistent workpiece-frame remaining-stock state, swept cutting-volume removal, allowance/gouge checks, and conservative reporting of uncertain regions. Only enable REST machining across orientations when stock transforms, ordering, cache invalidation, and precision have been validated.

**Exit:** multi-direction and undercut reference cases are checked against known solids; stock-based claims carry explicit resolution/error bounds.

### Step 15 — Commission, package, and document the first release

**Backlog:** MA-021, MA-022.

Measure the real machine's axes, pivots, signs, work offsets, tool lengths, travel limits, homing state, and relevant uncertainty. Confirm controller coordinated-axis capability and configured kinematic mode. Begin with off-machine simulation, then spindle-off no-stock motion using appropriate safe procedures, then a conservative test coupon after all gates pass. Record measured deviations and reject configurations outside the agreed error budget. CAM changes cannot establish hardware stiffness, backlash, or controller correctness by themselves.

Document supported topologies/tools/strategies, unsupported combinations, recovery procedures, export semantics, and reproducible examples. Add feature flags and defaults that cannot silently enable unverified modes. Profile representative large paths, cancellation behavior, save/reload, and deterministic output. Keep UI strings translatable and new dependencies compatible with project packaging and licensing requirements.

**Exit / M5:** documented tested scope with reproducible evidence. Do not label the result unrestricted five-axis CAM.

### Step 16 — Expand deliberately and contribute upstream

**Backlog:** MA-023, MA-024.

Add B/C trunnions, reversed hierarchies, head-head, and mixed head/table machines one at a time with independent fixtures and post contracts. Add additional finishing patterns, undercut access, swarf, and roughing as separate strategies with their own cutter-engagement and verification requirements. Keep safe refusal for unimplemented layouts.

Contribute small reviewed changes upstream, starting with reproducible correctness fixes and tests. Coordinate representation, workplane, and simulator design before a broad rewrite. Keep upstream discussions human-authored, disclose AI assistance, and do not assert human verification that has not occurred [S15]. A fork-local planning document is not an upstream endorsement.

**Exit / M6:** separately evidenced capabilities and reviewable contributions, rather than one unreviewable permanent fork delta.

## 5. Milestones and dependencies

| Gate | Deliverable | Required work packages |
| --- | --- | --- |
| M0 | Reproducible baseline and reviewed frame/output design. | MA-001–MA-002 |
| M1 | Physically correct reference-machine kinematics. | MA-003–MA-006 |
| M2 | Reliable reference-machine indexed machining and output. | MA-007–MA-010, plus M1 |
| M3 | Synthetic continuous pose pipeline; no machining-readiness claim. | MA-011–MA-015 |
| M4 | Restricted simultaneous finishing with collision and final-program verification. | MA-016–MA-019, plus M2/M3 |
| M5 | Stock verification, measured commissioning, packaging and documented scope. | MA-020–MA-022 |
| M6 | Additional machine/strategy coverage and upstream integration. | MA-023–MA-024 |

The work is ordered by correctness and integration dependencies, not calendar promises. Upstream coordination is ongoing even though MA-024 records the final integration checklist. Collision work and controller validation can start early once the frame/output contract exists; neither is optional for a machining-ready release.

## 6. Start working locally

For a fresh directory:

```bash
git clone --filter=blob:none --branch cam-multiaxis https://github.com/ntrp/FreeCAD.git FreeCAD-multiaxis
cd FreeCAD-multiaxis
git remote add upstream https://github.com/FreeCAD/FreeCAD.git
git fetch upstream
```

Use a small feature branch per work package, for example:

```bash
git switch -c cam/ma-003-single-axis-reachability cam-multiaxis
```

Follow MA-001 before claiming a test run. Do not blindly rebase shared branches or change the old default `master`. Incorporate upstream through reviewed merges or an agreed branch policy; record the resulting source SHA and re-run the acceptance set. The first code contribution should normally be the reproduced single-axis reachability regression, not the new surface generator.

## 7. Source references and verification limits

The pinned links below document the reviewed baseline. Subsequent implementation must recheck upstream changes. Source descriptions are not runtime results.

- **S1:** [Indexed machining PR #30106](https://github.com/FreeCAD/FreeCAD/pull/30106), merged 2026-06-04; describes supported operations and legacy-post/REST limitations.
- **S2:** [`Path/Op/Base.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Path/Op/Base.py): workplane transforms, shape proxies, index emission, and clearance handling.
- **S3:** [`Machine/models/machine.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Machine/models/machine.py): machine configuration and persistence.
- **S4:** [`rotation.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Path/Base/Generator/rotation.py): indexed solver and its validation/composition paths.
- **S5:** [`TestPathRotationGenerator.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/CAMTests/TestPathRotationGenerator.py): existing single-axis and other rotation tests.
- **S6:** [`RotarySurface.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Path/Op/RotarySurface.py): dedicated continuous four-axis operation.
- **S7:** [`surface_dropcutter.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Path/Base/Generator/surface_dropcutter.py): XYZ drop-cutter interface.
- **S8:** [`Command.h`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/App/Command.h), [`Command.cpp`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/App/Command.cpp): parameters, annotations, placement helpers, and serialization.
- **S9:** [`PathSegmentWalker.cpp`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/App/PathSegmentWalker.cpp): existing rotary preview convention.
- **S10:** [Simulation issue #29758](https://github.com/FreeCAD/FreeCAD/issues/29758), open as checked 2026-09-18; [`MillMotion.h`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/PathSimulator/AppGL/MillMotion.h) and [`GCodeParser.cpp`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/PathSimulator/AppGL/GCodeParser.cpp).
- **S11:** [`Processor.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Path/Post/Processor.py), [`linuxcnc_post.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Path/Post/scripts/linuxcnc_post.py), and [`PathOptimizationUtils.py`](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/src/Mod/CAM/Path/Post/PathOptimizationUtils.py).
- **S12:** [LinuxCNC five-axis kinematics](https://linuxcnc.org/docs/stable/html/motion/5-axis-kinematics.html), checked 2026-09-18. Match the actual controller version and sign conventions; do not assume its use of forward/inverse terminology matches this proposal.
- **S13:** [Shared workplanes PR #32722](https://github.com/FreeCAD/FreeCAD/pull/32722) and [transform-shape fix PR #32575](https://github.com/FreeCAD/FreeCAD/pull/32575), both open as checked 2026-09-18.
- **S14:** [ADR-002](../ADR/ADR-002.md) and [ADR-007](../ADR/ADR-007.md). The former is marked Legacy and the latter Draft at the baseline; neither alone proves current runtime behavior.
- **S15:** [Upstream AI policy](https://github.com/FreeCAD/FreeCAD/blob/691a041981b7972d3170ceadb96163019712d2b0/AI_POLICY.md) and [CAM roadmap](../README.md).
- **S16:** [LinuxCNC G-code reference](https://linuxcnc.org/docs/stable/html/gcode/g-code.html), particularly G93/G94/G95 and coordinate/motion modes; checked 2026-09-18.
