# Multiaxis validation matrix and release gates

Parent: [Implementation plan](README.md). Work packages: [Backlog](BACKLOG.md).

**Status:** every test below is PLANNED / NOT RUN by this documentation change. Test IDs define acceptance work; they do not assert that fixtures, harnesses, or passing results already exist. The audited source revision is `691a041981b7972d3170ceadb96163019712d2b0`.

## 1. Evidence principles

Use three complementary levels: independent document-free mathematics, FreeCAD operation/post integration, and target-controller or measured-machine validation. The planner and renderer may share a production transform implementation, but their agreement is not independent evidence. Do not use the inverse solver's own forward helper or `error_norm` as the only correctness oracle.

For every result record source SHA, machine/setup/tool configuration, input fixture, expected physical behavior, test command, actual error, limits, and outcome. Failed, skipped, unsupported, and inconclusive are distinct states. Unknown collision/clearance status must not be displayed as verified.

Validate the final posted program and the motion between endpoints, not only generated samples. Fault injection must demonstrate that verification rejects a wrong-but-plausible program. A renderer or a parser that ignores compensation modes cannot establish controller equivalence.

### Error budgets

Distinguish numerical residuals, surface tessellation error, cutter-contact error, toolpath interpolation error, controller blending/rounding, stock resolution, and hardware calibration uncertainty. Allocate these explicitly rather than applying one tolerance independently at every stage. Position and orientation tolerances are different units; orientation error can create position/clearance error over a long tool/holder lever arm.

For bounded, ideal reference fixtures, proposed initial math-test thresholds are 1e-6 mm position and 1e-8 rad direction error, evaluated with a numerically stable angular metric. These are test-design starting points, not manufacturing guarantees; adjust with justification for conditioning and scale. Actual machining tolerance, stock resolution, and clearance margins must be selected from the use case and measured configuration. Tests must not relax tolerance merely to hide a frame/sign defect.

## 2. Baseline and configuration

| ID | Test / fixture | Required outcome |
| --- | --- | --- |
| ENV-01 | Clean build at the pinned revision. | Exact environment and commands recorded; failures identified before implementation. |
| ENV-02 | Existing CAM suite, with new-test discovery checks. | Baseline captured; introduced tests actually execute and failures cannot be silently skipped. |
| ENV-03 | LinuxCNC version and reference simulation configuration. | Exact topology, sign convention, offsets, mode, and tool-reference semantics documented. |
| ENV-04 | Current upstream workplane/topology/simulation changes. | Overlap reviewed; pending PRs not represented as merged code. |
| KIN-01 | Cycles, disconnected required joints, invalid parents, NaN vectors, ambiguous joint frames. | Explicit configuration error; no arbitrary-root fallback in the new pipeline. |
| KIN-02 | A-only unreachable normalized `(0.866, 0, 0.5)`. | Rejected: an X-axis rotation cannot remove its X component. |
| KIN-03 | A-only reachable normalized `(0, 1, 1)`; corresponding head and table cases. | Correct signed orientation, independently verified, or a justified limit rejection. |
| KIN-04 | Zero/NaN direction and near-degenerate normal. | Deterministic input rejection with no stale usable output. |
| KIN-05 | Bounded rotary versus continuous/unwrapped rotary. | Travel and equivalence follow configuration; no illegal shortest-wrap substitution. |
| KIN-06 | A carries C versus C carries A, both at nonzero angles. | Distinct expected transforms match independent matrices. |
| KIN-07 | Nonzero pivot; verify `c + R(p-c)`. | Translation and rotation both correct; origin-only implementation fails this fixture. |
| KIN-08 | Nonintersecting rotary axes and translated setup origin. | Correct relative tool pose with offsets applied exactly once. |
| KIN-09 | Changed tool length and head/tool attachment offset. | Expected reference-point shift without direction error or double compensation. |
| KIN-10 | Separate table and head chains, sign reversals, and zero offsets. | Correct relative transform; unsupported topologies rejected explicitly. |
| KIN-11 | Random feasible poses and intentionally unreachable poses. | Independently checked FK/IK round trips, limit handling, and reproducible seeds. |

## 3. Indexed operations and safe transitions

| ID | Test / fixture | Required outcome |
| --- | --- | --- |
| IDX-01 | Profile, Pocket/PocketShape, Drilling, facing, and Adaptive at multiple fixed orientations. | Geometry and depths match independent transformed reference jobs. |
| IDX-02 | Translated workplane origin and nontrivial in-plane orientation. | Geometry/pattern frame remains correct without demanding an extra rotary degree of freedom. |
| IDX-03 | Recompute and repeated operation generation. | No source-placement mutation, cascading recompute, cumulative transform, or stale cached shape. |
| IDX-04 | Engrave/Deburr edges alongside face/solid-based operations and stock. | Arc recovery does not discard required faces/solids or invalidate element references. |
| IDX-05 | Supported surfacing versus unsupported operation/dressup/REST combinations. | Correct results for declared support; visible rejection elsewhere. |
| IDX-06 | Two differently oriented operations, then return to Z-up. | Full safe transition planned from actual previous state; explicit rotary reset remains correct. |
| IDX-07 | Reorder operations, change tool length, change work offset. | Transition regeneration; no inherited clearance assumption from old order/configuration. |
| IDX-08 | Fixture clear at endpoints but intersecting swept stock/table motion. | Unsafe index rejected or routed through a demonstrably clear region. |
| IDX-09 | Unknown starting pose, restart, and aborted/failed generation. | Entry requirements explicit; stale paths cannot be exported as current valid output. |

## 4. Pose data and simultaneous motion

| ID | Test / fixture | Required outcome |
| --- | --- | --- |
| DATA-01 | Save/reload, undo/redo, copy, and legacy document migration. | Frames, reference points, orientations, intent, and precision preserved. |
| DATA-02 | Array, mirror, rotation, placement, or dressup on pose paths. | Correctly transformed position and axis, or explicit unsupported-capability error. |
| DATA-03 | Change geometry, setup/workplane, tool, machine calibration, or tolerance. | All affected generated/display/verification caches invalidated. |
| DATA-04 | Machine-file serialization and old schema versions. | Defined migration and no silent reinterpretation of old coordinate semantics. |
| DATA-05 | Unknown pose schema, missing orientation metadata, or unsupported consumer. | Conservative error; no silent downgrade to XYZ machining. |
| MOT-01 | Continuous direction path crossing a 0/360-degree seam. | No accidental reversal/full revolution; winding follows controller contract. |
| MOT-02 | Near-vertical/singular direction sequence. | Stable free-axis choice, bounded replanning, or a clear singularity failure. |
| MOT-03 | Competing IK branches near rotary limits. | Continuous feasible branch, not pointwise shortest-angle jumps. |
| MOT-04 | Winding limit requiring an unwind. | Explicit noncutting safe transition; no unwind hidden inside a cut. |
| MOT-05 | Nonlinear kinematics with valid endpoint poses. | Between-endpoint tool error is bounded by the requested realization tolerance. |
| MOT-06 | Interior excursion missed by a single midpoint or sparse samples. | Refinement/bounding detects the violation or reports uncertainty and blocks verification. |
| MOT-07 | Intermediate linear/rotary limit crossing or large angular excursion. | Violation detected even when block endpoints are in limits. |
| MOT-08 | Pure orientation move at fixed tool tip; duplicate/zero-length sample. | Finite feasible duration or removal only when the complete pose is redundant. |
| MOT-09 | Rotary velocity/acceleration bottleneck with fast XYZ request. | Time scaling/replanning or explicit infeasibility, not an invalid feed declaration. |

## 5. Cutter geometry and collisions

| ID | Test / fixture | Required outcome |
| --- | --- | --- |
| CUT-01 | Ball nose on a plane with changing tilt at fixed ball center. | Tip changes according to `p = c - r*a`; cutter contact stays correct. |
| CUT-02 | Tilted plane, sphere/convex patch, and trimmed patch. | Correct contact, boundary handling, and pose continuity. |
| CUT-03 | Surface seam, reversed normals, and degenerate patch. | Consistent normals or explicit rejection, not uncontrolled tool flips. |
| CUT-04 | Lead/lag and side-tilt changes along a path. | Orientation changes do not violate contact, cutting hemisphere, or tolerance. |
| CUT-05 | Adjacent face/obstacle outside selected cutting surface. | Selection boundaries do not suppress full-part gouge/collision checks. |
| CUT-06 | Unsupported cutter shape or strategy parameter combination. | Clear capability error; no ball-nose formula applied to incompatible geometry. |
| COL-01 | Safe endpoints, unsafe swept holder/stock/fixture motion. | Continuous or conservatively bounded checks detect the collision. |
| COL-02 | Large rotary motion with nonintersecting pivots during indexing. | Full machine/workholding sweep is considered; Z clearance alone is insufficient. |
| COL-03 | Cutter engagement versus shank/holder/noncutting contact. | Intended cutting is distinguished from gouging and mechanical collision. |
| COL-04 | Mesh/interpolation/calibration uncertainty exceeds available margin. | Inconclusive/unsafe status, not false verified clearance. |
| COL-05 | Small allowable tilt adjustment avoids a collision. | Replanned contact, kinematics, timing, and full sweep all rechecked. |
| COL-06 | No solution found within a bounded orientation search. | Clear search-limit/failure report without claiming global infeasibility. |

## 6. Postprocessing and controller equivalence

| ID | Test / fixture | Required outcome |
| --- | --- | --- |
| POST-01 | Internal mm/s and radians at legacy/controller boundaries. | Verified conversion to target units, including mm/min and degrees where required. |
| POST-02 | TCP/controller-kinematic versus CAM-compensated variants of a reference path. | Independently reconstructed relative paths agree; compensation applied once. |
| POST-03 | Work offsets, tool changes, G43/tool-length semantics, and mode activation/cancellation. | Correct physical state and reference point across transitions. |
| POST-04 | G93 with repeated identical segment durations. | F is retained on every required feed block; `F = 60/dt_seconds`. |
| POST-05 | G93/G94 changes and metric/imperial output. | Inverse-time F is not length-scaled; conventional feed restored deliberately. |
| POST-06 | Tilted arcs/canned cycles unsupported by the selected mode. | Tolerance-bounded linearization/expansion or an explicit rejection. |
| POST-07 | Modal filtering, G0 collapsing, XYZ-unchanged rotary moves, and transition barriers. | Essential rotary motion, feeds, and safe ordering preserved. |
| POST-08 | Rotary wrapped/unwound/rezero policies. | Actual controller direction and offsets agree with planned motion, including boundary crossings. |
| POST-09 | Rapid interpolation, path blending, line precision, startup, shutdown, and restart. | Controller semantics included in tolerance and clearance assessment; unknown behavior blocks verification. |
| VERIFY-01 | Full indexed final program through independent interpretation. | Reconstructed relative poses, sequencing, and offsets match intent. |
| VERIFY-02 | Full simultaneous final program through independent interpretation. | Between-block realized path/contact/clearance match the validated plan. |
| VERIFY-03 | Inject wrong pivot/sign/tool length or double compensation. | Verifier fails despite a plausible visual toolpath. |
| VERIFY-04 | Remove a required G93 F word or a rotary-only move. | Program rejected or mismatch detected. |
| VERIFY-05 | Unsupported controller code/state in final output. | Explicit unverified/error state; unknown commands are not silently ignored. |

## 7. Simulation, stock, and completion

| ID | Test / fixture | Required outcome |
| --- | --- | --- |
| SIM-01 | Known pose in workpiece and machine frames. | Correct tool axis/reference and consistent frame labeling. |
| SIM-02 | Physical A/C hierarchy with offsets and separate moving components. | Playback matches independent transforms, not fixed Euler/common-center assumptions. |
| SIM-03 | Reference LinuxCNC/Vismach trajectory. | Component and tool-reference motion agree within the stated numerical/display tolerance. |
| SIM-04 | Existing three-axis simulator jobs. | No regression; preview-only versus verified modes remain explicit. |
| STOCK-01 | Multidirectional removal from known solids. | Remaining volume/shape agrees within the chosen representation's error budget. |
| STOCK-02 | Undercut and regions inaccessible to a single-direction height field. | Representation supports the case or rejects the claimed verification capability. |
| STOCK-03 | Cross-orientation REST and reordered operations. | Correct workpiece-frame stock state and complete invalidation when order changes. |
| STOCK-04 | Thin walls and features near stock-grid resolution. | Conservative error/uncertainty reporting; no false fine-feature accuracy claim. |
| HW-01 | Measured machine calibration and configured controller mode. | Pivots, signs, lengths, limits, uncertainty, and coordinated-axis capability documented. |
| HW-02 | Off-machine simulation followed by appropriate spindle-off, no-stock commissioning. | Observed motion agrees before any cutting trial is considered. |
| HW-03 | Conservative test coupon after applicable software/safety gates. | Measured results recorded against a declared tolerance and supported configuration. |
| PERF-01 | Representative long paths at increasing sample counts. | Measured time/memory scaling with explicit budgets; no unbounded object growth. |
| PERF-02 | Cancellation/recompute during generation and verification. | Responsive cancel; no partial/stale path promoted to verified output. |
| PERF-03 | Repeated generation with identical inputs and cache restore. | Deterministic equivalent motion and reproducible diagnostics within declared tolerances. |
| RELEASE-01 | Supported-scope matrix and negative cases on packaged builds. | Only independently tested combinations are advertised; unsupported cases fail visibly. |

## 8. Gate sign-off

| Gate | Required evidence before claiming completion |
| --- | --- |
| M0 — Baseline | ENV results, build log, current upstream reconciliation, and reviewed frame/unit/output decisions. |
| M1 — Kinematics | Relevant KIN cases, independent references, limit/offset/tool-length cases, and documented unsupported topologies. |
| M2 — Indexed | IDX cases, indexed collision/transition checks, applicable DATA/POST checks, and VERIFY-01. |
| M3 — Motion infrastructure | DATA and MOT cases, applicable feed/output checks; synthetic paths only do not establish machining readiness. |
| M4 — Restricted simultaneous | CUT/COL cases, complete applicable POST/VERIFY results, and SIM agreement for the reference machine. |
| M5 — Scoped release | STOCK, HW where hardware claims are made, PERF, and RELEASE evidence, plus documented recovery and limitations. |
| M6 — Expansion | New topology/tool/strategy/controller fixtures for every newly advertised combination; all earlier gates remain valid. |

No gate has been signed off by the planning commit. An actual release report must identify who reviewed the evidence and which machine/controller/software versions it covers.
