# OpenLoc Indoor/Outdoor Localization Architecture

The product target is continuous local LiDAR/IMU odometry with one global
alignment owner that can fuse ODOM motion, GNSS, map observations and optional
known-marker observations across indoor, outdoor and transition segments.
Source availability may change by environment; loss of global references must
not interrupt or re-anchor local ODOM. This is a target architecture, not a
claim that all source adapters are integrated or field-qualified; see
[`roadmap.md`](roadmap.md).

## Goal and boundary

L1 estimates continuous relative motion in a local `odom` frame from LiDAR
and IMU. A separate global-alignment estimator composes qualified or explicitly
diagnostic GNSS, map and optional known-marker observations with that ODOM
motion to estimate `T_map_odom`. The global output is formed as:

```text
T_map_body = T_map_odom * T_odom_body
```

No global source may initialize, correct, reset, or re-anchor the L1 `odom`
state. Outdoors, GNSS can provide horizontal global reference when its datum,
quality, time and antenna lever arm are valid. Indoors or during GNSS loss,
qualified map or known-marker observations may anchor/recover the global
alignment; without an available global observation, continuous ODOM remains
the usable relative output while global status degrades. Map, marker and GNSS
observations share one selected global owner; they are not fused by averaging
independent backend outputs. Source dependency and uncertainty must be
represented or left explicitly diagnostic.

Map matching and markers are optional global aids, not prerequisites for local
ODOM or for an outdoor GNSS-only operating mode. A map/marker reference and
its transform to the selected global frame must be explicit and versioned;
unknown marker identity/pose or an unverified map-to-GNSS transform cannot
bootstrap a trusted global pose. Online map building is outside this scope.

Map-assisted scan-to-map localization is one optional global observation path;
it never changes local-ODOM ownership. Its architecture, dataset limitations,
and staged implementation plan are maintained in the
[map-assisted localization plan](../../../ad_agents/context/modules/localization/knowledge/map-assisted-localization-validation-plan.md).

## Runtime layers and ownership

### Selected graph fusion paradigm

The target extension uses a tightly coupled local ESKF/LIO, one global
fixed-lag factor graph, and independent health supervision. The local estimator
alone owns continuous pose, velocity, biases, epoch, and history. The graph
owns global poses and an internal local-error nuisance chain, not a second
local estimator. It exports `T_map_odom`, with
`T_map_base = T_map_odom * T_odom_base`; no absolute factor feeds back into ODOM.
The existing four-DoF baseline and SE(3) CI diagnostic are alternative owners,
not additional outputs to fuse into the selected graph. They remain solely
for explicitly selected replay/ablation comparisons; they are not compatibility
wrappers around the graph or runtime fallbacks. Graph and diagnostic callers
use the same canonical observation value types without legacy aliases.

The current fast integration baseline is still explicitly diagnostic:
`DiagnosticGlobalAlignment` has real-data map/GNSS replay evidence, while
`FixedLagGlobalAlignment` is the target graph and has functional software
regressions but not yet equivalent multi-capture real-data acceptance. A known
marker is now adapted by `BuildMarkerObservation` as a global map-pose
observation with surveyed identity, explicit time/frames, extrinsics and
18D joint input covariance. It composes
`T_map_marker * inverse(T_sensor_marker) * inverse(T_base_sensor)` and retains
ambiguity for owner rejection. Detection/association and real marker
calibration remain host responsibilities; repeated survey/extrinsic errors
are shared across observations, not qualified independent noise.

The framework has three responsibilities: algorithms own local/global state;
diagnostics expose decisions, innovation, health and freshness without owning
a replacement estimate; offline evaluation selects scenarios and assertions,
checks evidence and produces reports. The
[delivery plan](../../../ad_agents/context/modules/localization/knowledge/openloc-indoor-outdoor-delivery-plan.md)
defines the executable indoor/outdoor scene list and the remaining real-data
and deployment gates. Synthetic functional scenes do not qualify frontends.

`FixedLagGlobalAlignment` is a PIMPL value-type entry over the existing
`GlobalAlignment` fixed-lag backend. Its `GraphOdometrySample` supplies the
full 15D covariance in the local error convention, and `GraphMotionConnection`
supplies `Cov(e_previous, e_current)` plus actual source sample identities.
Both endpoint integer timestamps, sequences and epoch must match the accepted
ODOM snapshots; the adapter cannot silently reassign a skipped motion increment.
The existing `ComputeMotionIncrement` performs joint covariance checks and
the backend builds its conditional error chain and constrained relative-pose
coupling. It does not add an independent ODOM Gaussian edge on top.
A positive joint covariance or a single cross-covariance does not establish
the required conditional/Markov model: that model must be qualified externally.

Fixed-lag marginalization preserves a Gaussian prior in its original reference
chart. If frozen separator linearizations stall active nonlinear hard
constraints, the retained window is relinearized using that same prior's
`A`, `b` and noise, with explicit Pose3 chart derivatives. Eliminated factors
are not reconstructed or counted again. This bounded relinearization remains
subject to the existing iteration, feasibility and publication budgets;
complete Bayes-tree delta updates avoid wildfire-truncated feasibility checks.
Evaluation records the retained relinearization count. This solves a numerical
failure boundary, not the native front-end's statistical qualification.

`retain_graph_on_full_pose_staleness` is an explicit default-off lifecycle
policy: an established graph may keep accepting partial constraints after its
full-pose evidence expires. Its full-pose freshness and trust still degrade;
partial updates cannot renew the lease. Explicit recovery, invalid/missing
motion and epoch changes still revoke the graph. This does not make GNSS XY
alone a complete SE(3) cold-start anchor or qualify joint directional coverage.

Calibration is a separate admission axis: the explicit, default-off
`allow_uncalibrated_inputs` mode permits uncalibrated covariance and diagnostic
geodetic references, but does not waive sufficient-model, quality, source
identity or dependency checks. It selects the same graph, not another solver or
a CI fallback. Its public result is always diagnostic-only and its assessment
cannot report trusted availability. Exact conditional null directions remain
constraints even when roundoff makes their covariance LLT appear positive.
Posterior covariance is propagated through the existing Bayes tree's triangular
conditionals, with zero noise for exact equations; it does not re-eliminate
constrained cliques or add artificial process noise.

An optional `JointLocalError` retains a producer's sufficient augmented error
basis instead of forcing all memory into 15D. Its full-rank covariance and
15-by-dimension output Jacobian must reproduce the published 15D marginal.
The connection then supplies the entire rectangular cross-time covariance;
its projection must reproduce the published cross-covariance. No hidden block
is discarded. Dimensions may grow or shrink across retained nodes, including
entry/exit from the legacy 15D path, but the producer must preserve all future
dependencies. The configured per-node bound defaults to 256; exceeding it is
an explicit rejection before decomposition, not truncation. Singular memory
must use a minimal full-rank basis, not artificial diagonal noise.

Map full/projected pose and antenna XY/XYZ inputs share one correction stream.
XY residuals are `(p_map_base + R_map_base * lever_base).head<2>() - z_xy`,
using the entire 2x2 noise matrix. Unmeasured Z/attitude entries are not checked
as observations, inserted as priors, or padded into a six-dimensional factor.
Partial factors do not initialize/recover a full pose or renew its lease.
Full-pose bootstrap/recovery retains the owner's distinct, consistent
multi-frame verification and ambiguity handling.
Graph initialization commits the correction revision only after the entire
candidate graph succeeds. Verification and failed bootstrap cannot publish a
candidate mean/covariance as a committed correction. ODOM graph maintenance
can legitimately advance the revision before recovery begins; recovery
decisions are evaluated against their own pre-submit snapshot, not the last
absolute-observation revision.

The graph facade requires a verified reference and explicit external motion,
source uncertainty and independence qualification declarations/evidence IDs.
These declarations are admission prerequisites, not library-generated proof.
Unconditional independence must cover both local motion and other absolute
sources, including map construction and GNSS preprocessing. Alternatively,
`GraphLocalObservationCorrelation` supplies `C = Cov(e, n)` at the exact
retained joint error node, with `n = measured - true` in the measured subspace.
The graph derives `B = C^T P^-1`, `R_cond = R - B C`, and jointly factors
`h(x) - z + B e` against that conditional covariance. Innovation gating uses
the current joint pose/error marginal, including cross blocks, rather than
adding independent pose and error variances. Invalid/nonpositive conditional
residual noise is rejected transactionally; it is not regularized.

This mode requires a matching motion model ID, exact ODOM sequence, explicit
conditional model qualification and evidence that the retained error state
mediates **all** local/temporal/source reuse. Conditional residuals must be
independent of the entire error history and previous/other source residuals.
The unconditional independence flags cannot also be set. This is an external
sufficient-state claim, not something inferred from `C`, positive covariance
or provenance flags. Current native stored scans/deskew/output memory do not
satisfy it; diagnostic map/ODOM/GNSS remain ineligible. Shared sources without
this model remain rejected. The backend retains the augmented state through
its conditional chain, pose coupling, joint innovation and marginalization.
The actual native sufficient geometry/output-memory model remains open;
supporting its representation is not evidence that the producer exports it.

The native entry's on-demand deterministic exports do not override this gate.
`OutputErrorTransition` retains the actual published step's internal/output
inputs and three correction-speed memory dimensions; its analytic Jacobian
uses the actual limiter branches. `DeskewErrorMap` composes point/end-pose and
raw-point derivatives with actual history interpolation, retaining corrected
left and pre-correction right clones instead of treating them as independent
samples. `StoredGeometryErrorMap` preserves frozen scan anchors and every
retained plane's original point support, mean/normal and anchor/deskew loading.
Repeated source/clone identities are shared. Undefined voxel, eigenvector,
limiter or rotation branches reject an error model without changing nominal
ODOM/geometry. A caller must still provide all history/geometry/output cross
terms; none of these derivative maps creates a stochastic covariance,
qualifies uncertainty, or permits current native LIO graph admission.

The opt-in native stochastic observer now composes those maps into one
first-order shared covariance model. Its exact sampled midpoint propagation
retains original IMU endpoint errors across neighboring/wheel-split intervals;
`native_imu_sample_period` explicitly declares the noise PSD-to-sample
conversion. Stationary mean/current-sample correlation, frozen actual LIO
gain/subspace/whitening, stored raw-point support, and 18D output memory share
the same bounded minimal basis. Outstanding asynchronous tasks pin their
original sufficient basis and subsequent primitive drivers until commit or
discard. A delayed commit joins candidate internal/history/geometry memory to
the actual live output without replacing published snapshots, even after
ordinary history retirement. Capacity overflow is rejection, not truncation.

`JointErrorLookup` and `JointErrorMotionBetween` expose this coherent model
separately from unchanged nominal diagnostic marginals. They do not qualify
calibrated uncertainty or source independence. Actual `NativeMotionBootstrap`
retains bounded raw registration/deskew support when the observer is enabled.
Frozen actual ICP gains, source/target voxel membership and normal derivatives
are propagated through the relative-pose chain, gyro interpolation and causal
translation deskew. Numerical sensitivities of the unchanged initializer finish
the startup model; initial-state/frontier-IMU covariance preserves reuse by the
next propagation. The owner's declared residual initial uncertainty remains a
separate uncalibrated assumption. Caller-supplied time/transform-only startup,
nonunique branches, missing or retired support remain explicitly unmodeled.
This does not qualify execution deadlines or actual map-source independence.

`PrepareJointLidarScan` retains the actual pre-correction deskewed source points
and their loadings in this same basis. Once per frontier, after serialized
commits and before graph export, it finalizes the augmented error snapshot
without changing the nominal ODOM or its published marginal. Stored scans and
their dependencies retire together. This is not permission to revise a node
already consumed by the graph.

The opt-in `ScanToMapRequest::joint_scan` path composes actual frozen ICP
source/seed derivatives with that native basis. It requires complete seed
support, a full observed pose and a deterministic versioned map assumption.
Range/voxel selection retains original source-point identity; shared raw/deskew
errors are not duplicated as independent pose noise. Its covariance and cross
blocks are separate from the unchanged residual-Hessian diagnostics. It does
not model map construction/survey uncertainty or qualify calibration.

Conditional motion and source noise may be positive semidefinite. A normalized spectral
decomposition represents zero modes as exact error-chain equations and keeps
positive modes stochastic, without artificial priors or epsilon process noise.
Fixed-lag marginalization and observed-subspace factors preserve those
constraints. Cross-time sources connect the original measurement pose to their
explicit retained conditioning-error node. Delayed cold acquisition assembles
the needed trajectory and anchor in one graph update, without an unanchored
intermediate solver. Conditional updates re-eliminate the same retained factors
pose-first instead of recombining constrained Bayes separators; this preserves
rank without adding noise, another solver or a fallback.

The opt-in Apollo consumer transports one bounded, immutable joint snapshot,
its actual motion cross-covariance/source IDs and optional shared scan after
the frontier's serialized LIO commits. The nominal ODOM wire is unchanged.
Explicit configuration selects exactly one diagnostic or target-graph owner,
including across alignment reset and consumer ACK barriers. Shared map ICP
uses the transported actual points/loadings and a fully modeled local seed;
it cannot re-decode an independent raw cloud or use an unmodeled posterior
seed. The deterministic-map assumption and external GNSS source declarations
remain explicit. The cross-process regression covers fully delayed actual raw
scans, same-graph map/GGA updates and unchanged local ODOM, not live throughput,
two-capture target-graph acceptance or calibrated accuracy.

This entry is synchronous and has no hidden queue: the host schedules delayed
observations after their exact retained ODOM node exists; neither graph
bootstrap nor subsequent updates interpolate nodes. Nodes predating the
verified bootstrap anchor are unavailable even if local history retains them.
The private backend still uses double-second keys. The facade retains bounded
original integer ODOM timestamps and rejects a measurement whose rounded key
matches a different integer epoch; it does not create sub-key graph resolution.
The host retains returned statuses and provenance for audit. `GetDecisions`
also supplies a bounded journal, including early adapter rejects and the exact
qualification declarations, with monotonic decision IDs across local epochs.
Results carry source/aggregate counts and journal evictions. Rejected or
retained-unanchored observations do not count as alignment updates.
ODOM freshness and full-pose evidence freshness are reported separately; partial
factor admission does not renew the latter. Missing motion
revokes global alignment while retaining continuous local input for recovery.
`GetResult` separates retrieval status from `assessment`; a retained estimate
may be stale or degraded. Its covariance belongs to the raw backend alignment,
not a smoothed/published pose. No trusted capability, publication smoothing,
TF emission, or process lifecycle is supplied by this entry.

The graph/adapter and diagnostic facade have targeted compile/unit-test
evidence, not production qualification. Production publication/health
integration and real-data statistical qualification remain separate work.

The point-factor Jacobian follows the body-right `Pose3` tangent used by
[GTSAM GPSFactorArm](https://github.com/borglab/gtsam/blob/4d2d8b5e8ac7a1cb5be09b1442e3ff703ebbafe9/gtsam/navigation/GPSFactor.cpp):
`[-R * skew(lever), R]`; XY selects the first two rows and retains the
complete measurement covariance. Full/projected pose factors retain their
existing left-map rotation residual and numerical derivative.
[LIO-SAM's addGPSFactor](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp)
illustrates single-graph source admission, but its disabled-elevation branch
substitutes current estimated height and still adds a 3D factor. That behavior
is intentionally not used here: absent height remains absent information.

### ODOM/GNSS baseline ownership

```text
LiDAR + IMU
    |
    v
L1 local LiDAR-inertial odometry
    continuous T_odom_body, covariance, diagnostics
    |
    +-------------------------+
                              |
GNSS/RTK -> input validation  |
    -> antenna/time/ENU model|
    -> quality and innovation gate
                              v
                  global alignment estimator
                  T_enu_odom, covariance, status
                              |
                              v
                  T_enu_body = T_enu_odom * T_odom_body
```

The L1 estimator exclusively owns `T_odom_body`; GNSS cannot feed back into
the local odometry state. The global estimator owns `T_enu_odom`, its
uncertainty, and GNSS acceptance/rejection status. In the current online API,
`map_frame`/`map_from_odom` denote the configured global frame and can represent
local ENU; they do not imply a prior point-cloud map. Global alignment is
invalid until an absolute GNSS position is accepted. A horizontal-only position
initializes XY, not height; `vertical_position_initialized` tracks an accepted
XYZ constraint separately. These flags express initialization history, not
current accuracy, integrity or full-pose observability. A heading-only observation
does not establish global position. During outages, validity remains set while
uncertainty grows. Runtime adapters own bag
decoding, sensor clocks, GNSS receiver protocol, datum conversion, sensor
calibration delivery, and host lifecycle.

GNSS is timestamped at its measurement epoch. The estimator must associate it
with the historical ODOM pose at that epoch, account for the antenna-to-body
lever arm, and define how delayed/out-of-order measurements update and
propagate global alignment. It must not apply a stale fix as if it were
measured at the current time.

## Input and frame contracts

- Preserve sensor and measurement timestamps, clock domain, frame, units,
  source identity, calibration revision, and quality/status.
- Verify LiDAR point layout and per-point timing, IMU semantics, axes, units,
  extrinsics, and LiDAR/IMU synchronization before selecting or configuring
  the L1 implementation.
- Represent transforms as `T_parent_child`, including direction and
  calibration provenance.
- GNSS inputs retain geodetic datum, antenna/reference point, solution type,
  covariance/quality, fix age, and source/processing provenance.
- `MapReference` binds horizontal CRS realization, decimal-year coordinate
  epoch, vertical datum, and the geodetic ENU origin. A verified reference
  requires all declarations and valid origin coordinates. An explicitly
  `diagnostic_only` reference may omit unresolved metadata; that state is
  propagated with the global result and is not production-admission evidence.
- Convert GNSS geodetic coordinates to a declared local ENU frame with a
  documented origin and conversion convention. Never compare latitude and
  longitude directly with metric ODOM coordinates.
- Runtime GNSS does not require a point-cloud map. Any local ENU origin
  initialization or reset must be explicit and must not silently redefine
  `odom`.

  `LocalizationEngine::Configure` validates the reference declaration, while
  the host adapter remains responsible for proving that the declared CRS,
  coordinate epoch, vertical datum, and geodetic-to-ENU conversion match its
  input data. The `diagnostic_only` state is an explicit opt-in for bounded
  software/data diagnostics with unresolved references; it does not make those
  references verified and must not be used for production GNSS admission.

## L1 — Continuous local odometry

FAST-LIO2 is the requested baseline candidate, subject to an ADR comparing
upstream execution behind an adapter, package integration, and Openloc's
private `LocalEstimator`/`LocalLio`. Compare license, dependency closure,
sensor and point-time compatibility, IMU model, calibration, runtime cost,
and testability before choosing. The private candidate is not FAST-LIO2 and
is not a production default.

L1 outputs timestamped continuous `T_odom_body`, covariance, epoch/session,
and health/observability diagnostics. IMU propagation supplies high-rate
output; LiDAR updates at scan rate. Its local map is bounded and never becomes
a prior map. During degeneracy, do not let unsupported directions contaminate
the local estimate. Covariance is not a calibrated error bound without
independent evidence. `LocalEstimator` can optionally initialize tilt and
gyro bias from an IMU-only quiet window, but this is disabled by default and
does not establish zero translational velocity or wheel-qualified motion
confidence. Consumers must distinguish `EvaluateIntegrity()` from
`Evaluate()`; without trusted wheel evidence, the latter reports degraded
motion eligibility even when the state is initialized and finite.

`NativeOdometry` is the explicit public PIMPL boundary for that same native
owner, linked with `//:native_odometry`. Local value contracts live under
`include/openloc/localization/local/`; private candidates include those exact
definitions instead of maintaining a second configuration/state model.
No private estimator, PCL, GTSAM or middleware type appears in the facade
header. Raw IMU/wheel/timed LiDAR, relative-motion initialization, exact state
lookup, integrity/capability and diagnostic tracing delegate to the existing
owner. Opaque delayed LiDAR tasks retain its owner identity, epoch, complete
IMU catch-up and single-commit rules. The host serializes owner calls and joins
a snapshot worker before commit; the facade introduces neither a scheduler
nor another estimator. Optional `NativeMotionBootstrap`
(`//:native_motion_bootstrap`) now exposes the existing relative startup
registration behind a PIMPL, without private/PCL/middleware types in its public
header. It consumes the same ordered calibrated IMU and previous raw scan,
then submits only relative poses to `NativeOdometry::InitializeMotion`;
no supplied map, absolute seed or second local owner is introduced.
The host still selects startup policy and feeds scans at the actual local
IMU frontier; asynchronous decode/worker policy and publication remain outside
the algorithm. Neither facade starts threads or automatically runs a sensor
application.
The native replay and delayed worker use this entry. Native LIO still revokes
joint-error qualification; the boundary does not manufacture graph motion,
accept absolute corrections, publish TF or establish production readiness.

Apollo's opt-in host consumer is outside this library, under
`modules/openloc_localization/`. Its IMU-only executable feeds this public
owner and a separate Cyber snapshot timer; diagnostic messages retain native
IMU-origin/local-ODOM semantics instead of masquerading as ENU/rear-axle
`LocalizationEstimate`. Diagnostic TF edges use epoch-specific frame names and
the original measurement time; stale samples carry no pose. This does not
move Cyber into OpenLoc. A second executable adds strict calibrated raw
point-cloud ingress, public relative startup and LIO in one bounded owner
worker. Three optimized recorded-data worker arms preserve exact native ODOM
and scan-frontier decisions; the recorded adapter bypasses Cyber sensor readers.
The host can now execute public LIO snapshots off-owner, retain accepted IMUs
and commit after exact catch-up without rewriting published poses; overdue
tasks are discarded. Startup geometry still shares the owner thread.
Three separate-process recorded Cyber runs also consume all source IMU/cloud
messages with explicit original receive-clock/replay labels and idempotent
admission acknowledgements. This is diagnostic delivery evidence, not a
qualification of open-loop live ingress or source latency.
A complete global application, independent startup scheduling, reset protocol and qualified 100 Hz output
are not implemented by this boundary.

The current wheel contract is a signed base-forward speed in m/s at the
estimator motion reference, with variance in (m/s)^2; sensor adapters own
encoder conversion, sign, frame and lever-arm calibration. Initialization
requires both the configured consecutive sample count and stationary dwell
(2 s by default). NHC and the three-axis zero-velocity update remain opt-in.
Enabling zero-velocity updates requires an explicit positive measurement
variance; it applies only after fresh wheel and IMU stationarity evidence and
preserves the local pose. These are algorithm contracts, not vehicle-specific
slip detection or accuracy qualification.

## GNSS input and global alignment

The global estimator consumes timestamped L1 poses and quality-qualified GNSS
positions. Its state is the transform `T_enu_odom`, with declared tangent
convention, covariance, and epoch. Its process uncertainty reflects L1 motion
and accumulated drift. Each GNSS observation is compared to the antenna
position predicted from the ODOM pose at the GNSS measurement time and the
calibrated lever arm.

The current four-DoF alignment parameterization has an important frame
precondition: the input `odom` axes must already be gravity-level, so the
remaining global rotation can be represented by yaw about `odom` +Z. If the
frontend publishes an arbitrary `camera_init` orientation, a fixed
`T_odom_level_camera_init` rotation must be explicitly supplied and applied by
the host adapter before the 4DoF estimator. The estimator must not silently
assume that `camera_init` +Z is vertical. Until that frame contract is
established, ENU horizontal gates and yaw interpretation are not valid for
the raw frontend frame; keep the data result diagnostic-only.

`LocalizationConfig::odom_gravity_contract` makes this assumption explicit:
`kGravityAligned` is a caller assertion backed by frame evidence, while
`kDiagnosticAssumption` keeps unresolved frame semantics labelled and
non-production. The engine rejects an unspecified contract and does not infer
the ODOM axes from body roll/pitch or apply a leveling transform. Synthetic
roll/pitch body poses are compatible with a gravity-aligned ODOM frame; the
frame contract concerns the parent axes, not whether the vehicle is level.
`LocalizationResult::observability` reports whether accepted absolute
measurements directly constrained horizontal position, height, or heading; it
is not a numerical rank or accuracy claim. `calibration_state` reports only
the configured geodetic-reference declaration, not sensor calibration or
covariance qualification. `LocalizationResult::odom_gravity_contract` carries
the caller's gravity declaration so consumers can identify diagnostic frame
assumptions without separately inspecting engine configuration.

Before fusion, validate GNSS fix type/status, covariance, age, datum, antenna
reference, time alignment, and physically plausible innovation. Rejected or
unavailable fixes do not alter alignment. Use only the declared observable
components of the GNSS measurement; GNSS position alone does not directly
observe orientation while stationary. If a receiver supplies course or
heading, its semantics and uncertainty require separate verification.

Online responsibilities are separated at the algorithm boundary:
`GnssQualityGate` owns common receiver screening and independent fixed-solution
dwell for position and heading; `LocalizationEngine` owns input/frame checks,
sensor-time ODOM association, factor admission, and result publication;
`AlignmentSmoother` owns the alignment estimate; and `GnssAlignmentRecovery`
owns shadow re-acquisition. Host adapters remain responsible for decoding and
calibrating receiver inputs. Position and heading share one receiver-quality
policy without sharing their stability history.
The gate validates its own policy; a failed reconfiguration preserves the
previous policy and dwell histories. Its structured outcome distinguishes
accepted, waiting-for-stability, rejected, and invalid input. The engine uses
that decision rather than inspecting dwell internals or matching reason strings.
Position accuracy has one source of truth in `GnssReceiverQuality`.

`DiagnosticGlobalAlignment` is the public boundary for the selected diagnostic
SE(3) alternative, not another estimator backend. Its PIMPL owns exactly one
existing diagnostic `GlobalAlignment`; no PCL/GTSAM or private candidate
types appear in its public header. It accepts explicit diagnostic pose
covariance, admitted full/projected map observations and prepared horizontal
point observations. The host owns matcher/receiver admission and calibration.
Observation contexts preserve integer measurement/receive times, source
sequence, ODOM epoch and full map/reference identity; completed decisions and
measurement-time-sorted pending inputs are bounded. ODOM epoch changes reject
incompatible pending observations.

The public context keeps diagnostic provenance distinct from geometric/noise
validity: acquisition/preparation identities and separate tri-state
ODOM-initialization, ODOM-deskew, shared-local-scan and GNSS-map-construction
dependencies. Unknown remains unknown; caller-declared absence does not prove
statistical independence. No tag enables a qualified model or changes fusion
weight. Final decisions retain that context and covariance provenance across
queue completion, rejection and epoch retirement. The prepared replay adapter
supports an optional provenance suffix without reinterpreting old inputs;
excluded-source events retain their provenance as well.

Map ambiguity and explicit recovery are forwarded to that same owner.
Ambiguity clears candidate verification; recovery revokes the active basin
while waiting for distinct consistent full poses. A single match does not
complete recovery. The bounded decision journal exposes monotonic IDs,
correction revisions at each decision and whether the alignment changed;
retained unanchored XY is not an applied correction. Journal eviction is
reported rather than silently hiding audit loss.

`GetResult(evaluation_time, ...)` separates the latest ODOM sample epoch from
the freshness evaluation epoch, without extrapolating motion. ODOM age and
last successfully applied full-pose measurement age have separate positive
budgets. Partial evidence never renews the full-pose age. A raw basin may be
retained during outages while freshness is false; these diagnostic flags
neither grant a trusted lease nor qualify any covariance.

An estimate is absent until distinct consistent full-pose observations verify
an alignment basin. Partial position evidence is retained without inventing a
global rotation or identity map transform. This facade cannot enable the
qualified graph and exposes raw diagnostic estimates only, without TF
publishing, smoothing or trusted capability. Its runtime must not also select
the 4DoF engine as a competing global owner. Shared geodetic declaration
validation lives in `online/validation`, not duplicated engine callbacks.

`diagnostic_alignment_replay` exercises this public boundary using separate
ODOM, prepared full/projected map, and antenna-XY CSV streams. It merges
delivery by arrival time, has no oracle-seeding entry, logs excluded as well
as rejected/pending events, and emits final decision revisions once. It
does not own scan matching, source qualification, correction publication or
independent assessment. The old seeded three-source runner remains a frozen
comparison path. They share CSV scalar/matrix serialization, not estimation
logic. Prepared-observation replay cannot establish that an upstream map or
matcher was causal; real-data provenance remains a separate gate.

The decision journal can be read before the first ODOM sample. Replay therefore
logs early rejections immediately, not at the first subsequent result read.
Replay instrumentation separates arrival delay, ODOM-bracket queue wait and
steady-clock event processing time. Correction metrics compare two alignments
at the same current body, excluding motion and treating initialization/epoch
changes as unavailable comparisons. Resource/latency measurements remain
offline diagnostics; they do not establish product budgets. Every selected
observation must account for exactly one final updated/rejected/retained
decision or one unresolved pending entry before the summary is written.

The map adapter preserves the original integer measurement timestamp and
converts that measurement time, not the associated local sample time, to the
legacy backend's double-second representation. Its explicit
`MapObservationTiming` input supplies independent receipt time and source
sequence; both are mandatory and no overload borrows local arrival/sequence.
The configured timestamp-error budget is
an association/conversion allowance, not permission to retime measurements.
The global owner checks supplied integer/double consistency before admission.
History lookup and source frontiers remain double-second operations; this is
not a nanosecond-resolution backend. Full and partial geometry/covariance must
be structurally valid at the adapter, and a failure leaves output unchanged.

`GlobalObservation::quality_valid` is a caller admission flag, not proof of a
receiver-reported quality state or calibrated covariance. Trusted
`GlobalAlignment` requires both `LocalState::uncertainty_qualified` and
`GlobalObservation::uncertainty_qualified`, in addition to the local
covariance-model contract. Missing qualification returns
`UNCERTAINTY_UNQUALIFIED`; the explicit diagnostic mode may still consume
assumed/uncalibrated covariance, but its output remains untrusted. Covariance
still controls relative influence through the fusion equations; a fixed CI
coefficient is not a source-quality score.

Accepted corrections update `T_enu_odom` only. ODOM stays continuous through
GNSS outages, rejected fixes, global alignment resets, and recovery. Expose
GNSS rejection reasons and outage/recovery state rather than silently
accepting low-quality data.

During GNSS recovery, distinguish actual bad receiver quality from waiting for
stable-fixed qualification after an otherwise good report gap. Both remain
ordinary measurement rejects. Actual bad quality, invalid inputs, time/frame
or motion failures clear the shadow recovery history. Stability waiting alone
preserves previously qualified history but contributes no training or
validation observation. The next qualified innovation reject still enforces
the candidate's entire 30 s age and subsequent 8-training/3-validation
observation contract. This prevents repeated short gaps from starving
recovery without accepting unqualified fixes or reusing expired evidence.
Publication stopped-reset confirmation is cleared even during stability
waiting; no reset confirmation is borrowed across a gap.

### Backend and publication boundary

`GnssHorizontalFix` carries antenna XY with a 2x2 covariance. It shares the
XYZ path's receiver checks, exact-time ODOM association, position admission
stream, selected-state factor gating and global owner. No synthetic height
is inserted; XY preserves the unmeasured Z mean/uncertainty. The horizontal
gate defaults to the 2D 99% chi-square threshold, separately from XYZ.
Neither representation carries a new independent measurement when it is
derived from the same receiver epoch; shared timestamp ordering rejects
duplicate XY/XYZ submissions.

The backend owns the statistical estimate; publication owns correction
limiting only. GNSS re-acquisition increments `alignment_revision` and replaces
only the global backend. It must not directly replace the published alignment
or emit a published reset. Ordinary position, heading and recovery all pass
through one publication policy using the latest ODOM body position:

```text
delta_p = (t_backend + R_backend * p_odom_body)
        - (t_published + R_published * p_odom_body)
```

Under the default `kHoldForStoppedReset`, small corrections publish directly
under existing bounds. Moderate corrections
limit this position delta and yaw delta using elapsed latest-ODOM publication
time, then reconstruct alignment translation around that body point. Large
position deltas remain held until consistent observations at distinct
publication epochs confirm them while stopped. Moving and rejected
observations clear confirmation; ODOM-only updates do not count. A confirmed
publication discontinuity increments `reset_counter` and sets the sticky
`reset_event`. Initial publication is explicit and retains its reset behavior;
backend revision starts over on explicit engine Reset.

The explicit diagnostic `kRateLimitedTracking` policy instead rate-limits
every post-initialization correction, including large offsets, at the current
body position. ODOM ticks advance publication toward the cached backend only
while the last accepted position measurement is within
`maximum_fixed_solution_gap_ns` of current ODOM sensor time. A rejected report
cannot renew that age. Stale ticks consume time without moving publication;
an ODOM gap is accounted for before processing queued fixes, so a newly
accepted fix cannot spend stale time as correction credit. Multiple updates
at one ODOM epoch cannot spend the same budget twice; zero budget preserves
the published parameters exactly. First publication still has explicit
initialization/reset semantics. Tracking does not modify ODOM or backend
estimation and never emits a post-initialization large reset.

The result exposes held/rate-limited/aligned publication state and the current
body position/yaw offset, recomputed as ODOM advances. `alignment_covariance`
remains the backend marginal. While `published_mean_matches_backend` is false,
it must not be presented as a covariance about the held/smoothed published mean.
The offset is deterministic, not an independent noise covariance or calibrated
error bound. Downstream reset handling, source integrity and calibrated
published uncertainty still require host/product qualification. A held global
correction does not inhibit continuous local ODOM.
`correction_source_fresh` reports accepted measurement-time recency;
`global_output_degraded` is set before position initialization, while
publication differs from the backend, or while that evidence is stale.
Neither a fresh source nor a cleared degraded flag certifies receiver integrity,
full-pose observability, calibrated covariance or physical accuracy.
If ODOM-only graph maintenance changes the backend after a previously aligned
publication, `kAwaitingPolicyUpdate` reports the new pending difference. It does
not apply another correction or count stopped confirmation without an accepted
absolute observation. This differs from an already rate-limited or held output.

The public engine fixes an internal reference `r = [first_odom_x,
first_odom_y, 0]` for each Configure/Reset cycle. GNSS antenna factors and
recovery candidates use `p_odom - r`, while the ODOM buffer remains unchanged.
Initial translation/yaw sigmas and diagonal process noise are defined at that
reference point, not the arbitrary caller ODOM origin. Heading axes and Z
origin do not change. Position-only initialization still does not establish
yaw observability.

For internal `[t_ref, yaw]`, returned alignment translation is
`t_raw = t_ref - R(yaw)*r`. The full covariance is `P_raw = J P_ref J^T`,
where `J` is identity with yaw-column translation derivative
`-R'(yaw)*r`. This retains translation/yaw cross terms; simply converting the
mean or diagonal is insufficient. The marginal still belongs only to the
backend mean. Translation-only XY origin controls verify composed physical
outputs, measurement NIS and publication decisions with equivalent
reference-point priors. They do not establish arbitrary rotated-frame or
different physical-reference/process-model invariance.

## Explicit diagnostic three-source alternative

`three_source_dataset_replay` selects the existing private conservative SE(3)
`GlobalAlignment` as its **only** global owner. It does not run the public
four-DoF smoother or qualified error-chain graph alongside it, and is not a
production adapter or a change to their defaults. SE(3) here avoids inventing
physical-up qualification for the supplied maps.

`GlobalConfig::diagnostic_only` allows a declared numerical local covariance
while retaining `LocalState::covariance_model_valid=false` in history. Numerical
positive-definiteness, identity, ordering and time guards still apply. This
mode cannot enable the qualified graph, marks `GlobalState::diagnostic_only`,
and `Evaluate` cannot return trusted output. `SeedDiagnostic` accepts one
explicit full alignment prior at the latest local epoch before observations;
it cannot reseed an active owner. The dataset adapter uses the same one-time
offline-informed prior in every ablation, not per-frame optimized poses.
In this mode a stale map full-pose lease does not revoke that diagnostic basin
and turn subsequent GNSS updates into unanchored evidence. Stale/integrity
qualification remains unavailable; the default non-diagnostic lease and
verification rules are unchanged.

`HORIZONTAL_POSITION` measures only antenna XY, with the declared body lever
and the top-left 2x2 measurement covariance. It never inserts a height or
heading measurement, cannot initialize/recover a full pose, and is explicitly
supported by the fixed-lag graph as a two-dimensional antenna factor, subject
to independent graph admission. The conservative owner gates innovations
and combines evidence using its existing fixed-weight covariance intersection
(0.5) for full poses. Diagnostic partial updates discount only the measured
prior information and retain its conditional null-space prior:
`I_new = I_prior - 0.5 H^T (H P H^T)^-1 H + 0.5 H^T R^-1 H`.
This avoids doubling a free height variance at every XY report. Retaining that
conditional model is a diagnostic assumption, **not** a qualified joint
unknown-correlation bound. The non-diagnostic generalized-CI update is unchanged.
GNSS covariance, local uncertainty and map geometric covariance determine
relative influence without asserting independence or calibrated uncertainty.
The three-source replay explicitly leaves uncertainty qualification false:
GNSS uses prepared sigma, ODOM uses distance-based diagnostic variances, and
map covariance is derived from registration geometry/residuals with configured
floors. Their differences can exercise covariance-sensitive fusion, but cannot
establish an operational signal-quality baseline until those uncertainty
models are independently calibrated and the receiver/map quality inputs are
qualified.

The opt-in `ScanToMapConfig::separate_ndt_support` is a **different diagnostic
admission model**, not a logging-only change. The unchanged legacy default
requires planar-supported overlap. The alternative requires distance coverage
above `minimum_overlap`, at least `minimum_points` valid planar correspondences,
and the existing projected-information rank/condition checks. It retains the
legacy planar overlap and all support counters in output; it does not fabricate
normals for nonplanar neighborhoods or promote unobserved directions. Solver,
fitness and correction-basin gates remain. This option forces
`ScanToMapResult::diagnostic_only` even with qualified map metadata.

ICP and scan-to-map uncertainty use the same pose tangent contract:
translation-first with left rotation about the current pose origin. ICP's
iteration update, final information matrix, and projected observation must
share this ordering and reference point. When `T_map_cloud` is converted to
`T_map_base`, transform the full covariance and observed projection/noise
through the cloud-to-base tangent Jacobian; copying cloud uncertainty to the
base pose loses lever-arm variance and cross terms. Regression tests cover a
translated map origin and a nonzero translated/rotated extrinsic. These tests
verify the software transformation only; matcher uncertainty remains
uncalibrated.

The replay consumes accepted `ScanToMapResult` pose, full covariance or observed
projection/covariance directly in memory at the exact ODOM event, not reconstructed
from score-only CSVs. Rejected results never become observations. Each next
matcher seed comes from that arm's current owner composed with immutable ODOM;
no global correction feeds back into ODOM. Outputs retain all events and source
decisions, full backend covariance, map subspace/noise, same-epoch correction
sizes and explicit diagnostic/untrusted labels. Direct backend composition is
an offline output, **not** the public engine's held/rate-limited publication.

`AdaptScanToMapObservation` is the typed boundary for that path. It accepts only
a converged, unambiguous match whose map ID/version/content, map/base frames and
extrinsic calibration ID match the replay contract; it also requires the match
timestamp to agree with the local snapshot within the declared nanosecond
tolerance and clock domain. It preserves full-pose versus projected-pose
covariance, but does not claim local independence: map registration was seeded
from ODOM. Calibration-qualified covariance is propagated as such; uncalibrated
registration covariance stays diagnostic-only. The dataset replay uses Unix
event timestamps for both local snapshots and scan matches to enforce that
association rather than using a relative replay clock.
`GlobalConfig` may additionally pin the expected map content ID and map/base
frames; configured owners reject direct observations that do not match those
identities, even if an adapter was bypassed. The observation retains the
matcher score/covariance source, calibration flag, diagnostic-only status,
physical-up qualification, and map-reference independence. A diagnostic match
is not admitted to a non-diagnostic owner even if a caller asserts calibrated
covariance; local independence is separately false because the registration
was initialized from ODOM.

### Three-source quality-model limits

The current recorded-data runner is a diagnostic replay, not a measured
receiver-quality or causal-arrival path. It supplies prepared GNSS sigma and
marks the diagnostic observation quality valid; it has no receiver arrival
timestamp/status chain. Its sparse ODOM interpolation can use a future
bracketing keyframe, and same-source post-update GNSS residuals are in-sample
consistency scores, not predictive or independent accuracy.

The diagnostic partial-observation update retains a conditional prior in
unmeasured directions. This is not a joint covariance-intersection guarantee
under arbitrary unknown cross-correlation; a strict-positive-definite linear
counterexample violates that stronger claim. Keep this path diagnostic-only
and do not infer statistical consistency from positive definiteness or
repeatability. Global alignment process drift rates are now defined at the
first ODOM reference pose and transformed into the alignment-origin tangent;
a translated-origin regression verifies covariance equivariance. Their rate
magnitudes remain uncalibrated assumptions. Map physical-up/gravity alignment
is not qualified by these captures. Do not claim qualified attitude or trusted
3D fusion until independent frame, quality, arrival, and held-out evidence is
available.

## Output contract target

The output carries:

- measurement timestamp and clock domain;
- continuous `T_odom_body`, covariance, epoch, and L1 status;
- `T_enu_odom`, covariance, and global-alignment validity/status;
- `T_enu_body` and covariance when global alignment is valid;
- GNSS fix type/quality/age, accepted/rejected status and reason;
- GNSS-to-body lever-arm/calibration identity and ENU-origin identity;
- per-source map/marker identity, reference, admission and correction status;
- measurement age and end-to-end latency.

Consumers use ODOM for continuous relative motion. Consumers of ENU position
must check global validity, active source quality, covariance, and reference
identity. Loss of GNSS/map/marker support degrades global validity; it does not
invalidate a healthy L1 ODOM estimate.

## Package and runtime ownership

Openloc owns middleware-neutral value contracts, estimator algorithms, and
health policy. Host/runtime adapters own sensor, GNSS, map and marker decoding,
clock synchronization, calibration delivery, process lifecycle, transport,
and output publication. The online library has no dependency on Apollo/Cyber
or ROS messages. A general map/marker database service is outside this
package's owner boundary.

## Evidence boundary

Software tests and LIO/GNSS agreement do not by themselves establish absolute
accuracy if the GNSS stream is also used by the estimator or is not
independently qualified. Accuracy and calibrated uncertainty claims require
independent truth, a declared evaluation protocol, and confidence bounds.
Record input provenance, configuration, rejected fixes, initialization,
metrics, commands, and artifacts; preserve inconclusive and failed results.
