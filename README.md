# openloc

Middleware-independent localization algorithms and value contracts. Host
repositories own sensor adapters, middleware, process lifecycle, and actuation.

See [`docs/README.md`](docs/README.md) for documentation navigation,
[`docs/principles.md`](docs/principles.md) for functional goals,
[`docs/architecture.md`](docs/architecture.md) for library contracts, and
[`docs/roadmap.md`](docs/roadmap.md) for implementation status and evidence.

**Current submission baseline:** [2026-10-28 diagnostic functional baseline](docs/baseline-20261028.md)
summarizes the indoor/outdoor goal, algorithms/diagnostics/evaluation split,
14-scenario field correspondence, known blockers, performance limits and
reproducible checks. It is not a production or real-time acceptance claim.

The package layout and ownership boundaries are defined in
[`docs/architecture.md`](docs/architecture.md). The online baseline and
extracted global-alignment candidate remain separate; neither is declared a
production default.

The explicit native local entry is `NativeOdometry`, from
`openloc/localization/local/native_odometry.h`, linked with
`//:native_odometry`. Its PIMPL owns exactly one existing `LocalEstimator`;
no Cyber, ROS, PCL or GTSAM header is needed by a local-only consumer.
Local value/configuration and motion-initialization records have one public
definition; private candidates reuse it without changing fields or defaults.
The entry accepts raw calibrated IMU, signed wheel speed and timed LiDAR,
plus explicitly relative poses for motion initialization. Optional
`NativeMotionBootstrap`, from
`openloc/localization/local/native_motion_bootstrap.h` and
`//:native_motion_bootstrap`, provides the existing relative raw-scan startup
algorithm behind another PIMPL. It uses only received IMU and previous raw
scans, never a supplied map or absolute pose, and submits through that same
local owner. Its public header also needs no PCL or middleware types.
The host still owns decoding, calibration, startup policy, exact-frontier
feeding, scheduling and publication; neither interface is an autonomous
raw-sensor application or a production-default frontend.
Opaque LiDAR snapshot tasks preserve the original owner identity, exact
catch-up frontier, epoch and single-commit checks. `EvaluateIntegrity()` is
not `Evaluate()`: finite propagation does not qualify capability, uncertainty
or graph admission. The interface has no absolute-pose or global-reset input
and does not expose an invented qualified LIO motion factor.
Exactly zero process noise uses `LocalState::covariance_square_root` to retain
the marginal as `P = L L^T` instead of repeatedly rounding an ill-conditioned
covariance. Existing factors survive wheel, zero-velocity, LiDAR and continuous
output updates using orthogonal factor reduction; ordinary positive-noise
propagation retains its existing dense path. The factor and rounded dense view
must agree, and unresolved factor rank still rejects. This is not a sufficient
cross-time/geometry model or a precision qualification. Dense-inverse motion
and graph consumers still reject covariance they cannot resolve; they do not
silently obtain temporal sufficiency from this marginal factor.
`MotionBetween()` now exports the owner's actual joint motion at exact retained
endpoints, including source identities and cross-covariance, without private
headers. It does not qualify uncertainty: unmodeled LIO/startup intervals
remain rejected and failed requests do not overwrite a caller's output.
`OutputErrorTransition()` exports the actual last published step's analytic
18D output/correction-velocity mapping and frozen internal/output inputs.
`DeskewErrorMap()` exposes actual retained history clones, including
pre-correction interpolation endpoints, shared point/end dependencies and
raw-point loading. Both are on-demand deterministic maps, not joint stochastic
models. They neither admit a scan nor authorize graph admission.
`StoredGeometryErrorMap()` retains actual frozen scan anchors and exposes
every plane's original point/sequence support, mean/normal derivatives and
anchor/deskew loading. Scan retirement removes the corresponding anchors;
voxel boundaries and nonunique normals reject an error model without changing
the actual geometry. These exports still require a sufficient joint
covariance/source model, not independent per-point or per-pose noise.

An opt-in stochastic observer (`LocalConfig::enable_native_joint_error`) now
propagates a bounded shared-error basis alongside that unchanged owner.
`JointErrorLookup()` and `JointErrorMotionBetween()` expose its coherent
marginals, output projection and rectangular cross-time covariance; they must
not be mixed with the original diagnostic covariance in `state()`/`Lookup()`.
It uses the exact sampled midpoint IMU mechanization derivatives and actual wheel gains,
shared history/deskew clones, independent raw-point driving errors, actual
stored-plane support and the analytic 18D output transition. LIO iterations
reuse the same prior/point/plane errors with their actual **frozen**
gain, whitening, subspace and correspondence decisions. Geometry retains its
original stochastic loading after the contributing history leaves the window.
No diagonal correction inflation is imported into this model as fake noise.
Memory rows, minimal stochastic dimension and exported snapshot count are
separately bounded; dependency/rank overflow rejects the observer without
dropping a physical dependency or changing nominal ODOM.

The IMU model requires explicit `native_imu_sample_period`: raw acceleration/
gyro sample variances are the configured continuous-time PSDs divided by that
period. Each original sample error is shared by both adjacent trapezoids and
all wheel-split interpolation intervals. Bias random-walk driving variables
act at each actual subinterval end. Stationary initialization propagates the
actual frozen mean's tilt/gyro-bias dependence and its correlation with the
retained last raw sample, alongside the owner's declared initial residual
uncertainty. Nominal IMU F/Q and diagnostic covariance remain untouched.

Delayed tasks pin the original stochastic fork and each subsequent original
primitive driver in both branches. Commit reconstructs the full shared joint
distribution, retains the LIVE output/correction-speed memory and all actually
published snapshots, and adopts only the candidate's internal/history/geometry
memory. Ordinary history can retire before commit without severing the bridge;
pins retire on task commit/discard. They consume real configured capacity.

With the observer enabled, actual `NativeMotionBootstrap` also retains the
frozen ICP iterations, raw source/target voxel membership, normal support and
deskew/gyro inputs. Reverse propagation through composed poses and causal
translation deskew combines those loadings with numerical sensitivities of the
actual initializer. Initial state and the original frontier IMU sample are
jointly retained for the next trapezoid, not assigned independent pose noise.
Synchronous/asynchronous LiDAR-rest and excited moving starts preserve the
original nominal result. Nonunique voxel/normal branches and retired or
over-capacity support explicitly reject the observer, without replacing nominal
startup. Caller-supplied time/transform-only initialization remains unmodeled.
This is an opt-in, uncalibrated first-order model, **not qualified E4 acceptance**.
Actual shared map-source export and explicit host target-graph selection are
implemented below; live deadlines and empirical calibration remain unqualified.
Native diagnostic flags and uncertainty qualification remain unchanged.

`native_lio_replay` and its delayed worker now consume this public entry;
three corrected local windows preserve the previous 6,000/6,000/5,999 ODOM
rows exactly except timing. Its paced third-capture map arm retains 141
global corrections with exact local-only isolation.
Evidence: `.agents/notes/native_timer_output_validation_20261023_v1/public_facade/audit.json`.
This removes the missing local API prerequisite, not the remaining Apollo
dependency/publication or native graph error-model work.

The extracted startup preserves all three local trajectories and startup/scan
decisions exactly except measured processing time. Its paced third-capture
map/timer arm establishes alignment with 141 accepted matches and 139
corrections, exact local isolation and 3,002 ticks with no whole-period misses.
Asynchronous global counts are not promised to be identical across runs.
Evidence: `.agents/notes/native_timer_output_validation_20261023_v1/public_startup/audit.json`.
Nonblocking decoder backlog can still change local reference/commit scheduling;
this extraction does not fix that separate source-frontier gap.

An independent Apollo consumer now exists at
[`modules/openloc_localization`](../../modules/openloc_localization/README.md).
It exercises the public local owner, a real diagnostic Cyber ODOM/TF publisher,
an explicit-time-offset IMU-only host, and a bounded raw IMU/LiDAR host using
public relative bootstrap and LIO. Source/held/stale/epoch contracts and
cross-process raw IMU/point-cloud transport are covered. Its optimized worker
consumes three recorded captures with exact frozen ODOM/frontier parity
(6,000/6,000/5,999 states, 300 scan decisions each). Recorded replay bypasses
Cyber sensor readers; it is not recorded middleware/deadline acceptance.
It publishes native IMU-origin
ODOM with untrusted status, not ENU/rear-axle `LocalizationEstimate` or production
TF. The host now also uses the public isolated `LidarTask` with exact IMU
catch-up and explicit deadline discard. All three prepared recordings have
been consumed through separate Cyber processes with original source clocks,
actual diagnostic subscriptions, admission acknowledgements and explicit
`recorded_input` labels. Eighteen cases cover this boundary. Admission-confirmed
replay is not open-loop runtime qualification: actual late tasks and sender
lateness remain visible. The independent global host, isolated startup
geometry and diagnostic reset acknowledgement now exist; actual connected
raw-native/GGA/map composition is recorded in the roadmap. Complete ingress
accounting, target native graph error model, production consumers and
accuracy/deadline acceptance remain open.

The target graph entry is `FixedLagGlobalAlignment`, from
`openloc/localization/global/fixed_lag_alignment.h`, linked with
`//:fixed_lag_global_alignment`. It wraps the existing fixed-lag graph owner,
not another solver: qualified 15D local error snapshots/cross-time covariance
feed its motion chain; map full/projected poses and antenna XY/XYZ feed the
same global graph. Exact retained ODOM nodes are required. It does not queue
observations, interpolate a graph node, feed corrections into ODOM, or publish
TF. The host owns scheduling, source qualification, and publication policy.
`GraphMotionConnection` names both endpoint integer times, sequences and epoch;
a stale/skipped connection is rejected instead of silently being reassigned.
Results expose separate ODOM/full-pose freshness and backend assessment.
`GetDecisions` retains a bounded journal of adapter/owner outcomes, provenance,
qualification declarations and actual correction revisions, including before
the first ODOM sample. Aggregate/source counters and dropped-decision counts
make bounded retention explicit; an OK unanchored position is retained evidence,
not a global correction.
Both entries use `GlobalMapObservation`, `GlobalHorizontalObservation` and
`ObservationDependency` directly; no diagnostic-named type aliases or adapter
compatibility overloads are retained. Diagnostic admission does not qualify
graph admission. The graph is the target implementation; the separate 4DoF
and conservative CI entries remain only as explicitly selected comparison
baselines with existing replay runners, never as implicit fallback backends.
Their output covariances and startup requirements are not interchangeable.
`DiagnosticAlignmentConfig::use_fixed_gain_update` selects an explicit
conservative comparison instead of the legacy covariance-intersection mean.
Full-pose updates use half the alignment residual; XY/XYZ updates change only
the measured translation mean, not orientation through covariance cross terms.
The first-order covariance bound retains all cross terms and assumes no
independence: `2[(I-KH)P(I-KH)^T + KRK^T]`. It is not a calibrated uncertainty
claim, optimal fusion, a FixedLag substitute or a new trusted default.
`native_lio_replay --diagnostic-fixed-gain` selects this policy explicitly and
writes `global_mean_policy.csv`; without the flag, the existing CI path is
unchanged. Source admission, ODOM isolation and full-pose freshness remain
separate requirements.
`FixedLagAlignmentConfig::allow_uncalibrated_inputs` explicitly admits declared
sufficient but uncalibrated models into this same graph. Its default is false.
Quality, stochastic-model validity, source dependencies and conditional
independence evidence remain mandatory. Results are marked `diagnostic_only`
and never receive a trusted assessment. A maintained raw-native regression
composes actual IMU/LiDAR joint snapshots and motion with independent synthetic
map/GNSS observations. A separate raw three-plane regression also runs actual
shared-scan ICP into this owner, including fully delayed cold acquisition,
cross-time source/error binding and fixed-lag retirement.
The graph also accepts explicitly qualified conditional observation/local-error
cross-covariance and singular conditional motion/source noise, retaining
shared-error coupling and exact deterministic directions. This does not
authorize legacy diagnostic covariances or qualify statistical calibration.
The same graph can now retain an optional bounded augmented error basis and
its 15D output projection, rectangular cross-time covariance and source loading.
Hidden shared memory survives dimension changes and fixed-lag marginalization;
inconsistent projections or negative hidden conditional noise are rejected.
`NativeOdometry::PrepareJointLidarScan()` finalizes a frontier once, after its
serialized LiDAR commits and before graph export. It retains the actual
pre-correction deskewed points and their same-driver loadings, without changing
ODOM or its published marginal. `ScanToMapRequest::joint_scan` requests the
actual frozen-ICP model against a deterministic versioned map; a complete seed
loading is mandatory. That model is separate from residual-Hessian diagnostics,
does not add independent source-point noise, and does not model map survey/
construction uncertainty. Delayed scans retain their measurement time while
`GraphLocalObservationCorrelation::conditioning_time` identifies the actual
retained error node. The opt-in
[`Cyber host`](../../modules/openloc_localization/README.md#opt-in-single-target-graph-owner)
now transports coherent bounded joint/motion/shared-scan packets and explicitly
selects this sole graph owner. An actual all-delayed raw-native cross-process
map/GGA regression covers the diagnostic functional boundary, not dataset,
live-deadline or accuracy acceptance.

The graph/adapter and diagnostic API regressions now build and pass in the
managed development container. See the [roadmap](docs/roadmap.md) for exact
validation scope; this is not a production-readiness or recorded-data claim.

### Indoor/outdoor algorithms, diagnostics and evaluation

The [delivery plan and scenario checklist](../../ad_agents/context/modules/localization/knowledge/openloc-indoor-outdoor-delivery-plan.md)
separate online algorithms, existing decision/health diagnostics and offline
acceptance. The executable matrix covers indoor map/marker without GNSS,
projected-map/GNSS complementary degradation, joint loss/reentry, bad receiver
quality, outliers, ambiguous markers, delayed observations and unanchored cold
start. These are synthetic observation-boundary checks, not raw LiDAR,
detector, calibration or field-accuracy acceptance.

`BuildMarkerObservation`, from
`openloc/localization/global/marker_observation.h` and
`//:marker_observation`, composes
`T_map_marker * inverse(T_sensor_marker) * inverse(T_base_sensor)` into the
existing `GlobalMapObservation`; either global owner can consume that value.
Its full 18D joint covariance uses one `[translation_parent, left_rotation_parent]`
block per input pose and preserves correlations. Marker ID, reference,
content, calibration, frame and clock mismatches reject without modifying the
output. Ambiguity is preserved for owner rejection, not silently resolved.
It is not a detector or qualification claim: repeated survey/extrinsic errors
need an explicit temporal/shared model before graph independence is declared.

`FixedLagAlignmentConfig::retain_graph_on_full_pose_staleness` is default-off.
When explicitly enabled, an established graph can continue partial-source
updates after the full-pose lease expires; freshness and trust still degrade.
It does not permit position-only SE(3) cold startup, renew full-pose evidence,
relax observation gates or bypass explicit recovery/model/epoch revocation.
Host profiles and default behavior are unchanged.

```bash
env -u LD_LIBRARY_PATH bazel test -c opt //:marker_observation_test \
  //:fixed_lag_global_alignment_test
env -u LD_LIBRARY_PATH bazel build -c opt //:localization_scenarios
env -u LD_LIBRARY_PATH python3 -m unittest discover \
  -s test/integration/map -p 'evaluate_localization_scenarios_test.py'
env -u LD_LIBRARY_PATH python3 test/integration/map/evaluate_localization_scenarios.py \
  --binary bazel-bin/localization_scenarios --output "$NEW_OUTPUT"
```

Run in the mapped-user managed container. Output must be new; it includes
source/runtime pins, per-source terminal decisions, pre-update residual/NIS,
freshness/trust, JSON and HTML. The current graph matrix has 23 functional
scenarios, 1,026 state rows and 1,165 recomputed NIS samples. The original S01-S15
remain functional regressions; W01-W03 add 1.5/3/15 s absolute-source loss
around a 3 s window, and R01/R00 compare a 15 s rejected GNSS burst with its
omitted-source control. Their poses and full alignment covariances match.
Three long map-loss cases retain GNSS XY for 30/60/120 s, then require actual
map commits and renewed full-pose freshness. Public API regressions separately
check analytically derived covariance-bound growth and recovery after 120 s of
accepted 0.1 m GNSS bias. These are not general accepted-fault robustness proofs.
Public full-pose freshness now preserves the committed integer measurement
time, including the lease's -1/0/+1 ns boundary, without partial-source renewal.
S15 freezes field-derived sparse map/GNSS availability, not real
measurement noise or front-end geometry.
GNSS XY-only cold start deliberately reports **no full pose**, not zero error
or a successful localization result; refusal of unobservable startup is correct
behavior, not itself a functional failure. The
[current synthetic report](.agents/notes/synthetic_production_gate_20261029_v2/scenes/index.html)
also includes known local drift during joint absolute-source loss: increasing
error is accepted when degradation, ODOM isolation and source recovery match
the contract. Coverage records observed node intervals, dimensions and delays,
not a guarantee over untested ranges or real field data. Real-record FixedLag and robust
indoor/outdoor product acceptance remain open.

The [field-state report](.agents/notes/field_state_matrix_20261028_v1/final/index.html)
contains four captures and 16 source-ablation replay arms. The ChengTou 180 s
diagnostic combination has a **990.55 m** peak paired GNSS discrepancy
(same-source consistency, not independent truth). D01 separately exercises
`DiagnosticGlobalAlignment` with correlated positive-definite bounds and a
small rotation residual: an accepted, fresh correction can amplify translation.
Its passing characterization regression does **not** establish acceptable
accuracy. The actual large correction is reconstructed from logged covariance
and residual to about 1e-11 m; it is not explained by normal joint-source loss.
No online gains, gates or covariance policy were changed.

## Build and tests

### Committable diagnostic functional baseline

The ChengTou first-30-second baseline selects `DiagnosticGlobalAlignment`,
not the qualified target graph. Its four arms share raw IMU/LiDAR, calibration,
local configuration and startup settings. The explicit treatment adds only
`--diagnostic-level-odom-gnss-bootstrap` to GNSS-bearing arms: level-axis
XY/yaw acquisition is not a full-pose GNSS solution. Corrections and finite
states are functional evidence, not calibrated accuracy, trusted availability,
production readiness or live-rate acceptance. In particular,
`CORRECTION_BUDGET_EXCEEDED` output must not be relabeled as usable.

Use the maintained runner below, not a session-local driver. From this package
directory, supply mounted-workspace paths `RAW_INPUT`, `GNSS_INPUT`, `MAP_ROOT`,
fresh `BASELINE`/`CANDIDATE` output roots and the managed test `CONTAINER`.
Supply independently recorded manifest/metadata pins, the seven `SEED_*`
components and `SEED_DESCRIPTION` identifying the offline, potentially
future-informed map hypothesis. Preserve the raw `config.csv` and extrinsics;
the seed is global-only. Build `//:native_lio_replay` with `-c opt` in the
managed container first. The runner verifies manifest-listed files and records
the executable/loader identity; verify the raw manifest pin separately.

```bash
printf '%s  %s\n' "$RAW_MANIFEST_SHA256" "$RAW_INPUT/manifest.json" | sha256sum --check
common=(--container "$CONTAINER" --binary bazel-bin/native_lio_replay
  --input "$RAW_INPUT" --global-owner diagnostic-ci
  --motion-bootstrap --motion-bootstrap-voxel-m 0.2 --imu-trace)
gnss=(--gnss-input "$GNSS_INPUT" --gnss-manifest-sha256 "$GNSS_MANIFEST_SHA256")
map=(--map-root "$MAP_ROOT" --map-metadata-sha256 "$MAP_METADATA_SHA256"
  --seed "$SEED_X" "$SEED_Y" "$SEED_Z" "$SEED_QX" "$SEED_QY" "$SEED_QZ" "$SEED_QW"
  --seed-description "$SEED_DESCRIPTION")
runner=test/integration/map/run_native_map_diagnostic.py
python3 "$runner" "${common[@]}" --arm odom --output "$BASELINE/odom"
isolation=(--compare-odometry "$BASELINE/odom/replay/odometry.csv")
python3 "$runner" "${common[@]}" "${isolation[@]}" "${gnss[@]}" \
  --global-holdback-ns 100000000 --arm gnss --output "$BASELINE/gnss"
python3 "$runner" "${common[@]}" "${isolation[@]}" "${map[@]}" \
  --global-holdback-ns 100000000 --arm map --output "$BASELINE/map"
python3 "$runner" "${common[@]}" "${isolation[@]}" "${map[@]}" "${gnss[@]}" \
  --global-holdback-ns 100000000 --arm both --output "$BASELINE/both"
python3 "$runner" "${common[@]}" "${isolation[@]}" "${gnss[@]}" \
  --global-holdback-ns 100000000 --diagnostic-level-odom-gnss-bootstrap \
  --arm gnss --output "$CANDIDATE/gnss"
python3 "$runner" "${common[@]}" "${isolation[@]}" "${map[@]}" "${gnss[@]}" \
  --global-holdback-ns 100000000 --diagnostic-level-odom-gnss-bootstrap \
  --arm both --output "$CANDIDATE/both"
```

Use the same explicit seed epoch/tolerance on both map-bearing arms when that
epoch is known. Do not mix pacing, asynchronous workers or changed local flags
into this deterministic source ablation. Runner ODOM comparison alone is not
the acceptance gate: retain all scan-frontier decisions, original GGA arrivals,
final owner outcomes and actual map/GNSS correction revisions. Reject-only map
execution and retained-but-unanchored GNSS positions are not fusion passes.

For the retained immutable ChengTou v1/v2 evidence, regenerate the separate
report-only comparison from this package directory:

```bash
python3 test/integration/map/compare_native_lio_baselines.py \
  --baseline .agents/notes/chengtou_functional_baseline_20261027_v1 \
  --candidate .agents/notes/chengtou_functional_baseline_20261027_v2 \
  --gnss-input .agents/notes/chengtou_native_fusion_20261025_v1/gnss_30s \
  --output "$NEW_REPORT_OUTPUT"
```

Set `NEW_REPORT_OUTPUT` to a new directory under `.agents/notes/`; do not
overwrite a completed or failed report. This frozen-evidence comparison requires
the original aggregate `report.json`, `contract.json` and their referenced
replay/input files, obtained separately from the evidence owner; they are not
included in a source commit. New runner outputs above have their own per-arm
reports, not that frozen aggregate schema. Source-time antenna XY consistency
against reused GNSS/map evidence is not independent truth or full-SE(3)
ATE/RPE. The unchanged combined-arm trajectory must not be presented as a
fusion accuracy improvement.

Submit source, adjacent tests, BUILD declarations and required patches in the
independent `openloc` repository; submit the optional Cyber consumer separately
in Apollo-Lite. Never use a workspace-wide `git add .`. Keep raw bags, clouds,
maps, replay CSVs, HTML/images, runtime logs and frozen contracts outside Git
under ignored `.agents/notes/` or an external evidence store. Preserve their
hashes and access instructions separately. A commit does not contain ignored
evidence or validate registry publication; package lockfile/toolchain review
and exact multi-repository revisions remain delivery prerequisites.

### Functional replay versus Apollo runtime replacement

The reusable workspace procedure is
[`openloc-fusion-ablation`](../../.agents/skills/openloc-fusion-ablation/SKILL.md).
The report's `fusion_diagnostics` separates source admission from actual
corrections, deduplicates pending/final journal entries per observation, and
reports original-source-clock startup/global-initialization delays, local
capability reasons and map rejection/rank evidence.
`metrics.gnss.event_diagnostics` accounts for every arrival: gate/adapter
failure, native ODOM unavailable, unanchored position retention, owner
rejection or committed correction. Empty owner messages for accepted position
evidence no longer hide the absence of a global anchor. These are diagnostic
findings, not calibrated health or wall-clock timing claims.

End-to-end diagnostic replay and production localization are separate
acceptance levels. A functional replay must consume raw IMU/LiDAR, generate
native local ODOM, actually match a supplied prior map, and produce a
single-owner global result with retained rejection evidence. A successful
local-only replay does not satisfy that requirement.

The current Apollo RTK and NDT hosts publish localization/TF from their GPS
input callbacks. They are not OpenLoc hosts and do not establish an indoor
IMU-driven publication path. Diagnostic replay output must not be connected
to their production topics as a replacement. Native joint-motion uncertainty,
absolute-source qualification, independent health, host scheduling and output
publication remain separate requirements; an explicitly diagnostic global
owner cannot be relabeled as the qualified fixed-lag graph.

The explicitly selected map-assisted functional entry is
`test/integration/map/run_native_map_diagnostic.py`. It consumes prepared raw
inputs and a pinned supplied map package. Its only global owner is
`DiagnosticGlobalAlignment`; GNSS is absent in the default map-only arm and
optional in the explicit `both`/`gnss` arms. Target-graph availability and global
trust remain false. The required map-side pose hypothesis applies to
the first eligible scan after native startup, not the beginning of the bag,
and never initializes or corrects local ODOM. This is seeded map matching,
not autonomous global relocalization.

From the package directory inside the managed container, after building
`//:native_lio_replay` and preparing raw inputs:

```bash
env -u LD_LIBRARY_PATH python3 test/integration/map/run_native_map_diagnostic.py \
  --input <raw-input-directory> --map-root <supplied-map-package> \
  --map-metadata-sha256 <pinned-metadata-sha256> \
  --seed <x> <y> <z> <qx> <qy> <qz> <qw> \
  --seed-description <explicit-hypothesis-provenance> \
  --seed-reference-ns <hypothesis-epoch-ns> --seed-maximum-offset-ns 1000000 \
  --global-owner diagnostic-ci --motion-bootstrap \
  --binary bazel-bin/native_lio_replay --output <new-output-directory>
```

Success requires accepted scan-to-map matches, committed global correction
decisions and initialized global output, in addition to native LIO updates.
Rejected-only map execution is not an end-to-end pass. Outputs retain local
ODOM, local scan decisions, map decisions, global decisions and global poses;
the JSON/HTML report explicitly retains source dependencies and limitations.
The default target uses the existing scan-to-map local window (15 m radius,
2 m cells), not the entire supplied map as overlap support. Explicit
`--map-window-radius-m` and `--map-window-cell-m` settings are bounded and
recorded; matcher overlap thresholds and geometry gates are unchanged.

### KITTI LiDAR-only diagnostic

`test/integration/map/kitti_lidar_odometry.cc` and
`run_kitti_lidar_odometry.py` provide a separate bounded scan-to-scan ICP
diagnostic for KITTI odometry data. It estimates relative motion from
consecutive Velodyne scans, anchors the reference trajectory at the first
scan, and uses reference poses only for scoring and the XY visualization.
KITTI lacks raw IMU, GNSS observations, and per-point timing in this local
package, so this experiment does not validate OpenLoc native LIO, sensor
fusion, deskew, mapping, or loop closure. Its short-window metrics are not the
official 100–800 m KITTI segment score.

The first recorded functional pass uses `20261012103814`, first 30 seconds:
113 native LIO updates, 73 accepted matches out of 114 map attempts, and
71 committed diagnostic global corrections. The output contains 2,344
initialized global rows (including observation-triggered rows, not unique
IMU epochs) and 6,000 health rows with no trusted capability. Evidence is at
`.agents/notes/native_map_e2e_20261023_v1/20261012103814_v2/report/`.
Its initial map-side hypothesis comes explicitly from an offline map keyframe,
using future/map-preparation information; this result does not establish
autonomous relocalization, GNSS fusion, target-graph qualification or Apollo
runtime replacement.
An ensuing input audit found exact-zero XYZ no-return sentinels retained by the
native preparer and consumed as geometry. These numbers are historical
execution evidence; corrected-input acceptance requires new prepared inputs
and replay, not relabeling the old outputs. The defect belongs to sensor
adaptation and does not prove that the raw bag or IMU is faulty.

Corrected-input native three-source replay is now available with `--arm both`,
`--gnss-input <prepared-gnss-directory>` and
`--gnss-manifest-sha256 <pinned-manifest-sha256>`. Prepare GNSS with
`test/integration/map/prepare_native_gnss.py --bag <GPS-bag> --nmea <RTK-bin>
--profile <sensor-profile> --metadata <map-metadata.json>
--metadata-sha256 <pin> --input <raw-input-directory> --sigma-m <explicit-value>
--gnss-map-construction present --output <new-gnss-directory>`.
The sigma is an uncalibrated diagnostic assumption, not receiver accuracy.
`--compare-odometry <local-only-odometry.csv>` verifies local isolation.
The other explicit arms are `odom`, `gnss` and `map`; horizontal-only GNSS
does not invent a full-pose anchor.

For no-map functional acquisition, explicitly add
`--diagnostic-level-odom-gnss-bootstrap` to the native binary or the wrapper's
`gnss`/`both` arm. It declares a **diagnostic assumption** of gravity-level
ODOM/global XY and reuses the existing eight-position fit plus three later
holdouts, unchanged baseline/residual gates and antenna lever arm. The single
diagnostic owner then tracks XY/yaw; it never sends an absolute input to local
ODOM. Stationary, insufficient or inconsistent windows remain uninitialized.
After an accepted-position outage exceeds the existing global timeout,
acquisition requires a new training/holdout window rather than an immediate
return to aligned status.

The public equivalent is `DiagnosticAlignmentConfig` with both
`enable_horizontal_bootstrap` and `assume_gravity_aligned_odom` set.
`DiagnosticAlignmentResult::horizontal_estimate` is separate from the full-pose
`estimate`: it supplies alignment XY/yaw, base XY, original observation time
and uncalibrated parameter covariance at the declared ODOM reference point.
It does not establish height, roll/pitch, full-pose freshness or trust.
Native `horizontal.csv` and JSON/HTML retain this partial result separately;
`global.csv` still reports no full pose until actual map evidence establishes
one. Defaults and the qualified fixed-lag graph contract are unchanged.
Two real first-30-second windows (`20261012105149`, `20261013170722`) establish
this partial alignment with 3/10 committed corrections, versus zero in default
GNSS-only arms. Six same-binary runs retain exact respective 6,000/6,001 ODOM
rows and 300 scan decisions except timing. Source/runtime pins and reports:
`.agents/notes/native_horizontal_acquisition_20261026_v1/audit.json`.
This is functional acquisition/tracking evidence, not independent accuracy,
target-graph qualification or the still-missing online global application.

The corrected first-30-second `20261012103814` result is 188 LIO updates,
117/189 map matches, and same-owner commits from 115 map observations and two
GNSS positions. All 29 GNSS events are retained: 23 float rejects, two dwell
waits, four gate passes; two passes commit and two are rejected as older than
the committed map frontier. This is an honest delayed-fusion limitation, not
a receiver-quality failure. Local-only/map-only/both have identical 6,000 ODOM
rows except measured timing. Evidence and comparison:
`.agents/notes/native_gnss_e2e_20261023_v1/20261012103814/comparison_report.json`;
the initialized trajectory is in `both_v2/report/index.html`.
This remains offline-seeded diagnostic CI with untrusted health and no
qualified target-graph or Apollo runtime replacement claim.

`--global-holdback-ns <0..500000000>` explicitly enables a bounded replay-host
reordering window for global observations; default zero preserves immediate
submission. Received map/GNSS observations wait for the processing-time watermark
and are submitted in measurement order, with stable arrival order for ties.
The queue is limited to 256 observations; exhaustion fails explicitly.
`global_admission.csv` records original measurement/arrival, enqueue and submit
processing times, selected holdback and final submission status. Only global
admission is delayed: native ODOM and sensor timestamps are unchanged. Future
measurements on arrival retain immediate owner validation and are outside the
causal reordering guarantee. Arrivals beyond the window can still be rejected;
this does not implement historical factor-graph refusion. At replay end, only
the processing clock advances by the selected holdback to drain received
observations; no sensor sample or arrival is invented.
The same-binary first-capture comparison at 100 ms retains 117/189 map matches
and 115 map commits while increasing committed GNSS positions from two to four;
all 6,000 local ODOM rows remain identical except timing. Maximum observed
queue residence is 104.617 ms, including discrete processing wakeups, not a
wall-clock deadline guarantee. Evidence:
`.agents/notes/native_gnss_e2e_20261023_v1/20261012103814/holdback_comparison_v1.json`
and `both_holdback_v1/report/index.html` in the same capture directory.

The no-GNSS `20261015185840` map arm also completes with explicit 0.2 m startup
resolution: 155/162 accepted map matches, 153 committed map corrections, and
all 5,999 ODOM rows identical to local-only except timing. Evidence:
`.agents/notes/native_corrected_startup_20261023/20261015185840/map_voxel_02_v1/report/index.html`.
It uses a pinned, epoch-matched offline same-capture map hypothesis with future
information, not causal relocalization or independent accuracy evidence.
Gate-accepted GNSS arriving before native ODOM exists is explicitly journaled
as `native_odometry_unavailable`, not submitted with an invented epoch and not
classified as bad receiver input.
The second capture's corrected native three-source arm accepts 74/135 map
matches and commits 71 map + 12 GNSS corrections. Its 29 GNSS events retain
two dwell waits and 27 gate passes: 14 native-not-ready exclusions, one
unanchored position and 12 committed updates. All 6,000 ODOM rows match
local-only except timing. Evidence:
`.agents/notes/native_corrected_startup_20261023/20261012105149/both_voxel_02_v2/report/index.html`.

The current-code design-goal validation re-executes three corrected 30-second
local-only runs, second-capture map+GNSS and GNSS-only, and third-capture
no-GNSS map, all with one pinned binary and explicit 0.2 m startup resolution.
Local functionality, numeric integrity, exact ODOM isolation, diagnostic source
integration and refusal of unqualified health pass in this bounded scope.
GNSS-only retains 29 arrivals without inventing a full-pose anchor or changing
ODOM. Output-envelope and synchronous 100 Hz feasibility goals fail; causal
global acquisition, native target-graph integration, Apollo publication and
independent accuracy/transition acceptance remain uncovered.
Frozen commands, input/source/binary hashes, reports and the goal matrix are in
`.agents/notes/native_design_goal_validation_20261023_v1/design_goal_report.json`
and `index.html`. Completing validation does not complete the production design.

The replay's `health.csv` runs the independent `LocalizationHealth` supervisor
on native numeric/timing outputs. Relative-state validity and diagnostic
global initialization are recorded separately from availability and trust.
Unqualified raw replay uncertainty does not acquire global, continuity or
trusted capabilities merely because map alignment initializes.
Health protobuf poses use the distinct generated type `HealthPose`, so health
and the public global `Pose` value type can coexist in one program. Existing
protobuf field numbers/layout are unchanged, but callers explicitly naming
the former generated health `Pose` type must use `HealthPose`; no conflicting
compatibility alias is retained.

Focused replay/health checks:

```bash
env -u LD_LIBRARY_PATH bazel test //:native_health_diagnostic_tests
env -u LD_LIBRARY_PATH python3 test/integration/map/native_lio_replay_test.py \
  bazel-bin/native_lio_replay
env -u LD_LIBRARY_PATH python3 test/integration/map/native_map_diagnostic_test.py \
  bazel-bin/native_lio_replay
```

### No-ground-truth evaluation

The diagnostic report now includes an `evaluation` audit of pinned CSVs,
source-event accounting, correction revisions, arrival/holdback ordering and
diagnostic trust ceilings. Map attempts and GNSS arrivals are counted separately
from submissions and actual corrections. Unresolved submitted events fail
accounting; missing epoch/consumer evidence stays explicitly unavailable.
This is not a complete target-graph factor ledger or covariance qualification.

Run the fast, dependency-free evaluation tests inside the mapped-user managed
test container:

```bash
env -u LD_LIBRARY_PATH python3 -m unittest discover \
  -s test/integration/map -p 'analyze_global_innovations_test.py'
env -u LD_LIBRARY_PATH python3 -m unittest discover \
  -s test/integration/map -p 'evaluate_native_fusion_test.py'
env -u LD_LIBRARY_PATH python3 test/integration/map/native_gnss_diagnostic_test.py \
  bazel-bin/native_lio_replay
```

Reanalyze an existing frozen diagnostic report into a **new** directory:

```bash
env -u LD_LIBRARY_PATH python3 test/integration/map/evaluate_native_fusion.py \
  --report "$RUN/report/report.json" --replay "$RUN/replay" \
  --baseline "$LOCAL_ONLY/replay" --repeat "$IDENTICAL_RUN/replay" \
  --output "$NEW_EVALUATION"
```

`--baseline` checks ODOM and scan isolation; `--repeat` checks deterministic
ODOM, scan, map, GNSS and global journals (only `imu_ms`/`processing_ms` are
excluded). These optional checks stay `not_run` if omitted. JSON and offline
HTML expose per-event outcomes, NIS tails, whitening/QQ, sample-lag
autocorrelation, innovation component scales and untested evidence.
Cross-source correlations require a common explicit `residual_basis` describing
frame, projection, component ordering and units; legacy journals without it
are not silently paired by dimension. Statistics are descriptive, not calibrated
coverage or accuracy; innovation scales are not posterior pose error bounds.
New journals also export the actual gate, covariance model and, where available,
the alignment-prior, observation and local-state contributions to innovation
covariance. The evaluator checks their sum and NIS/gate consistency; JSON/HTML
show each pre-update gate decision. Missing legacy fields remain unavailable.
A large residual can pass when modeled uncertainty is large; this is not
evidence that the observation is correct or the uncertainty calibrated.

Prepared-GNSS fault variants preserve original input pins and a per-event mask:

```bash
env -u LD_LIBRARY_PATH python3 test/integration/map/prepare_native_gnss.py inject \
  --input "$GNSS_INPUT" --output "$NEW_FAULT_INPUT" \
  --start-ns "$START_NS" --stop-ns "$STOP_NS" --shift-xy 50 -30
```

Use `--time-offset-ns` for measurement-clock faults or `--dropout` instead of
position/time shifts. Arrival times, sequence and quality gates remain unchanged;
the interval is half-open in original measurement time. These faults are at the
prepared GNSS boundary, not corrupted raw NMEA/checksum tests. Replaying a fault
may change later map associations; output differences measure response, not
known true localization error. Existing `--global-odom-drift`/`--global-odom-step`
alter only the global owner's ODOM copy, never native local state.

The test-only `//:fixed_lag_evaluation` target archives committed factors when
explicitly enabled before initialization. Its batch reference optimizes those
same factors without timestamps or eviction; it is not a second production
owner. Pose NEES uses the actual pose marginal, not the public alignment bound.

```bash
env -u LD_LIBRARY_PATH bazel build -c opt //:fixed_lag_evaluation
env -u LD_LIBRARY_PATH python3 -m unittest discover \
  -s test/integration/map -p 'evaluate_fixed_lag_test.py'
env -u LD_LIBRARY_PATH python3 test/integration/map/evaluate_fixed_lag.py \
  --binary bazel-bin/fixed_lag_evaluation --odometry "$LOCAL_ONLY/replay/odometry.csv" \
  --output "$NEW_EVALUATION" --trials 128 --nodes 12 --lag-seconds 60
```

Real replay poses become **fictional** nominal truth for independent, modeled
and unmodeled AR/shared-bias experiments. The 60 s setting is a deliberately
non-marginalizing control for a shorter nominal trajectory; use a shorter lag
to exercise actual eviction. The frozen gate/Huber limit is an evaluation
policy, not production tuning. Per-trial failures remain in `failures.csv`;
survivor NEES cannot establish consistency. A successful runner exit is not
acceptance. The 2026-10-28 repair retains each marginalized Gaussian's original
`A`, `b`, noise and reference chart, differentiates that chart on Pose3, and
relinearizes only the bounded retained nonlinear window when fixed separators
prevent hard-constraint convergence. It does not reconstruct eliminated
observations. Complete Bayes-tree delta updates also prevent wildfire
truncation from violating exact constraints. Lag, solve budget, maximum
iterations, source gates and the 1e-6 feasibility tolerance are unchanged.
`retained_relinearizations` makes this work visible in evaluation CSV/JSON.

The earlier matched 8-node/5 s and 12-node/60 s experiments each completed all 640
trials; the old failures remain frozen evidence. Regression includes both
previously failing no-eviction seeds and a turning/shared-source/dimension-change
case that failed before the repair. Correctly modeled semi-synthetic errors
show no detected mean-NEES inconsistency in these experiments; deliberately
unmodeled correlation remains inconsistent. Neither establishes calibration
or real-record FixedLag acceptance. See the
[current graph diagnostics](.agents/notes/goal_graph_20261028_v1/validation/index.html).

The current evaluator adds overconfident/conservative observation-covariance
arms to those five correlation models. It verifies the same sampled errors
with declared map/GNSS variance scaled by .25/1/4, using `generated.csv`;
local covariance is unchanged. The frozen 8-node, 128-seed, 5/60 s study has
1,792 successful runs, paired across models and windows, not 1,792 independent
samples. Mean NEES at 5 s is 6.1223 for matched independent noise, 19.4024 for
the overconfident declaration and 2.8338 for the conservative one. Lower-tail
conservatism is distinguished from overconfidence, not automatically a bug.
See the [statistical report](.agents/notes/window_covariance_matrix_20261029_v2/statistics_lag5/index.html)
and the canonical plan's 18-mode classification for executed and still-missing
coverage. These fixed-admission synthetic trials do not qualify production
robustness, native input capacity or field accuracy.

Use `--map-loss BEGIN_S END_S` with `evaluate_fixed_lag.py` to omit map
submissions in a source-relative half-open interval while keeping GNSS.
All map noises are still sampled/journaled, including omitted observations,
so changing availability does not change the paired random sequence.
`admission_schedule.json` independently checks every submitted source/node;
an interval that never triggers cannot pass.
The [long-loss study](.agents/notes/synthetic_production_gate_20261029_v2/verification.json)
uses 24 nodes over 170.35 s, 15 s lag, seven models and 128 paired seeds for
each of two schedules: no map return before the endpoint, and return after
120 s loss. Correctly modeled mean NEES lies inside the simultaneous six-test
reference interval; misspecified correlation/variance controls trigger.
This remains a fixed-admission synthetic study, not online fault detection or
production qualification. The diagnostic fixed-half exponential-bound failure,
native model/capacity, full fault-mode coverage and product budgets remain open.

For source exclusion, native replay accepts
`--global-map-holdout START_NS STOP_NS` with an explicit diagnostic map. It
journals each half-open interval exclusion before deskew, seed lookup or
matching, without excluding local LiDAR. Combine it with a prepared GNSS
`--dropout` variant for separate source holdouts. Raw preparation accepts up
to 180 s. Same-capture GNSS-built maps and informed seeds remain shared
information; held-out GNSS is only a weak discrepancy reference. If no map
match succeeds after reentry, the reentry-jump check is **not triggered**.

### Native raw IMU/LiDAR diagnostic entry

`LocalConfig::diagnostic_lidar_tracking_gain` is a **default-off diagnostic**
nominal tracking mode. Replay opts in with
`--diagnostic-lidar-tracking-gain` and records `local_gain_policy.csv`.
It keeps a private conditional gain matrix, propagates it with the existing
analytic IMU transition/process noise, and applies one fixed-prior Joseph
posterior per accepted scan, including the attitude reset. Delayed tasks
commit that private memory with their internal trajectory. Plane geometry is
treated as fixed for this nominal gain; its shared scan/history errors are
**not** thereby qualified. The original conservative published covariance,
residual gates, diagnostic ceiling and ODOM/global ownership remain unchanged.
Do not export this private matrix as pose uncertainty or use the flag to
bypass native joint-model / FixedLag admission.

Geometric observability uses point/plane geometric noise separately from
anchor marginal weighting: an increasing absolute anchor marginal must not
erase the rank of otherwise unchanged relative geometry. By default the
correction still uses the original conservative anchor variance. The explicit
diagnostic mode instead uses geometry-only noise for its nominal gain, not
for residual admission or published uncertainty.

The retained 180 s ChengTou experiment at
`.agents/notes/goal_continuity_20261027_v1/index.html` compares the original,
rank-only and explicit diagnostic-gain paths: accepted local scans increase
from 196 to 1,149 to 1,702 of 1,800; the diagnostic mode accepts all 1,500 scans
after 30 s. One owner commits 407 map and 126 GNSS corrections with exact local
isolation. This is functional recovery, not accuracy or calibrated covariance.
Map 30/60 s exclusion does reenter, but its first same-query global position
responses are 6.65/22.15 m; map 120 s exclusion does not reenter.
Three further captures cover local-only 30 s windows, not full-route or
indoor/outdoor acceptance. Original failures and default settings are retained.

Use `bazel build -c opt //:native_lio_replay` for deployment-oriented timing
diagnostics. The default unoptimized binary is suitable for contract checks,
not production latency estimates. A pinned GTSAM source patch explicitly
includes `<cassert>` in `SL4.cpp`, avoiding its optimized-build dependency on
an incidental transitive include; assertions and compiler flags are unchanged.
Observed optimized scan times do not establish asynchronous 100 Hz publication.

`native_lio_replay` runs OpenLoc's private `LocalEstimator`/`LocalLio` on raw
sensor inputs, not external FAST-LIO odometry. It is an incremental frontend
validation entry, **not** the completed map/global/health/Apollo replacement.
The supplied Mid-360 adapter ignores the invalid IMU orientation, converts
acceleration from g, uses the manual `T_I_L`, retains integer point offsets and
bag delivery timestamps, and applies no external pose or gravity seed.

From the host, prepare a new bounded input directory (requires the already
available `rosbags` and PyYAML environment):

```bash
python3 pkgs/openloc/test/integration/map/prepare_native_lio.py \
  --bag <raw-LIDAR.bag> --profile <sensor-profile.yaml> \
  --config pkgs/openloc/test/integration/map/native_lio_diagnostic_config.json \
  --duration 30 --output <new-input-directory>
```

Build and execute inside the managed container, as its mapped non-root user:

```bash
env -u LD_LIBRARY_PATH bazel build //:native_lio_replay
env -u LD_LIBRARY_PATH python3 test/integration/map/native_lio_replay_test.py \
  bazel-bin/native_lio_replay
env -u LD_LIBRARY_PATH bazel-bin/native_lio_replay \
  <input-directory> <new-output-directory>
```

An explicit `--motion-bootstrap` option additionally attempts local relative
scan registration and native IMU alignment while stationary initialization
is unavailable. Its target is the previous raw scan, **not** the supplied
prior map. It uses provisional gyro rotation-only deskew, point-to-plane
registration with existing geometry gates, and a bounded relative pose window.
The joint alignment solves velocities, gravity and accelerometer bias without
a zero-speed or bias prior; scaled data-Jacobian rank, residual, gravity and
bias checks reject unobservable or inconsistent windows. Gyro bias is fitted
against integrated IMU rotation increments; it does not assume constant angular
rate between scan poses. Bias-corrected IMU rotations also drive specific-force
integration within each pose interval. This alignment is a diagnostic startup
candidate, not a qualified raw-trajectory batch estimator: translation deskew
and relative-pose uncertainty are not modeled. No metric accuracy claim follows
from acceptance.

The explicit replay option `--motion-bootstrap-voxel-m <0.1..0.5>` selects
relative-registration resolution (default 0.5 m); it requires
`--motion-bootstrap`. A common 0.2 m first-30-second diagnostic completed native
LIO on three corrected captures without changing any acceptance gate:

| Capture | Initialization delay | Valid ODOM rows | LIO updates | Correction-budget-exceeded rows |
| --- | ---: | ---: | ---: | ---: |
| `20261012103814` | 9.441 s | 4,112 | 203 | 1,412 |
| `20261012105149` | 16.335 s | 2,733 | 134 | 0 |
| `20261015185840` | 13.681 s | 3,263 | 161 | 63 |

The replay records `CORRECTION_BUDGET_EXCEEDED` as degraded capability when
native integrity still passes; it must not abort as if this were a malformed
state, nor promote these rows to usable driving output. Integrity failures
such as IMU gaps remain fatal. These diagnostics are not a successful output
envelope or 100 Hz deadline qualification. Evidence:
`.agents/notes/native_corrected_startup_20261023/common_voxel_02_comparison_v1.json`.

`capability.csv` preserves degraded decisions with source/receive times,
integrity status, and measured translation/rotation/velocity discrepancies
against the declared limits; correction acceleration is the applied,
rate-limited value. JSON/HTML include the first and last envelope exceedance.
Paired replays isolate every first/third-capture exceedance to the internal
versus continuous-output translation gap: maxima 4.976 m and 2.128 m against
2 m. Rotation and velocity discrepancy limits were not exceeded. These are
output tracking gaps, **not** errors against independent truth, and the new
diagnostics do not change ODOM. Evidence:
`.agents/notes/native_corrected_startup_20261023/correction_envelope_diagnostics_v2.json`.

`local_corrections.csv` separates accepted per-scan position/velocity changes,
rotation and bias norms from the internal/published states. Reports summarize
net correction demand, total correction path and maximum output tracking gap;
these are not independent truth errors. Rejected scans must not change position
or velocity. The iterated local plane update uses one fixed prior per scan,
including the prior-to-current attitude chart, instead of reapplying the scan
as a new observation on each iteration. Numerical covariance remains
unqualified and is inflated once from the total correction, never promoted.

The fixed-prior repair has fresh corrected-input evidence under
`.agents/notes/native_fixed_prior_validation_20261023_v1/comparison.json`.
Three-capture budget violations are now 1372/0/0 rather than 1412/0/63;
first-capture output tracking remains outside its declared envelope.

Internal deskew history now retains the propagated and corrected states at
each IMU frontier separately. Interpolation before a LiDAR correction ends
at its propagated state; the exact correction epoch and following interval
use the corrected state. This avoids smearing an estimator jump into earlier
physical motion and does not rewrite published ODOM history.

Use optional `--imu-trace` on `native_lio_replay` or
`run_native_map_diagnostic.py` to write `imu_trace.csv`. It records actual
propagation acceleration, specific force, angular rate, midpoint attitude,
bias vectors, before/after position/velocity and unqualified covariance-block
diagnostics through the existing estimator callback. The default does not
enable this callback or write the trace. Trace work adds diagnostic overhead;
its latency is not a deployment estimate. Reports account for every IMU and
LiDAR position/velocity contribution and reject nonfinite, noncausal,
equation-inconsistent or unexplained changes. HTML plots show signed XYZ
acceleration, velocity and biases, not independent truth or accuracy.

For deterministic local-only model diagnosis, `native_lio_replay` accepts
`--native-joint-audit-period-s 0.005`; this is the IMU sample period used to
convert noise PSD, not an audit interval. Optional positive integer
`--native-joint-max-dimension`, `--native-joint-max-memory-rows` and
`--native-joint-max-snapshots` override only the explicitly enabled observer's
separate capacity bounds. They require the audit flag and do not change defaults
or qualify uncertainty. `native_joint.csv` records the effective limits and
configured local history duration, not a claimed retained support span.
Increasing capacity does not repair rank loss, missing physical dependencies
or target-graph bootstrap/source-provenance requirements. An audit failure
preserves its decision and aborts without replacing nominal ODOM covariance.

Latest three-capture trace evidence is under
`.agents/notes/native_imu_trace_validation_20261023_v2/`.
After the history-boundary repair, scan-frontier tracking gaps are
4.661/1.440/1.930 m, with 1372/0/0 budget violations. The first capture still
fails; the third gap increases compared with the fixed-prior-only version.
State accounting closes to numerical precision, but cannot distinguish real
acceleration from attitude/bias error without independent evidence.
The remaining local-error model, causal recovery, graph and host goals are
not complete; no thresholds or trust qualifications were relaxed.

### Bounded diagnostic host scheduling

`--arrival-paced` replays recorded receive-time spacing against a monotonic
wall clock and flushes each emitted ODOM row. `host_timing.csv` retains the
scheduled and actual emission times for every IMU row, including invalid
startup. JSON separates valid-row 10 ms deadline misses and wall-clock gaps;
catch-up bursts do not erase earlier misses. This measures the diagnostic
CSV host, including point-file I/O, not an Apollo publisher or isolated target
hardware.

`--arrival-rate 1`, `2` or `5` requires `--arrival-paced` and compresses only
the wall schedule. Original sensor/receive timestamps stay unchanged;
`replay_clock.csv` declares the rate and origin. Timing analysis verifies
the scaled schedule and keeps the measured wall lateness. Accelerated
diagnostic timers do not qualify live sensor clocks or Apollo consumers.
Bounded-worker errors remain explicit failures, not silently dropped input
or successful overload recovery.

Use `--async-global --arrival-paced` with a map and/or GNSS arm to move its
single global owner, matcher and journals onto one worker. The producer sends
copied native states/scans; the worker never accesses `LocalEstimator`.
FIFO admission is bounded to 256 outstanding tasks, including the running
task, and 64 MiB of declared payload, not total process memory.
Overflow fails explicitly without blocking the producer or silently dropping
an observation. Worker exceptions propagate to the caller; queued work is
journaled as cancelled after failure. `global_scheduler.csv` retains every
admitted task's enqueue/start/finish times, payload and outcome. Normal EOF
drains and joins the worker before output closure. There is no hidden
backend switch, reset handling or production publication.

The historical same-binary, receive-paced second-capture comparison retains identical
ODOM and global decisions, poses and admission chronology. In one shared
container run, valid-row deadline misses decrease from 2025 synchronous to
215 with the global worker, but the latter still has 164 wall-clock gaps
above 10 ms. At that revision native local LIO, startup and point-file loading
were synchronous. Third-capture asynchronous
map-only and first-capture paced local-only likewise retain failed deadline
checks. Evidence:
`.agents/notes/native_async_global_validation_20261023_v1/`.
The current opt-in `--async-local-lio-ms <5..100> --arrival-paced` runs one
local-LIO snapshot off-thread and catches it up with at most 128 received
IMU samples before owner-thread commit. Owner/epoch/frontier, competing
updates, age and integrity guards protect commit; published history and the
live output smoother are retained. Wheel-aided instances reject this path.
`local_lidar_tasks.csv` retains busy, expired, rejected and EOF-uncommitted
outcomes. A missing future IMU at EOF is never synthesized.

Use `--prefetch-lidar --arrival-paced` to decode at most one future point
file off-thread. Consumption still requires recorded scan arrival and the
measurement frontier. `lidar_decode.csv` records consumed, not-ready and
unused scans; malformed, oversized and failed reads propagate errors.
Default synchronous behavior remains unchanged.

Frozen three-capture prefetch runs with a 50 ms local commit policy have
4/7/5 valid-row deadline misses and 59/36/168 wall gaps above 10 ms.
Of these gaps, 51/32/163 also span recorded receive intervals above 10 ms.
Reports distinguish receive gaps from emission delay without declaring the
remaining input or scheduling problem solved. Second both and third map
preserve local ODOM exactly except timing under identical local commit
schedules, with 84 map + 12 GNSS and 156 map commits respectively.
First-capture tracking still fails; native graph qualification, causal
recovery and timer-driven Apollo publisher/TF/reset remain incomplete.
Evidence: `.agents/notes/native_async_local_validation_20261023_v2/`.
Original frozen reports are unchanged; `audit.json` separately pins the
reporter used for additional receive-gap attribution.

### Seed-free conditional map acquisition

`run_native_map_diagnostic.py --recovery-policy <csv>
--recovery-policy-sha256 <sha256>` replaces the offline seed options for map
arms. The pinned policy uses `name,value` rows: `field_min_{x,y,z}`,
`field_max_{x,y,z}`, `search_min_{x,y,z}`, `search_max_{x,y,z}`,
`observed_min_{x,y,z}`, `observed_max_{x,y,z}`, `yaw_min_rad`, `yaw_max_rad`,
`tilt_q{x,y,z,w}`, `field_resolution_m`, `distance_sigma_m`,
`inlier_distance_m`. Search bounds must lie strictly inside the field.
Coverage is declared distance-scoring support, not navigability or visibility;
fixed tilt is a conditional attitude prior, not marginalized gravity error.
The native bundle contains `recovery.csv` instead of `seed.csv`; their hash
occupies the same third position in `--map-input-sha256`. Any offline seed
or seed-time file in a recovery bundle is rejected.

Existing particles generate at most eight basins from 1,024 samples; unchanged
point-to-plane matching verifies full-rank, unambiguous geometry. A persistent
fixed-alignment tracker requires at least three distinct frames and 0.2 s.
Only a single surviving verified basin can propose a diagnostic alignment.
The field is bounded to 500,000 cells and two million map points. Search mass
is not pose probability or covariance. Search and geometry refusals remain in
`causal_search.csv` and `causal_candidates.csv`, including candidate poses.

With `--async-global`, one separate recovery snapshot worker prevents slow
candidate refinement from blocking global ODOM/GNSS processing. The matcher
is never used concurrently; results retain the exact source ODOM pose and
epoch, and proposals older than 500 ms are rejected. `causal_worker.csv`
accounts for busy queries, results, stale rejection and unused EOF work.
No proposal initializes, resets or corrects native ODOM.

For this explicit recovery-policy path, expiration of the existing two-second
full-pose evidence lease now restarts a fresh bounded search generation.
Partial map/GNSS updates do not renew that lease. A returned proposal must pass
the current scan's full-rank, unambiguous matching gates before tracking it;
recovery then still needs the single owner's distinct-frame verification.
While verification or holdback is pending, an old retained alignment cannot
overwrite the candidate. The diagnostic initialized indicator follows the
current owner, not whether any historical output existed.
`map_tracking.csv` records lease expiry, current-scan verification and committed
alignment transitions; `causal_search.csv` includes the recovery generation.
Failure to reacquire leaves full-pose freshness false, without resetting ODOM
or granting trusted capability. Offline-seed runs retain their original policy.

Third-capture raw regional acquisition commits 146 map corrections without
an offline pose seed and preserves synchronous local-only ODOM exactly except
timing. A wider full-map-bounds search produces no verified proposal and
remains unanchored. These are conditional-region/tilt diagnostics, not proven
unknown-location recovery. The paced regional arm completes with 85 map
commits after search isolation, but decoder deferrals change startup and local
commit timing: controlled paired ODOM isolation is **not established** for
that arm. All health remains untrusted; qualified recovery/handover, graph and
Apollo publication goals are still open. Frozen evidence:
`.agents/notes/native_causal_map_validation_20261023_v2/audit.json`.

The conditional lease/reacquisition path has a separate frozen third-capture
baseline at
`.agents/notes/native_tracking_recovery_validation_20261023_v1/audit.json`.
Its normal arm retains 148 accepted matches / 146 commits. An explicitly
injected 2.5-second LiDAR-arrival outage drops 25 scans, retains all IMU, and
produces 119 accepted matches / 113 commits; reacquisition commits 1.800061 s
after recorded lease expiry in measurement time, not a wall-clock deadline.
Both arms preserve their respective local-only 5,999 ODOM rows except timing.
The injected input is labeled as modified, not an unmodified recording or an
independent accuracy reference. This does not prove full-map recovery.
An earlier broader regression aborted in `localization_map_tests`,
in the unchanged NDT planar-geometry test with
`double free or corruption (out)`. PCL also reports an aligned-allocation
warning, but that does not establish the crash's exact cause. Failure logs/XML
are frozen alongside the raw replay evidence, which is not relabeled as
complete acceptance.

Follow-up investigation did not reproduce the abort under GDB, thirty
in-process suite repetitions, or ten independent normal Bazel test processes.
Final native Python contracts also pass. The original crash remains
unexplained, not fixed or erased; it is still a release concern.
Avoid login shells around validation: shell initialization can reintroduce a
different workspace's `LD_LIBRARY_PATH` after `env -u LD_LIBRARY_PATH`.
The diagnostic wrapper now records `runtime_identity.json` before execution,
checks the ELF and actual `ldd`-resolved shared-library hashes again afterward,
and rejects foreign Bazel output/configuration libraries. Container mode
inspects inside the same mapped-user container and loader environment.
Reports carry `execution_runtime_identity`; report-only historical calls leave
it null rather than inventing past runtime pins. This guards repeatable
execution, not calibration, dynamic `dlopen` modules or a complete build
dependency closure. A new ELF hash alone is not a complete native baseline.

The default-off `--timer-output` requires `--arrival-paced`. An independent
10 ms thread publishes only the latest immutable local snapshot to
`timer_output.csv`; it neither extrapolates nor renews measurement timestamps.
New, held, unavailable, epoch-change and skipped-tick records are distinct.
Freshness includes sensor receive delay and elapsed time since the original
scheduled arrival, not the later processing handoff. Callback errors surface
to the replay owner; a slow callback does not hold the estimator handoff lock.
This is a diagnostic CSV host, not an Apollo localization/TF publisher.

Three corrected raw 30-second windows now have frozen timer/local comparisons.
Every usable timer pose exactly matches its own source ODOM snapshot. The
first two captures preserve cross-run ODOM exactly except timing. Third-capture
timer-only comparison fails strict equality, with maximum position-component
difference `1.00533e-11 m`; a no-timer repeat is exact, and the unique numerical
cause remains unproven. No comparison tolerance was changed. Timer-only runs
miss 3/5/2 whole-period deadlines; their held outputs are not new 100 Hz ODOM.
The third-capture async causal-map/timer arm preserves local ODOM and records
3,000 ticks without a whole-period miss, but commits **zero** map corrections.
Its search has ten stale-result rejections, ten `HISTORY_UNAVAILABLE` search
rejections and exhausted restart attempts. It is not an end-to-end map pass.
All seven executions and both failed acceptance flags are retained in
`.agents/notes/native_timer_output_validation_20261023_v1/final_audit.json`,
with per-arm CSV/HTML reports under `raw_v2/` and the strict-comparison
investigation beside them. Native worker/timer C++ cases and 14/5/3 Python
contracts pass; production replacement, graph qualification and deadlines
remain open.

The subsequent online acquisition repair rejects truncated particle-mode sets
**before** map refinement. Previously those sets ran eight costly matcher
calls despite being unconditionally inadmissible afterward; this consumed
snapshot age and disrupted the bounded search lifecycle. Particle updates
still accumulate and uniqueness/geometry/age gates remain unchanged.
The same third-capture paced map/timer arm now accepts 143/162 matches and
commits 141 corrections, with 5,999 ODOM rows identical to the failed arm
except timing. Four truncated snapshots trigger zero geometry calls;
3,001 timer ticks have no whole-period miss in this run.
Evidence: the timer investigation's `search_fast_rejection/audit.json`.
The earlier zero-commit report is retained, not overwritten.
The declared 2.5-second scan-arrival outage also completes with paced async
recovery: 110 map accepts / 106 commits, one lease expiry and two tracking
commits, with 2.200062275 s from expiry to recovery on the measurement clock.
All 5,999 ODOM rows match the corresponding fault-input local-only arm except
timing; 3,003 timer ticks have no whole-period miss. This remains an explicitly
modified-input regional recovery check, not a wall-clock recovery guarantee.
Evidence: `search_fast_rejection_gap/audit.json` in the same investigation.

Both GGA recordings also pass separate paced, async-global seeded-map/timer
functional runs: first capture commits 122 map + 4 GNSS corrections, second
78 map + 12 GNSS; both preserve their matching local-only ODOM exactly except
timing. Their original explicit bootstrap resolutions differ (0.5/0.2 m), so
these are wiring checks, not a common-configuration algorithm comparison.
The first still misses two whole-period timer deadlines; all health remains
untrusted and no GGA heading is fabricated. These seeded arms do not prove
causal unknown-pose acquisition or independently referenced accuracy.
Commands, source manifests, runtime identity and reports are in
the same investigation's `three_source/audit.json`.

Two additional options remain experimental and default-off:
`--motion-translation-deskew` uses the previous accepted local scan interval's
velocity for constant-velocity point compensation; it does not solve gyro-bias
feedback or acceleration within the scan. `--lidar-stationary-initialization`
allows a separate LiDAR-confirmed-rest hypothesis only after every scan
interval meets existing stationary speed/angular limits and IMU force/gravity/
gyro checks pass. That path explicitly assumes zero velocity and zero
accelerometer bias; it does not identify that bias or acquire uncertainty
qualification. Both require `--motion-bootstrap`, and neither alone resolved
the third capture at the default 0.5 m resolution. Point-level rolling-plane,
causal velocity, rest admission/rejection and atomic unqualified commit
regressions are in `//:native_motion_bootstrap_tests` and
`//:motion_initializer_tests`.

`motion_initialization.csv` retains per-startup-scan numerical evidence:
whether the linear system was evaluated, its minimum scaled singular value,
whether a finite solution exists, and gravity, bias, angular and fit residuals.
The evaluation/solution flags distinguish missing evidence from zero error.
Rejected solutions remain visible in the JSON report and the HTML gravity/bias
plot; numerical rank and small residuals do not establish accuracy.
Focused regressions cover analytic variable translation/attitude and coherent
accelerative pose drift: the latter can preserve rank and fit residual while
violating the gravity budget, so the physical rejection must remain enabled.

`LocalEstimator::InitializeMotion` commits once at the actual received IMU
frontier, establishes a local gravity-aligned gauge, and leaves both covariance
model and uncertainty qualification false. No map/GNSS correction is used to
initialize local ODOM. The library retains its own bounded startup IMU evidence;
the caller supplies only metric relative scan poses at the IMU origin.

```bash
env -u LD_LIBRARY_PATH bazel test //:motion_initializer_tests \
  //:local_estimator_tests
env -u LD_LIBRARY_PATH bazel-bin/native_lio_replay \
  <input-directory> <new-output-directory> --motion-bootstrap
```

Scans are deskewed to a real committed IMU frontier after their original end
and arrival; raw point timestamps are never rounded to IMU samples. Startup
scans without initialized history are explicitly recorded. Unexpected native
errors stop replay with a nonzero exit; absence of initialization or successful
LIO updates also fails. Outputs retain per-IMU call latency, validity/integrity,
motion eligibility, scan reference time, observed rank and rejection detail.
These latencies exclude decoding/CSV and do not certify host deadlines.
Generate a hash-checked JSON report and standalone local-trajectory HTML with
the standard-library reporter (no visualization packages are required):

```bash
python3 test/integration/map/native_lio_report.py \
  --input <input-directory> --replay <replay-output-directory> \
  --output <new-report-directory> --binary bazel-bin/native_lio_replay
```

The report separates startup from runtime scan decisions and scan latency from
IMU-call latency. It never plots nominal covariance as qualified uncertainty,
invents a factor graph, or treats measurement frequency as runtime throughput.
The explicit diagnostic noise/budget configuration is uncalibrated; bag record
time does not establish hardware arrival qualification.
The runner retains `WHEEL_STALE` and `CORRELATION_UNQUALIFIED` as capability
diagnostics when `EvaluateIntegrity()` still establishes propagation integrity;
neither is converted to a healthy assessment or graph qualification.
The internal GNSS quality policy has a separate, default-off
`allow_missing_horizontal_accuracy_diagnostic` option for position-only replay
when receiver accuracy is absent (NaN). Heading remains strict; invalid numeric
accuracy and reported accuracy above its limit remain rejected. Receiver
quality and dwell checks remain active. This does not qualify uncertainty or
permit a diagnostic noise scale to masquerade as receiver-reported accuracy.

The first native raw-data check is a **failed startup**, not a localization
baseline: `20261012103814`, first 30 seconds, 6,000 IMU samples and 300 scans,
produced no initialized state or LIO update under the unchanged native quiet
window. A read-only first-30-second inventory of all seven captures likewise
found no interval satisfying the native instantaneous thresholds for its
2-second dwell. A subsequent whole-recording raw-IMU inventory confirmed that
none of the seven complete recordings contains a qualifying window under
those unchanged thresholds; the longest interval is 0.214 seconds.
This does not establish actual vehicle motion or receiver/IMU health.
Evidence, including `full_startup_inventory.json`, is retained in
`.agents/notes/native_lio_20261023_v1/`. FAST-LIO's separate
startup preflight uses different criteria and does not qualify this initializer.
No thresholds, initial states, noise scales or source qualifications were
changed to turn that outcome into a pass.

With `--motion-bootstrap`, the same 30-second raw input subsequently initialized
at approximately 18.44 seconds and produced 2,311 valid IMU-rate states and
113 successful LIO updates. The earlier motion candidate incorrectly tested
instantaneous gyro samples against a constant scan-interval angular rate;
integrated rotation alignment removes that assumption without relaxing gates.
The frozen diagnostic output is
`.agents/notes/native_lio_20261023_v1/20261012103814/motion_output_v3/`.
This is frontend execution evidence only: local covariance remains unqualified,
no global/prior-map trajectory was produced, and accuracy has not been measured.
IMU-call latency was below 2 ms in this run, but scan processing took hundreds
of milliseconds synchronously; neither this replay nor its output rate proves
a concurrent 100 Hz runtime deadline.

The unchanged motion candidate was also exercised on two additional raw
captures, each using its first 30 seconds with no external ODOM/GNSS input:

| Capture | First valid state after start | Valid IMU states | LIO updates |
| --- | ---: | ---: | ---: |
| `20261012103814` | 18.445 s | 2,311 | 113 |
| `20261012105149` | 20.335 s | 1,933 | 94 |
| `20261015185840` | Not initialized | 0 | 0 |

Each capture retains `motion_output_v3/` and `report_v3/index.html` beneath the
same evidence root. The third capture exits nonzero: 90 windows fail physical
initialization budgets, with median gravity-norm error 1.807 m/s2 and estimated
accelerometer-bias norm 2.711 m/s2 despite small linear residuals. Its median
integrated angular residual is 0.0224 rad/s, so the earlier constant-angular-rate
bug no longer explains this failure. These observations do not isolate
registration, deskew, calibration or excitation as the remaining cause.
Runtime scan median latency in the two initialized captures is approximately
419/414 ms in this diagnostic build, excluding CSV decoding. Do not infer
production throughput from the IMU measurement rate or extrapolate these short
windows to full-recording/indoor-outdoor readiness.

Native LIO currently invalidates its joint covariance model after a successful
correction and exposes no graph-eligible motion connection. This entry must
not override that result to feed `FixedLagGlobalAlignment`, nor substitute the
4DoF diagnostic backend and call it a target-graph validation. Supplied prior
maps remain inputs for the subsequent scan-to-map integration stage.

The selected DR diagnostic baseline is **wheel speed + raw gyro Z**, with
zero fitted gyro bias and unchanged speed scale. It is a comparison reference,
not a production default, qualified covariance source or implicit fallback.
The selected LIO/global workstream reuses GNSS-off FAST-LIO2 ODOM; GNSS and
scan-to-map must share one global-alignment owner and never rewrite ODOM.
See the [roadmap](docs/roadmap.md) for the 2026-10-15 chain audit and gates.

The active two-capture goal is **normal scan-to-map + GNSS + ODOM fusion
first**, then source-quality recovery, then exceptional inputs. The selected
XY release is only an ODOM/GNSS sub-baseline. Default ICP and finer-target NDT
both accepted 0/4 startup observations; a separate explicit diagnostic support
model and single-owner replay now exercise actual accepted map observations.
This is not a production switch or calibrated accuracy claim. See the
[current evidence and remaining integration contracts](docs/roadmap.md#active-priority-normal-three-source-fusion-before-recovery).
The selected normal diagnostic is `openloc-three-source-normal-baseline-v1`,
packaged under
`.agents/notes/two_capture_validation_20261015/normal_three_source_baseline_release_v1/`.
It includes the frozen executable, both maps, selected scans, all event inputs,
source/config snapshot and golden outputs. On the host, reproduce all eight
normal arms with the copied executable in the managed test container:

```bash
python3 pkgs/openloc/.agents/notes/two_capture_validation_20261015/normal_three_source_baseline_release_v1/check_replay.py \
  --package pkgs/openloc --container <managed-test-container> \
  --output pkgs/openloc/.agents/notes/two_capture_validation_20261015/<new-check>
```

The verifier checks every payload hash and all output fields except measured
processing time. This is an offline comparison baseline: startup is
future-informed, uncertainty uncalibrated, map coverage incomplete and backend
3D corrections can exceed 4 m. Do not use it as trusted/smoothed runtime output.

This is a standalone Bazel/Bzlmod package. From this directory:

When running these standalone tests inside the managed Apollo container, unset
the root workspace `LD_LIBRARY_PATH`; otherwise tests can load stale shared
libraries from a different Bazel output tree.

```bash
env -u LD_LIBRARY_PATH bazel test //:online_contracts_test \
  //:diagnostic_global_alignment_test \
  //:fixed_lag_global_alignment_test \
  //:online_gnss_quality_test \
  //:online_gtsam_fixed_lag_spike_test \
  //:online_alignment_recovery_test \
  //:online_alignment_smoother_test \
  //:online_localization_test
env -u LD_LIBRARY_PATH bazel test \
  //:localization_algorithm_tests //:localization_map_tests
env -u LD_LIBRARY_PATH bazel test //:localization_health_tests
env -u LD_LIBRARY_PATH bazel test //:scan_to_map_tests
env -u LD_LIBRARY_PATH bazel test //:four_wheel_kinematics_tests
env -u LD_LIBRARY_PATH bazel test //:planar_wheel_dr_tests
env -u LD_LIBRARY_PATH bazel test -c opt //:native_odometry_tests \
  //:native_motion_bootstrap_tests //:local_estimator_tests \
  //:motion_initializer_tests
```

The algorithm regressions reject non-rigid LiDAR extrinsics and non-unit local
quaternions. `AdaptScanToMapObservation` is the typed internal boundary from an
accepted `ScanToMapResult` to the private `GlobalAlignment`: it checks source
time against a bounded double-history conversion budget, clock, map
content/version, frames and calibration identity, retains full/projected
covariance provenance, and never claims the
map match is independent of its ODOM seed. Its regression checks that accepted
map evidence reaches the same owner without changing ODOM; the existing
synthetic GNSS/map case still checks shared-owner updates and outlier rejection.
This remains a diagnostic adapter, not the public scan-to-map runtime API; it
does not qualify native LIO covariance or establish a complete online pipeline.
The explicit `MapObservationTiming` overload carries independent arrival time
and source sequence. The observation retains integer measurement/arrival
timestamps; the legacy owner still associates history in double seconds and
does not promise nanosecond resolution. Independent arrival and source sequence
are mandatory; no overload borrows them from local history. Adapter failures leave
the caller's previous output intact; non-rigid poses, invalid covariance and
rank-deficient projection rows are rejected before submission.

The public diagnostic entry is now `DiagnosticGlobalAlignment`, from
`openloc/localization/global/diagnostic_alignment.h`, linked with
`//:diagnostic_global_alignment`. It wraps the existing diagnostic SE(3)
owner behind a PIMPL; public headers expose neither PCL/GTSAM nor candidate
types. Configure an explicitly diagnostic reference, then call `AddOdometry`,
`SubmitMapObservation`, `SubmitHorizontalObservation`, and `GetResult`.
The host submits admitted matcher observations and prepared antenna XY,
including exact measurement/arrival times, source sequence, epoch,
map/calibration/content identity and covariance provenance. This interface
does not replace receiver admission.

Future observations are queued with a fixed capacity and applied at their
historical ODOM time; rejections and completed decisions are bounded and
explicit. Epoch changes retire incompatible queued observations. Without a
verified full-pose basin, the result has no alignment estimate: XY evidence
does not create an identity transform or synthetic orientation/height.
Distinct consistent map observations can bootstrap the diagnostic owner
without a future-informed seed. Its result is the raw backend, not a
publication-smoothed or trusted runtime output. It has no qualified graph
switch, TF publisher or local-state feedback. Use one selected global owner,
not this facade plus the public 4DoF engine in the same localization chain.
The existing oracle-seeded offline runner below remains a separate frozen
comparison adapter; these API tests do not make its startup causal.
The partial-update conditional null-space model is not a conservative joint
bound for arbitrary unknown cross-correlation.

`GlobalObservationContext::provenance` separately records acquisition and
preparation identities plus ODOM-initialization, ODOM-deskew, shared-local-scan
and GNSS-map-construction dependencies. Each is `kUnknown` by default, or a
caller-declared `kAbsent`/`kPresent`. Missing evidence stays unknown; neither
absence nor matching/different identity strings establishes statistical
independence. These fields never enable a qualified backend or change fusion
weights. Final decisions preserve both this context and `covariance_source`
on application, rejection or epoch retirement, so provenance is not lost when
a queued observation leaves the pending queue.

Map observations also carry explicit `ambiguous` and `recovery` declarations.
Ambiguity interrupts verification; recovery needs distinct consistent full
poses, never a single match or projected constraint. Decisions have monotonic
`decision_id`, the owner's `correction_id` at the decision, and
`alignment_updated`: OK unanchored XY evidence is retained, not an alignment
update. IDs survive epoch changes; `decisions_dropped` reports journal eviction.
Use `GetResult(evaluation_time, ...)` to check `odometry_fresh` and
`full_pose_evidence_fresh` at an explicit clock epoch without extrapolating
ODOM. `timestamp` still identifies the composed ODOM sample.
`last_full_pose_time` is the original measurement time of the last applied
full pose; partial updates do not renew it. A retained estimate, recent full
pose, and fresh ODOM are separate facts, none of which qualifies trust.

### Prepared-observation diagnostic replay

The public facade has a separate seed-free adapter; it does not run or tune
scan-to-map and does not modify the frozen offline-informed baseline:

```bash
env -u LD_LIBRARY_PATH bazel build //:diagnostic_alignment_replay
env -u LD_LIBRARY_PATH python3 \
  test/integration/map/diagnostic_alignment_replay_test.py \
  bazel-bin/diagnostic_alignment_replay \
  --artifacts .agents/notes/<new-software-check>
bazel-bin/diagnostic_alignment_replay <odom.csv> <map.csv|-> <xy.csv|-> \
  <contract.csv> <new-output-directory> all --diagnostic-only
```

Run in the managed test container. Arms are `odom`, `gnss`, `map`, `all`;
every arm keeps every input event, including excluded sources as
`not_requested`. Optional streams may be `-` or header-only. Each stream must
be arrival-ordered; the deterministic merge orders equal arrivals as ODOM,
map, then XY, retaining file order. It never sorts delivery by measurement
time. ODOM measurement times must advance, including epoch changes.
All times use positive Unix nanoseconds. A map/XY observation may await an
ODOM bracket; unresolved tail observations remain pending and are reported,
not silently flushed using invented motion. ODOM delivery cannot precede
measurement time; the library's existing observation time budgets still apply.

Exact unquoted input headers:

```text
# contract.csv: exactly one row; caller-owned identities, not verified hashes
map_id,map_version,map_content_id,calibration_id,map_frame,odom_frame,base_frame
# odom.csv
arrival_ns,timestamp_ns,session,generation,px,py,pz,qx,qy,qz,qw,covariance
# map.csv
arrival_ns,timestamp_ns,session,generation,sequence,source,kind,px,py,pz,qx,qy,qz,qw,covariance,projection,projected_covariance,covariance_source,ambiguous,recovery
# xy.csv
arrival_ns,timestamp_ns,session,generation,sequence,source,x,y,covariance,lx,ly,lz,covariance_source
```

The `#` lines above label files and are not part of their CSV contents.
Matrix cells use `;` between columns and `|` between rows, preserving cross
terms. ODOM uses a 6x6 covariance in the public ODOM tangent convention;
XY uses a 2x2 antenna-position covariance and an explicit body-fixed lever.
Map `kind=full` requires a 6x6 covariance and `-` in both projection fields.
Map `kind=projected` requires `covariance=-`, a 1..5 by 6 projection and the
corresponding square projected covariance. Covariance provenance is required.
`ambiguous`/`recovery` are 0/1. File text fields cannot contain CSV quoting,
commas or newlines; LF and CRLF are supported.

Map and XY files may append this exact optional header suffix and six
corresponding cells to every row:

```text
acquisition_id,preparation_id,odometry_initialization,odometry_deskew,shared_local_scans,gnss_map_construction
```

The last four cells must be `unknown`, `absent` or `present`. Without the
suffix, identities stay empty and dependencies stay unknown, not independent.
Both event and final-decision logs retain these declarations and the supplied
covariance source, including sources excluded by an arm. No qualification is
inferred from provenance tags; input errors in tags are explicit failures.

The new directory contains `events.csv` (all input decisions and raw estimates),
`decisions.csv` (each final queued/immediate decision once), `metrics.csv`
(per-event timing and correction sizes), and `summary.csv`.
Uninitialized global fields are empty, not identity or zero covariance.
Summary updates count actual revisions; retained unanchored evidence and
non-OK decisions are separate. Non-OK includes verification waits, not just
outlier rejection. The decision journal is checked for gaps.
`GetDecisions` exposes the journal before ODOM is available; early rejections
are logged at their decision event rather than assigned an artificial wait
until the first ODOM sample. Final decisions include `decision_time_ns` and
`queue_wait_ns` in the replay arrival clock. Metrics preserve signed
`arrival_delay_ns` (including rejected future measurements). Correction
translation/rotation compares old and new alignments at the same current
ODOM body pose, so motion is not counted as a correction; initialization,
missing basins and epoch changes have empty correction fields.
Summary includes queue peak/wait, maximum correction, per-event processing
P50/P95/max and process peak RSS in KiB on Linux. P50/P95 use nearest-rank
order statistics. Processing time uses a steady clock from event dispatch
through result/event serialization, excluding subsequent metric/decision
serialization. RSS includes input loading and runtime libraries, not just the
owner. These are offline replay measurements, not online/target-hardware
latency or memory qualification; wall-time/RSS fields are not deterministic
golden outputs.

Only a successful exit and complete summary establish a completed software
run; failed runs may leave partial output for diagnosis and are not overwritten.
All outputs remain `diagnostic_only=1,global_trusted=0`.
The test command can preserve synthetic inputs, outputs, command logs, test
source and a SHA-256 manifest; it is not a recorded-data baseline release.

This adapter assumes the host already prepared map/XY observations. It does
not establish receiver quality, source independence, causal map construction
or causal matcher initialization. Before a new recorded-data baseline, freeze
and approve upstream map/frontend/calibration/provenance and genuine arrival
exports. In particular, do not fill missing arrival times with measurement
times or reuse the old offline-informed seed. GNSS-only input deliberately
does not initialize a full SE(3) basin in this adapter; use the selected public
4DoF path when its gravity contract is appropriate.

The normal-only four-arm diagnostic adapter uses the existing conservative
SE(3)/covariance-intersection owner, not the public four-DoF engine:

```bash
env -u LD_LIBRARY_PATH bazel build //:three_source_dataset_replay
env -u LD_LIBRARY_PATH bazel test //:localization_algorithm_tests \
  //:scan_to_map_tests
env -u LD_LIBRARY_PATH python3 \
  test/integration/map/three_source_replay_test.py \
  bazel-bin/three_source_dataset_replay
bazel-bin/three_source_dataset_replay <map.pcd> <events.csv> <prior.csv> \
  <new-output.csv> all --diagnostic-only
```

Arms are `odom`, `gnss`, `map`, `all`. Events use the exact header
`timestamp_ns,px,py,pz,qx,qy,qz,qw,gx,gy,sigma,gnss_present,scan_path,local_position_variance,local_rotation_variance`.
`scan_path=-` means no scheduled map query; GNSS availability is 0/1. Prior
contains one headerless `x,y,z,qx,qy,qz,qw` alignment, declared identically in
every arm. Local variances are explicit uncalibrated diagnostic inputs, not
qualified LIO covariance. The fixed manual antenna lever is
`[-0.131,-0.02329,0.00412] m`; use this dataset-specific runner only with that
declared calibration assumption.

The matcher uses the opt-in separate NDT support model described in the
[architecture](docs/architecture.md#explicit-diagnostic-three-source-alternative).
Accepted covariance/subspace reaches the owner directly in memory. Output
includes all source decisions, map support/rank, covariance and current-body
correction sizes; matrix CSV cells use `;` between columns and `|` between rows.
`diagnostic_only=1` and `global_trusted=0` always remain explicit. Direct offline
backend output does not exercise public correction smoothing, real receiver
admission, full-rate arrival, recovery or calibrated uncertainty.

The public `LocalizationEngine` accepts `GnssHorizontalFix` through
`SubmitGnssHorizontalFix`: measured antenna XY and a 2x2 covariance, with no
invented Z. It shares receiver quality, sensor-time ODOM association, ordering,
admission rate, recovery and publication with the existing XYZ path. XY and
XYZ share one position stream; do not submit both forms of a receiver epoch.
Records distinguish `ABS_POS2` from `ABS_POS3`. Horizontal initialization sets
`global_alignment_valid` but not `vertical_position_initialized`; neither flag
qualifies full-pose accuracy or integrity.

The public 4DoF engine also requires an explicit `odom_gravity_contract`:
`kGravityAligned` is a caller declaration requiring frame evidence, while
`kDiagnosticAssumption` labels an unresolved ODOM up-axis. The engine does not
infer parent-frame axes from vehicle roll/pitch or apply a leveling transform.
`LocalizationResult::odom_gravity_contract` preserves that caller declaration
in the output, including diagnostic assumptions.
`LocalizationResult::observability` reports directly constrained horizontal
position, height and heading; `calibration_state` reports the geodetic-reference
declaration only. Neither certifies rank, sensor calibration, integrity or
accuracy.

All accepted updates, including recovery and first heading after an existing
publication, use one correction publication policy. Translation limits apply
to the composed **current body position**, not alignment translation
parameters. Moderate corrections are rate-limited at the latest ODOM time;
large corrections are held until consecutive consistent stopped observations
at distinct publication epochs confirm a reset. Moving or rejected observations
break confirmation. Startup still explicitly initializes the first publication.
Limits remain provisional and recovery remains default-off.

`LargeCorrectionPolicy::kRateLimitedTracking` is a separate explicit diagnostic
opt-in. It bounds every post-initialization correction (including small and
large ones) by 0.3 m/s and 1 degree/s by default, advancing on ODOM ticks only
while an accepted position measurement is at most 1.5 s old. Stale intervals
do not accumulate correction credit, including a new fix queued across an ODOM
gap. It does not issue a large reset. The default remains
`kHoldForStoppedReset`; no measurement gate is relaxed by either policy.
Check `correction_source_fresh`, `correction_pending` and
`global_output_degraded`; a cleared degraded flag is not an integrity or
accuracy certification.

Backend XY translation priors/process noise are now defined at the fixed first
ODOM XY reference, exposed as `alignment_reference_in_odom`. Caller ODOM and
reported transforms retain their original coordinates. The returned full
alignment covariance includes the corresponding translation/yaw cross terms;
Z retains its original origin. This corrects translated-XY-origin sensitivity,
not arbitrary rotation, gravity qualification or yaw observability.

`backend_map_from_odom` and `alignment_covariance` describe the backend.
`map_from_odom` describes the potentially held/rate-limited publication.
`publication_state`, `correction_pending` and `publication_pose_offset` expose
the difference at the current body pose. `published_mean_matches_backend`
indicates mean agreement only: the backend covariance is **not** a covariance
for a different published mean, and the deterministic offset is not stochastic
noise. No calibrated published covariance is synthesized.
`alignment_revision` counts backend re-anchors within an engine reset cycle;
`reset_counter`/`reset_event` describe published discontinuities or explicit
engine Reset, not a held backend re-anchor.

The separate public-path diagnostic runner exercises receiver gates, pending
ODOM association, recovery and correction publication through the public API:

```bash
env -u LD_LIBRARY_PATH bazel build //:public_xy_dataset_replay
env -u LD_LIBRARY_PATH python3 \
  test/integration/map/public_xy_replay_test.py \
  bazel-bin/public_xy_dataset_replay
bazel-bin/public_xy_dataset_replay <odom.csv> <fixes.csv> <new-output.csv> \
  baseline --diagnostic-synthetic-quality <lever.csv>
```

`baseline` keeps the single-antenna startup bootstrap but disables subsequent
GNSS re-acquisition. Use `recovery` to enable the guarded post-initialization
re-anchor path; use `tracking` to additionally opt into rate-limited
publication. ODOM
schema is `timestamp_ns,px,py,pz,qx,qy,qz,qw`; fixes use
`arrival_ns,timestamp_ns,gx,gy,sigma,quality` (quality 0/1); the single-row
lever schema is `lx,ly,lz`. The mandatory diagnostic flag declares synthetic
receiver fields: fixed/invalid from the fixture quality flag, 20 satellites,
HDOP 0.8, differential age 0.2 s and horizontal accuracy 1 m. These are not
receiver measurements. The adapter uses a diagnostic 2 m accuracy limit and
2.5 s sparse ODOM interpolation bound; ordinary stability, consistency, rate,
NIS and publication limits remain unchanged. Neither override changes library
defaults. Supply body quaternions and a declared lever, not invented rotations.

Global yaw has two distinct sources. A qualified `GnssHeading` is a direct
dual-antenna baseline measurement and initializes yaw through the scalar
heading factor; position fixes then initialize global translation. Without an
accepted dual-antenna heading, the default single-antenna bootstrap does not
initialize from the first position fix. It fits a global XY/yaw candidate from
eight quality-admitted GNSS/ODOM antenna observations spanning at least 5 m,
then requires three later holdout fixes to pass the candidate innovation
check, all within a 30 s window. Until that succeeds, global position remains
invalid. This course-over-ground-derived yaw is an alignment candidate, not a
direct GNSS heading measurement, and correlated GNSS errors are not ruled out
by the holdout. The bootstrap does not alter local ODOM and does not relax the
ordinary NIS gate. Its diagnostic record is an alignment bootstrap, not an
inserted position factor; the visualization distinguishes it from graph factors.
Validated recovery re-anchors are also shown separately from inserted position
factors.

Output includes every ODOM/fix event, original measurement and processing
times, backend/published body and antenna positions, full backend covariance,
correction-only jumps at a common ODOM epoch and publication/reset state.
Companion `.factors.csv` logs pending-to-final record updates by record index;
`.summary.csv` explicitly reports unresolved pending fixes and offline
runtime/peak RSS. Output files cannot overwrite previous results.

### Public XY replay visual diagnostics

The replay's event and factor CSVs can be rendered without rerunning the
estimator. From the package root in the managed container, pass the directory
containing `inputs/`, the replay CSV prefix, and an unused output directory:

```bash
python3 test/integration/map/visualize_public_xy_replay.py \
  .agents/notes/<capture-evidence> \
  --gga /mnt/synology/3dmakerpro/<capture>/gga.txt \
  --output-dir .agents/notes/<new-visualization>
```

The output includes a trajectory overlay, backend covariance timelines, NIS
and admission decisions, and a factor-graph schematic. The GGA overlay is
optional; when supplied, it is checksum-checked and compared against the
prepared fixes; quality-0 no-fix sentences are counted separately from
malformed or incomplete valid-fix sentences. Matplotlib, NumPy and pyproj must
already be available in the managed container; the tool does not install
dependencies. Bootstrap and recovery re-anchors are marked separately and are
not represented as inserted graph factors. The initial 60 s
ODOM alignment is used only to visualize same-input divergence. The GGA and
prepared fixes share a receiver and are not independent truth. Covariance is
backend marginal covariance, and the graph image is reconstructed from
`state_id` records rather than exported from the optimizer. These figures are
diagnostic, not accuracy or graph-qualification evidence. Plot conventions
are informed by [evo](https://github.com/MichaelGrupp/evo) trajectory tools
and [GTSAM](https://github.com/borglab/gtsam) factor-graph representations.

The two-capture driver `test/integration/map/public_xy_validation.py` verifies
frozen source associations/hashes, reconstructs the declared body/antenna chain,
runs two short prefixes then the 24-run three-arm matrix through a specified managed
container. Its synthetic profile, provisional frames/noise and same-source
scoring do not qualify receiver integrity, causal arrival or physical accuracy.
From the host, for an approved frozen two-capture experiment:

```bash
python3 pkgs/openloc/test/integration/map/public_xy_validation.py \
  --package pkgs/openloc --container <managed-test-container> \
  --output pkgs/openloc/.agents/notes/two_capture_validation_20261015/<new-run>
```

The historical `public_xy_v3` run exposed held publication and origin-prior
sensitivity. The completed `public_xy_v7_reference_tracking` iteration fixes
translated-origin parameterization and verifies bounded moving publication.
It contains 35 executions / 99,542 events, including controls; no unresolved
fixes remain. Recovery+hold and recovery+tracking have identical backend
estimates/covariance and ODOM. Same-source published residual P95 improves
from 23.838 to 2.543 m (A nominal) and 17.019 to 3.280 m (B nominal).
These are diagnostic consistency results, not independent accuracy.

### Runnable diagnostic version

The local release is
`.agents/notes/two_capture_validation_20261015/diagnostic_xy_release_v2/`.
It contains the frozen Linux executable, A/B inputs, source snapshot, usage,
evidence and SHA-256 manifest. Run as the mapped non-root user in the managed
test container, from `/apollo/pkgs/openloc`; choose an unused output path:

```bash
r=.agents/notes/two_capture_validation_20261015/diagnostic_xy_release_v2
env -u LD_LIBRARY_PATH "$r/bin/public_xy_dataset_replay" \
  "$r/inputs/A/odom.csv" "$r/inputs/A/nominal_fixes.csv" \
  /tmp/openloc-xy-tracking-new.csv tracking \
  --diagnostic-synthetic-quality "$r/inputs/A/lever.csv"
```

`inputs/B` selects the independent retained-prefix capture; other frozen fix
files exercise outage, outlier and omission. This is a **diagnostic integration
version**, not a production localization release. Accepted map fusion,
measured receiver/arrival replay, physical frame/noise calibration and
independent truth remain open. See the latest [roadmap entry](docs/roadmap.md).

### Selected data-validated engineering baseline

`openloc-xy-tracking-baseline-v1` supersedes the diagnostic release for the
external GNSS-off ODOM + public XY/GNSS workstream. Its selected profile is
explicit recovery-on + bounded tracking; **library defaults do not change**.
It is a sparse two-capture engineering baseline, not a production, native-LIO
or accepted-map-fusion baseline.

The frozen artifact is
`.agents/notes/two_capture_validation_20261015/xy_tracking_baseline_release_v1.tar.gz`
(also available as a directory). It includes the executable, selected profile,
A/B inputs and unmodified scoring reference, acceptance report, source and
hash manifests. From `/apollo/pkgs/openloc` in the managed test container:

```bash
r=.agents/notes/two_capture_validation_20261015/xy_tracking_baseline_release_v1
env -u LD_LIBRARY_PATH "$r/bin/public_xy_dataset_replay" \
  "$r/inputs/A/odom.csv" "$r/inputs/A/nominal_fixes.csv" \
  /tmp/openloc-xy-baseline-new.csv tracking \
  --diagnostic-synthetic-quality "$r/inputs/A/lever.csv"
```

The baseline retains all 773 A / 670 B reference epochs during outage and
outlier tests. It scores the injected outlier against the **unmodified**
reference, not the corrupted +1000 m input. Periodic withheld GNSS observations
never enter public submission/factors/recovery. Withheld-point P95 is
3.057 m (A, versus seeded ODOM-only 17.568 m) and 4.838 m (B, versus22.795 m).
They remain same-receiver/reference consistency, not independent truth.

The additional 22-run acceptance matrix verifies exact repeated outputs,
coverage, rate/staleness/reset invariants, withheld observations and terminal
GNSS loss. It found and fixed recovery starvation: good-quality reports still
waiting for stable-fixed qualification no longer erase earlier qualified
candidate history. Waiting reports are rejected and never enter the candidate;
invalid quality still clears it, and its 30 s total-age /8+3 observation /5 m
baseline limits remain. No noise, receiver, motion or innovation gate was tuned.

To rerun the reusable verifier from the host with the existing frozen inputs:

```bash
python3 pkgs/openloc/test/integration/map/public_xy_baseline.py \
  --package pkgs/openloc --container <managed-test-container> \
  --reference pkgs/openloc/.agents/notes/two_capture_validation_20261015/public_xy_v8_intermittent_recovery \
  --output pkgs/openloc/.agents/notes/two_capture_validation_20261015/<new-baseline-check>
```

Run its focused scoring/coverage tests in the managed test container:
`python3 test/integration/map/public_xy_baseline_test.py`.
`xy_tracking_baseline_v2/evaluation.json` records the passed acceptance;
`xy_tracking_baseline_v1/failure.json` preserves the original starvation failure.
Nominal fresh-and-aligned duration is only72.2%/58.9% for A/B (not calibrated
accuracy availability); stale/transition/startup intervals remain accounted
and explicitly degraded. GNSS-off ODOM remains available independently.

An `awaiting_policy_update` publication state means an ODOM-only backend update
changed the estimate after publication was aligned; the published transform is
unchanged, pending is explicit, and no additional stopped confirmation is counted.

The historical private-backend runner remains available unchanged. It supports
sparse, source-prepared diagnostics, not public receiver admission/publication
or a successful scan-to-map factor bridge:

```bash
env -u LD_LIBRARY_PATH bazel build //:horizontal_alignment_dataset_replay
env -u LD_LIBRARY_PATH python3 \
  test/integration/map/horizontal_alignment_replay_test.py \
  bazel-bin/horizontal_alignment_dataset_replay
```

Its strict CSV schema is
`timestamp_ns,px,py,pz,ax,ay,az,gx,gy,sigma,distance_m,gnss_present,map_rejected`.
`p` and `a` are the body and antenna positions in the same explicitly declared,
fixed diagnostic frame; `g` contains measured horizontal position only.
`distance_m` is the nonnegative traveled-distance increment between states.
Modes are `lio`, `gnss`, `map-rejected`, `gnss-map-rejected` and
`gnss-recovery`. Recovery requires a separate per-epoch diagnostics output:

```bash
bazel-bin/horizontal_alignment_dataset_replay \
  <input.csv> <output.csv> gnss-recovery <recovery_diagnostics.csv>
```

The runner reuses the private recovery helper, with a total 30 s sensor-time
limit for training plus subsequent holdouts. It does not exercise receiver
admission or the public publication policy. The public engine keeps its XYZ
factor contract; an XYZ rejection can seed XY/yaw recovery only when the
selected state's horizontal marginal
innovation also exceeds the 2D gate. Z-only outliers cannot initiate it.
`enable_gnss_reacquisition` remains false by default. Temporal consistency
cannot distinguish coherent receiver bias from genuine global-alignment drift.

The map flag records rejected evidence without inserting a map factor; equality
of those arms does **not** validate accepted-map fusion. The two-capture run used a
future-informed initial rotation with unqualified physical up. It exposed
long innovation-rejection streaks and slow post-outage readmission; see the
[roadmap](docs/roadmap.md). It is not a complete causal or production pass.

Audited scalar CSVs can also drive the separate planar wheel-odometry
diagnostic. From the package root inside the managed test container:

```bash
python3 -m unittest discover -s test/integration/chassis_imu_dr \
  -p 'test_wheel_odometry.py'
python3 test/integration/chassis_imu_dr/wheel_odometry.py \
  --chassis <record>_chassis.csv --imu <record>_imu.csv \
  --pose <record>_pose.csv --steering-coefficient <frozen-other-record-value> \
  --output-dir <new-output-directory>
```

This compares chassis-speed/raw-gyro integration with chassis-speed/transferred
steering integration. It is not a `LocalEstimator` replay or a calibrated
vehicle model. Input gaps above 50 ms cause explicitly reported segment
reinitialization; reference pose is used only to seed each segment and score
it. Same-record localization comparisons are not independent accuracy evidence.

`PlanarWheelDr` is a separate private candidate for time-associated planar
body velocity and gyro-Z integration. It accepts an explicit local seed,
clock, gap bound and fixed gyro-bias compensation; it does not initialize
itself from GNSS, learn bias, publish qualified covariance, or replace the
15-state estimator. Nonzero body-lateral velocity can represent a qualified
rear-steering reference model. A gap invalidates the candidate until an
explicit diagnostic reseed. Callers remain responsible for sensor-time
association, frame/tilt eligibility, speed semantics and slip rejection.

The test-only C++ candidate and existing ESKF runner can be compared without
changing either production algorithm:

```bash
env -u LD_LIBRARY_PATH bazel build //:planar_wheel_dataset_replay \
  //:local_estimator_dataset_replay
python3 test/integration/chassis_imu_dr/compare_planar_replay.py \
  --apollo-root .agents/notes/ackermann_crosscheck_20261014 \
  --zhongji-root .agents/notes/chassis_imu_dr_calibration_20261013/forward_150054_rfu \
  --output-dir <new-output-directory> \
  --planar-runner bazel-bin/planar_wheel_dataset_replay \
  --eskf-runner bazel-bin/local_estimator_dataset_replay
```

This requires the existing frozen audit/CSV/baseline artifacts, not raw record
extraction. It verifies their hashes, reruns the Python control, checks C++/Python
Apollo parity, reproduces the Zhongji ESKF baseline byte-for-byte, and compares
both candidates at common initialized sensor times. Reference-unbracketed states
are explicitly listed, never extrapolated. `protocol.json`, `commands.json`,
`summary.json`, generated inputs, trajectories and paired scores are written to
a new directory. The original Apollo export lacks gyro X/Y and sensor_rgb
has insufficient stationary dwell; the completed-IMU experiment below
addresses these only under explicit diagnostic conditions. The Zhongji
planar input is a forward-only diagnostic, not a qualified rear-steering model.
See the [roadmap](docs/roadmap.md) for results and interpretation limits.

With the separately completed raw gyro-X/Y export, Apollo ESKF diagnostics
can also run in two explicitly conditional modes:

```bash
python3 test/integration/chassis_imu_dr/apollo_ekf_diagnostic.py \
  --root .agents/notes/ackermann_crosscheck_20261014 \
  --output-dir <new-output-directory> \
  --eskf-runner bazel-bin/local_estimator_dataset_replay \
  --planar-runner bazel-bin/planar_wheel_dataset_replay
python3 -m unittest discover -s test/integration/chassis_imu_dr -p 'test_*.py'
```

The required `full_imu_20261014/manifest.json` pins measured axes and proves
all previously exported IMU fields are unchanged. Mode A shortens only the
test runner's stationary dwell to 1 s and compares motion after startup.
Mode B uses the later quiet-start attitude/bias and the first chassis speed
as an explicit future-informed offline seed, to exercise the main curved
route. It is not an online moving-start implementation or calibrated
initial-state covariance. The new runner flags
`--diagnostic-stationary-duration` and `--offline-diagnostic-seed` exist only
in the test executable; without them the original default behavior is
unchanged. Neither mode established ESKF accuracy or statistical consistency.

The selected map artifact can be replayed through the standalone scan-to-map
component using the verified map keyframes, GNSS-off FAST-LIO2 ODOM export,
and projected RTK track. The replay adapter verifies the map inventory and
input provenance before setting the component's map-integrity declaration.
It writes scan diagnostics, a per-epoch comparison CSV, and a JSON run summary
to an explicit output directory:

```bash
env -u LD_LIBRARY_PATH bazel build //:scan_to_map_dataset_replay
python3 test/integration/map/run_scan_to_map_dataset_replay.py \
  --map-root /path/to/verified/map \
  --odom-csv /path/to/exported/odometry.csv \
  --frontend-manifest /path/to/manifest.json \
  --scan-association /path/to/scan_pose_association.json \
  --gnss-csv /path/to/gnss_pose3.csv \
  --gnss-source-csv /path/to/gps.csv \
  --gnss-contract /path/to/gnss_contract.json \
  --raw-gga /path/to/gga.txt \
  --runner bazel-bin/scan_to_map_dataset_replay \
  --output-dir /path/to/replay-output
```

The replay accepts independent `--scan-voxel-m`, `--map-voxel-m`, and
`--ndt-resolution-m` values, plus `--ndt-step-size-m`,
`--ndt-transformation-epsilon-m`, and `--ndt-max-iterations`. A map voxel
value of `0` inherits the scan voxel size; otherwise it must be positive.
The default PCL NDT settings are 0.5 m scan/map voxel, 1.0 m resolution,
0.1 m step size, 0.001 m transformation epsilon, and 40 iterations.

The 2026-10-15 two-capture startup pilot is recorded in the
[roadmap](docs/roadmap.md). All four NDT startups converged but were rejected
for insufficient observed map support; no causal trajectory or fusion replay
followed. NDT `overlap` counts nearest-target support with valid target normals,
not only geometric proximity. ICP-specific source/target overlap fields are
not populated by that NDT path.
The replay CSV additionally exposes post-voxel registration source/target
counts, `ndt_support_available`, distance-supported and normal-supported
counts, and the quoted rejection `detail`. NDT support counters are unavailable
on ICP or when rejection occurs before the observed-support stage.

An optional `--matcher point-to-plane-icp` selects OpenLoc's custom
point-to-plane ICP backend; NDT remains the default. ICP options are exposed
by the replay runner, including the packager-oriented registration settings.
For ICP, source overlap is the fraction of downsampled scan points with a
nearest target point within the correspondence distance; target overlap is
the fraction of points in the selected, downsampled and capped local map
window with a nearest aligned scan point within that distance. It is not a
fraction of the entire global map. ICP gates are independent:
`--icp-min-source-overlap` applies to the scan fraction, and
`--icp-min-target-overlap` applies to the local-map fraction. Both default to
the prior 0.4 baseline; lowering the target gate (for example to 0.25) is an
experimental setting, not a calibrated acceptance threshold.
`--prewarm-local-windows` constructs each distinct request window before
matching, provided the set fits the configured cache capacity. Its window
count and preparation time are reported separately from per-frame latency;
prewarming moves work to startup and does not reduce total construction cost.
The 2026-10-12 full-record diagnostic returned 19 `MATCHED` scans out of 196
and missed the provisional latency target; it is not a qualified localization
result. The run configuration, per-frame diagnostics, and limitations are
recorded in the
[map-assisted validation plan](../../ad_agents/context/modules/localization/knowledge/map-assisted-localization-validation-plan.md).
The follow-up ablation found that an experimental 0.25 target-overlap gate
raises the same-record `MATCHED` count to 129/196 but does not establish match
correctness; the 0.4 source and target defaults remain unchanged. Prewarming
four windows plus a 10-iteration cap measured 58.02 ms P95 / 71.73 ms max,
with fewer convergence flags (136/196 versus 145/196). No tested configuration
met both provisional latency goals (P95 <50 ms and max <100 ms). Exact
per-frame results and hash-pinned provenance are archived in the ignored
`scan_to_map_icp_followup_a17c39bf` result directory.
The replay runner also accepts `--profile-icp-timing` to emit local-window
construction, target voxelization/KD-tree/normal preparation, per-scan nearest
search, linearization, and solver durations in its per-frame CSV. It is
default-off and adds per-query timing overhead; use it to locate relative
stage costs, not to qualify latency. Normal-preparation totals are repeated
for frames sharing a cached window.

This adapter replays the map's frontend-deskewed keyframe PCD subset, not the
raw LiDAR bag. The selected map and GNSS track share the same recording and
GNSS contributes to map georeferencing; resulting RTK residuals are
same-record horizontal consistency diagnostics only. The configured antenna
lever arm remains unverified, and vertical accuracy is not evaluated.

The optional 3DMakerPro data smoke test uses a mounted session without copying
it into the repository:

```bash
OPENLOC_3DMAKERPRO_ROOT=/path/to/session \
  env -u LD_LIBRARY_PATH bazel test --test_env=OPENLOC_3DMAKERPRO_ROOT \
  //:openloc_3dmakerpro_smoke_test --test_output=errors
```

Without the environment variable, synthetic adapter tests run and the external
data case is explicitly skipped. See
[`docs/plan/3dmakerpro-data-smoke-test.md`](docs/plan/3dmakerpro-data-smoke-test.md)
for the tested session layout and limitations.

These software tests do not establish field accuracy, calibrated protection
levels, runtime performance, safety, or production readiness.
