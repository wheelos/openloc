# openloc

Middleware-independent localization algorithms and value contracts. Host
repositories own sensor adapters, middleware, process lifecycle, and actuation.

See [`docs/README.md`](docs/README.md) for documentation navigation,
[`docs/principles.md`](docs/principles.md) for functional goals,
[`docs/architecture.md`](docs/architecture.md) for library contracts, and
[`docs/roadmap.md`](docs/roadmap.md) for implementation status and evidence.

The package layout and ownership boundaries are defined in
[`docs/architecture.md`](docs/architecture.md). The online baseline and
extracted global-alignment candidate remain separate; neither is declared a
production default.

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

The graph/adapter and diagnostic API regressions now build and pass in the
managed development container. See the [roadmap](docs/roadmap.md) for exact
validation scope; this is not a production-readiness or recorded-data claim.

## Build and tests

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

Use `recovery` instead of `baseline` to enable recovery only; use `tracking`
to keep that same backend and opt into rate-limited publication. ODOM
schema is `timestamp_ns,px,py,pz,qx,qy,qz,qw`; fixes use
`arrival_ns,timestamp_ns,gx,gy,sigma,quality` (quality 0/1); the single-row
lever schema is `lx,ly,lz`. The mandatory diagnostic flag declares synthetic
receiver fields: fixed/invalid from the fixture quality flag, 20 satellites,
HDOP 0.8, differential age 0.2 s and horizontal accuracy 1 m. These are not
receiver measurements. The adapter uses a diagnostic 2 m accuracy limit and
2.5 s sparse ODOM interpolation bound; ordinary stability, consistency, rate,
NIS and publication limits remain unchanged. Neither override changes library
defaults. Supply body quaternions and a declared lever, not invented rotations.

Output includes every ODOM/fix event, original measurement and processing
times, backend/published body and antenna positions, full backend covariance,
correction-only jumps at a common ODOM epoch and publication/reset state.
Companion `.factors.csv` logs pending-to-final record updates by record index;
`.summary.csv` explicitly reports unresolved pending fixes and offline
runtime/peak RSS. Output files cannot overwrite previous results.
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
[map-assisted validation plan](../../wheelos-service/context/modules/localization/knowledge/map-assisted-localization-validation-plan.md).
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
