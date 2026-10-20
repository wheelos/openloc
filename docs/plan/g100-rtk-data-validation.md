# G100 LiDAR ODOM + GNSS Validation Plan

**Dataset:** `/mnt/synology/3dmakerpro/20261008141912/`

**Scope:** validate LiDAR/IMU local odometry and GNSS-aided global alignment.
A prior point-cloud map is not needed. Dataset/schema findings are maintained
in `/mnt/synology/3dmakerpro/schema/`; this plan tracks the validation
sequence and decision gates.

## Confirmed input evidence

- Files present: `LIDAR_20261008141912.bag`,
  `GPS_20261008141912.bag`, `RTK_20261008141912.bin`,
  `diff_data.rtcm`, `gga.txt`, calibration metadata, and image/thumbnail
  products. Only LiDAR/IMU and GNSS-related inputs are in scope.
- The paired ROS bags use ROS 1 bag v2.0. LiDAR is
  `/livox/lidar` (`livox_ros_driver2/CustomMsg`); IMU is `/livox/imu`
  (`sensor_msgs/Imu`). Use embedded sensor timestamps, not bag receive times.
- Checked LiDAR timestamps are Unix/POSIX UTC nanoseconds and agree
  numerically with GPS/NMEA timelines; this does not prove PPS wiring or
  absolute clock synchronization accuracy.
- Mid-360 profile `mid360-fast-lio2-jmk2-20261008` records raw IMU
  acceleration in `g`, gyro in rad/s, invalid all-zero orientation, and the
  active FAST-LIO2 adapter scale to m/s² (`9.80665`). Use the profile's exact
  FAST-LIO extrinsic convention/parameters; this is a nominal manual-derived
  transform, not a surveyed calibration.
- `rtk_factor_admission-20261008.md` records 162 `/GPSGGA` epochs reporting
  RTK fixed (quality 4), matching MSM4 corrections, and a GNSS-only
  reference-relative diagnostic. This does not establish calibrated
  uncertainty or independent absolute truth.
- The GNSS factor admission decision is currently **disabled** because
  uncertainty calibration and an independent reference are absent. GPS/PPS
  lock, absolute timing discipline, surveyed correction-station coordinates,
  and independent validation of the receiver antenna lever arm also remain
  unresolved. The user supplied `T_L_G = ([-0.12, 0, -0.04] m, I)`; this is
  nominal input, not a surveyed calibration.

The detailed source evidence and limitations are in:

- `/mnt/synology/3dmakerpro/schema/confirmed-findings.md`
- `/mnt/synology/3dmakerpro/schema/evidence-details.md`
- `/mnt/synology/3dmakerpro/schema/fast-lio2-mid360-profile-20261008.yaml`
- `/mnt/synology/3dmakerpro/schema/rtk-factor-admission-20261008.md`

Keep source bags and RTK artifacts read-only. Store derived outputs under the
offline-map repository's results directory, separate from the source data.

## Completed baseline evidence

### GNSS-off FAST-LIO2 replay

- The 163-second replay used LiDAR+IMU only; GNSS input/factors were disabled.
  The sensor/initialization gate passed. Configuration SHA-256
  `e5280ecb...ec4db2a` matches the selected profile. The exact FAST-LIO
  source revision, dirty diff, executable, input bag, and derived outputs are
  recorded in
  `/home/wfh/01code/map-infra/offline_map/results/20261008141912/fast_lio_odom_baseline_full_gnss_off/manifest.json`.
- The wrapper/container exited with code 137 during finalization; the exact
  cause is unknown, and there is no evidence establishing OOM. Playback had
  reached `Done.` at 162.989/162.998 seconds. The recorded bag was readable:
  162.395 seconds, 1,625 `/Odometry` and 1,625 `/cloud_registered_body`
  messages. It was exported and evaluated without replaying the input.
- The reference-free integrity gate **passed**: 1,625 poses, 162.401 seconds
  (99.63% coverage), maximum adjacent-pose speed 1.475 m/s, maximum timestamp
  gap 0.111 seconds, and no configured gross-speed failures. The 3D path was
  121.314 m with a 16.206 m Z range. This establishes finite, time-ordered
  output and gross continuity only; it is not an accuracy result. Runtime
  resource use and repeatability were not measured.

### Same-record GNSS consistency

- Compared all 162 GNSS reference epochs over 161 seconds at zero time
  offset, using a fixed SE(2) alignment fit over the first 60 seconds.
  Relative-translation error was: 1-second RMSE 0.111 m (p95 0.202 m),
  10-second RMSE 1.007 m (p95 1.913 m), and 60-second RMSE 2.720 m
  (p95 4.271 m). The fixed-alignment horizontal residual was RMSE 1.803 m,
  p95 4.150 m.
- The configured gross-trajectory sanity gate passed. These are
  **same-record, reference-relative consistency diagnostics**, not independent
  accuracy: GNSS uncertainty, PPS/absolute clock discipline, and antenna
  lever arm are unresolved. Zero timestamp offset was used, not calibrated.
  GNSS factors were not enabled.
- Full result:
  `/home/wfh/01code/map-infra/offline_map/results/20261008141912/fast_lio_odom_baseline_full_gnss_off/evaluation_odom_gnss_consistency_fullwindow.json`.

## Validation sequence

### 1. Reconcile the active runtime configuration

- Verify the specified FAST-LIO2 executable/source and
  `offline_map/ros_ws/src/mapping_baseline/config/fast_lio_3dmakerpro.yaml`
  against the profile's recorded hashes before replay.
- Confirm the actual `20261008141912` bag uses the profiled sensor identity,
  LiDAR/IMU topics and field semantics. Preserve `header.stamp`, LiDAR
  `timebase`, per-point `offset_time`, and IMU timestamps.
- Apply the profile's `g` to m/s² conversion exactly once for this FAST-LIO2
  runtime; never use all-zero orientation or external gravity/bias seeds.
- Confirm first-IMU initialization is suitable; do not assume the other
  recording's first samples represent this session.
- Retain point zero-return counts and configured tag/range filters in the
  run report.

**Gate:** executable/config hashes, input topics/types, sensor identity, and
resolved parameters match the profile; otherwise stop and reconcile before
running. The run's config and sensor contract passed. Its actual FAST-LIO
source/executable identity differs from the older profile diagnostic build
and is pinned in the run manifest; do not claim byte-for-byte parity with that
older build.

### 2. Produce the GNSS-off LIO baseline

- Run bounded LiDAR+IMU replay with GNSS factors disabled.
- First check startup/initialization, scan and IMU consumption, timestamps,
  point counts, finite output, and runtime stability. Do not silently trim or
  extrapolate missing trajectory intervals.
- If startup is healthy, record ODOM continuity, scan-rate trajectory,
  relative drift diagnostics, runtime, and resource use. Keep all output
  attributable to exact executable/config/input hashes.

**Gate:** a complete LIO ODOM trajectory exists with no unexplained time
gaps, resets, or non-finite output. The 2026-10-08 run passes the configured
reference-free trajectory integrity gate; deterministic repeatability and
runtime resource limits remain untested.

### 3. GNSS reference-relative diagnostic

- Reuse the recorded GNSS-only diagnostic for track coverage and receiver
  reported status. Do not treat quality 4 as a calibrated covariance.
- Align the GNSS track to the LIO trajectory only with an explicit local ENU
  origin, time association, and declared antenna-to-estimator reference.
- Report residuals/consistency only; the same GNSS stream cannot be both an
  independent truth source and a factor used to estimate the trajectory.
- Quantify the effect of unresolved antenna lever arm and timing on residuals
  before interpreting a discrepancy as LIO drift.

**Gate:** the comparison is time/frame consistent and labeled
non-independent. The current same-record comparison passes only the configured
gross sanity gate and does not verify GNSS aiding or absolute accuracy.

### 4. Isolated OpenLoc GNSS-aided algorithm diagnostic — completed; provenance pending

The user authorized an isolated OpenLoc diagnostic replay on
`20261008141912` with provisional 1.0 m horizontal sigma. This is not
production admission and must not be used to claim absolute accuracy or
improvement. Use the GNSS-off FAST-LIO2 ODOM export and same-record GGA
payload-time coordinates; do not use the offline PGO trajectory as input or
fit a clock offset.

The data-driven OpenLoc adapter uses the user-provided LiDAR-frame lever arm
composed with the profile's `T_I_L` to express the GNSS antenna in FAST-LIO
`body`/IMU coordinates: `[-0.131, -0.02329, 0.00412] m`. The source lever arm
and body-frame relationship remain unverified. The adapter alone uses a
120 ms maximum ODOM interpolation gap, above the observed maximum GNSS/ODOM
bracket of 105.135 ms and maximum adjacent ODOM interval of 110.811 ms;
larger gaps remain rejected. No timestamp shift is applied. Vertical GNSS
variance is a `1e6 m²` placeholder, so only horizontal same-input consistency
is reportable.

The first run stopped at the library's default 100 ms bracket limit before
processing GNSS updates; no GNSS factor was fused in that attempt. The
120 ms test-only bound is data-driven, not a change to the library default.
With that bound, the OpenLoc diagnostic processed all 1,637 ODOM samples and
162 GNSS epochs; the maximum observed GNSS/ODOM bracket was 105.135 ms.
OpenLoc accepted 148 fixes, rejected 3 because the fixed solution had not yet
met the 3-second stability requirement, and rejected 11 for GNSS/ODOM distance
disagreement. Global alignment became valid. The 147 accepted non-initial
prefactor residuals had horizontal median 0.799 m, p95 1.868 m, and maximum
2.453 m. These are same-input innovation diagnostics after gating, not
independent accuracy or a calibrated uncertainty/NEES result. The run used
the provisional 1.0 m horizontal sigma, `1e6 m²` vertical placeholder, and
unverified composed lever arm; production admission remains false. Result
JSON: `/tmp/openloc_gnss_diagnostic_20261008/openloc_result.json` in the
managed test container.

Production admission requires independent route reference or calibrated GNSS
uncertainty, verified timing/PPS semantics, and the GNSS antenna lever arm.
Accepted GNSS must update only `T_enu_odom`; LIO ODOM must remain unchanged.

### 5. OpenLoc global-alignment algorithm validation — current workstream

The selected frontend for this workstream is the existing, pinned
GNSS-off FAST-LIO2 ODOM baseline. Do not develop or run OpenLoc's native L1
bootstrap as part of this validation. OpenLoc consumes the FAST-LIO2
`camera_init -> body` trajectory and same-record `/GPSGGA` fixes; the
middleware-neutral global estimator may update `T_enu_odom` only. The supplied
antenna lever arm composed with profile `T_I_L` remains a conditional,
unverified diagnostic input.

The latest schema evidence strengthens input handling but does not change
admission:

- Associate GNSS using embedded GGA payload UTC time, not the delayed ROS
  bag-record/publication timestamp. The raw RTK NMEA sentence cross-check
  supports this interpretation for the audited delayed sample.
- All 162 GGA payload epochs are inside the FAST-LIO2 ODOM interval. Nearest
  scan-end residual is 0.000016 s median signed, 2.450 ms P95 absolute, and
  11.678 ms maximum absolute, with no offset fitted. This is sampled
  time-axis agreement, not PPS/clock-discipline or absolute-time accuracy.
- FAST-LIO2 internally estimates gravity direction and gyro bias from early
  IMU samples and has no external seed in the selected profile. This supports
  the already completed GNSS-off frontend baseline only; it is not an
  OpenLoc initializer or an independent accuracy measurement.
- The receiver's quality-4 status, RTCM/NMEA parsing, and a valid timestamp do
  not provide calibrated GNSS covariance or independent truth. Keep the
  1.0 m horizontal sigma and `1e6 m²` vertical placeholder explicitly
  experimental.

#### Plan and gates

**P0 — Freeze the GNSS-off frontend baseline (complete).** Reuse
`fast_lio_odom_baseline_full_gnss_off/` and its manifest. Do not replay source
LiDAR/IMU or use the GNSS-fused PGO trajectory as OpenLoc input. Preserve
source bags/RTK files read-only.

**P1 — Close provenance for the prior OpenLoc diagnostic (pending).** Retrieve
the diagnostic result JSON, exact command/environment, and code/config/input
identity from the managed test container; archive them in the ignored
results area. If temporary artifacts are no longer available, record their
unavailability; do not rerun only to recreate them.

**P2 — Verify global-alignment software invariants (synthetic coverage exists;
integrated evidence to review).** Against
`T_enu_body = T_enu_odom * T_odom_body`, verify:

- a qualified position fix can establish only the declared global alignment;
  position-only input does not invent an independently observed heading;
- fixes are associated to historical ODOM at payload measurement time and
  outside-history, duplicate, regressing, or over-gap fixes fail closed;
- invalid status/quality, covariance, and innovation are rejected with
  explicit per-fix reasons;
- accepted GNSS changes only `T_enu_odom`; FAST-LIO2 ODOM samples and later
  local trajectory output remain unchanged;
- global uncertainty grows through outages, validity/recovery follows the
  specified lease and verification policy, and recovery does not reset ODOM.

Keep the library's default maximum ODOM interpolation gap at 100 ms. The
previous data diagnostic used 120 ms only in its test adapter because observed
adjacent ODOM gaps reached 110.811 ms and the largest GGA/ODOM bracket was
105.135 ms. That experiment must remain separately identified, with larger
gaps rejected; it is not a general algorithm default.

**P3 — Repeat/extend the data-driven diagnostic (requires explicit approval
for a new GNSS-on replay).** Before execution, freeze input hashes,
implementation/config revisions, the 120 ms adapter-only bound, experimental
noise assumptions, exact outputs, and acceptance checks; then obtain explicit
user approval. Replay existing GNSS-off FAST-LIO2 ODOM in timestamp order,
submit GGA at payload time, account for every fix, and report per-reason
counts, alignment/validity, covariance, recovery behavior, and ODOM
pre/post-identity. Separate measured same-record consistency from synthetic
fault injection (poor status, delay/missing epochs, innovation outlier,
outage, and recovery). Do not fit a clock offset, silently extrapolate, call
GNSS truth, or claim accuracy gain/NEES calibration.

**P4 — Production admission (blocked on external evidence).** Keep GNSS
production fusion disabled until independent route truth or held-out
uncertainty calibration is available, PPS/absolute timing is verified, the
antenna lever arm and uncertainty are surveyed, and FAST-LIO2 `body`/IMU
semantics are confirmed. Reassess quality and innovation thresholds only
against that evidence.

#### Current status

- GNSS-off FAST-LIO2 baseline integrity gate: passed; deterministic
  repeatability/resource qualification remains open.
- OpenLoc global-alignment synthetic tests and one explicitly authorized
  same-record diagnostic: completed. The diagnostic processed 1,637 ODOM
  samples and 162 GGA epochs; 148 accepted, 3 rejected before stable-fixed
  initialization, 11 rejected by distance consistency; global alignment
  became valid. Gated residuals are same-input consistency only.
- Original diagnostic JSON provenance: unavailable on the host; the formerly
  reported managed-container `/tmp` path is absent. The 1,637-row P1-1 report
  below is a new, deterministic host reconstruction, not the original artifact.
- Independent accuracy, calibrated uncertainty, absolute timing, verified
  antenna extrinsic, and production GNSS admission: not established.

OpenLoc native L1 remains a separate, blocked future workstream. The existing
sensor screen found no qualifying 1/3/5/10-second quiet window under gyro
p95 < 0.05 rad/s and acceleration-norm standard deviation < 0.02 g. Do not
weaken those gates or use this current global-alignment validation to claim
OpenLoc L1 validation.

## User-specified Phase 0–3 plan: execution evidence

This status follows the supplied A–D tasks and Phase 0–3 decision order.
P0-1, P0-2 software contract, P0-3, P0-6, P1-1, ODOM input reconciliation,
and per-epoch GPS 1074 association were advanced. No new GNSS-on replay or
graph change was performed. The original OpenLoc result JSON previously
reported under the managed container's `/tmp` is unavailable; the 1,637-row
P1-1 result below is a separate, reproducible reconstruction from the pinned
exports.

All same-record comparisons below are **consistency diagnostics**. No
sensitivity sweep was performed in this update, and no independent truth or
calibrated covariance is available. Keep `calibration_state=uncalibrated`.

## Next work: two parallel evidence tracks

The immediate algorithm objective is the **minimum odometry + global
correction loop**, not production GNSS qualification. A diagnostic GNSS input
may be used with explicit, uncalibrated assumptions and rejection reporting.
This does not enable GNSS in FAST-LIO2, establish GNSS truth, or admit factors
for production. Do not repeat the completed A–D export audit, alter the
4DoF state graph, tune gates to maximize acceptance, or enable a gravity
leveling transform.

### Minimum algorithm loop — implementation present

```text
FAST-LIO2 GNSS-off T_odom_body
    -> timestamp-ordered alignment states A_k
    -> odometry-derived random-walk factors between A_k
payload-time GGA position
    -> interpolate T_odom_body at the measurement time
    -> antenna lever-arm model
    -> quality / covariance / innovation checks
    -> position factor on A_k
GTSAM fixed-lag optimization
    -> T_enu_odom and alignment covariance
    -> T_enu_body = T_enu_odom * T_odom_body
```

The current `LocalizationEngine`/`AlignmentSmoother` implements this
middleware-neutral loop. It does not modify, reset, or feed back into
FAST-LIO2 ODOM. Position-only GGA does not claim to observe full orientation;
the graph remains the existing four-DoF alignment chain. Invalid status,
reference mismatch, bad covariance, unavailable ODOM brackets, stale/out of
order data, and excessive innovations remain explicit rejection paths.
Outages grow global-alignment uncertainty while local ODOM continues, and
accepted fixes can reacquire global alignment.

GNSS certainty is not a prerequisite for this diagnostic algorithm loop.
Instead, its input state and covariance provenance must be explicit:
`diagnostic_only`, `calibration_state=uncalibrated`, provisional horizontal
sigma 1.0 m, vertical variance placeholder `1e6 m²`, and the conditional
user-supplied lever arm. Receiver-reported quality is used for gating, not as
proof of accuracy. Results are same-record consistency only.

Existing software evidence covers first-fix initialization, payload-time
ODOM interpolation, lever-arm application, ODOM immutability, status and
innovation rejection, outage covariance growth, recovery, and deterministic
replay. The previously completed 20261008141912 diagnostic processed 1,637
GNSS-off ODOM samples and 162 same-record GGA epochs: 148 accepted, 3 rejected
before stable-fixed initialization, and 11 rejected by the distance gate.
That demonstrates an algorithmic data loop under the stated assumptions, not
accuracy or production admission. Its original result JSON is unavailable;
the reconstructed rejection report is retained separately.

**Algorithm-loop exit:** focused online tests pass and the diagnostic run
reports all input/factor decisions while preserving the ODOM stream. This
gate is independent of the production-evidence gates below.

### Architecture review and test interpretation (2026-10-11)

**(a) Review result:** the estimator ownership and factor-graph split are
structurally appropriate for the minimum loop: FAST-LIO2 remains GNSS-off and
owns local `T_odom_body`; OpenLoc's fixed-lag 4DoF chain estimates only
`T_enu_odom`; GNSS position factors affect global alignment; the output
composition is `T_enu_body = T_enu_odom * T_odom_body`. This review did not
change graph structure or run a new data replay.

There is, however, an unclosed coordinate-frame precondition. A yaw-only 4DoF
alignment assumes the ODOM axes are gravity-level (or that the host adapter
applies a known, validated fixed leveling rotation before the estimator).
The selected FAST-LIO2 schema identifies the exported frame as
`camera_init -> body` and does not guarantee `camera_init` is horizontal.
`LocalizationEngine::CheckOdometryConsistency` currently compares
`odometry_delta.head<2>()` to ENU `gnss_delta.head<2>()`, and its direction
check rotates the ODOM delta by yaw about +Z only. Those comparisons are not
frame-consistent if ODOM has roll/pitch relative to the horizontal frame.
This invalidates the horizontal gate/yaw interpretation for an unlevelled
input; it is a plausible contributor to the data rejections, not evidence
that it caused any particular rejection. Keep the existing 4DoF graph, make
the input frame precondition explicit, and add a synthetic nonzero-roll/pitch
regression before treating the gate as valid. Do not enable a leveling
rotation until P0-3 passes.

**(b) Regression coverage:** the current online tests cover the alignment
contract in axis-aligned synthetic cases, including ODOM immutability,
interpolation, factor rejection, outage uncertainty, and recovery. They do
not test tilted ODOM axes against ENU horizontal distance/direction gates.
`LocalizationResult` exposes global validity, alignment covariance and
factor decision records, but has no explicit observability or
`calibration_state` field; that remains an output-contract gap against the
required reporting contract. The P0-2 focused test initially called
`GetResult` before supplying any ODOM, correctly receiving state-unavailable;
the fixture was corrected to add ODOM before querying. The subsequent focused
test run passed, so this was a test precondition error, not an estimator
failure.

**(c) Data diagnostic:** the existing reconstruction uses 1,637 GNSS-off
ODOM samples and 162 selected same-record GGA epochs: 148 accepted, 3 rejected
before stable-fixed initialization, and 11 rejected by the distance gate.
The 11 are all receiver quality 4, with ages 1 s for 10 and 2 s for 1; their
distance-gate excess is only 0.0022–0.0549 m (median 0.0162 m) in the
reconstruction. These are same-record consistency facts, not independent
truth. All 11 root causes remain `UNRESOLVED`; GNSS quality fields do not rule
out measurement error, and the frame precondition issue is not enough to
assign blame to the model or ODOM.

**(d) Decision-gate answer:** the high-level architecture is suitable for the
requested minimum loop, but the current raw-ODOM integration is not fully
validated because the level-frame precondition and required observability /
calibration output fields are not closed. Synthetic test success proves the
tested software contracts only; it does not validate the production frame,
measurement model, absolute accuracy, or covariance calibration. Next
algorithm work should first add the tilted-frame regression and explicitly
enforce/represent the adapter frame transform without changing the 4DoF
graph; production use remains gated on the independent P0 evidence.

### Track A — close input and coordinate-frame contracts

1. **P0-2 configuration/startup contract:** `MapReference` now carries
   horizontal CRS realization, decimal-year coordinate epoch, vertical datum,
   and geodetic ENU origin. `Configure` requires complete, valid declarations
   for a verified reference and rejects unspecified state; GNSS fixes and
   headings must match the complete reference identity. An explicit
   `diagnostic_only` state permits unresolved fields for bounded diagnostics
   and is preserved in results. For this dataset, the schema records the
   diagnostic projection as UTM EPSG:32650, the local origin as the first GGA,
   and ellipsoidal height via GGA `H+N`; the diagnostic reference now carries
   those known values. The GNSS source's precise CRS realization and
   coordinate epoch, geoid-model provenance, and an independently surveyed
   origin remain unconfirmed. Do not present this diagnostic origin as a
   surveyed production reference; G-P0 remains open.
2. **P0-3 runtime gravity evidence:** the recovered estimator `mat_pre`
   snapshots are internal FAST-LIO state evidence only. A follow-up must pair
   timestamped runtime gravity state with a physically confirmed stationary
   interval, source/frame/sign documentation, multiple-window stability
   statistics, and an independent physical up reference. Until that evidence
   is available and transformation direction is verified, retain
   `UNRESOLVED` and do not enable `R_level`.
3. **P0-4/P0-5 frame and lever arm:** confirm FAST-LIO's actual published
   `body` origin against the physical IMU; document `T_I_L` and `T_L_I`
   directions with a non-identity composition test. Independently measure
   IMU-origin-to-antenna-phase-center lever arm, attach measurement method,
   uncertainty, profile identity, and records/photos, then check residual
   direction against turns. The reported user lever arm remains conditional,
   not a measurement.
4. **P0-7 station/height provenance:** obtain the correction station's
   surveyed coordinate source, datum/epoch and vertical reference, and an
   independent rover height/control reference. RTCM 1005 values alone only
   establish the coordinates advertised in the stream.

**Track A / production gate:** G-P0 remains closed until required coordinate
declarations are explicit and the frame, gravity, lever-arm and
base-coordinate evidence is reviewed. A verified reference with any missing
declaration must fail closed; only the explicitly selected diagnostic-only
state may omit unresolved fields. This does not block the minimum diagnostic
algorithm loop and cannot promote its current origin into a production
reference.

### Track B — diagnostics and independent truth

- The source
  `/mnt/synology/3dmakerpro/20261008141912/LIDAR_20261008141912.bag` was
  inspected read-only with the ROS 1 bag reader. It contains
  `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`, 1,640 messages) and
  `/livox/imu` (`sensor_msgs/msg/Imu`, 32,799 messages), 34,439 messages
  total. It contains no `/Odometry`, registered-cloud, FAST-LIO health,
  effective-point-count, or covariance topics. The raw input is sufficient
  for a possible GNSS-off diagnostic replay, but it does not preserve the
  runtime FAST-LIO internal diagnostics needed for historical per-rejection
  health attribution.
- Before any new replay, freeze an instrumented FAST-LIO GNSS-off protocol
  and obtain explicit approval. Preserve the original sensor bag unchanged;
  pin executable/config/source/input hashes and record timestamp-aligned
  scan counts, effective point counts, estimator health/degeneracy indicators,
  and covariance only where the estimator exposes a well-defined value.
  Label unavailable or uncalibrated telemetry rather than inventing a health
  score. Re-run the same 1,637-row/162-epoch diagnostic only if telemetry
  changes an evidence question; do not rerun merely to regenerate the A–D
  outputs.
- Prepare a static control-point capture with a surveyed point, declared
  datum/epoch/vertical reference, antenna reference point, and receiver raw
  observations. Define occupation and acceptance thresholds with the survey
  provider before capture; no universal duration or threshold is assumed
  here.
- Prepare dynamic PPK truth by arranging synchronized raw rover carrier-phase
  observations and base observations (or a qualified reference service),
  surveyed base coordinates, antenna calibration/phase-center information,
  and trajectory time/coordinate metadata. The currently audited `.bin`,
  RTCM stream, and same-record GGA are not independent PPK truth; the mounted
  schema records no verified RINEX observation/navigation pair or completed
  PPK solution.
- Use only independently surveyed/PPK-qualified data to investigate whether
  future discrepancies are GNSS-, ODOM-, or model-related and to calibrate
  covariance. NEES/NIS evaluation remains blocked until independent truth,
  a stated covariance model, and a predeclared evaluation protocol exist.

**Track B gate:** no root-cause label beyond `UNRESOLVED`, absolute accuracy
claim, or covariance-calibration claim until the corresponding telemetry or
independent truth is available.

### Test-result reporting boundary

Completed software evidence supports only that the audited scripts and
diagnostic processing paths behave as tested: focused Python audit tests,
the raw-GGA C++ diagnostic-adapter test, four documented OpenLoc online Bazel
targets, C++ formatting for the changed C++ file, and result-integrity
assertions passed. Full-repository lint was blocked by unrelated existing
Clang-format violations. Python format/lint checks were not run because
Black, isort, and Flake8 were unavailable in the managed containers. These
results do not establish production GNSS ingestion, physical coordinate or
lever-arm correctness, localization-model correctness, independent accuracy,
or production readiness.

### Phase 0 — input correctness

| ID | (a) Change / (b) diagnostic | (c) Acceptance | (d) decision |
|---|---|---|---|
| P0-1 | Added a raw-file GTest for the existing test-only C++ GGA adapter and ran it against `/mnt/synology/3dmakerpro/20261008141912/gga.txt`. It parsed/checksummed the raw GGA rows and looked up the `06:27:55` payload epoch through the same C++ `ReadGgaEllipsoidalHeights` path used by the dataset diagnostic. The sample is `H=93.5572 m`, `N=-2.0266 m`, `h=91.5306 m`; the first selected epoch `06:27:56` is `91.5139 m`. | The targeted C++ GTest passed against the mounted real file. Independent arithmetic, units, raw checksums, duplicate consistency, and the 162 payload-time joins also pass. This proves the diagnostic adapter's parse/conversion path, not a production middleware adapter or independent altitude truth. | The P0-1 data-path check passes. G-P0 remains **not passed** because datum/fixed-origin startup checks, frame semantics, and surveyed lever arm remain unresolved. |
| P0-3 | Reconstructed the initialized FAST-LIO state from the first processed IMU batch and recovered the existing run's read-only `mat_pre` state log; no estimator/source/config was changed and no replay was required. The report contains all 20 payload timestamps and vectors, source hashes, runtime-state snapshots, and rotation checks at `results/20261008141912/gravity_p0_3_20261010/gravity_frame_audit.json`. | The `32.747263 deg` value is `acos((mean_acc / ||mean_acc||)_z)`, comparing inferred up with camera_init `+Z`; it is not a world-frame leveling rotation. The batch is raw `/livox/imu`, `livox_frame`, acceleration in `g`, timestamp span `95.706 ms`; the runtime adapter scales by `9.80665` to `/mapping/imu_si` in m/s². FAST-LIO source initializes `grav=-mean_acc/||mean_acc||*9.81` but does not rotate the initial attitude. Runtime `mat_pre` has 1,627 pre-update states: the first gravity differs from source reconstruction by at most `0.00084 m/s²`, its norm stays `9.808993–9.809007 m/s²`, and up changes at most `0.9512 deg` from the first row. It has 1,627 rows versus 1,625 ODOM poses, so no row-wise pairing is claimed. A separate physically confirmed static interval and surveyed gravity/frame reference are unavailable. | **UNRESOLVED**, not FAIL: runtime/source are internally consistent, but physical gravity and frame semantics lack independent static/reference evidence. The provisional rotation passes `RᵀR=I` (Frobenius error `1.95e-17`), `det(R)=1`, maps inferred gravity to `-Z`, and leaves zero translation origin fixed. Do not add it to the profile or enable it; `0.2 deg` is not an activation threshold. Source/runtime provenance is pinned by the selected run manifest: FAST-LIO revision `7cc4175de6f8ba2edf34bab02a42195b141027e9`, worktree-diff SHA-256 `760f3c4b4b15fc8f93c8a95a0830ad341a8c7b21d237f651508674bbb3f18d7f`, executable SHA-256 `07f74b1cb33cca0a5123fc30ca9ead93eb011514b87c32c0e628a5867d0fa484`, `IMU_Processing.hpp` SHA-256 `a456016e268680555919f58ae066354fd297a27f49be197292452f4147e1e0f5`, `laserMapping.cpp` SHA-256 `5ab48570ed4d9754ad13433af24064d875d0f3bf9b0f82ff7c7d7599fd11f327`, and config SHA-256 `e5280ecbaa721067f12117cfd0b8db73a539c7804adb897d566982c99ec4db2a`. The saved `mat_pre` file timestamp is 0.405 s before this run's odometry-bag completion; it is now copied with hash into the diagnostic artifact directory. |
| P0-6 | Recomputed raw-row and unique-epoch quality/age/satellite/HDOP statistics from `gga.txt`; cross-checked against `rtk_quality_report_20261008.json`. All 546 checksum-valid rows have quality, differential age, satellites, and HDOP; no missing values were observed. Duplicate quality-4 epoch ages agree exactly. | 546 sentences reduce to 261 unique payload epochs (quality 4/2/5: 252/8/1). Of 461 quality-4 raw rows, all carry age; the 252 unique quality-4 epochs have ages: 1 s: 156, 2 s: 63, 3 s: 5, 4 s: 1, 5 s: 1, 6–12 s: one each, 14–21 s: one each, 28–36 s: one each, 53 s: 1, 54 s: 1. Thus `>5 s=26`, `>10 s=21`; unique-epoch age min/max 1/54 s. Satellites min/median/max `11/32/37`; HDOP `0.5/0.6/1.4`. In the selected 162-epoch bag overlap, all are quality 4, ages 1/2/3 s = 120/40/2, `>5 s=0`, `>10 s=0`, satellites `18/31/37`, HDOP `0.5/0.6/1.4`. Raw `.bin`: 911 valid RTCM frames; `diff_data.rtcm`: 1,710 CRC-valid frames, including 276 GPS 1074 messages; GPS 1074 has one 39 s gap from 06:31:09 to 06:31:48 UTC, after selected GGA/ODOM overlap. | P0-6 passes as descriptive statistics only. Receiver-reported quality/age and RTCM continuity do not measure position error or provide calibrated covariance. |
| P0-2 | Added geodetic-reference metadata to `MapReference` and fail-closed `LocalizationEngine::Configure` validation. Verified references require CRS realization, decimal-year coordinate epoch, vertical datum, and finite in-range ENU-origin coordinates. GNSS fixes/headings must match the complete reference, including geodetic metadata. The dataset diagnostic declares EPSG:32650, its first-GGA origin (`22.7152590047° N, 114.3831726317° E`, ellipsoidal height `91.5139 m` from GGA `H+N`), and leaves the unverified coordinate epoch unset under the explicit `diagnostic_only` state. | All four documented online Bazel targets pass; `online_gnss_alignment_dataset_diagnostic_test` compiles without running the data replay. Tests cover omitted/invalid verified declarations, diagnostic-only propagation, and rejection of a mismatched vertical datum. Schema-backed diagnostic projection/origin/height are represented; precise source CRS realization/epoch and independent survey provenance are not established. | **Diagnostic reference is explicitly represented; production evidence remains UNRESOLVED.** G-P0 is not passed and diagnostic-only references are not production-admissible. |
| P0-4/P0-5 | Traced FAST-LIO publisher `publish_odometry`: `/Odometry` is `camera_init -> body`, stamped at `lidar_end_time`; the profile defines `T_I_L` from LiDAR coordinates into IMU coordinates. Conditional composition gives `ell_{I->A}=t_I_L+R_I_L t_L_G=[-0.131,-0.02329,0.00412] m`. `body == IMU` remains an unverified runtime semantic. | No independent body/IMU semantic check, external-frame composition test against a measured fixture, physical lever-arm survey, uncertainty record, or turn-residual validation is available. | **UNRESOLVED**; retain the lever arm as conditional and uncalibrated. |
| P0-7 | Decoded the repeated RTCM 1005 station-271 reference ECEF coordinates and compared their derived ellipsoid height with the selected rover GGA `H+N`. | RTCM 1005 has one unique ECEF tuple across 27 messages: `[-2429517.1522, 5361839.4008, 2447213.4855] m`, corresponding to approximately `36.000797 m`; first selected rover GGA is `91.513900 m`, a `55.513103 m` difference. The byte-level coordinate and arithmetic are internally consistent with the prior parser result. | **UNRESOLVED**: RTCM 1005 proves what the stream advertises, not the surveyed station location. No independent base survey provenance or rover height reference distinguishes actual terrain separation from a stale/incorrect base coordinate. |

The diagnostic `R_level` is the minimal source-to-level rotation derived from
the reconstructed up-vector `[-0.540930, -0.002247, 0.841065]`; it has no
geographic-north yaw:

```text
R_level =
[[ 0.841068, -0.000660,  0.540930],
 [-0.000660,  0.999997,  0.002247],
 [-0.540930, -0.002247,  0.841065]]
```

It maps inferred up to +Z and the corresponding negative gravity vector to
`-Z`. The matrix is a pure rotation about the unchanged zero origin and passes
orthogonality/determinant checks. It is not an ENU transform and is not
authorized for runtime use. The archived runtime state is an estimator
consistency check, not independent physical truth.

### Phase 1 — existing-data attribution

| ID | (a) Change / (b) diagnostic | (c) Acceptance | (d) decision |
|---|---|---|---|
| P1-1 | Extended the read-only `test/integration/3dmakerpro/attribute_distance_rejections.py` with adjacent accepted epochs, motion/distance comparisons, unavailable LIO and vertical/frame diagnostics, and evidence-based `UNRESOLVED` attribution. Added `test/integration/3dmakerpro/audit_dataset_provenance.py` plus focused Python tests; these generate the 12-row ODOM suffix and 162-row GPS-1074/GGA association artifacts. Rebuilt all 162 decisions from the complete 1,637-row production-capture GNSS-off ODOM export, without row filtering. | The 1,637-row run reproduces 148 accepted, 3 stable-initialization rejects, and 11 distance rejects, with zero quality rejects. The 1,625-row baseline is an exact prefix: every common timestamp and seven pose fields are identical, with no baseline-only rows; the 12 full-run-only rows are a contiguous tail after the shorter 163.0 s recorded segment. For all 162 selected epochs, nearest GPS 1074 after GPS-week/TOW-to-UTC normalization is station 271 and has 0 ms encoded epoch difference; GGA age remains a distinct receiver-reported field. All 11 rejection epochs have a corresponding row in the RTCM association table and remain `UNRESOLVED`. | G-P1 remains **inconclusive**: receiver-reported quality 4 and matching GPS MSM4 do not prove correction validity or independent truth; exported data lack IMU acceleration, LIO health/point count/covariance, surveyed lever arm, and independent truth. No root cause is asserted. |

#### Reproducible audit artifacts and provenance

All paths below are derived outputs under the ignored map-infra results tree;
the source bags and RTK files remain read-only.

- ODOM reconciliation: `results/20261008141912/openloc_odometry_reconciliation.json`
  and `openloc_odometry_excluded_tail.csv`. The latter has the 12 omitted rows,
  timestamps, poses, and per-row exclusion explanation. The 1,625 baseline CSV
  SHA-256 is
  `9e7618643555fa8acb0403c1809840c27f62ca2ac2e3ffc4a17b5c04ed5825cf`;
  the 1,637-row CSV SHA-256 is
  `311be3df17dce70d5e0b01684dbc340209876d35f98b7c7cee1b9f0cb92bb802`.
  The common 1,625 timestamps and pose fields match exactly; the additional
  rows span `1791441038.0002563`–`1791441039.1004221` Unix seconds. The
  manifests record segment durations 163.0 s and 163.997 s respectively.
  Thus the 12-row difference is consistent with the shorter baseline output
  horizon, not estimator row filtering. Exact original runner invocation was
  not preserved in its manifest/log; do not infer unrecorded flags.
- Per-epoch RTCM association:
  `results/20261008141912/openloc_rtcm_gga_epoch_association.csv` (162 rows)
  and `openloc_rtcm_gga_epoch_association.json`. Inputs are pinned there:
  raw `gga.txt` SHA-256
  `0db40e817b1488ebd8aa6940ec775a45dee2ef63981c8456289d63f48611cc00`,
  `diff_data.rtcm` SHA-256
  `ce94f385b33d2e48bde3cf5ccbd8cadf97e04a78e6a46893e846c122a8d9a15d`,
  GPS CSV SHA-256
  `738eddc258bc49ace0957e942aace7b41679c6af91f5e2eedbb971ca180a6997`,
  and ENU reference SHA-256
  `68ea0c8709b76ceccf083f475f6cd415fc39b9efb28ada0ff9f1c79e8f0073fb`.
  The table separates GGA payload UTC, receiver-reported differential age,
  raw GPS 1074 week/TOW, normalized UTC, station/type, byte offset, and match
  method. RTCM transport/arrival timestamps are unavailable; byte offsets show
  frame order only. GPS-UTC `18 s` is used for the recorded date; the GGA field is
  0.01-second resolution and RTCM TOW is 1 ms. This discrete-field match is
  not PPS evidence, clock-accuracy validation, or proof the rover applied a
  particular correction. GLONASS/Galileo/QZSS/BeiDou MSM are not assigned
  per-epoch matches because their time encodings were not normalized here.
  RTCM 1005 is station metadata without an epoch timestamp; 1006 is absent.
  All 1,710 parsed correction-stream frames passed CRC; each observed MSM
  station ID and all 27 RTCM 1005 messages use station 271.
- Per-rejection report:
  `results/20261008141912/openloc_p1_1_rejection_attribution_1637_host.json`.
  It includes the adjacent accepted epochs, quality/age/satellites/HDOP,
  ODOM speed/yaw/bracket/distance residuals, RTCM epoch association, and
  explicit unavailable IMU/LIO/covariance/frame metrics for each of 11 rows.
  The exact reconstruction command is recorded in this JSON. OpenLoc HEAD at
  audit time was `5926abe78de59e6f327e12aec622c44771fb005f`; map-infra HEAD
  was `be727669d5e10a082eb16fc59cc3a3282816a6bf`. FAST-LIO baseline revision
  was `7cc4175de6f8ba2edf34bab02a42195b141027e9`; the production manifest
  records source SHA-256
  `5ab48570ed4d9754ad13433af24064d875d0f3bf9b0f82ff7c7d7599fd11f327`
  but no revision string. ODOM/GNSS joins use all 162 strict, ordered epochs;
  no timestamp fitting, epoch thinning, or row filtering was applied.
  Dataset-audit script SHA-256:
  `6ae83cd586d07048bff61e4e1477c4daa2117c9010a44df15e3ef77906485148`.

| UTC epoch | `d_odom` / speed | `d_GGA` | error / gate | ODOM angular rate | GGA quality / age / sats / HDOP | LIO health |
|---|---:|---:|---:|---:|---|---|
| 06:29:10 | 1.152 m / 1.152 m/s | 1.285 m | 0.133 / 0.123 m | 0.120 rad/s | 4 / 1 s / 22 / 1.2 | unavailable |
| 06:29:12 | 1.094 m / 1.094 m/s | 1.241 m | 0.147 / 0.122 m | 0.073 rad/s | 4 / 1 s / 21 / 1.2 | unavailable |
| 06:29:21 | 1.017 m / 1.017 m/s | 1.140 m | 0.123 / 0.120 m | 0.087 rad/s | 4 / 1 s / 20 / 1.2 | unavailable |
| 06:29:24 | 0.937 m / 0.937 m/s | 1.102 m | 0.165 / 0.119 m | 0.696 rad/s | 4 / 1 s / 20 / 1.3 | unavailable |
| 06:29:46 | 0.786 m / 0.786 m/s | 0.918 m | 0.132 / 0.116 m | 0.368 rad/s | 4 / 2 s / 32 / 0.6 | unavailable |
| 06:29:52 | 1.191 m / 1.191 m/s | 1.322 m | 0.131 / 0.124 m | 0.007 rad/s | 4 / 1 s / 28 / 0.7 | unavailable |
| 06:29:57 | 1.090 m / 1.090 m/s | 1.222 m | 0.132 / 0.122 m | 0.234 rad/s | 4 / 1 s / 24 / 1.0 | unavailable |
| 06:29:59 | 1.043 m / 1.043 m/s | 1.209 m | 0.166 / 0.121 m | 0.383 rad/s | 4 / 1 s / 22 / 1.0 | unavailable |
| 06:30:01 | 1.057 m / 1.057 m/s | 1.233 m | 0.176 / 0.121 m | 0.155 rad/s | 4 / 1 s / 20 / 1.2 | unavailable |
| 06:30:04 | 1.176 m / 1.176 m/s | 1.301 m | 0.126 / 0.124 m | 0.084 rad/s | 4 / 1 s / 20 / 1.2 | unavailable |
| 06:30:31 | 0.357 m / 0.357 m/s | 0.230 m | 0.127 / 0.107 m | 0.941 rad/s | 4 / 1 s / 35 / 0.6 | unavailable |

The largest horizontal distance discrepancy is `0.176 m`; each row only
establishes that the configured gate fired. Age, satellite count, HDOP, and
sampled payload/ODOM time proximity do not identify whether the discrepancy
came from GNSS noise/multipath, a frame or lever-arm model, residual timing,
or ODOM motion error. No independent truth exists. Do not classify a
distance-gate rejection as a confirmed model error or tune the gate from this
same-record sample.

### Gate summary and remaining phases

- **G-P0: not passed.** P0-1 passed the targeted C++ diagnostic-adapter test
  against raw `gga.txt`; this does not resolve the fixed datum/origin,
  independent physical validation of runtime gravity and frame semantics,
  `body` semantics, or the surveyed lever arm.
- **G-P1: inconclusive.** No model error is confirmed, but the available
  exports cannot rule one out and do not contain FAST-LIO health telemetry.
- **P1-2 through P1-5:** not run. No configuration replay, covariance sweep,
  age-gate ablation, or reason-code schema expansion occurred.
- **Phase 2 / G-P2:** not started. The 4DoF graph and GNSS admission behavior
  were not changed; outage/recovery/revocation gates were not re-exercised.
- **Phase 3 / G-P3:** blocked. No independent control-point/PPK truth or
  calibrated covariance is available; `calibration_state` remains
  `uncalibrated`.

## Reportable outcomes

- **Now supportable after a successful LIO run:** ODOM availability,
  continuity, bounded-run timing/resources, and GNSS-reference-relative
  residuals with limitations.
- **Not yet supportable:** absolute localization accuracy, calibrated
  covariance/NEES, GNSS-enabled production benefit, independent truth, or
  target-hardware performance without the corresponding evidence.
- No map matching, prior-map localization, map relocalization, or map accuracy
  tests are part of this dataset task.
