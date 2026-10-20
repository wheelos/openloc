# LiDAR Odometry + GNSS Implementation Plan

**Scope:** continuous local LiDAR-inertial odometry with GNSS-aided global
alignment. A prior point-cloud map is not required or part of this plan.
FAST-LIO2 is the requested baseline candidate; integration choice remains
subject to a source-backed ADR.

The architecture target is in [`../architecture.md`](../architecture.md);
implementation evidence and blockers are tracked in
[`../roadmap.md`](../roadmap.md).

## Objective

Produce:

- continuous local `T_odom_body` from LiDAR + IMU;
- separate `T_enu_odom` alignment updated by timestamped, quality-qualified
  GNSS observations;
- `T_enu_body = T_enu_odom * T_odom_body` when the global alignment is valid;
- explicit uncertainty, input quality, rejection, outage, and recovery status.

GNSS must never pull, reset, or re-anchor the L1 ODOM state. There is no
prior-map matcher, map package, map relocalization, or map-to-ENU transform
deliverable in this scope.

## Validation dataset

Use only `/mnt/synology/3dmakerpro/20261008141912/` for this validation.
Dataset/schema findings are maintained in `/mnt/synology/3dmakerpro/schema/`;
the runtime profile identifies the JMK2/Mid-360 device, LiDAR/IMU inputs,
time handling, and FAST-LIO2 adapter scale. The LIO run must still verify that
the actual bag and resolved runtime configuration match that profile.

The schema audit records 162 `/GPSGGA` epochs reporting RTK-fixed quality 4,
but no calibrated uncertainty or independent reference. The existing
GNSS-only path diagnostic is not independent truth. A full-recording
GNSS-off FAST-LIO2 baseline and same-record consistency comparison have now
been completed; this is external-baseline evidence, not a run of Openloc's own
L1 implementation or GNSS fusion. Production GNSS factor admission remains
disabled. Keep source data read-only and derived outputs outside the source
tree.

## Decisions before implementation

1. Compare FAST-LIO2 external backend/package integration with the private
   `LocalEstimator`/`LocalLio` candidate. Evaluate license, dependencies,
   input compatibility (including point time), calibration, runtime, and
   testability; document the decision in an ADR.
2. Establish exact sensor schemas, units, frames, timestamps/clock domains,
   active extrinsics, and LiDAR/IMU synchronization.
3. Use the current dataset audit for GNSS/RTK stream and fix semantics; keep
   datum, antenna lever arm, PPS/absolute timing, and calibrated uncertainty
   marked unresolved where evidence is missing.
4. Define the local ENU origin, geodetic conversion, reset/epoch behavior, and
   the `T_enu_odom` state convention. This is a GNSS coordinate frame, not a
   map anchor.
5. Define historical ODOM lookup, delayed/out-of-order GNSS policy, innovation
   gating, and global-alignment propagation during outages.

## Staged execution

### Stage 0 — Reconcile the current dataset profile

- Follow the findings and runtime profile in `/mnt/synology/3dmakerpro/schema/`
  for `/mnt/synology/3dmakerpro/20261008141912/`.
- Confirm the actual bag streams and resolved FAST-LIO2 executable/config
  match that profile before replay. Preserve sensor/header timestamps,
  `timebase`, per-point `offset_time`, and IMU timestamps.
- Apply the profile's acceleration conversion exactly once. Do not use the
  all-zero IMU orientation or inject external gravity/bias seeds.
- Retain all input/drop/filter counts and point sentinel/tag handling in the
  run record.
- Carry forward the current RTK admission decision: GNSS factors disabled
  until uncertainty/reference requirements are met. Do not treat quality 4 as
  calibrated covariance.

**Gate:** profile/config/input identity matches and initialization is
observable. Stop if required runtime semantics conflict or startup is
unobservable.

### Stage 1 — GNSS-off LIO baseline

- Run bounded LiDAR+IMU replay using the selected sensor/config profile with
  GNSS factors disabled; pin source and executable identity separately.
- Verify first-sample initialization suitability; do not reuse an initializer
  seed from the other recording.
- Establish complete ODOM output, timestamp/gap behavior, continuity, reset
  handling, resource use, and deterministic replay.

**Current evidence:** the first GNSS-off FAST-LIO2 baseline passed the
reference-free integrity subgate for 1,625 poses over 162.4 seconds at 99.63%
coverage. The Stage 1 qualification remains open pending runtime resource
measurement and deterministic repeatability; this external baseline does not
validate Openloc's own L1 implementation.

### Stage 2 — L1 continuous ODOM

- Make the FAST-LIO2 integration decision through the Stage 0 ADR.
- Implement a bounded input adapter that preserves measurement times, frame,
  units, point timing, calibration identity, and session/reset generation.
- Verify scan-rate correction and high-rate IMU propagation, continuous ODOM,
  epoch/reset behavior, initialization, local-map bounds, and degeneracy
  behavior.
- Do not invent unsupported point fields or silently discard per-point time.
- Keep GNSS completely outside L1 state updates.

**Gate:** synthetic tests and a bounded segment from this dataset demonstrate
input handling, repeatability, ODOM continuity, reset behavior, and resource
sanity. Treat these as same-record development evidence, not held-out
accuracy evaluation.

### Stage 3 — GNSS global alignment

- Convert verified GNSS positions into a declared local ENU frame; preserve
  datum, origin identity, antenna reference, covariance, fix quality, and
  timestamp provenance.
- Estimate `T_enu_odom` from GNSS antenna positions and the historical L1 pose
  at each GNSS measurement epoch, applying the calibrated lever arm.
- Gate on fix status, age, covariance, clock association, and innovation.
  Rejected fixes must not update alignment and must expose a reason.
- Define observable state dimensions honestly: position-only GNSS does not
  provide orientation while stationary. Treat any heading/course input as a
  separate sensor contract.
- During GNSS outages, continue ODOM and propagate global-alignment
  uncertainty. On reacquisition, gate before correcting the global output.
- Specify out-of-order/delayed fix policy and output timestamp semantics.

**Gate:** tests show accepted GNSS changes only `T_enu_odom`, bad/stale fixes
are rejected, ODOM remains bitwise/contractually continuous, outages grow
global uncertainty, and recovery is explicit.

**Current software evidence:** `LocalizationResult` exposes
`global_alignment_valid`. It remains false until an absolute GNSS position is
accepted; a heading-only observation does not establish global position; after
initialization it remains true through GNSS outages while alignment covariance
grows. Synthetic assertions cover initialization, heading-only input, quality
rejection, caller-owned ODOM sample immutability, outage uncertainty, and
accepted GNSS recovery. The focused
`//:online_contracts_test`, `//:online_gtsam_fixed_lag_spike_test`,
`//:online_alignment_smoother_test`, and `//:online_localization_test` Bazel
targets pass in the managed development container. This verifies software
behavior only; it is not a GNSS-on replay or production admission. A
synthetic API test confirms a GNSS update does not mutate its caller-owned
ODOM sample; an integrated external L1 isolation test and full dataset
integration remain open. Timing/PPS, antenna-lever-arm, calibrated-uncertainty,
and independent-reference gates are unresolved.

### Stage 4 — Single-dataset GNSS diagnostic and experimental fusion

- Reuse the recorded GNSS-only track diagnostic for receiver-status/coverage
  context; do not label it truth.
- After the GNSS-off LIO gate passes, compare the time/frame-associated
  trajectory against the GNSS path as non-independent consistency evidence.
- Do not enable production GNSS factors under the current admission decision.
  A separately approved sensitivity experiment may use explicit provisional
  noise bounds, separate from production settings, with no accuracy claim.
- Report LIO continuity/drift, GNSS residuals/rejections, lever-arm/time
  sensitivity, outage/recovery behavior, and latency/resources. Accuracy or
  NEES requires independent reference and justified covariance.

**Gate:** run is reproducible and evidence is classified as either
GNSS-reference consistency or approved experimental factor sensitivity. The
same-record, zero-offset consistency diagnostic is complete and remains
non-independent; no GNSS-factor sensitivity run was performed.

### Stage 5 — Runtime integration and performance

- Integrate the middleware-neutral algorithm through separately owned sensor,
  GNSS, clock, lifecycle, and publication adapters.
- Measure scan latency, output rate/age, CPU, memory, GNSS update latency, and
  outage/recovery behavior on intended hardware.
- Verify the consumer-facing distinction between continuous local ODOM and
  GNSS-referenced ENU output.

**Gate:** reviewed runtime budgets pass on target hardware. Software and
desktop replay results are not vehicle qualification.

## Evidence and change control

For each stage record exact input identifiers/hashes, configuration,
initialization, time/frame conversions, metrics and thresholds, command/
artifact references, pass/fail/blocked/inconclusive result, and limitations.
Retain negative results. Accuracy claims require independent truth. Any
scope expansion to prior-map matching requires a separately approved plan.
