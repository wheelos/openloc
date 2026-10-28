# Native LIO Error-Model Inventory

## Purpose and decision rule

This inventory records which dependencies of native LIO output are represented
by the current error-export path, which are cut by an explicit identity/epoch
boundary, and which remain assumptions. It is an implementation inventory,
not evidence that the stochastic model matches physical error or that global
fusion is accurate.

The target graph may report `qualified` only when its required motion and
source models are explicitly qualified, the full-pose estimate and ODOM are
fresh, the graph assessment is healthy, and candidate verification is not
pending. Any admitted uncalibrated or diagnostic-only input caps an otherwise
fresh result at `diagnostic`. Missing, stale, unhealthy, or unanchored output
is `untrusted`. Qualification flags and evidence IDs are producer declarations;
the graph does not independently validate their truth.

## Dependency inventory

| Error source or event | Current representation | Status | Remaining caveat |
| --- | --- | --- | --- |
| IMU sample noise and bias propagation | The opt-in native joint observer propagates sampled midpoint errors and bias noise through the 15D local state; `native_imu_sample_period` declares the PSD-to-sample conversion. | **Modeled** | Sensor noise densities, bias stability, temperature dependence, and the sample-time model have not been independently calibrated for the deployed unit. |
| Reuse of an IMU endpoint across adjacent propagation intervals | The observer retains shared primitive sample identities and a bounded joint error basis instead of treating neighboring intervals as independent. | **Modeled** | A fixed-lag Markov/sufficient-state claim still requires review against actual producer state and retention behavior. |
| Per-point acquisition time and LiDAR-to-body transform | Each point carries a time; the scan carries its frame and calibration identity and transform. | **Assumption unverified** | Point-time semantics, clock offset/drift, frame convention, and physical extrinsic calibration must be checked against the actual sensor recording. A declared ID is provenance, not calibration evidence. |
| Deskew dependence on the scan-end pose and interpolated IMU/LIO history | `DeskewErrorMap` records the actual history endpoints, pre/post-LiDAR-correction clones, raw-point loading, and point Jacobians. The observer composes these in the shared basis. | **Modeled** | The map is a first-order derivative model. Clock/time errors, unmodeled interpolation error, and nonlinear branch changes are not calibrated by this export. |
| Local-map/plane dependence on earlier scans | `StoredGeometryErrorMap` retains plane point support, anchor states, deskew loading, and shared source identities; the stochastic observer propagates retained geometry support. | **Modeled** | Voxel membership, correspondence selection, plane fitting and normal/eigenvector changes are piecewise/non-smooth. Undefined branches reject an error model; scene dynamics and physical map error remain unqualified. |
| LIO measurement update and state correction | The observer uses the frozen update gain, observed subspace, whitening, and actual correction branch; source identities are retained. | **Modeled** | Innovation gates, robust selection, correspondence changes, and configured point/plane noise have not been shown statistically calibrated on independent truth. |
| Continuous published-output correction and limiter memory | `OutputErrorTransition` linearizes the actual published step, including the 18D output/correction-velocity state and limiter branch. | **Modeled** | This models propagation of the declared local error, not the physical correctness of limiter thresholds or the resulting pose accuracy. |
| Delayed scan commit and history catch-up | Pending work retains its original frontier and error basis; a delayed commit joins candidate memory to the current output without replacing published snapshots. | **Modeled** | Capacity exhaustion is an explicit rejection. The complete production scheduler/overload behavior and deadlines are not qualified by the library model. |
| Epoch changes and local reset | Epoch/session/generation and exact sample identities prevent a motion connection from silently crossing a local epoch. | **Structurally eliminated** | Cross-process reset sequencing, consumer acknowledgement, global-alignment retirement, and publication handover remain host responsibilities and are not established by this boundary. |
| Motion-bootstrap/startup error | Bootstrap-specific deterministic sensitivities and native joint-state construction are available to the observer. | **Modeled** | Initialization priors and branch selection are not independently calibrated; a successful startup is not an absolute pose or an accuracy result. |
| GNSS observation noise, quality and temporal association | The global adapter preserves source identity, measurement/receive timestamps, quality decisions, and declared covariance/lever arm. | **Assumption unverified** | The current 3DMakerPro stream lacks independent heading and receiver accuracy evidence; timing, datum, antenna reference, and covariance calibration remain open. |
| Map observation uncertainty and map/GNSS dependence | Map factors identify the map and covariance source; graph admission supports explicit independence declarations or a conditional shared-error model. | **Assumption unverified** | A map built/georeferenced from the same capture/GNSS is not independent truth. The present diagnostic replay's future-informed seed is not causal initialization; map survey/construction error is not modeled as independent noise. |
| LIO-to-map/GNSS residual correlation | The graph API can represent a qualified conditional cross-covariance at an exact retained joint-error node; otherwise it requires explicit independence qualification and rejects known shared dependencies. | **Assumption unverified** | A positive covariance, Hessian, sparse keyframes, or evidence ID alone does not establish a sufficient-state/conditional-independence model. The real diagnostic map/GNSS sources do not currently qualify this contract. |

## Trust-level behavior

`FixedLagAlignmentResult::trust_level` is a runtime ceiling, not a quality
score:

- `kQualified`: fresh ODOM and full-pose evidence, a healthy graph assessment,
  no pending verification, and no diagnostic-only input mode.
- `kDiagnostic`: a fresh estimate exists, but the result is diagnostic-only or
  the graph assessment is not healthy enough for qualified use. Horizontal-only
  acquisition is diagnostic, never a qualified full-pose result.
- `kUntrusted`: no fresh estimate/evidence, stale ODOM, or verification pending.

The implementation does not infer a physical assumption's truth from its
boolean qualification declaration. While any dependency above remains
unverified, the producer must use the diagnostic path or keep the graph input
unqualified; it must not claim `kQualified`.

## Evidence required to change a status

1. **Model review:** document state/error conventions, primitive source
   identities, retention, reset boundaries, Jacobian validity, and the
   conditional/Markov assumptions required by the graph.
2. **Functional replay:** run the same immutable raw inputs and local settings
   through diagnostic and target-graph owners; preserve factor/owner outcomes,
   actual correction IDs, degeneracy reasons, and exact ODOM/scan isolation.
3. **Consistency diagnostics:** compute innovation statistics only for factors
   with a valid pre-update prediction, declared residual dimension/covariance,
   and modeled source dependence. With no independent truth, residuals are not
   NEES or accuracy measurements. Post-selection/post-gating NIS is biased and
   must be labeled accordingly.
4. **Empirical dependence checks:** inspect time-ordered innovations,
   autocorrelation, and before/after relocalization or loop-correction jumps.
   These can reveal unmodeled structure; they cannot prove independence,
   calibrate covariance, or justify an automatic inflation factor.
5. **Independent reference:** record the reference instrument/method and
   uncertainty, surveyed frame/lever arms, clock synchronization and offset
   evidence, and why reference errors are independent of the tested map and
   estimator. Until that is available, conclusions remain functional or
   same-source consistency only.

## 3DMakerPro input and independent-reference plan

The prepared `20261013170722` recording demonstrates that this session has
usable raw IMU and Livox `CustomMsg` scans for further algorithm experiments:
the archived preparation preserves integer IMU/header times, bag receive
times, LiDAR `timebase + point.offset_time`, and original point coordinates
before its declared zero-return filtering. This establishes that a
semi-self-developed observation model can be exercised on this recording; it
does not prove PPS discipline, absolute clock accuracy, or sensor calibration.
The source ROS bag and prepared manifest remain the authorities for a replay.

### Measured time/frame audit for the prepared 30 s window

The pinned raw and GNSS manifests identify the exact sample used for this
audit (`raw_30s/manifest.json` SHA-256
`d0d7f7b85c93bb9f1d9d106cad9350a262dc4d33299353f276e6caf177ad7293`;
`gnss_30s/manifest.json` SHA-256
`7864af837085b60a02e21652d917b5fae6daaba22f46f3bb3a18c2167e89b708`).
Comparing the preserved integer timestamps in those prepared files gives:

| Comparison | Min | Median | P95 | Max |
| --- | ---: | ---: | ---: | ---: |
| IMU bag receive minus IMU measurement | 0.321 ms | 0.712 ms | 4.917 ms | 16.451 ms |
| LiDAR bag receive minus scan-end time | 1.599 ms | 3.074 ms | 6.265 ms | 10.910 ms |
| GGA bag receive minus GGA measurement | 28.114 ms | 29.418 ms | 32.868 ms | 35.315 ms |
| Scan end minus nearest IMU measurement | 0.0000004 ms | 0.0000011 ms | 0.0000017 ms | 0.0000019 ms |

These are differences among recorded timestamp fields, not hardware
arrival-latency or synchronization accuracy. The point-time rule is preserved
as `timebase + original offset_time`; the first sample's maximum point time
equals its scan end, and the final sample's maximum selected point time is
4.947 us before the recorded raw scan end. Numeric overlap therefore supports
the replay's receive/measurement ordering only; it does not establish common
PPS, absolute clock accuracy, or a calibrated offset/drift model.

The raw replay's body reference is the integrated IMU origin. Its
`extrinsic.csv` contains translation `[-0.011, -0.02329, 0.04412] m` only;
the adapter currently supplies identity rotation. The GNSS manifest separately
records a user-provided identity-rotation `T_L_G` translation
`[-0.12, 0, -0.04] m`, composed into antenna-in-IMU
`[-0.131, -0.02329, 0.00412] m`. Neither transform has independent survey
evidence or a stated uncertainty, so these remain diagnostic nominal values,
not calibrated extrinsics/lever arms. The global frame is horizontal local
UTM-grid offsets from the first accepted fixed GGA; the map's vertical is
frontend-relative, `T_enu_map` is unavailable, and no qualified geodetic
height/up relation exists.

**P1 outcome:** recorded timestamp arithmetic and the adapter's declared
translation composition are reproducible for this pinned window. Hardware
clock synchronization, absolute timing, rotation calibration, transform
uncertainties, and a complete map/ENU/vertical frame relation remain
unverified; do not promote them to calibrated parameters.

No independent reference is currently pinned for this route. The following is
a proposed acquisition contract, not a claim that the hardware is available:

| Item | Proposed minimum evidence |
| --- | --- |
| Dynamic route reference | A second survey-grade GNSS/INS logger, separate from the receiver used by the estimator, retaining raw observations and a post-processed trajectory. Record receiver/antenna serials, firmware, base/CORS source, ambiguity/quality status, and the reference solution's position and attitude covariance. |
| Absolute anchors | Surveyed control points/checkpoints tied to the declared horizontal CRS, coordinate epoch, and vertical datum; independently measure the reference antenna phase center and vehicle-to-reference lever arm. Use a total station or equivalent surveyed method for static checkpoints where satellite geometry is poor. |
| Reference precision | Before the run, compare the product accuracy budget with a traceable reference uncertainty budget. Require reference uncertainty no worse than one third of the applicable acceptance limit in each scored component; report the 95% uncertainty bound and control-point closure. There is no approved product limit yet, so a numeric absolute-accuracy pass threshold cannot be set from current evidence. |
| Time association | Preserve each device's measurement and receive clocks. Prefer a shared hardware PPS/trigger recorded by both loggers; independently estimate and report residual offset/drift over the run. The required time-transfer bound must be derived from speed and the accuracy budget, not inferred from matching UTC-looking timestamps. |
| Independence argument | Do not feed reference poses, raw measurements, base corrections, control points, or reference-derived initialization into the tested estimator or its map. Build/freeze the evaluated map before the reference run. Document common satellite/correction services and bound their shared errors; use independent surveyed checkpoints to detect common-mode reference/map bias. |

If a survey-grade dynamic reference cannot be sourced immediately, collect
static surveyed checkpoints and preserve the route data. That supports frame,
map, and drift diagnostics, but does not replace a time-aligned dynamic truth
trajectory for whole-route accuracy scoring.

## Same-input owner comparison and empirical dependence

The frozen 2026-10-27 four-arm artifact is a diagnostic baseline only.
Its map/global decision journals retain source, status, correction revision,
and dependency declarations, but do not retain the pre-update residual vector
and innovation covariance required to compute an auditable NIS series. The
current raw replay adapter constructs only `DiagnosticGlobalAlignment`; it
does not instantiate `FixedLagGlobalAlignment`. Therefore the archive cannot
be post-processed into a target-graph result, an NIS result, or a residual
autocorrelation test without new instrumentation and an actual replay.

The next same-input experiment must keep the raw manifest, local configuration,
startup policy, and scheduling identical, and create two fresh result trees:

1. `diagnostic`: existing `DiagnosticGlobalAlignment`, four arms
   (ODOM/GNSS/map/both).
2. `target-graph`: `FixedLagGlobalAlignment`, the same four arms and source
   timestamps. Record graph factor admission/rejection reason, source and
   correction IDs, rank/degeneracy, solve time, and exact ODOM/scan isolation.
   Do not relabel an unqualified source as independent to force graph admission;
   preserve fail-closed rejections as experiment results.

For each factor, instrument the **pre-update** residual vector, innovation
covariance, observed dimension, source/time identity, gate result, and whether
the sample was selected or rejected. NIS is meaningful only with that matching
pre-update covariance and a defensible residual model. Report accepted-only
and rejected/gated samples separately because gating and robust selection
truncate the residual distribution. With no independent truth, do not report
NEES or absolute accuracy.

For admitted residuals, inspect time-ordered component autocorrelation and
whitened innovation sequences at declared lags, plus pose/correction jumps
before and after relocalization or loop events. Preserve the raw sequences and
sample counts. These checks can find visible temporal structure and motivate
a separately named covariance-sensitivity experiment; they cannot establish
independence or choose a validated inflation multiplier. The only permitted
positive conclusion without stronger evidence is “no inconsistency observed
in this sample under the declared model,” never “calibrated.”

## Current 3DMakerPro boundary

The 2026-10-27 four-arm run uses raw IMU/LiDAR and `DiagnosticGlobalAlignment`.
It demonstrates source corrections and ODOM isolation for a short recorded
window. It does not execute `FixedLagGlobalAlignment`, establish independent
accuracy, calibrate covariance, qualify a causal map seed, or demonstrate
live deadlines. A same-input target-graph replay is a separate required
experiment and must not overwrite the diagnostic baseline.

## 2026-10-27 innovation and fault-injection follow-up

A fresh, separately pinned four-arm replay is retained at
`pkgs/openloc/.agents/notes/current_fusion_runtime_20261027_v3_innovation/`.
Each arm consumed 6,001 raw IMU rows and 300 scans; GNSS arms retained all 29
GGA arrivals. All four arms have exact local ODOM and scan-decision isolation
(ignoring only `imu_ms` and `processing_ms`, respectively). Actual global
correction counts were 0 (ODOM), 10 GNSS, 41 map, and 105 map + 20 GNSS.
Pre-update residual/covariance records were independently re-evaluated by the
NIS analyzer: 9 GNSS-only, 40 map-only, and 124 combined-arm innovations were
recomputed; all logged NIS values matched. Missing prior innovations remain
explicitly counted (11, 3, and 3, respectively). Accepted-sample residual
autocorrelation is stored per source and lag in each arm's
`innovation_analysis.json`; it is descriptive only and does not establish
independence or calibration. The machine-readable cross-arm summary is
`p0_p2_summary.json`.

Named same-window diagnostic injections are retained separately:

| Injection | Observed behavior | Limit |
| --- | --- | --- |
| One GGA position spike at sequence 17: +25 m X, -20 m Y | 20 final owner decisions, 7 recovery-training-window and 13 recovery-training-inconsistent outcomes, 0 corrections | No innovation had an available prior state, so this run does not show NIS-based spike detection. |
| GGA measurement clock +50 ms, receive clock unchanged | 10 owner decisions rejected as measurement-in-future; remaining outcomes include missing/bracket and holdout reasons; 0 corrections | No innovation was available for these decisions. This is a functional fail-closed timing response, not a measured clock-tolerance bound. |
| Constant GNSS XY frame translation +50 m, -30 m | 10 corrections and 9 recomputable innovations; NIS mean/median match the unshifted GNSS-only arm | The horizontal translation is absorbed by the free alignment gauge. This demonstrates that single-source innovations cannot detect a common frame offset; it is not a map+GNSS shared-fault test. |
| Raw IMU X accelerometer bias +0.01 m/s^2 after 15 s | 4,071 valid ODOM rows, maximum post-injection position difference 1.041 m; 10 GNSS corrections and 9 innovations | An upstream IMU bias is only a proxy for LIO drift, not a direct injected LIO pose/model fault. |
| Raw IMU Z gyro-rate +0.05 rad/s for 500 ms after 18 s | 4,071 valid ODOM rows, maximum post-injection position difference 7.213 m; 10 GNSS corrections and 9 innovations | This is an IMU disturbance response, not a direct LIO discontinuity test. |

The GNSS and IMU injection artifacts are in
`current_fusion_runtime_20261027_v4_fault_injection/` and
`current_fusion_runtime_20261027_v5_imu_fault_injection/`; their input
transformations and derived journals are hashed. The raw source manifests
remain unchanged. No independent truth was available, so none of these results
establishes accuracy, NIS calibration, or detection completeness.

The target-graph host now writes a separate `*.innovations.csv` journal with
measurement/receive timestamps, source identity, decision status, prior frame
and state, residual, innovation covariance, NIS, and prior pose/covariance.
Its `native_global_worker_test` passes. This adds target-host observability but
does not complete a same-input real-data target-graph run. The prior 3DMakerPro
joint-error audit stopped at 1,950 IMU rows/98 scans with
`CORRELATION_UNQUALIFIED` because its configured joint basis exceeded capacity
or lost published rank; no graph corrections were made. The current raw
CSV replay still instantiates only `DiagnosticGlobalAlignment`. Direct LIO
drift/step injection, a joint map/GNSS common-mode fault, and an actual
same-input `FixedLagGlobalAlignment` replay remain open.

### Direct global-owner ODOM fault-injection follow-up

The raw replay now accepts `--global-odom-drift START_NS X_MPS Y_MPS` and
`--global-odom-step START_NS X_M Y_M`; the maintained runner validates and
records these settings. The perturbation is applied only to the ODOM sample
submitted to `DiagnosticGlobalAlignment`, after native local LIO. It does not
alter raw local ODOM or local scan-decision/frontier logs. The resulting global
alignment can affect subsequent scan-to-map pose predictions and match
outcomes. The deterministic perturbation leaves the original covariance
unchanged. Each injected sample is retained in `global_odom_injection.csv`.

Three fresh runs use the same pinned 30 s raw IMU/LiDAR input
(`d0d7f7b8...a7293`) and unchanged local settings. ODOM rows are identical
except `imu_ms`; all 300 local scan-decision rows are identical except
`processing_ms`.

| Scenario | Global-owner perturbation | Map outcome | Actual source corrections | Recomputed pre-update innovations |
| --- | --- | --- | --- | --- |
| Drift | From 15 s, +0.1/-0.05 m/s; final injected offset +1.5001/-0.7501 m | 106/202 map matches accepted; 96 failed | 104 map + 20 GNSS | 103 map + 20 GNSS; 3 unavailable priors |
| Step | From 18 s, +5/-3 m | 42/202 map matches accepted; 160 failed | 40 map + 20 GNSS | 39 map + 20 GNSS; 3 unavailable priors |
| Shared GNSS/ODOM frame translation | GNSS and global-owner ODOM both shifted +50/-30 m; GNSS-only owner | No map source in this arm | 10 GNSS | 9 GNSS; 11 unavailable priors |

The step materially reduced accepted map matches relative to the drift run,
but this is a functional disturbance response, not an accuracy or detection
guarantee. In the shared-translation run, the recomputed GNSS NIS mean/median
match the unshifted GNSS-only baseline to floating-point precision
(absolute differences below `2.4e-18`); the free horizontal alignment absorbs
the common offset. This confirms that these single-source innovations cannot
expose a shared frame translation.

Evidence and reproducibility pins are in
`pkgs/openloc/.agents/notes/current_fusion_runtime_20261027_v6_global_odom_faults/`
(`audit.json`, `artifact_hashes.json`, per-run commands, reports, replay
journals, runtime identities, and NIS analyses). The prepared raw manifest
still declares `external_odometry_used=false`; the shifted GNSS copy has a
separate manifest and does not change the source bundle.

These are global-owner boundary injections, not perturbations of the local
LIO output or its internal error model. They remain diagnostic-only with no
independent truth or calibrated covariance. A direct local-LIO output fault,
a coupled map+GNSS common-mode fault, and a same-input real-data
`FixedLagGlobalAlignment` run are still open. The latter remains blocked by
the previously recorded `CORRELATION_UNQUALIFIED` joint-error audit; no
graph qualification or threshold relaxation is claimed.
