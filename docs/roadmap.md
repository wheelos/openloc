# OpenLoc Indoor/Outdoor Localization Roadmap

This roadmap tracks the product target updated 2026-10-27: indoor/outdoor
continuous LiDAR/IMU ODOM with one global owner able to fuse ODOM motion, GNSS,
map observations and optional known-marker observations. Local ODOM remains
independent of global corrections. Existing raw-native map/GNSS evidence is
diagnostic; target-graph real-data acceptance and indoor/outdoor transition
qualification remain incomplete.
Functional principles remain in
[`principles.md`](principles.md), and the target contracts are in
[`architecture.md`](architecture.md).

Until independent truth and pre-approved product budgets are available, the
no-ground-truth evaluation ceiling is L3. Existing evaluation tools now
recompute NIS and analyze whitening/correlation, source accounting, fault
response, same-factor batch and semi-synthetic NEES, with known failure
boundaries preserved. A complete raw factor ledger, calibrated covariance,
successful real-record FixedLag and independent physical reference remain
unqualified. The staged evaluator plan and evidence wording are
maintained in the
[global integration plan](../../../ad_agents/context/modules/localization/knowledge/openloc-global-integration-plan.md).

## Current implementation and evidence

### 2026-10-28: diagnostic baseline prepared for submission

The [package baseline snapshot](baseline-20261028.md) is the current compact
entry for product goals, algorithm/diagnostic/evaluation ownership, 14 synthetic
scenario contracts and their incomplete field correspondence. Commit
preparation checks cover 12 C++ test targets (356 cases), three built
evaluation/replay binaries and 56 Python cases; no lint was run.
Detailed sections below retain their original scope and historical counts.

This baseline includes the numerical graph repair, marker adapter, innovation/
evaluation tools and behavior-preserving map/CSV/LIO optimizations. Local LIO
compute gains do not establish end-to-end gains: latest 1x CPU/deadline results
worsened and increased map cost is unresolved. Real native FixedLag admission,
marker/continuous indoor-outdoor field coverage, calibrated accuracy and
production runtime/consumer acceptance remain open. Preparing a baseline does
not mark these gates complete or create a release.

### 2026-10-28: nonlinear fixed-lag failure repaired

The graph now preserves the original marginalized Gaussian chart explicitly
while permitting active nonlinear constraints to be relinearized within the
retained window. Full Bayes-tree delta updates repair a separate no-eviction
wildfire truncation failure. No lag, iteration, solve budget, source/trust gate
or exact-feasibility tolerance was relaxed.

Matched 8-node/5 s and 12-node/60 s runs each complete 640/640 trials; the former
previously failed 40/40 audited trials and the latter 2/640. Correctly modeled
independent/AR/shared-bias errors show no detected mean-NEES inconsistency under
these synthetic assumptions; deliberately unmodeled correlation remains
inconsistent. This is not real covariance qualification. Turning/shared-error
and original failure-seed regressions cover the repair. The
[current source/runtime-pinned report](../.agents/notes/goal_graph_20261028_v1/validation/index.html)
also distinguishes the 13 functional source scenarios from remaining real
native-model admission, marker field and deployment qualification gaps.
Older failures below are historical evidence, not the current solver status.

### Indoor/outdoor scenario framework

The [delivery plan and validation list](../../../ad_agents/context/modules/localization/knowledge/openloc-indoor-outdoor-delivery-plan.md)
separate algorithm state ownership, decision/health diagnostics and offline
evaluation. `BuildMarkerObservation` now converts associated known-marker
poses and calibrated transforms through full joint covariance propagation
into the existing global observation entry. A marker detector and actual
marker observations are not implemented or validated by this adapter.

The explicit default-off `retain_graph_on_full_pose_staleness` policy addresses
the graph being destroyed when full-pose evidence expires despite ongoing
partial constraints. It retains updates but not full-pose freshness or trust;
default behavior, observation gates and explicit recovery remain unchanged.

The actual public graph now completes 13 synthetic observation-boundary
scenarios: indoor map/marker without GNSS, complementary and joint degradation,
intermittent markers, source transitions/loss/reentry, faults, delayed map and
an explicitly unanchored GNSS-only cold start. The
[frozen report](../.agents/notes/indoor_outdoor_scenarios_20261027_v5/index.html)
contains 312 state rows, 474 terminal observations and 367 recomputed NIS.
Synthetic exact geometry is not raw-sensor or statistical-consistency evidence.
The preceding 2026-10-27 180 s raw local/combined replays preserve 1,702/1,800 local scans,
407 map + 126 GNSS corrections and exact local isolation, still using the
diagnostic owner and explicit local gain mode. Real native graph-model
gaps, global reentry jumps, field marker/indoor-outdoor and runtime/consumer
acceptance are not closed.

### Native target-graph Cyber functional boundary

The host now has default-off, explicit sole `FixedLagGlobalAlignment` selection.
The existing diagnostic owner is not constructed as a parallel/fallback owner.
Local post-commit exports atomically transport the actual joint basis/output
projection, rectangular cross-time motion, primitive identities and optional
actual shared deskewed scan. Original nominal ODOM remains immutable.
Packets, dimensions, tasks and queues are bounded; incomplete or unavailable
models cannot be replaced by configured diagnostic covariance.

Off-owner shared ICP includes the stochastic local seed loading and uses the
declared deterministic versioned map. Its source covariance/cross-covariance
feed that same selected graph. Raw GGA retains real quality/dwell admission
and needs an explicit external source-model declaration. Both full-pose and
GNSS decisions remain auditable; output identifies the selected owner/model
and stays diagnostic-only/untrusted.

The actual cross-process regression consumes 19 raw IMUs and four native scans,
all committed two frontiers late, then performs shared ICP and three map
updates plus admitted raw GGA in that one graph. It exercises delayed cold
acquisition, lag retirement and original source references without modifying
ODOM. A separate target-owner regression checks reset/consumer ACK fencing,
reacquisition and rejection without diagnostic fallback. Source/worker failures
also withdraw the separate joint channel immediately, with cross-process
receipt checked before any graph snapshot exists. The preceding nine-target
host regression passed, including the raw-host suite; its Cyber suite has ten cases.
No lint, calibration sweep or accuracy tuning was run.

The subsequent full raw sender/local host/target-graph host regression now
passes with explicitly selected offline source-clock graph replay: 224 raw IMUs,
four raw scans and two GGA packets produce three map and one GNSS correction,
with the final subscribed source frontier at 224. The native owner computes
the models inside the local executable, not before transport. Local/global
profiles must both opt in; source times stay unchanged and actual processing
delay is published separately. Ordinary ODOM and live graph freshness are
unchanged. A locally withdrawn model no longer advances the motion endpoint.

Earlier wall-freshness attempts remain failed coverage: even delayed LIO and
recorded backpressure did not meet the unchanged window. The passing offline
case observed model processing above two seconds and sender lateness above
seven seconds. It is a maintained synthetic raw-input functional regression,
not a real-capture baseline, live throughput fix or deadline qualification.
After this change, all nine affected host targets pass together, including
the full 13-case raw-host suite and ten-case global Cyber suite. No lint ran.

This closes the host transport/selection functional gap, not all E3-E6:
the new target path currently requires an explicit bounded map hypothesis
rather than the separate diagnostic regional search. New target-graph
two-capture replay, production consumers, live processing/overload budgets
and independent-reference acceptance remain outside this evidence.

### Actual native joint model into the selected graph

An explicit default-off `allow_uncalibrated_inputs` mode separates functional
graph composition from empirical uncertainty calibration without weakening
model sufficiency, source dependence or quality admission. The public result
remains diagnostic-only with an untrusted assessment.

A maintained regression feeds 19 actual raw IMU samples and four local LiDAR
scans through the native stochastic owner, then supplies its coherent joint
snapshots and rectangular motion cross-covariances to the same fixed-lag graph.
Independent synthetic map and GNSS measurements bootstrap and update that owner;
absolute corrections do not change local ODOM. This exposed constrained
Bayes-tree marginal re-elimination losing numerical rank. Covariances now use
triangular propagation through the existing posterior, preserving exact
zero-noise equations rather than introducing epsilon noise.

This independent-source regression is functional composition evidence, not
host target-graph selection, accuracy or E4 completion.
The following section covers the separate actual relative-startup model.

### Actual shared raw scan-to-map model

Native geometry now retains each accepted scan's actual pre-correction
deskewed points and same-driver loadings. `PrepareJointLidarScan` finalizes the
augmented frontier once before graph consumption, preserving nominal ODOM and
its published marginal. Synchronous and delayed commits retain the original
measurement reference; their scan support retires with native geometry.

The opt-in public scan-to-map path composes actual frozen ICP source/seed
derivatives with this joint state. It preserves point identity across range
filtering and voxelization and requires complete stochastic seed support.
The model explicitly conditions on a deterministic versioned map; map survey/
construction uncertainty and empirical calibration remain outside this claim.
It does not substitute the existing residual-Hessian diagnostic covariance.

All source errors can be mediated by the joint state, so zero conditional
source noise is represented by exact equations, not an epsilon floor. Delayed
observations bind their original pose node to a separately named retained
error node. Cold bootstrap assembles that trajectory and source in one update.
The same owner uses pose-first retained-factor re-elimination for conditional
updates, avoiding constrained-separator rank loss without another solver.

Maintained raw IMU/three-plane scan regressions cover synchronous and fully
delayed acquisition, actual finite-iteration ICP, repeated same-graph updates,
fixed-lag retirement, uncalibrated labels and unchanged nominal ODOM. Five
package targets and the three existing worker/publication targets pass.
The host transport/selection and actual cross-process regression are now
implemented as described above; calibrated/operational E4 acceptance is separate.

### Opt-in frozen-linearization stochastic observer (E4 still incomplete)

The native owner now has executable shared covariance propagation, not just
deterministic derivative exports: exact sampled IMU propagation, actual wheel gains,
pre/post-correction history clones, deskew/raw points, retained geometry,
iterated LIO and the 18D output smoother share one bounded error basis.
Stored-point loadings survive retirement of their originating history; plane
support reuses those loadings instead of independent anchor/point marginals.
Every LIO iteration uses the original same-scan random errors, not another
independent observation. Its gain, correspondence, whitening and observed
subspace are frozen at the actual nominal iteration.

`JointErrorLookup`/`JointErrorMotionBetween` expose separate coherent,
uncalibrated first-order snapshots and rectangular cross-time covariance.
The original nominal state, diagnostic covariance, source decisions and
qualification flags are unchanged. Minimal-basis compression handles singular
shared physical memory; no epsilon process noise is introduced. Independent
limits cover physical rows, stochastic dimension and exported snapshots.
Overflow rejects the observer, rather than truncating history dependencies.

The raw sampled IMU model now reuses each original acceleration/gyro error
across adjacent intervals and causal wheel interpolation, with explicit
`native_imu_sample_period` specifying PSD-to-sample variance conversion.
The actual midpoint state/sample Jacobians pass finite differences.
Stationary initialization includes the frozen mean/current-sample correlation;
the owner's declared residual initial uncertainty is not relabeled calibrated.

Delayed tasks now pin the original sufficient basis and later primitive
drivers. Their joint candidate/live covariance survives ordinary history
retirement. Commit preserves the actual live output memory and previously
published snapshots, while retaining candidate internal/history/geometry
loadings; all pins and dependencies remain resource-bounded. Analytic shared
driver/gauge, raw-sample, wheel-split, stationary-mean and retired-history
bridge regressions exercise these paths without epsilon noise or looser graph
admission.

Actual relative bootstrap now retains raw scans and frozen ICP steps, with
voxel membership and target-normal derivatives. Reverse propagation through
relative composition, gyro interpolation and causal translation deskew
preserves reuse across adjacent registrations. Numerical differentiation of
the unchanged initializer combines pose and original IMU inputs; its initial
state and frontier IMU share one joint covariance. The existing declared
residual initial uncertainty is retained, not relabeled calibrated.

Maintained regressions exercise real raw-scan synchronous/asynchronous
LiDAR-rest startup and excited moving startup, then export joint motion across
the next original IMU trapezoid. They compare nominal state exactly against the
observer-disabled owner. Unretained/retired support, nonunique voxel/normal
branches and capacity overflow reject only the observer. Bare pose-only
initialization remains unmodeled. All four package regression targets pass.

This does **not** establish qualified E4 acceptance. Host target-graph
transport/selection is now implemented as described above.
The shared map-source functional model is described above. These startup computations do
not establish live processing deadlines or statistical calibration.

### Augmented joint-error representation and native motion export

The existing graph owner now accepts a bounded full-rank joint error basis,
its projection into the published 15D state, and rectangular cross-time
covariance. Its chain, exact pose coupling, conditional source loading and
innovation retain all supplied memory instead of collapsing back to 15D.
Endpoint/source dimensions and projected marginals/cross-covariances must
agree. Negative hidden conditional noise and capacity violations fail
explicitly; no truncation or artificial memory priors are introduced.
An analytic regression keeps shared source/output memory through delayed
historical observations, dimension growth/retirement and lag marginalization.
The public facade exercises augmented bootstrap and malformed-model
non-consumption.

`NativeOdometry::MotionBetween()` now exposes the existing owner's actual
cross-time motion and source identities without a private header or a
duplicate value definition. IMU motion is available without relabeling
uncalibrated uncertainty; unmodeled raw LIO intervals remain explicitly
rejected. This resolves an API/export prerequisite and the graph's fixed-15D
representation barrier, **not E4 completion**. Actual native stored geometry,
deskew/startup and output-memory joint stochastic propagation remain incomplete.

The native owner now exports on-demand analytic continuous-output dynamics,
including its three correction-velocity memory dimensions and frozen endpoint
inputs. It also exports the actual raw-scan deskew dependencies: interpolation
uses corrected-left/pre-correction-right clones, repeated clones are shared,
and raw-point/end-pose loadings retain lever arms and rotations. Nonunique
rotation/limiter boundaries fail without overwriting a caller's model.
These are deterministic derivatives of the unchanged nominal path, not a
covariance assembled from independent endpoint marginals. Stored-geometry
reuse and the full startup/history/output joint covariance/source model still
need integration; native model/uncertainty flags remain unchanged.

Stored local planes now also expose their actual raw-source sequence/point
support and analytic center/normal derivatives, composed with the frozen
anchor pose and deskew-point loading. Anchors are retained and retired with
the existing bounded scan window. Voxel-assignment boundaries, repeated normal
eigenvalues and ambiguous source identities cannot produce an error model.
Plane relinearization, shared rigid gauge, scan retirement, actual public-owner
exports and unchanged raw live/recorded/three-process composition are covered.
This preserves geometry reuse explicitly but still does not supply its joint
covariance or qualify E4.

### Reversible output-envelope withdrawal

The raw native worker now forwards the owner's existing
`CORRECTION_BUDGET_EXCEEDED` assessment instead of publishing it as healthy
integrity-only ODOM. Out-of-envelope publications retain original source
identity/times but contain no pose/velocity/TF. The owner continues unchanged;
later in-envelope samples can restore diagnostic publication without an epoch
reset. A focused raw-scan regression proves withdrawal, recovery and exact
position/velocity/orientation/covariance parity with the direct owner. Worker,
publication, Cyber publisher, global worker and actual connected-host targets
pass. No budget or uncertainty qualification was changed; a first-capture
envelope breach remains a failure, not a reason to enlarge its limits.

### Conditional source factors in the existing graph (2026-10-26)

The selected graph now models qualified source/local-error cross-covariance
instead of requiring every observation to be unconditionally independent.
For `C = Cov(e,n)`, it derives the loading and conditional noise and factors
`h(x)-z+B e` against that noise. Innovation uses the full current pose/error
joint marginal, including cross blocks. Exact native node sequence/model
identity, sufficient-state qualification and conditional independence from
all history/other residuals are mandatory. Invalid conditional covariance
cannot consume startup evidence or commit a graph revision.
Independent sources retain their existing path; diagnostic sources do not
gain qualification. No local feedback or additional global owner is added.

The graph also accepts singular PSD conditional motion noise using exact
constraints in deterministic directions, rather than adding epsilon process
noise. Regressions exercise all-deterministic and rotated low-rank chains
through fixed-lag marginalization, analytic shared-noise posterior gains,
delayed shared evidence, projected factors, conditional full-pose bootstrap,
invalid model identity/covariance and original-observation non-consumption.
One existing test expectation was stale: unmodeled local covariance returns
`CORRELATION_UNQUALIFIED`, not `INVALID_INPUT`; production behavior was not
changed to satisfy that assertion.

Evidence: `.agents/notes/native_graph_conditional_sources_20261026_v1/audit.json`.
This implements graph-side E4 primitives, **not native E4 completion**.
Native stored-plane/point/deskew reuse and continuous-output correction-memory
still require a sufficient joint state and actual model export. The present
15D native output does not establish that claim. No native qualification flag
or recorded-data covariance was relabeled. The instrumented five-run bounded
shutdown investigation did not reproduce the earlier timeout; it remains
unresolved and is not counted as fixed.

### Terminal online health withdrawal (2026-10-26)

Fatal raw source/worker failures now latch a terminal unavailable state in
the existing native publisher before process teardown. Original native
identity/timestamps are retained, with no pose/velocity/TF or renewed source
time, and later owner updates cannot restore output. Actual Cyber receipt,
not just attempted write, is required by the four ingress-fault process cases.
The existing global worker withdraws local availability on these messages;
held ODOM/GNSS cannot revive it. Transport failure remains an explicit failure,
not a delivery guarantee. This closes a functional health gap, not E4, live
deadline/overload qualification or production-consumer acceptance.

Opaque startup tasks also retain their originating bootstrap identity:
matching revision/frontier from another bootstrap no longer authorizes
commit or consumes the original task. The original bootstrap can still
catch up and commit normally; no local initialization policy is changed.

A subsequent archive run exposed first-map-packet loss after reader discovery
(39/40 unique inputs) and one live-test shutdown timeout. The recorded map
boundary now optionally reuses bounded enqueue ACKs and byte-identical
last-packet retries, keeping every original cloud and the strict owner gate.
Conflicting retries/gaps and live opt-in are rejected; the connected regression
enables the protocol explicitly. The earlier failed run remains preserved.
The shutdown timeout did not reproduce in the next corrected run; its cause
is unresolved, not declared fixed. Bounded child waits now retain thread wait
channels on failure rather than only discarding the process. This remains
separate from qualified shutdown/deadline/overload acceptance.

All 50 focused cases across seven local-owner/publication/global-worker/Cyber
targets pass after the recorded-map boundary fix; all four terminal statuses
are archived from actual subscribers. Evidence, including the prior failed
run and the unresolved shutdown boundary:
`.agents/notes/native_terminal_health_20261026_v1/audit.json`.
This does not close the native joint-error/source-correlation implementation.

### Actual connected raw-native/global functional milestone (2026-10-26)

The raw sensor sender can now observe the separately running global host
while the local host owns startup/LIO and publishes actual ODOM. It never
substitutes a prepared ODOM trajectory. An actual global heartbeat precedes
sensors; both subscribers must reach the final native source frontier.
Optional original raw-GGA packets interleave by receive time. Recorded map
transport is separate from local ACK retries, preserving identical clouds
while leaving the global duplicate-observation guard unchanged.

The third-capture no-GNSS, no-pose-seed conditional-region run admits all
5,999 IMU and 300 clouds, initializes via relative async startup, commits 161
local LIO tasks and 139 map corrections, and receives 1,405 actual full-pose
publications. Evidence:
`.agents/notes/native_connected_hosts_20261026_v2/audit.json`.
Declared region/identity tilt and same-capture map remain unqualified priors.

Second-capture connected GNSS-only and map+GNSS runs each admit all 6,000 IMU,
300 clouds and 29 original GGA packets, and commit 135 local LIO tasks.
GNSS-only commits one horizontal correction and no full pose; map+GNSS
commits 70 map plus 11 GGA corrections through one owner and receives 1,324
full-pose publications. Evidence:
`.agents/notes/native_connected_three_sources_20261026_v3/audit.json`.
This map arm preserves its historical epoch/time-limited, future-informed,
same-capture **global-only** hypothesis, not seed-free recovery.
Original reconstructed GGA payloads/identities are preserved; double-header
receive-time rounding and unavailable raw-driver chunking are disclosed.
Scheduling is recorded admission-paced, not open-loop live qualification.

These runs close an actual online-composition gap, not E4 or all E3-E6.
Native ODOM still has no absolute-feedback interface; asynchronous repetitions
are not claimed as deterministic ODOM-isolation ablations. No threshold,
queue/deadline budget or uncertainty qualification was relaxed to obtain
corrections. Target conditional-error graph, unknown whole-map/tilt recovery,
production consumers, output-envelope failures and full overload/scenario
acceptance remain implementation/acceptance work, not hidden precision tuning.

The connected composition is also a maintained three-process regression in
the existing host suite, not only a temporary evidence driver. Its synthetic
800 IMU/40 clouds/one raw-GGA fixture must yield actual local admissions,
all 40 unique map events, correction commits and global full-pose publication
through the final native frontier. Profiles, inputs and journals are archived
as Bazel test outputs. All 24 affected host/config/worker/ingress cases pass;
evidence: `.agents/notes/native_connected_regression_20261026_v1/audit.json`.

### Explicit live ingress functional milestone (2026-10-26)

The raw Cyber host now optionally requires genuine single-producer
`Header.sequence_num` continuity independently for IMU and LiDAR.
Missing identities, visible gaps, duplicates and regressions fail before
enqueue; uint32 rollover is valid. A flushed `.source.csv` records the
original source identity, actual admission and fatal adapter/worker/publisher
outcomes. Failed enqueue is not admission. Exception-path reader shutdown
precedes destruction of captured worker/journal state.
Default headerless behavior and recorded idempotent retries remain unchanged.

All 23 focused host/config/worker/ingress cases pass, including four actual
cross-process admission/fault ledgers and missing point-coordinate rejection.
Frozen source/runtime hashes, test logs/XML and fault journals:
`.agents/notes/native_live_ingress_20261026_v1/audit.json`.
This is synthetic contract evidence, not another raw-native algorithm run.
The first observed sequence is only a baseline; earlier loss, trailing loss,
producer loss before numbering, restart identity and transport-versus-reader
attribution remain unqualified. Detection is not reliable delivery or 100 Hz
overload qualification. The next online composition must consume actual
native publications, not substitute replayed ODOM for a connected local host.

### Isolated relative startup host functional milestone (2026-10-26)

Opt-in `startup_task_timeout_ms` now moves provisional relative registration
and motion initialization off the IMU owner thread. Public opaque startup
tasks copy native/bootstrap state, replay every subsequently accepted IMU,
and commit only at the exact live frontier of the original uninitialized
owner. Local/global absolute sources remain unrelated to startup. Default
synchronous behavior and the existing startup geometry/observability checks
are unchanged; no stopped, calibrated or trusted claim is added.

One startup task, a 128-IMU catch-up bound, steady/source-age deadlines and
explicit shutdown retirement prevent hidden unbounded work or late adoption.
A live owner that independently initialized cannot be overwritten.
Async failures surface even when no new sensor event arrives.
Focused public-owner and host cases cover exact catch-up, wrong owner,
duplicate consumption, continuing IMUs during deliberately blocked geometry,
deadline/shutdown rejection and async failure propagation.

Three real raw-input, arrival-paced Cyber publisher runs preserve all 17,999
IMU source identities/times, initialize via startup tasks and subsequently
commit delayed LIO tasks. Current startup snapshot-to-frontier maxima are
56.3/64.9/131.9 ms; these are source receive-time catch-up intervals, not
measured compute times or a live 100 Hz result. End-of-input LIO work without
a new IMU remains uncommitted. Evidence:
`.agents/notes/native_async_startup_host_20261026_v2/audit.json`.
Native uncertainty remains unqualified. The target conditional-error graph,
unknown whole-map/tilt recovery, complete ingress-drop accounting, production
consumers and full scenario acceptance are still not delivered by this step.

### Raw-map Cyber acquisition/recovery functional milestone (2026-10-26)

The independent host now accepts raw calibrated LiDAR plus continuous native
ODOM history. One bounded asynchronous scan-to-map task waits for the actual
ODOM frontier, deskews copied poses, and supplies full/projected observations
to the existing single diagnostic global owner. ODOM/GNSS/reset processing
does not wait for geometry; busy, missing history, deadline, stale source and
epoch/reset retirement have explicit outcomes. No absolute feedback reaches
native ODOM. This is not complete live ingress-drop or deadline qualification.

Acquisition supports either an epoch/time-limited explicit hypothesis or a
hash-pinned conditional region/tilt/coverage policy. Public
`DiagnosticRegionalSearch` extracts the existing replay particle search,
geometric candidate verification and stable unique tracker; it introduces
neither a second estimator nor new geometry thresholds. Truncated/competing
evidence cannot supply a pose. Full-pose lease loss or local/global identity
changes restart candidate verification before the same owner's acquisition.

Real seed-free third-capture evidence:
`.agents/notes/native_regional_host_20261026_v1/audit.json`.
The current raw-native replay commits 146 map corrections and exactly
preserves the 5,999 ODOM rows and local scan decisions except measured timing.
Rehosting its ODOM with 120 original point clouds commits 106 map corrections,
with no GNSS, external ODOM or pose seed. Region, identity tilt and declared
coverage remain explicit diagnostic priors; the same-capture draft map does
not establish independent accuracy.

`.agents/notes/native_regional_host_faults_20261026_v1/audit.json` separately
proves full-pose withdrawal and regional reacquisition after 26 dropped scans
(72 commits), and reset/consumer retirement followed by next-generation
acquisition (69 commits). Both retain byte-identical ODOM inputs.
Earlier seeded raw-map/raw-map+GNSS/raw-map+raw-GGA controls remain historical
evidence in `.agents/notes/native_online_map_host_20261026_v4/`.
This completes conditional raw-map runtime acquisition/recovery, not unknown
whole-map/tilt search, the native conditional-error graph, production consumers,
or qualified live deadlines/overload. Independent startup scheduling is
implemented in the newer milestone above.

### No-map horizontal acquisition functional milestone (2026-10-26)

Explicit `--diagnostic-level-odom-gnss-bootstrap` now enables GNSS-only
XY/yaw acquisition and tracking within the existing diagnostic owner.
It reuses the eight-position fit and three later holdouts with unchanged
baseline/residual gates; no map seed, second global owner or absolute input
to local ODOM is introduced. Gravity-level ODOM/global XY is an explicit
diagnostic assumption, not qualified physical-up evidence. Height/full pose
remain absent, and the target graph and trust remain unavailable.
Accepted-position gaps beyond the existing global timeout require a new
training/holdout window; stationary and failed-holdout windows do not acquire.

Same-binary first-30-second `odom`/default `gnss`/opt-in `gnss` arms pass for
`20261012105149` and `20261013170722`: horizontal acquisition plus tracking
commits 3 and 10 corrections respectively, while default GNSS-only commits
zero. All six runs preserve their respective 6,000/6,001 ODOM rows and 300
scan decisions exactly except measured processing time. All 29 GNSS arrivals
per arm remain accounted, and no full pose or trusted health is emitted.
Commands, source/input/runtime pins, reports and exact isolation checks:
`.agents/notes/native_horizontal_acquisition_20261026_v1/audit.json`.
Focused owner/recovery/online/graph tests and native map/GNSS regressions pass.
This completes a no-map **diagnostic horizontal** function, not E3-E6.
The prepared-input global host step below now advances online publication;
live source adapters and the reset-request boundary remain functional work.
Accuracy calibration is a separate acceptance track, not a reason to postpone
that implementation.

### Prepared-input Cyber global host functional milestone (2026-10-26)

The independent consumer `modules/openloc_localization` now includes an
opt-in `native_global_diagnostic` executable. One bounded diagnostic owner
consumes existing local ODOM publications, explicitly prepared,
receiver-admitted GNSS antenna XY and opt-in prepared map observations.
A separate 10 ms timer publishes diagnostic XY/alignment or verified fresh
map full pose, without absolute feedback, production topics, trusted output
or target-graph qualification. Held/stale sources, missing
local output, epoch retirement/reacquisition, rejects and worker overflow have
explicit outcomes. Source frame/epoch/reference checks and journals remain
host responsibilities; no private candidate headers are exposed.

Two frozen raw-native 30-second ODOM/GNSS gate ledgers rehost through typed
Cyber with all 6,029/6,030 events accounted and 3/10 actual correction commits
plus partial publications. GNSS integer identities are preserved; the current
ODOM wire uses double seconds and can round converted ns. This rehosting uses
explicit uncalibrated host covariance, not retained native covariance.
Evidence: `.agents/notes/native_global_host_20261026_v3/audit.json`.
The cross-process regression separately covers actual Cyber delivery and
stale/epoch output, including bilateral discovery and explicit reader shutdown.
This is not a new raw-sensor algorithm run, independent accuracy or live
100 Hz/overload acceptance. Prepared full/projected map observations now share
that same owner. Map-only initialization, horizontal-to-full handover, both
source revisions, epoch retirement and full-pose freshness are implemented;
GNSS/projected evidence cannot renew the full-pose lease. Journals distinguish
sources and retain dependency declarations. A final-event watermark makes the
maintained replay wait for the corresponding publication rather than an older
correction at the same ODOM epoch.

Current host-wiring evidence:
`.agents/notes/native_map_host_20261026_v1/audit.json`. The two unchanged
no-map native ledgers still commit 3/10 corrections; a separate synthetic
prepared-map/GNSS stream commits two map plus one GNSS correction and publishes
full diagnostic pose. Mixed-source cross-process coverage is also synthetic,
not raw-map matching evidence. Raw GGA adapter/projection is now implemented
as described below. Live scan-to-map
scheduling, reset-request/consumer acknowledgement and E4 remain unfinished;
E5 is advanced, not complete. Continue these software functions rather than
turning the next milestone into accuracy tuning.

### Raw GGA Cyber adapter functional milestone (2026-10-26)

The same independent global host now optionally subscribes
`apollo.drivers.gnss.RawData` instead of prepared GNSS XY. Bounded streamed
GPGGA/GNGGA framing/checksum, exact UTC time-of-day with nearest receive-day
association, shared receiver fixed-quality/dwell admission and explicitly
configured WGS84/UTM origin/lever adaptation run on the existing global worker.
Public `GnssPositionGate` wraps the existing private admission implementation;
no second global estimator or host dependency enters the OpenLoc library.
The projection uses an explicit pipeline, not a host `proj.db` dependency.
Prepared GNSS mode remains unchanged.

Both original 30-second GGA recordings now feed actual typed Cyber with frozen
native ODOM: all 6,029/6,030 events and 29 GGA sentences per capture are
accounted, with 1/8 actual diagnostic corrections. New ODOM epoch establishment
restarts two-second receiver stability, explaining two fewer commits than the
unchanged prepared controls (3/10). Measurement ns remain exact; RawData's
double receive-time header can round associated original bag receive ns.
This reconstructs packets from original checksum-valid sentences and associated
GPSGGA bag receive times, not observed raw-driver arrival/chunking. Separate-
process synthetic cases exercise recorded and headerless live input; focused
cases cover fragmented/multiple sentences, quality/checksum rejection, midnight,
duplicates, gaps, epoch retirement and bounded incomplete shutdown.
Evidence: `.agents/notes/native_raw_gga_host_20261026_v2/audit.json`.

ODOM messages are unchanged and cannot receive absolute feedback. Raw GNSS
still establishes no height, full pose, target graph or trust. This removes a
runtime adapter gap without accuracy tuning; it is not a raw-LIO rerun or live
100 Hz/overload acceptance. The diagnostic reset/consumer boundary is now
implemented below; live map-matcher scheduling remains a functional gap. E3-E6
and the qualified native conditional-error model are not declared complete.

### Explicit global reset/consumer ACK functional milestone (2026-10-26)

The independent diagnostic host now optionally accepts an explicit alignment
reset request for the current host session/generation. It retires only the
single global owner's history/alignment/pending observations, increments a
separate alignment generation, withdraws global poses and waits for every
configured named consumer to acknowledge retirement. No consumer timeout
implicitly re-enables output. ODOM continues unchanged and the reference,
origin, calibration and content remain immutable. Local `ResetStopped` is
not called or weakened: its independent stopped-wheel prerequisite is absent
from the current raw IMU/LiDAR host.

Prepared GNSS/map require the new session/generation token; delayed old or
untagged observations are rejected. Raw GGA clears partial framing/dwell and
cannot use measurement epochs at/before the final ACK's ODOM frontier.
Idempotent last-request/ACK retries, conflicting or stale requests, unknown
consumers, missing ACK and fresh reacquisition have explicit outcomes.
Alignment identity and sticky reset request/reason are published even when
global pose is unavailable; correction IDs are scoped to alignment generation.
Consumer ACK is a retirement declaration, not authentication or a fabricated
trusted/qualified result.

Worker and separate-process synthetic regressions exercise withdrawal,
two-consumer barrier, stale source rejection, continued ODOM and actual
horizontal reacquisition. A maintained 24-event synthetic map reset replay
commits once before and once after reset, with unchanged ODOM generation.
Two unchanged frozen native ODOM/prepared-GNSS controls still account all
6,029/6,030 events and commit 3/10 corrections. Evidence:
`.agents/notes/native_global_reset_host_20261026_v1/audit.json`.
This removes the bounded diagnostic reset/ACK runtime gap without precision
tuning. Production consumers, CRS/origin migration, live map scheduling,
target conditional-error graph and full E3-E6 acceptance remain unfinished.

### Native raw-input end-to-end milestones (2026-10-23)

The current implementation sequence and input-status classification are owned
by the [canonical integration plan](../../../ad_agents/context/modules/localization/knowledge/openloc-global-integration-plan.md).
The stages below must not be confused with the older external-ODOM baselines.

| Milestone | Current status |
| --- | --- |
| E1 raw IMU/LiDAR to native ODOM, supplied map, one diagnostic owner and health | Latest history-boundary repair: second capture 83/135 map accepts, 78 map corrections; third capture without GNSS 155/162 accepts, 153 corrections; explicit offline hypotheses, not causal relocalization |
| E2 raw GNSS in the same native map/global chain | Second capture 12 GNSS commits in the map+GNSS arm; default GNSS-only remains unanchored. Explicit no-map horizontal acquisition is now available as described above. Historical first-capture 100 ms holdback comparison commits four GNSS rather than two |
| E3 causal global acquisition and reliable multi-capture startup | Three native starts, seed-free conditional regional acquisition and lease-expiry/reacquisition are demonstrated; whole-map unknown-pose search, qualified startup and production handover remain incomplete |
| E4 qualified native motion into the selected fixed-lag graph | Diagnostic functional model, actual shared-map source, bounded Cyber transport and sole target-owner selection implemented; calibration, live budgets and new target-graph dataset acceptance remain unqualified |
| E5 bounded high-rate scheduling and Apollo publication | Independent raw IMU/LiDAR ODOM/TF host with isolated startup/LIO tasks, single-global-owner prepared/raw map + GGA publisher, conditional regional recovery and diagnostic reset/consumer ACK implemented; production consumers, complete ingress-drop accounting and qualified live deadlines/overload remain incomplete |
| E6 indoor/outdoor/transition and S5/S6 acceptance | Not complete; independent reference and approved product budgets remain separate from functional software evidence |

Completion means the integrated design goals, not a replay exit code or an
individual milestone. Calibration, hardware clock evidence and independent
truth remain recorded qualification limits, not excuses to stop runtime or
algorithm implementation. E3-E5 are software work items, not missing-input
blockers. Prior short-window results below are pinned historical evidence,
not automatic acceptance of later algorithm changes.

### Active implementation gaps

1. Fixed-prior iteration and internal-history correction-boundary fixes are
   implemented. Preserve attitude-error chart transport and unqualified
   uncertainty; re-run all three captures and map/GNSS isolation after any
   further trajectory change.
2. Resolve sustained local correction demand versus the continuous-output
   tracking budget; do not increase the envelope merely to clear violations.
3. Complete host-owned high-rate publication, reset and explicit overload
   contracts beyond the implemented bounded diagnostic workers. Separate
   recorded receive gaps from host emission delays. Optimized native-only
   baseline scan medians are 6.667-8.634 ms, with maxima up to 13.186 ms;
   unoptimized 0.4 s timings are not deployment estimates.
4. Extend the implemented conditional regional acquisition to unknown-pose
   search and qualified recovery/handover beyond diagnostic lease expiry and
   reacquisition. The native conditional model and selected single-owner Cyber
   diagnostic publication boundary are implemented above; extend their actual
   dataset/live acceptance rather than substituting offline future-informed
   seeds or CI for those targets.

Conditional diagnostic recovery now restarts the bounded particle/tracker
generation when the existing two-second full-pose lease expires. A proposal
must pass current-scan full geometry before adoption, then distinct-frame
verification by the same global owner. Pending holdback cannot replace this
hypothesis with a retained old alignment. The initialized diagnostic follows
the current owner during verification; health remains untrusted.
The third-capture normal arm retains 148 map accepts / 146 commits. A declared
2.5-second scan-arrival outage (25 dropped scans, unchanged IMU) triggers loss
and reacquisition, with 119 accepts / 113 commits and 1.800061 s from expiry to
commit on the measurement clock. Each arm matches its corresponding local-only
5,999 ODOM rows except timing. These are conditional regional, injected-fault
functional results, not unknown-pose, independent accuracy or wall-time
qualification. Commands, source/binary/input pins and reports:
`.agents/notes/native_tracking_recovery_validation_20261023_v1/`.
An earlier broader regression aborted with `double free or corruption (out)`
in the unchanged NDT `MapMatcherTest.PlanarGeometryCannotProduceTrustedSixDofPose`
case. At that failure, the native binary built and worker tests passed, but
chained Python checks had not executed. PCL's aligned-allocation warning is
a diagnostic lead, not a proven root cause. Do not infer full regression
acceptance from the successful ICP/raw recovery runs.
Follow-up GDB investigation and repeated normal tests did not reproduce the
abort: thirty in-process repetitions and ten separate Bazel test processes
passed. Final native Python regressions pass. The original failure remains an
unresolved release concern, not a claimed allocator fix.
The replay wrapper now verifies the executable plus loader-resolved library
hashes before/after execution and rejects libraries from other Bazel
configurations/workspaces. Both direct-container and container-invoking
third-capture wrapper arms preserve the existing 5,999-row local-only output.
Evidence: `.agents/notes/native_runtime_identity_validation_20261023_v1/`.
Historical reports without such execution pins are not retroactively promoted.

Independent diagnostic publication now uses a default-off `--timer-output`
with arrival pacing. Its 10 ms thread samples immutable latest ODOM, preserves
source timestamps and distinguishes new/held/stale/epoch changes and skipped
ticks; callbacks never hold the estimator handoff lock. Freshness starts at
original sensor arrival, not delayed handoff. It is not a Cyber publisher/TF.
The stabilized binary passes ten worker/timer C++ cases and 14/5/3 native
Python contracts. Three raw 30-second timer-only arms record respectively
1,913/1,240/1,529 new measurements and 138/111/99 held publications, with
3/5/2 whole-period deadline misses. Every usable publication matches its own
source snapshot exactly. Cross-run ODOM is exact for the first two captures;
third-capture timer-only differs by at most `1.00533e-11 m` per position
component, while a no-timer repeat is exact. The strict comparison remains
failed; its unique cause is not proven and no tolerance was relaxed.
Third-capture async causal-map/timer produces 3,000 ticks and exact local ODOM,
but zero map commits: ten stale worker results, ten history-budget search
rejections and exhausted attempts prevent acquisition. A clean timer cadence
therefore does not establish the map E2E goal. Frozen reports and explicit
failed acceptance flags:
`.agents/notes/native_timer_output_validation_20261023_v1/final_audit.json`.
The seven executions are complete; strict isolation, real-time acceptance and
overall design acceptance are not.

Follow-up online acquisition repair moves the existing truncated-mode
rejection ahead of geometry refinement. The rejected sets previously spent
eight matcher calls without any possibility of admission; no acceptance,
search-domain or age budget was relaxed. Same-input third-capture paced
map/timer now accepts 143/162 matches, commits 141 corrections and preserves
5,999 local ODOM rows exactly except timing. Four truncated snapshots cause
no geometry calls; the recorded arm has 3,001 ticks and zero whole-period
misses. Evidence:
`.agents/notes/native_timer_output_validation_20261023_v1/search_fast_rejection/audit.json`.
The original zero-commit and strict-numerical-mismatch evidence remains.
The declared 2.5-second scan-arrival outage now also passes a paced async
regional recovery check: 110 map accepts / 106 global commits, one lease expiry,
two tracking commits and 2.200062275 s measurement-clock expiry-to-recovery.
Its 5,999 ODOM rows match its fault-input local-only arm except timing;
3,003 ticks have no whole-period miss. The fault input remains labeled as
modified, and this is not a wall-clock recovery guarantee. Evidence is in the
same investigation's `search_fast_rejection_gap/audit.json`.
Paced seeded three-source checks also complete for both GGA recordings:
122 map + 4 GNSS and 78 map + 12 GNSS commits, with exact respective
local-only ODOM isolation. They preserve historical explicit 0.5/0.2 m
bootstrap configurations rather than pretending a common algorithm baseline.
First-capture timer still misses two whole-period deadlines.
See the same investigation's `three_source/audit.json`.
These complete selected diagnostic acquisition/wiring checks, not E3/E5/E6.

The missing public local-ODOM boundary is now implemented as `NativeOdometry`
(`//:native_odometry`). A PIMPL delegates to the same private local owner;
public local value types retain the original fields/defaults, with no competing
model or private/PCL/GTSAM header in the facade. Opaque delayed LiDAR tasks
retain owner identity, exact IMU replay frontier and commit guards.
Public-only consumer tests plus local owner and motion-initializer tests pass.
Native replay and its delayed worker now consume the facade; 14/5/3 Python
contracts and bootstrap tests pass. Fresh three-capture local runs exactly
preserve the old 6,000/6,000/5,999 ODOM rows except timing.
Third-capture paced causal-map/timer still accepts 143/162 matches and commits
141 corrections, with exact local-only isolation and 3,003 timer ticks.
Evidence: the timer investigation's `public_facade/audit.json` and C++ XMLs.
Subsequently the same relative startup algorithm was extracted from the test
host into public `NativeMotionBootstrap` (`//:native_motion_bootstrap`).
Its PIMPL consumes the same IMU/previous raw scans and submits to the same
`NativeOdometry`; geometry, solver and acceptance budgets are unchanged.
Public-only consumer/owner guards and existing bootstrap regressions pass;
three corrected windows retain exact ODOM and scan/startup decisions except
processing time. The paced third-capture arm establishes alignment with
141/162 map accepts, 139 corrections and 3,002 timer ticks without whole-period
misses, preserving exact local-only isolation and publication identity.
This is a fresh asynchronous execution, not an exact global-count comparison.
Evidence: the timer investigation's `public_startup/audit.json` and two XMLs.
The public startup algorithm prerequisite is resolved, but a raw sensor
application, source-frontier stability under decoder backlog, Apollo
dependency/host and sufficient LIO graph errors are not declared complete.

The next host-output submilestone is implemented in the independent
`modules/openloc_localization/` consumer: public native ODOM to actual Cyber
subscribers, diagnostic-only TF, preserved source timestamps, new/held/stale
identity and epoch-specific frames. An explicit-epoch-offset IMU-only
executable now runs the native owner; three boundary cases, one owner/Cyber
case and two cross-process raw-input cases cover publication, TF, stale/epoch,
clean shutdown and malformed-input failure.
Raw LiDAR integration now uses strict source/calibration profiles, bounded
decode/FIFO queues, public relative startup and LIO with explicit overload
failure. A third cross-process case consumes synthetic raw clouds through the
standalone executable and reaches geometry/LIO acceptance. Four worker and
two strict-profile cases bring the consumer total to 13.
Optimized recorded worker replays retain all 6,000/6,000/5,999 ODOM rows and
900 total scan decisions exactly against the frozen native baseline. These
replays use a real publication timer but bypass Cyber sensor readers and do
not count subscriber delivery. The initial unoptimized overflow arm remains
preserved rather than being hidden with a larger queue.
Post-initialization geometry can now run on a public isolated LIO snapshot
with bounded exact IMU catch-up, declared source receive-time deadlines and
explicit shutdown discard. Three real prepared recordings also passed actual
separate-process Cyber IMU/cloud readers and diagnostic subscriptions:
17,999 admitted IMUs and 900 scans, unchanged source/reference scan frontiers,
178/122/158 LIO correction commits, and 25/12/2 explicit late-task discards.
Replay preserves original header clocks, labels output `recorded_input` and
uses admission acknowledgement/idempotent retry; sender lateness is recorded,
not hidden as live performance. Eighteen consumer cases cover the expanded
boundary. Evidence and trajectory overlays are in the consumer notes
`native_cyber_reader_20261023_v3/`.
Startup geometry still runs on the owner worker; complete live ingress-drop accounting,
independent startup tasks, the global application, reset acknowledgement,
fresh 100 Hz ODOM and production publication remain open.
See the consumer README and its ignored validation artifacts for exact scope.

The optimized baseline and per-scan correction demand are preserved in
`.agents/notes/native_optimized_validation_20261023_v1/`. A reproducible
GTSAM `<cassert>` patch fixes the optimized dependency build; existing
PCL alignment and ignored OpenMP warnings remain visible and are not
runtime correctness or deadline evidence.

Fixed-prior iterated LIO repair has now been revalidated on the same corrected
raw windows without gate changes. Output-budget rows change from 1412/0/63
to 1372/0/0; scan-frontier tracking gaps are 4.680/1.643/1.782 m.
The first capture remains outside its envelope, and the second gap increases:
this is not a blanket accuracy improvement claim. Native update counts and
initialization coverage remain 203/134/161 and unchanged, respectively.
Fresh second-capture both accepts 79/135 map matches and commits 74 map plus
11 GNSS corrections; third-capture map accepts 159/162 and commits 157.
Both absolute arms preserve their new local-only ODOM exactly except timing.
Nine focused C++ and fifteen Python contract cases pass. Paired hashes,
correction summaries and commands are in
`.agents/notes/native_fixed_prior_validation_20261023_v1/comparison.json`.
This repair does not complete the remaining E3-E6 implementation.

The next source-verified defect was interpolation through same-time LiDAR
corrections in internal deskew history. The corrected history retains the
propagated endpoint for the preceding interval and the corrected state for
the exact epoch/following interval. Published ODOM history is unchanged.
Synthetic coverage includes translation, attitude, velocity and the following
interval. The three raw windows retain startup/LIO coverage; tracking gaps
are now 4.661/1.440/1.930 m and violations remain 1372/0/0.
Second-capture both commits 78 map and 12 GNSS corrections; third-capture map
commits 153. Same-binary absolute arms still preserve local ODOM exactly
except timing. These changes are not a uniform accuracy improvement.

Default-off `--imu-trace` exposes the existing propagation callback as a
CSV and signed XYZ acceleration/velocity/bias HTML plots. The reports verify
IMU propagation equations and account for all IMU/LiDAR position/velocity
contributions; they do not declare measured acceleration to be sensor bias.
First-capture post-startup IMU velocity impulse is
[1.951, 0.669, -0.487] m/s and LIO contributes
[-1.540, 0.255, 0.263] m/s, while net LIO position correction is
[-7.612, 1.476, 1.003] m. Accounting residuals are below 1e-8,
not independent trajectory error. Trace adds explicit diagnostic overhead.
Evidence:
`.agents/notes/native_imu_trace_validation_20261023_v2/comparison.json`.
Next implement sufficient local estimation/output error dynamics and bounded
host scheduling; do not clear the first-capture failure with a larger budget
or qualification flags.

The diagnostic host now implements `--arrival-paced` and optional
`--async-global`: one bounded FIFO global worker owns map/GNSS processing,
accepts copied states/scans and never mutates the native estimator.
Capacity is 256 outstanding tasks/64 MiB declared payload; overflow and worker
failure are explicit, not drops or producer waits. Worker failures retain
cancelled queued-task evidence. EOF drains before closing output.
`host_timing.csv` records actual flushed ODOM emissions against recorded
receive-time targets; `global_scheduler.csv` accounts for every submitted
event and queue latency. Slow-task, outstanding-task/byte-limit and failure
handoff regressions pass.

One historical receive-paced comparison on the second raw capture retains identical
ODOM, global poses, decisions and source-time admission. Valid-row deadline
misses are 2025 synchronous versus 215 async-global; the async run still has
164 output gaps above 10 ms, and queued-task peak is 224/256.
Third-capture map-only worker peak is 47/256; first-capture local-only and both
asynchronous global runs fail the wall-clock 10 ms check. These shared-host
observations are not deterministic production deadlines or accuracy.
At that revision, local LIO/bootstrap and point-file I/O were synchronous.
Evidence: `.agents/notes/native_async_global_validation_20261023_v1/`.

The current default-off `--async-local-lio-ms 50 --arrival-paced` computes
one estimator snapshot off-thread, catches it up with at most 128 received
IMU samples, and commits on the owning thread at the declared source frontier.
Owner, epoch, source counts, exact IMU frontier, age and integrity guards
reject inconsistent results. Published history and the output smoother are
not replaced. Busy, missed-deadline, rejected and EOF-uncommitted tasks remain
explicit in `local_lidar_tasks.csv`; EOF never invents a future IMU.
Wheel-aided instances are not supported by this task path.

Default-off `--prefetch-lidar` additionally decodes at most one future point
file without advancing estimator admission before recorded arrival. Its
`lidar_decode.csv` accounts for consumption, readiness deferral and unused
decoding; bounded size, malformed input and worker failures fail explicitly.
All five paced raw arms consumed 300 scans without readiness deferral.
Snapshot copying, catch-up and bootstrap still run on the producer.

The frozen prefetch revision preserves synchronous-default ODOM against the
previous baseline. Three paced local arms commit 203/134/160 corrections;
the third has one explicit tail task without a future commit frontier.
Tracking gaps are 4.738/1.497/1.939 m: the first still fails.
Second-capture both commits 84 map plus 12 GNSS updates and third-capture map
commits 156. Both preserve paired local ODOM exactly except timing and have
identical local commit schedules. All health remains untrusted.

| Paced arm | Deadline misses | Wall gaps >10 ms | Those gaps with receive gap >10 ms |
| --- | ---: | ---: | ---: |
| First local | 4 | 59 | 51 |
| Second local | 7 | 36 | 32 |
| Second both | 7 | 33 | 33 |
| Third local | 5 | 168 | 163 |
| Third map | 12 | 168 | 161 |

The same captures contain 52/33/165 valid receive intervals above 10 ms.
These counts separate observed source-arrival gaps from host timing; they
do not establish hardware clock discipline or uniquely attribute causality.
An event-driven CSV emission path cannot establish timer-driven 100 Hz Apollo
publication. No deadline, physical gate or trust criterion was relaxed.
Fifty-two local C++ cases, five worker/decoder C++ cases and 18 Python cases
pass. Frozen commands, binary/source/input/output pins, HTML and native XML:
`.agents/notes/native_async_local_validation_20261023_v2/`.
Its original reports remain immutable; `audit.json` adds receive-gap analysis
with a separately pinned reporter revision.
Next: causal global acquisition, sufficient native error dynamics, and the
single-publisher/TF/reset boundary; these are still implementation work.

### Conditional causal map acquisition (2026-10-23)

`recovery.csv` replaces, and cannot coexist with, `seed.csv`/`seed_time.csv`.
The host reuses the existing likelihood field, particle search and persistent
fixed-anchor hypothesis tracker. Each bounded mode is refined with the existing
point-to-plane matcher and unchanged geometry gates. Only one nontruncated,
full-rank, unambiguous refined basin, confirmed over distinct frames and a
minimum span, proposes a diagnostic alignment. No proposal modifies ODOM.
Search region, score-coverage region, yaw interval and fixed tilt are explicit
conditional priors, not inferred free space, calibrated gravity or heading.
No trusted capability follows from a proposal or search weight.

The third capture has a seed-free regional raw run with 148 map accepts and
146 diagnostic global commits. All ODOM rows match local-only except timing.
This uses manual regional bounds, full yaw search and an unqualified identity
tilt prior; it is not arbitrary-location or attitude recovery. The wider
map-cloud-bounds experiment finds no verified proposal and remains unanchored,
with zero global commits. That is retained failure evidence, not successful
localization or proof the correct basin is absent.

A first paced attempt overflowed the unchanged 256-task global FIFO:
candidate-refinement tasks took 145-462 ms. The repaired `--async-global`
path isolates recovery in one immutable snapshot worker, serializing matcher
access until the result is joined. Global ODOM/GNSS processing continues while
search is busy. Result application retains the source ODOM pose and checks
epoch, monotonic reference and 500 ms age before transporting its hypothesis
to the current scan's independent matcher validation. Busy, stale, no-proposal
and EOF-unconsumed outcomes are explicit; no queue bound was enlarged.

The repaired paced raw arm finishes with 13 search queries, 33 busy skips,
one stale result reject, one consumed proposal and 85 map commits. Global
FIFO peak is 52/256; 12 valid-row deadlines and 141 wall gaps exceed 10 ms
(137 also span receive gaps). Decoder readiness deferred 48 times, changing
bootstrap/local commit timing against the historical local-only arm:
**that paced ODOM isolation comparison is not established**. Synchronous
source isolation and synthetic asynchronous isolation remain separate
controlled evidence. No 100 Hz, production graph or Apollo acceptance follows.

Native map/local/worker suites contain 30/52/5 passing C++ cases; Python
contracts contain 11 local, five map and three GNSS cases. Commands, immutable
reports/HTML, source/binary/CSV pins and test logs/XML:
`.agents/notes/native_causal_map_validation_20261023_v2/`.
The failed pre-isolation paced attempt remains under the corresponding `v1/`.
The diagnostic host still lacks automatic lost-tracking recovery, map handover,
source-frontier-stable feeding under decoder backlog and Apollo publication.

With no-return sentinels removed before sampling, the first corrected
`20261012103814` replay produced 188 native LIO updates and retained all
29 GNSS arrivals (23 float rejects, two fixed-dwell waits, four accepted but
unanchored positions). It did not establish a global correction: 189 map
attempts were rejected (169 iteration-limit, 18 overlap, two planar-support).
The old offline map hypothesis was reused at a newly earlier scan frontier,
with approximately 7.8 seconds of epoch mismatch. This is a replay seed
integration issue, not evidence that GNSS or the supplied map is invalid.
Explicit map-side hypothesis re-association and a fresh output directory are
required; no matcher gate relaxation is authorized.
The subsequent epoch-matched `both_v2` run passed the bounded three-source
integration: 188 LIO updates, 117/189 map matches, 115 map and two GNSS
correction commits. All 29 GNSS arrivals remain in the journal; two of the four
gate-accepted positions were ultimately rejected by the owner's committed-time
frontier. This is a delayed-admission limitation, not a receiver-quality failure.
Matched-seed map-only produced 121/189 matches and 119 corrections. All 6,000
native ODOM rows match local-only, map-only and both arms except measured
latency. A 1 ms seed-reference guard rejects the old stale hypothesis; no
geometry gate changed. Fourteen focused regressions passed. Source/hash/
comparison evidence:
`.agents/notes/native_gnss_e2e_20261023_v1/20261012103814/comparison_report.json`.
This remains offline-seeded diagnostic CI, not qualified fixed-lag fusion.

An explicit default-off `--global-holdback-ns` now provides bounded causal
global-observation reordering in the replay host (0-500 ms, 256 observations).
With 100 ms, the first capture retains 117/189 map matches and 115 map commits,
and all four gate-accepted GNSS positions commit. Original source timestamps
and local ODOM are unchanged. `global_admission.csv` audits enqueue/submit
times and statuses; the measured maximum queue residence is 104.617 ms,
including discrete IMU wakeups, not a proven wall-clock bound. Beyond-window
arrivals remain rejectable and future-on-arrival observations keep immediate
owner validation. Explicit end-of-input processing-clock advancement drains
only already received observations. This is not historical graph refusion.
Evidence: the same first-capture directory's `both_holdback_v1/report/`.

Input adapters already convert IMU acceleration from g, ignore invalid raw
orientation and retain per-point Livox timestamps. Missing trusted single-antenna
heading, unverified calibration/noise and unqualified map vertical reference
must not be labeled as proven sensor failures. Graph motion not exported and
no OpenLoc Apollo host are implementation gaps, not proof that
the supplied bags/maps are unusable.

The independent health supervisor is now invoked by native replay. It retains
relative-state validity separately and grants no trusted capabilities to
uncalibrated diagnostic output. The global/health type collision was fixed
with the distinct protobuf `HealthPose`, not by hiding conflicting definitions
behind a translation-unit boundary. Native local LIO directly revokes covariance
model qualification after correlated corrections.

E1 evidence and trajectory HTML:
`.agents/notes/native_map_e2e_20261023_v1/20261012103814_v2/report/`.
Existing mapper gates are unchanged; the explicit local-map support window is
15 m with 2 m cells. Deterministic native ODOM is unchanged when map fusion is
enabled. This does not grant qualified graph or production publication status.
Subsequent source audit identified an adapter defect: exact-zero XYZ no-return
sentinels were retained and consumed as geometry. The recorded counts above
remain historical execution evidence, not corrected-input acceptance. The raw
preparer must filter declared sentinels before sampling, retain rejection counts
and original timestamps, and regenerate inputs for validation. This does not
prove the cause of the third capture's startup failure.
Corrected-input replay of `20261015185840` still failed to initialize:
865,221 raw no-return points removed, 5,999 IMU samples / 300 scans retained,
zero valid states or LIO updates. All 89 physical-budget rejection windows
exceeded the unchanged gravity-error threshold. Raw point/IMU timestamps and
hashes were independently checked. Evidence:
`.agents/notes/native_corrected_startup_20261023/20261015185840/`.
This narrows the remaining startup problem but does not identify a faulty
sensor, and different historical/current binaries preclude paired attribution.

Motion initialization now exposes finite-solution and linear-system-evaluation
flags plus minimum scaled singular value and physical/fit residuals for rejected
windows. Replay retains `motion_initialization.csv` and a gravity/bias HTML plot.
Analytic variable-translation/attitude regression passes; coherent accelerative
pose drift preserves rank and fit residual but correctly fails the unchanged
gravity gate. This isolates a concrete failure mechanism, not the recorded
capture's unique cause. A diagnostic-only rerun preserves all 5,999 ODOM rows
and 300 scan decisions except measured timing: 91 evaluated linear systems,
89 finite rejected solutions, gravity errors 0.431419-8.549471 m/s^2.
Evidence: the same capture directory's `numeric_report_v1/index.html`.

### Corrected multi-capture resolution baseline and replay capability fix

The previous third-capture failure is specific to the default 0.5 m relative
registration resolution. A bounded `--motion-bootstrap-voxel-m` alternative
keeps all geometry, rank, residual, gravity and bias gates unchanged.
With a common 0.2 m resolution and the same binary, corrected first-30-second
captures produce:

| Capture | Initialization delay | Valid ODOM | LIO updates | Output-envelope exceedances |
| --- | ---: | ---: | ---: | ---: |
| `20261012103814` | 9.441 s | 4,112 | 203 | 1,412 |
| `20261012105149` | 16.335 s | 2,733 | 134 | 0 |
| `20261015185840` | 13.681 s | 3,263 | 161 | 63 |

The replay previously aborted on `CORRECTION_BUDGET_EXCEEDED`, conflating
capability degradation with an integrity failure. It now preserves those
untrusted rows and their reason; IMU gaps and invalid states still fail.
No correction budget was increased. A completed diagnostic is not acceptance
of the output envelope, calibrated accuracy or high-rate runtime.
Evidence: `.agents/notes/native_corrected_startup_20261023/common_voxel_02_comparison_v1.json`.
Paired capability journals isolate all 1,412/63 first/third-capture violations
to the internal-to-published translation gap (max 4.976/2.128 m versus 2 m).
No rotation or velocity discrepancy limit was exceeded. Applied correction
acceleration remains capped; it is not an uncapped requested acceleration.
`capability.csv` and JSON/HTML retain detailed first/last decisions and limits.
This instrumentation preserves ODOM and establishes neither truth error nor
the cause of the underlying LIO corrections; smoothing limits are unchanged.
Evidence: `.agents/notes/native_corrected_startup_20261023/correction_envelope_diagnostics_v2.json`.
The third no-GNSS map arm accepts 155/162 matches and commits 153 corrections;
all 5,999 ODOM rows match local-only except timing. The seed is explicitly
offline same-capture, epoch-matched and future-informed.
Its report is under the third capture's `map_voxel_02_v1/report/`.

Current-code design-goal validation now re-executes six bounded arms with one
pinned binary: three local-only, second-capture both/GNSS-only, third-capture
map-only. The local/absolute ODOM comparisons remain exact except timing;
GNSS-only does not fabricate a full-pose anchor. Six bounded functional and
honest-degradation checks pass, while output envelope and synchronous 100 Hz
feasibility fail. Causal global acquisition, native target graph, actual Apollo
publisher deadlines and independent accuracy/transition acceptance remain
uncovered, not passed. Commands and hashes are preserved under
`.agents/notes/native_design_goal_validation_20261023_v1/design_goal_report.json`
and `index.html`; E3-E6 remain incomplete.

Default-off translation deskew and LiDAR-confirmed-rest hypotheses have
focused regressions but do not independently solve the third capture at
0.5 m. The former uses only the last accepted relative-scan velocity; the
latter requires existing rest/IMU checks and explicitly assumes zero velocity
and accelerometer bias without qualifying uncertainty. Neither is promoted
to production or used to claim the 0.2 m result. Source/model uncertainty and
the specific recorded registration error remain unqualified.
The second-capture native three-source arm also completes: 74/135 map matches,
71 map + 12 GNSS commits, and all 6,000 ODOM rows unchanged except timing.
All 29 GNSS arrivals are retained: two dwell waits, 14 gate-accepted
native-not-ready exclusions, one unanchored position and 12 updates.
Pre-initialization events are no longer mislabeled invalid observations
through an invented generation-zero epoch. Evidence: the second capture's
`both_voxel_02_v2/report/`.

### Remaining-plan implementation pass

The graph owner now commits bootstrap mean/covariance and revision only after
successful candidate graph initialization. Failed initialization retains the
previous correction revision rather than publishing a success-shaped update.
Additional regression sources cover failed bootstrap atomicity, outlier
non-commit, distinct-frame recovery, ambiguity restarting verification,
out-of-order full poses preserving evidence time, age/window rejection, and
four source-selection arms over identical immutable ODOM.

The four-arm software contract deliberately leaves ODOM-only and GNSS-XY-only
without a full SE(3) anchor. It does not introduce a gauge pose to make those
arms appear globally initialized. Map-only and map+GNSS select the same graph
owner. This is not a substitute for the causal recorded-data baseline.

The first recovery regression failed because its expected revision omitted
one legitimate ODOM graph update before recovery. The corrected assertion
checks no revision/update during candidate verification and one commit on
successful recovery; no algorithm threshold or source data was changed.
The failure is documented here rather than presented as a successful run;
subsequent validation evidence is retained separately.

This pass completed the selected S1-S4 library software scope, not the full
S0-S6 plan. The managed-container regression passed all nine targets:
193 C++ cases, including all 120 `localization_algorithm_tests` cases and
14 public graph-facade cases. Eight targets executed; `online_contracts_test`
was a cached pass. The diagnostic replay rebuilt and all 12 prepared-observation
contracts passed in the managed test container. The lifecycle C++ files passed
Clang-Format checks and `git diff --check` passed.

Evidence is retained in
`.agents/notes/global_integration_20261020/graph_completion_v1/`:
the regression log, per-target XML/logs, source hashes and replay payloads,
outputs and manifest. Health unit tests establish the health component's
existing contracts, not graph-to-host health integration. Existing dependency,
protobuf and PCL/FLANN warnings were not suppressed or repaired.

The remaining plan is not entirely a source-code task:

| Gate | Remaining requirement |
| --- | --- |
| S5 causal recorded baseline | Pinned prepared sources with defensible arrival/clock semantics, causal startup/source preparation and approved four-arm protocol |
| S6 qualification and host integration | Qualified local joint-error/source-correlation models, independent reference/time/lever evidence and truth, approved accuracy/latency/recovery budgets, SE(3) publication policy, reset-consumer semantics and selected host entrypoint |

Those gates remain open. Existing CI replay and synthetic graph qualification
fixtures must not be relabeled as their acceptance evidence.

### Graph integration review and targeted validation

The selected fusion paradigm is local tightly coupled ESKF/LIO plus a single
global fixed-lag pose/error-chain graph and separate health supervision.
This development pass adds `FixedLagGlobalAlignment`, a thin public entry over
the existing graph owner, and native antenna-XY graph factors. Map
full/projected and antenna XY/XYZ use that one owner. Full 15D local covariance,
cross-time covariance and actual source identities are explicit inputs;
exact-node association is enforced during bootstrap as well as updates.
Motion endpoints include original integer times/sequences/epoch; aliased
observation times cannot select another integer ODOM epoch through a rounded
double key. Bounded decisions retain adapter/backend rejects, provenance,
qualification declarations and actual revisions across epochs, with explicit
source/aggregate counts, evictions and independent ODOM/full-pose freshness.
No local feedback, diagnostic fallback, extra solver, or default-backend
switch is introduced.

Shared observation value types and a private status/time/pose adapter are
reused by graph and diagnostic entries. Diagnostic-named aliases are removed
and every caller uses the canonical value types directly. The map adapter has
one explicit-timing signature; it no longer borrows arrival/sequence from ODOM.
Graph admission requires externally qualified motion/uncertainty and
independence from both local and other absolute sources. Known shared
dependencies are rejected rather than represented by independent factors.
The facade does not itself prove these claims or qualify current source data.

The upstream source review used GTSAM commit
`4d2d8b5e8ac7a1cb5be09b1442e3ff703ebbafe9` and LIO-SAM commit
`0be1fbe6275fb8366d5b800af4fc8c76a885c869`, linked from the architecture.
Point factors now use the analytical body-right Jacobian; true XY residuals
remain two-dimensional rather than following LIO-SAM's substitute-height
branch. Graph-level map-content/frame checks supplement owner admission.
Rejected motion connections revoke the graph but retain current valid ODOM
so the next legal connection can advance without a local reset.

Focused regressions cover XY dimensionality/null space, correlated
2x2 noise, antenna lever arms, delayed exact-node factors, partial leases,
single-owner routing, failed motion, qualification rejection and no
interpolated bootstrap. The previous code-only pass is superseded by:

| Validation | Result |
| --- | --- |
| `fixed_lag_global_alignment_test` | 10 cases passed |
| `diagnostic_global_alignment_test` | 11 cases passed |
| `localization_algorithm_tests`, selected `FixedLagGraphTest.*` and `MapObservationAdapterTest.*` | 22 cases passed; not the full algorithm suite |
| `scan_to_map_tests`, selected `EmitsTimestampedDiagnosticMatchFromBoundedMapWindow` | 1 case passed |
| `diagnostic_alignment_replay` build and prepared-observation Python contracts | Built; 12 cases passed in the managed test container |
| Scoped C++ Clang-Format and `git diff --check` | Passed; not a full repository lint run |

The replay payloads, outputs and hash manifest are retained under
`.agents/notes/global_integration_20261020/graph_review_20261020T010301/`;
selected unit-test XML/logs and source hashes are retained alongside them.
The replay still exercises the CI comparison backend, not a qualified
recorded-data graph baseline. Existing dependency-version, FLANN OpenMP and
PCL aligned-malloc/AVX warnings remain; no flags or dependencies were changed
to suppress them.

The 4DoF and conservative CI entries remain explicit replay/ablation baselines,
not production compatibility wrappers or automatic fallback paths.
Publication policy, host integration, independent health wiring and real-data
qualification remain incomplete; this is not S0-S6 or production acceptance.

### Active priority: normal three-source fusion before recovery

**Selected normal diagnostic: `openloc-three-source-normal-baseline-v1`.**
The first stage now has an actual recorded-map/GNSS/ODOM four-arm baseline,
frozen as `normal_three_source_full_v4/` and packaged in
`normal_three_source_baseline_release_v1/`. This is a sparse offline algorithm
comparison, not calibrated 3D accuracy, a public-engine integration or an
online publication baseline. Recovery and fault-injection expansion remain
subsequent stages.

First establish normal-operation quality-aware scan-to-map + GNSS + ODOM
fusion with one global owner and unchanged continuous ODOM. Only after its
recorded-data baseline passes, extend source-quality recovery and then
exceptional-input testing. The selected XY release below is a reusable
ODOM/GNSS sub-baseline, not completion of the three-source target.

Two bounded startup comparisons preserve all four historical same/cross-capture
inputs, frozen maps and one-time offline-informed seeds. Neither is causal or
an independent accuracy test. Results are retained under
`.agents/notes/two_capture_validation_20261015/`:

| Experiment | Accepted / scheduled | Observed rejection |
| --- | ---: | --- |
| `normal_three_source_startup_v1`: existing point-to-plane ICP, default gates | 0 / 4 | A-on-A reaches 40 iterations; three other cells converge but target coverage is 0.1840-0.3002, below 0.4 |
| `normal_three_source_ndt_target_v2`: original NDT, only target voxel changes from 0.5 m to packaged 0.2 m | 0 / 4 | All converge; normal-supported overlap is 0.2124-0.3126, below 0.5 |

In the finer-target NDT run, distance support remains 99.1-99.8%, but
1,400-1,653 correspondences per cell fail neighborhood planarity, with no
rank/other-normal loss. Finer voxelization alone does not fix eligibility and
increases startup processing to 9.25-16.53 s. NDT's normal check uses ten
neighbors and ratio 0.2; the configurable fifteen-neighbor/0.1 settings apply
to ICP, not this NDT check.

These experiments distinguish two different overlap definitions: NDT requires
normal-supported scan coverage, while ICP also requires scan coverage of the
selected target window. Near-complete distance support does not establish
either criterion. The NDT planarity loss and ICP target-denominator effect
need a justified observation-quality model; switching algorithms or simply
increasing map density did not establish usable observations. No gate was
lowered, rejected pose admitted, map changed or recovery experiment added.

#### Implemented normal diagnostic model and recorded-data comparison

Added an opt-in **different admission model**, `separate_ndt_support`, rather
than silently lowering the legacy overlap gate: require geometric coverage,
the existing planar minimum count, and projected-information observability.
The original default is unchanged; the alternative forces diagnostic-only
output. Synthetic mixed planar/nonplanar geometry distinguishes coverage from
information support while preserving insufficient-count rejection.

Selected the existing conservative SE(3) owner explicitly for the diagnostic,
not the public four-DoF engine or qualified error-chain graph. It receives
accepted matcher covariance/projection directly in memory and lever-aware
GNSS XY with no invented height. Full-pose updates use CI; partial diagnostic
updates retain the conditional unmeasured prior, an explicit approximation,
not a qualified joint correlation bound. The local covariance-model flag
remains false, the qualified graph is prohibited, and output cannot become
trusted. See the [architecture contract](architecture.md#explicit-diagnostic-three-source-alternative).
The trusted owner now separately requires qualified local and global
uncertainty inputs; diagnostic mode is the only path that accepts the current
uncalibrated replay assumptions. This closes the API ambiguity where
`quality_valid=true` could be mistaken for calibrated source uncertainty. It
does not qualify the frozen three-source results as an operational
quality-aware baseline.

**Code checkpoint — 2026-10-16.** `GlobalAlignment` now fails closed with
`UNCERTAINTY_UNQUALIFIED` unless both local and global uncertainty are
explicitly qualified on the trusted path. The three-source diagnostic runner
marks its assumptions unqualified and remains unable to produce trusted
output. In the managed container, `//:localization_algorithm_tests` passed all
105 tests in 16 suites and `//:three_source_dataset_replay` built; the changed
C++ ranges passed Clang-Format checking. The archived four-arm numerical
replay was not rerun after this API guard was added, so its metrics remain the
previous frozen diagnostic evidence, not a fresh run of this code checkpoint.

Use the original GNSS-off raw keyframes. Fixed diagnostic local variances are
`0.01 + 0.01 * cumulative_distance_m` in m2 and
`0.0001 + 0.00001 * cumulative_distance_m` in rad2, not calibrated LIO quality.
GNSS retains prepared XY/sigma without fabricated receiver qualification.
All four arms use one identical frozen offline-informed startup alignment.
Thereafter matcher seeds come only from the current arm's owner plus ODOM.
Map queries are fixed at every thirtieth keyframe; all ODOM/GNSS events remain
in every arm, with idealized sensor-time arrival.

| Query / target map | ODOM-only XY P95 | ODOM + GNSS XY P95 | ODOM + map XY P95 | All three XY P95 | Actual map updates in all-three arm |
| --- | ---: | ---: | ---: | ---: | ---: |
| A / B | 17.688 m | 0.989 m | 17.733 m | 0.654 m | 13 / 92 scheduled |
| B / A | 25.089 m | 0.884 m | 29.078 m | 0.701 m | 20 / 81 scheduled |

These are **post-update same-source GNSS antenna consistency** scores, not
independent accuracy or centimeter performance. The maps already used GNSS
and share calibration dependencies. All773/670 prepared GNSS epochs actually
update the corresponding GNSS/all-source owners. All3521/3100 events per arm
are retained:2748/2430 raw ODOM keyframes plus773/670 GNSS epochs. Eight full
runs account for26,484 events; the two four-arm64-keyframe prefixes add692.
ODOM poses, epoch, covariance input and invalid-model flag remain unchanged.
Final v4 also logs each original ODOM quaternion component unchanged, including
quaternion sign, rather than reconstructing it through a rotation matrix.

Map availability is limited, not hidden: in the all-source arm A has58
unavailable-window and21 registration rejections; B has47 and14. Map-only
admits13/4 and performs poorly, so this is not a map-only or continuous
three-source-coverage pass. GNSS continues to update when maps are absent.
All output remains diagnostic/untrusted, with a backend prior rather than
causal startup. Maximum all-source **3D backend** correction is4.153/4.023 m;
this direct replay does not implement the public engine's bounded publication
policy or establish vertical accuracy. It must not drive runtime consumers
as smooth/trusted global localization.

Two meaningful failed development versions are retained. `full_v1` exposed
stale full-map lease invalidation that stopped GNSS updates; success could
mean stored unanchored evidence, and B XY P95 reached26.250 m. Diagnostic basin
retention and actual revision-based update accounting fix that, without
changing the default lease. `full_v2` then exposed exponentially growing
free-height covariance and26-37 m 3D correction tails despite excellent XY
residuals. The diagnostic partial-information discount now preserves the
conditional null prior. A repeated-XY regression and every GNSS-only replay
epoch verify exactly the declared free-height process growth; final maximum
GNSS-only Z alignment variance is10.150/9.248 m2, not exponential. The
conditional model and all-source map covariance remain uncalibrated.

Focused mapper/owner/CLI contracts cover model isolation, quality-dependent
influence, free-height behavior, stale-map isolation, covariance, ODOM identity,
CRLF, required diagnostic flag and no overwrite. The first new minimum-count
fixture incorrectly exceeded the voxelized map size; that failure is retained
in the build log and fixed by testing one more than the actual supported
count. No default matcher gate, map, ODOM or public-engine behavior is retuned.

#### Expert audit follow-up: uncertainty transforms and remaining qualification

The ICP solver and its final information matrix now use a consistent
translation-first tangent with rotation about the current pose origin. The
pose-centered update removes the global-origin lever term from the iterative
normal equations. `ScanToMapLocalizer` now transforms full-rank covariance and
partial observed projection/noise through the cloud-to-base tangent Jacobian,
then canonicalizes the transformed subspace. Regressions cover a translated
map origin and a nonzero translation-plus-rotation cloud extrinsic. In the
managed test container, `localization_map_tests` passed 30/30,
`scan_to_map_tests` passed 9/9, and `localization_algorithm_tests` passed with
the new process-noise and diagnostic-only regressions. Targeted Clang Format
passed on all eight changed C++ files. The workspace-wide format check still
reports numerous unrelated existing violations; the
independent `buildifier` executable is absent in the test container, although
the updated BUILD target was parsed and built by Bazel.

The following audit findings remain **unqualified**, not solved by the P0
regressions:

- Physical gravity/up and roll-pitch observability are not established for
  these captures. Process-noise rate magnitudes remain uncalibrated; the new
  reference transform proves coordinate equivariance, not a physical error
  bound. Do not present this diagnostic process model as calibrated.
- Diagnostic partial updates retain a conditional null-space prior. A strict
  positive-definite algebra counterexample shows this is not a conservative
  joint bound for arbitrary unknown cross-correlation. The path stays
  diagnostic-only; positive definiteness, successful admission, and repeatable
  output do not establish consistency.
- Prepared GNSS quality/sigma and idealized arrival are not receiver evidence.
  Sparse interpolation uses future bracketing ODOM, and post-update
  same-source residuals are not a predictive or independent score. A causal
  receiver-quality replay and held-out assessment require source-bound
  arrival/status/covariance and independent reference data.

No map admission threshold or global fusion weight was changed. The normal
three-source result remains a reproducible functional diagnostic, not a
statistical/physical quality baseline.

The public 4DoF engine still lacks a typed map entry and supplied ODOM quality
process contract. The newer `DiagnosticGlobalAlignment` facade exposes the
existing SE(3) diagnostic owner separately; it does not provide qualified
deployment output, calibrated uncertainty, real receiver admission, full-rate
timing or independent qualification.

The self-contained release has233 hashed payload files. Its copied executable
replayed all eight full-route arms /26,484 events with every CSV field exactly
equal to the frozen result except measured processing time. Results are in
`normal_three_source_release_check_v1/verification.json`. Executable SHA-256:
`4a38067d5567d2d8e3f1fc504ab3a893753bc9f992a33a57f58a9c39912fb3c8`.
The package contains its own payload hash manifest and requires only the
existing managed-container runtime; it does not reference original cloud/map
paths during verification.

### 2026-10-15 selected data-validated XY engineering baseline

Selected **`openloc-xy-tracking-baseline-v1`** for the external GNSS-off
ODOM + public XY/GNSS workstream. Its opt-in recovery+tracking profile is
frozen in `xy_tracking_baseline_release_v1/` and the adjacent tar.gz.
This is a reproducible engineering comparison baseline on the two supplied
captures, not a production/full-LIO/accepted-map-fusion baseline. Library
recovery-off and stopped-reset defaults remain unchanged.

The new baseline verifier fixes the scoring protocol: all 773 A /670 B
unmodified nominal GNSS epochs remain in every scenario, including the
60-epoch outage, omitted fix and injected +1000 m point. Historical corrupted
input residuals remain separately available, not treated as independent truth.
Seeded ODOM-only freezes each run's first accepted global publication, without
future route fitting; startup epochs retain invalid status and are separately
counted. Time-weighted state coverage includes the entire ODOM span.

Before tuning any parameter, the new periodic withheld-GNSS test exposed
recovery starvation. Omitting every tenth epoch from20 onward creates 2 s
report gaps; the normal1.5 s continuity/3 s stability guards leave about six
qualified reports per cycle. Clearing recovery on every stability wait made
the required8+3 evidence impossible. A's original withheld P95 was23.363 m,
worse than seeded ODOM-only17.568 m; `xy_tracking_baseline_v1/failure.json`
preserves that failed acceptance.

Fixed only candidate lifetime: stable-fixed waiting does not insert a
measurement or clear previously qualified evidence. Actual invalid quality,
input/time/frame/motion errors still clear it; total age30 s,8 training plus3
later holdouts,5 m baseline and ordinary gates are unchanged. Focused tests
verify successful recovery across waits, invalid-quality reset, age expiry
and no moving publication reset. The complete v8 three-arm matrix passes.

| Query / scenario | Published clean-reference P95 | Seeded ODOM-only P95 after initialization |
| --- | ---: | ---: |
| A nominal | 2.543 m | 17.619 m |
| B nominal | 3.280 m | 22.817 m |
| A 60-epoch outage | 3.679 m | 17.619 m |
| B 60-epoch outage | 6.876 m | 22.817 m |
| A injected outlier | 2.192 m | 17.619 m |
| B injected outlier | 3.283 m | 22.817 m |

The published column scores all epochs (including10/7 preinitialization
epochs); seeded ODOM has no valid global estimate before initialization, so
the seeded comparison and acceptance also use an explicit initialized-only
subset. Each scenario's tracking P95 beats conservative publication and
seeded ODOM on its declared subset. A and B are evaluated separately; B is
still prefix-only. No new centimeter or production accuracy criterion is
invented.

Additional fixed holdout indices20,30,40,... remove76 A /65 B observations
before public submission; they are absent from all factor/recovery inputs.
Their published P95 is3.057/4.838 m versus seeded ODOM17.568/22.795 m. This
tests generalization to unused epochs, not independence from the receiver,
calibration or shared frame errors. Terminal-loss controls submit only the
first100 fixes, retain all ODOM, and confirm stale/degraded output with no
further correction, not successful global accuracy without GNSS.

`public_xy_v8_intermittent_recovery/` contains35 executions /99,542 events.
`xy_tracking_baseline_v2/` adds22 executions /71,062 events: two repeats of
each eight tracking scenarios (byte-identical CSVs/ledgers versus v8), four
holdout runs and two terminal-loss runs. All57 executions /170,604 events
have zero unresolved fixes, unchanged ODOM and no invented Z. Four online
targets, four public runner contracts and three baseline scoring/coverage
contracts pass. Scoped C++ formatting passes; Python style tools remain
unavailable, not a full Python lint pass.

After outage resubmission, first ordinary readmission is3/14 s (A/B);
first fresh backend/publication mean agreement is22.200/33.300 s. Later
re-anchors remain possible, so these are not permanent integrity recovery
times. Nominal fresh-and-aligned time is72.2%/58.9%; startup22.500/13.500 s,
stale167.992/247.999 s and transition63.492/77.288 s are retained. Sparse
timing and the1.5 s freshness policy explain why a baseline pass cannot be
reported as all-time trustworthy output. B outage maximum published residual
is18.290 m; the lifecycle fix does not monotonically improve every metric
(v7 B outage P95 was5.824 m).

The package pins executable/profile/source/Bzlmod identity, prepared inputs,
clean reference, v8 and acceptance reports and manifests. Source maps,
recordings and map-infra remain read-only. Receiver fields are still
synthetic, arrivals idealized, tilt/manual lever/noise unqualified, and truth
same-source. Successful map factors, native frontend/full-rate validation and
independent accuracy remain separate evidence gates.

### 2026-10-15 runnable XY diagnostic: reference coordinates and tracking

Completed the user-approved diagnostic release, keeping recovery default-off
and stopped-reset publication as the default. Added explicit
`LargeCorrectionPolicy::kRateLimitedTracking`: all post-initialization current
body corrections obey the existing 0.3 m/s and 1 degree/s bounds. Publication
advances on ODOM ticks with accepted position evidence no older than 1.5 s;
neither rejected fixes nor an outage/queued-fix gap bank correction credit.
Pending or stale output is explicitly degraded. No feedback enters ODOM or
backend estimation.

The backend now anchors XY translation/yaw priors and process noise at the
fixed first ODOM XY position. Caller ODOM is unchanged; external alignment
means and full covariance are transformed back, including translation/yaw
cross terms. Z keeps its existing origin and remains uninitialized under XY.
This intentionally changes the old origin-sensitive prior parameterization;
the old v3 results are historical, not byte-equivalent algorithm baselines.

Frozen `public_xy_v7_reference_tracking/` includes 24 primary executions
(8 recovery pairs plus 8 publication pairs), 79,086 primary events, two
prefixes, six invalid-quality controls and three source-shaped synthetic
controls: **35 executions / 99,542 total events**, zero unresolved fixes.
Recovery+hold and recovery+tracking backend means/covariance are exactly equal
at every corresponding event; original ODOM poses/epochs match in all arms.
Every tracking event satisfies position/yaw rate bounds, stale-evidence
isolation and no post-initialization reset. Four online targets and four
public runner contracts pass, including long-ODOM-gap queued fixes.

| Query / scenario | Backend XY P95: recovery on | Published XY P95: hold | Published XY P95: tracking |
| --- | ---: | ---: | ---: |
| A nominal | 2.472 m | 23.838 m | 2.543 m |
| B nominal | 2.753 m | 17.019 m | 3.280 m |
| A 60-epoch outage | 3.000 m | 17.207 m | 3.679 m |
| B 60-epoch outage | 4.989 m | 18.888 m | 5.824 m |

These are all-epoch same-input antenna consistency residuals, not independent
accuracy. Both publication arms have the same 1/2 nominal and 1/3 outage
backend re-anchors for A/B, and no primary published reset. Tracking removes
policy holds, not all transient error: maximum pending body offset is about
10.47 m in B outage; its published residual maximum remains about 18.27 m.
At the capture tails, source freshness expires and degraded remains set even
when published/backend means agree. Offline primary runtime is approximately
2-4 s per run and peak RSS approximately 13-14 MiB, not full-rate latency.

Equivalent translated-XY-origin controls now have maximum composed backend
difference 4.9e-8 m, published difference 6.1e-12 m, identical admission and
publication decisions, and matching measurement-space NIS. Maximum covariance
entry error is 2.5e-5 m^2 at a high-uncertainty epoch; scale-normalized error is
1.1e-8. Acceptance explicitly checks entry error divided by
`sqrt(max(1,Pii)*max(1,Pjj)) < 1e-6`; small-variance entries retain 1e-6
absolute tolerance. This is a floating-point comparison, not covariance
calibration or arbitrary SE(3) invariance.

The synthetic coherent-bias control still re-anchors while moving without a
publication jump, then performs one approximately 100.845 m stopped reset.
Initialization is accounted for separately (about 0.307 m); invalid receiver
quality delays the stopped reset by 6 s. Temporal consistency is not integrity.
`public_xy_v4_reference_tracking` preserves the old startup-counter assumption
failure; v5 preserves the unscaled-covariance assertion failure. v6 passed
before the final queued-fix/zero-credit boundary hardening; v7 is the release
evidence. Earlier failed compile/test runs were stopped and diagnosed.

The runnable local artifact is `diagnostic_xy_release_v2/` beside these runs,
with executable, A/B inputs, source snapshot, evidence and hashes; the package
README gives the managed-container invocation. It is not a production release:
synthetic receiver qualification, idealized arrival, provisional tilt/manual
lever, uncalibrated sigma, sparse external GNSS-off FAST-LIO keyframes and
same-source scoring remain explicit. No accepted map factor, native OpenLoc
frontend qualification or independent global accuracy is claimed. Map
observation admission/correlation and real receiver/frame/noise qualification
remain separate gates; no maps, source recordings or map-infra code changed.
The v1 package is superseded only to correct documentation that quoted the v6
tracking residuals instead of v7 after gap-credit hardening; both packages pin
the same final executable. Previous packages and experiments remain untouched.

### 2026-10-15 public XY two-capture replay and publication findings

Reused the frozen A/B input streams, raw GNSS-off quaternion keyframes and
declared graph antenna lever. All source hashes match. Reconstructing the
fixed-transform body/antenna chain agrees with the old prepared positions to
2.3e-13 m. No optimized trajectory replaces ODOM, no map is changed and no
raw sensor/frontend replay is added.

Added a test-only `public_xy_dataset_replay` and reproducible
`public_xy_validation.py`. The public engine receives original ODOM samples,
fixes at idealized measurement-time arrival and interpolates only after the
right bracket arrives. Outputs retain all ODOM/fix events, pending/final
factor updates, original measurement time, composed backend/published body and
antenna positions, full backend marginal, publication offset, revisions and
resets. Unresolved pending fixes are explicitly counted; none remain in the
full measured-input runs. Existing ordinary receiver-continuity, motion, NIS,
admission-rate and publication gates are retained.

Receiver qualification is **synthetic**, not recovered from the recordings:
fixed reports, 20 satellites, HDOP 0.8, age 0.2 s and nominal accuracy 1 m.
The diagnostic adapter explicitly selects an accuracy limit of 2 m and sparse
ODOM bracket allowance 2.5 s; library defaults are unchanged. The 1 m factor
sigma and future-informed fixed tilt/lever remain unqualified. Sparse speed
is a displacement-rate proxy, not certified stationarity. Missing evidence
limits the claim rather than rejecting the entire experiment.

Two 64-keyframe prefixes and the 8-pair/16-execution matrix completed, with
52,724 primary output events. Four additional invalid-quality runs and three
synthetic controls also completed: 25 executions including prefixes, 66,559
total output events and zero unresolved pending fixes. All source ODOM samples
are accounted for in each applicable run, unchanged across arms; XY does not
initialize height or change its alignment mean. Four online test targets and
three runner contract tests pass.

| Query / scenario | Ordinary XY admitted: off / on | Backend re-anchors: off / on | Backend same-source XY residual P95: off / on | Published same-source XY residual P95: off / on |
| --- | ---: | ---: | ---: | ---: |
| A nominal | 567 / 716 | 0 / 1 | 22.365 / 1.014 m | 23.834 / 23.834 m |
| B nominal | 513 / 550 | 0 / 2 | 4.669 / 2.550 m | 10.997 / 17.119 m |
| A 60-epoch outage | 507 / 656 | 0 / 1 | 22.361 / 2.913 m | 17.280 / 17.280 m |
| B 60-epoch outage | 158 / 472 | 0 / 3 | 18.958 / 5.028 m | 18.958 / 18.958 m |

These are all-scheduled-epoch post-update consistency comparisons, using the
measurement-epoch antenna and latest available alignment at its right bracket;
not independent accuracy, raw prefactor NIS or latest-body versus stale GNSS.
No whole-route fit or rejected-tail truncation is used. The public off/on
experiments are not equivalent to the older private runner: state segmentation,
quality continuity, startup and publication differ. No historical 10/11 s
recovery claim is transferred to this path.

There are **zero published resets in the primary runs**. Large corrections
remain held while moving or lacking consecutive stopped confirmation; local
ODOM remains usable independently. A nominal recovery does not imply a usable
published global correction: B's published P95 worsens despite the improved
backend. Longest holds are A nominal 183.5 s, A outage 710.1 s, B nominal
recovery 271.0 s, B outage recovery 504.1 s. Maximum held current-body offset
in the reported nominal/outage runs reaches 27.36 m. Backend reacquisition
and output recovery must therefore stay separate. B's post-outage first
ordinary readmission changes 475 -> 14 s; A is 3 s in both public arms.
Offline runner wall times are approximately 2-3 s per primary run, peak RSS
approximately 13-14 MiB; neither is a full-rate latency qualification.

Two fail-fast stops are retained:

- `public_xy_v1`: first complete A baseline exposed an ODOM-only fixed-lag
  update with ~6.4 cm backend/published difference but stale `aligned` status.
  After approval, added `kAwaitingPolicyUpdate`: pending mismatch is explicit,
  publication is unchanged and no stopped confirmation is counted. The full
  event checker, which previously failed, now passes (including ODOM-only
  events); focused online/runner tests remain green.
- `public_xy_v2`: historical all-future outlier/omission state-parity assertion
  failed. The outlier is rejected by motion consistency and at that processing
  epoch leaves backend/publication identical to omission. A fixed receiver
  report nevertheless preserves quality continuity; omission produces a 2 s
  gap exceeding the default 1.5 s and restarts stable-fixed qualification.
  A's next fix is admitted versus rejected as not stable. With approval,
  `public_xy_v3` verifies immediate rejection isolation, records later
  differences and separately runs invalid-quality reports; no gate is relaxed.

Synthetic controls use source-interpolated A epochs 160..209 with separately
constructed +100 m coherent GNSS bias and 20 stopped samples. They are not
additional recorded motion. The backend re-anchors while output is held; after
five stopped confirmations it publishes exactly one 100 m reset. Invalidating
one quality report delays that reset by 6 s. This is a counterexample to
integrity: temporal consistency plus stopped confirmation cannot determine
whether a coherent GNSS shift is real or biased.

The local-origin stress control subtracts 1000 m from ODOM X while retaining
physical GNSS, using the engine's unchanged default parameter prior. It is
not an equivalent transformed-prior experiment. After initialization it
produces backend differences up to 3.765 km and published differences up to
26.86 m before later recovery. This exposes serious initialization/prior
sensitivity, NOT a passed origin-invariance gate. Proposed repair: explicit
position-only initialization/local reference parameterization, transform
alignment priors and process covariance including XY/yaw cross terms, and
compare physically equivalent priors before claiming invariance. Do not
recenter caller ODOM silently or label position-only initialization trusted yaw.

Frozen sources, protocol, exact commands, per-event decisions and scores,
data-gap repair actions and hashes are in
`.agents/notes/two_capture_validation_20261015/public_xy_v3/`.
`public_xy_v1/` and `public_xy_v2/` preserve the stops; 162 archived artifact
hashes, 51 source/input pins and the final binary identity have been
independently verified. Production recovery remains off.

Next iteration should address initialization/prior sensitivity and define an
explicit host policy for long held corrections (degraded global output,
operator-approved reset or separately approved transition), not merely tune
noise to improve residuals. Real receiver admission needs source status/
accuracy and receive timestamps; physical qualification needs gravity/lever/
datum calibration, full-rate ODOM and independent truth.

Map integration remains decision-only: a successful observation must bind
sensor time and ODOM epoch, map/version/calibration identity, observed subspace,
noise provenance and frontend/map/GNSS dependency identifiers to the single
global owner. Timestamp selection and unknown-correlation fusion must be
explicit. The actual map geometry/normal-support evidence is still rejected;
synthetic contract success cannot qualify it. No new public map API, accepted
map factor, fake independence flag or NDT gate relaxation is included here.

### 2026-10-15 architecture-first publication and public XY iteration

Implemented the next two bounded software gates, without another dataset run:

- GNSS global re-anchor no longer writes published alignment directly. It
  increments a separate backend `alignment_revision` and uses the same
  publication policy as ordinary GNSS and heading, including first heading
  after an existing publication. A held re-anchor is not a published reset.
- Translation decisions use the composed latest ODOM body position, not the
  alignment translation parameter. Moderate position/yaw corrections are
  limited in pose space and converted back to an alignment transform.
  Publication timing uses latest ODOM, while measurement association/records
  retain original sensor time.
- Existing large-correction limits are retained. Confirmation requires
  consecutive consistent stopped observations at distinct publication epochs.
  Moving or rejected observations break confirmation; multiple accepted
  sources processed at one epoch cannot multiply confirmation.
- Result state distinguishes aligned, rate-limited and held publication,
  pending correction and deterministic current-body XYZ/yaw offset.
  `alignment_covariance` remains explicitly backend-only when means differ;
  no offset-as-noise or calibrated published covariance is invented.
- Public `GnssHorizontalFix`/`SubmitGnssHorizontalFix` accepts only measured
  antenna XY and 2x2 covariance. One internal position path shares receiver
  qualification, ordering, queue, time association, recovery and publication
  with XYZ; records use `ABS_POS2`/`ABS_POS3`. XY does not initialize or shrink
  unobserved height; a separate flag records prior XYZ height initialization.

Quick validation passed four existing targets: online contracts, recovery,
smoother and localization engine. Public-engine synthetic A/B covers both
XY and XYZ: re-anchor while moving changes backend revision but holds output
without reset; five subsequent consistent stopped epochs permit one published
reset. Additional cases cover a near-zero alignment translation with a
yaw-induced 10 m body displacement, a 0.5 m yaw-induced displacement limited
to 0.3 m, delayed-factor versus publication clocks, same-epoch confirmation,
bad-quality interruption, XY pending/quality/NIS/duplicate handling, and no
fabricated vertical observation. Caller ODOM/epoch remain unchanged.

The first XY gate test stopped as required: its synthetic 250 ms inter-fix
interval hit the default 500 ms admission-rate guard before NIS. After user
approval, only that test's admission interval was set to zero to isolate the
intended gate. Product admission limits and recovery default-off are unchanged.

This establishes bounded software ownership/publication contracts, not complete
localization qualification. The frozen `recovery_v3_hardened` numbers below
still describe the private sparse runner and its historical direct re-anchors;
they do not measure this public publication policy. No source receiver replay,
native LIO run, successful map factor, new geometry qualification or independent
accuracy evidence was added. Next: source-bound public XY replay when eligible
quality/arrival inputs exist, followed by the successful typed map-observation
bridge under its existing frame/subspace/correlation gates. Default-off recovery,
unqualified coherent-bias integrity and downstream reset handling remain open.

### 2026-10-15 offline-map-informed recovery hardening

Reviewed the sibling `map-infra/offline_map` fusion evidence in `BASELINE.md`,
`docs/CPP_FUSION.md`, `docs/SCAN2MAP_BASELINE.md`, and
`docs/UPSTREAM_BACKEND_COMPARISON.md`. Its selected engineering baseline uses
Gaussian relative Pose3 odometry, source-admitted Cauchy GNSS XY, separate
soft relative receiver height, and strictly verified loops. Its repeated-pass
geometry still passes 0/3 and 0/12 pairs. This motivates preserving negative
evidence and separating consistency from integrity, not copying an offline
batch pose optimizer or changing kernels/noise to reduce same-source residuals.
OpenLoc continues to own only global alignment; source ODOM is immutable.

Implemented two bounded corrections without changing ordinary XYZ/XY factor
admission or production defaults:

- XYZ factor rejection is no longer sufficient to initiate XY/yaw recovery.
  The selected state's horizontal marginal innovation must independently
  exceed the unchanged 2D gate, using that state's innovation covariance,
  not the latest published pose or an XYZ conditional residual. An isolated
  height failure cannot trigger a horizontal re-anchor.
- The 30 s recovery bound now covers the total sensor-time span of the
  retained training window plus subsequent holdouts, not merely the gap
  between adjacent observations. Expired candidates restart training with
  an explicit timeout reason and retain strict timestamp ordering.

Three focused C++ targets and three CSV-runner contract tests pass. Added
regressions cover Z-only rejection through the public engine, training expiry,
holdout expiry and its inclusive boundary, and no repeated-time admission
after expiry. A negative synthetic control demonstrates that a coherent
20/-10 m GNSS bias can still validate a candidate: temporal agreement does not
provide independent integrity evidence.

The paired diagnostic reused all frozen prepared nominal, 60-epoch-outage,
+1000 m outlier, and omitted-outlier streams for both captures: eight pairs,
16 executions and 11,544 output epochs. Every baseline remains byte-identical
to its historical control, and both nominal hardened-recovery outputs remain
byte-identical to `recovery_v1`. Source/code/binary hashes were frozen before
execution and rechecked afterward. All ODOM fields and timestamps are
accounted for; re-anchor Z means are unchanged and Z variances do not shrink.
The isolated outlier's state matches its omitted-fix control in both arms.

| Query capture | Nominal GNSS admitted: baseline / recovery | Nominal same-source XY residual P95: baseline / recovery | Post-outage first ordinary factor admission: baseline / recovery | Post-outage first re-anchor |
| --- | --- | --- | --- | --- |
| A `20261013143752` | 614/773 / 762/773 | 21.653 / 1.401 m | 196 / 11 s | 10 s |
| B `20261013170722` | 613/670 / 648/670 | 4.063 / 2.496 m | 477 / 11 s | 10 s |

First re-anchor and first ordinary factor admission are different events.
Neither is a trusted-accuracy qualification. The normal runs' current-epoch
antenna XY jumps at re-anchor are 4.82 m for A and 3.63/5.45 m for B; outage
runs reach 12.76 m. These are composed pose-space jumps, distinct from the
previously reported alignment-translation changes up to 27.33 m. Publication
safety was an explicit blocker at this snapshot; the later architecture-first
software policy above does not retroactively validate these private-runner
jumps or downstream handling. Recovery still defaults off.

The runner has prepared XY inputs, provisional 1 m GNSS sigma, sparse ODOM,
and no receiver arrival/quality stream. It cannot exercise the new public XYZ
subspace eligibility rule; the public-engine synthetic regression covers
that rule. No total-age expiry occurred in these real prepared streams; only
the targeted tests establish expiry behavior. No raw frontend, map matching,
new relative-height factor or accepted-map bridge was executed.

Artifacts and the executable experiment snapshot are in
`.agents/notes/two_capture_validation_20261015/recovery_v3_hardened/`.
The failed `recovery_v2_hardened/` is retained: the first fault baseline used
plain GNSS while its historical outage control included one rejected-map
ledger entry. The sole difference was that flag, not any state/GNSS decision;
the approved correction restored historical modes without relaxing parity.

Next stages: define backend/publication separation and actual pose-space jump
policy with explicit reset/degraded uncertainty; add a typed horizontal
public-engine observation path and source-bound receiver replay; then bridge
successful map observations into the single global owner. Do not lower NDT
normal-support gates or claim that failed map geometry is qualified.

### 2026-10-15 explicit GNSS global reacquisition and two-capture A/B

OpenLoc now has a shadow GNSS global-alignment recovery path, disabled by
default. It only observes fixes that passed the engine's existing time,
quality, and ODOM-consistency checks but were rejected by its XYZ innovation
gate. The later hardening above additionally requires horizontal rejection.
The private XY replay instead uses the 2D gate. The candidate requires eight
rejected observations, at least
5 m of horizontal ODOM baseline, a robust XY/yaw fit with at most 2 m training
RMS, and three later temporal holdouts below the existing 2D NIS threshold
9.21034. These thresholds are provisional diagnostics, not calibrated
receiver or product requirements. A validated candidate re-anchors only the
global alignment graph; local ODOM samples/epoch remain unchanged, and the
unobserved Z mean/variance are preserved. Re-anchor state IDs remain
monotonic, and a repeated re-anchor at the same timestamp is rejected.

The synthetic recovery, smoother re-anchor, and localization-engine A/B tests
pass. A test-only sparse replay then compared the frozen GNSS baseline with
the same prepared inputs using the recovery helper:

| Frozen map ← query capture | Epochs | GNSS admitted, baseline → recovery | Re-anchors (elapsed from first epoch) | All-epoch same-input antenna XY residual median / P95 / max, baseline → recovery |
| --- | ---: | ---: | --- | --- |
| A ← B | 670 | 613 → 648 | 2 (306 s, 646 s) | 0.833 / 4.063 / 19.950 m → 0.121 / 2.496 / 5.293 m |
| B ← A | 773 | 614 → 762 | 1 (757 s) | 0.650 / 21.653 / 25.699 m → 0.197 / 1.401 / 4.642 m |

The baseline output is byte-identical to the prior frozen diagnostic. All
source ODOM fields match exactly between arms, and re-anchors changed neither
Z mean nor Z variance. Re-anchor translation jumps were 9.73 m and 12.49 m
for A ← B, and 27.33 m for B ← A; these are large global corrections, not hidden
continuity or accuracy improvements. Each run still has rejected epochs, with
a maximum rejection streak of 11.

This is **source-prepared diagnostic evidence only**: the inputs contain
prepared graph GNSS XY positions and an uncalibrated 1 m sigma, not raw
receiver-quality/arrival fields. The runner exercises the private alignment
backend, not the complete public-engine receiver-admission path. Later
holdouts are temporal samples from the same record/source, not independent
truth or statistically independent GNSS. Lower same-input residuals therefore
do not establish absolute accuracy, calibrated uncertainty, or production
readiness. Large re-anchor jumps and possible correlated GNSS bias still
require explicit safety policy and independent validation. Keep
`enable_gnss_reacquisition` false in production.

Commands, input/source/binary hashes, per-epoch recovery decisions, all-epoch
residuals, and results are preserved under
`.agents/notes/two_capture_validation_20261015/recovery_v1/`.

### 2026-10-15 horizontal-only fusion diagnostic and NDT rejection classification

An unchanged-input four-cell startup replay now partitions every nearest-target
normal rejection. A/A, A/B, B/B and B/A lose 1,477, 1,730, 1,649 and 1,354
distance-supported points, respectively; **all** fail the ten-neighbor
`lambda_min > 0.2 * lambda_middle` nonplanarity check. Second-eigenvalue
degeneracy, insufficient neighbors and eigensolver/other failures account for
zero of these losses. Original score, overlap, convergence, iterations,
support counts and rejection status remain exactly unchanged.
This establishes the immediate post-NDT certification failure, not whether
voxel scale, neighborhoods crossing surfaces, scene geometry or fused layers
caused it. No threshold or registration setting was relaxed. The four startup
cells still have no admitted map observation; their 12 later queries each
remain blocked, not silently reseeded or omitted from accounting.

The approved private four-DoF backend extension adds an explicit XY GNSS
factor, using the measured 2x2 covariance and the two-dimensional 99% NIS
threshold 9.21034. XYZ callers retain their existing contract. Focused checks
cover unchanged Z mean/variance, unobserved single-position yaw, invalid
covariance, outlier rejection without estimate mutation, delayed in-window
admission, expired-state rejection and the distinct XY gate. Both map test
targets and the existing online targets pass. The test-only CSV runner also
checks LF/CRLF parity, duplicate-time/nonfinite rejection and rejected-map
ledger isolation. An initial CRLF parser failure is preserved in
`horizontal_v1/`; the corrected run uses a separate `horizontal_v2/`.

The replay uses the audited raw GNSS-off **keyframes**, interpolated at all
prepared horizontal GNSS epochs with canonical integer-nanosecond ODOM times,
linear translation and shortest-arc quaternion interpolation. Its sparse
2.5 s bracket allowance comes from the frozen graph diagnostic and does not
change the public 100 ms default. Prepared graph positions are XY-only with
uncalibrated 1 m sigma; relative H and graph recovery/reset flags are unused.
One fixed, offline future-informed initial transform includes provisional
tilt; physical gravity/up, receiver integrity and actual arrival times remain
unqualified. The target-origin XY shift is not a verified physical 3D map
registration. The backend uses its existing random-walk defaults and an
explicit diagnostic prior sigma `(1000 m, 1000 m, 100 m, pi rad)`.

| Query / diagnostic target | Raw keyframe route | GNSS admitted | LIO-only antenna XY residual median / P95 / max | XY-fused median / P95 / max |
| --- | ---: | ---: | --- | --- |
| A / B | 3,294.82 m | 614 / 773 | 8.25 / 17.54 / 18.64 m | 0.65 / 21.65 / 25.70 m |
| B / A | 2,914.28 m | 613 / 670 | 10.14 / 22.76 / 22.95 m | 0.83 / 4.06 / 19.95 m |

Residuals above include **every** prepared epoch, including rejected tails,
and compare the composed antenna position with the same GNSS used for
updates. They are not independent accuracy, and a low accepted-only residual
would be selection-biased. A's P95 worsens: its final 159 epochs are rejected,
spanning 160 s. The current model/gate is therefore not a reliable whole-route
fusion or recovery result.

All four arms retain identical input ODOM. `LIO+rejected-map == LIO-only`
and `LIO+GNSS+rejected-map == LIO+GNSS` hold exactly for backend means,
covariance diagonals and GNSS decisions. The map arms record the actual
startup rejection at the first GNSS processing epoch, retaining its original
sensor timestamp in the manifest; no later map queries were run.
This proves operation of the isolated GNSS path when no map factor is
submitted, **not** a complete runtime map-admission pipeline or successful
joint GNSS/map fusion.

A synthetic +1,000 m horizontal outlier at epoch 40 is rejected for both
captures, without changing the predicted mean/covariance versus an omitted-fix
control; the next ordinary epoch is admitted. Suppressing 60 GNSS epochs
(88 s / 67 s from first suppressed epoch to first resubmission) retains all
ODOM states and grows uncertainty, but readmission occurs only **196 s / 477 s**
after resubmission. This is first factor admission, not trusted recovery.
The first returning NIS is 9.428 / 20.797, above the unchanged 9.210 gate.
The private smoother retains its predicted alignment while rejecting; no
explicit reacquisition state machine is exercised. Distance-based uncertainty
growth alone is not a sufficient recovery policy. The size of the rejection
period is measured; attribution to covariance calibration, restricted state
model or frontend drift remains unqualified. Do not hide it by increasing the
gate, adding fictitious Z or resetting ODOM.

Immutable inputs, source/binary hashes, commands, per-epoch decisions,
fault controls, antenna trajectory SVGs, all-epoch residual CSVs and an
explicit non-pass disposition persist under
`.agents/notes/two_capture_validation_20261015/{normal_v3,horizontal_v2}/`.
The bounded diagnostic execution is complete; the full P0-P5 plan is **not
passed**. Accepted-map API/subspace/correlation/epoch integration, explicit
global recovery, physical frames, receiver admission/arrival, full-rate causal
replay and independent qualification remain open. Production remains disabled.

### 2026-10-15 two-capture input audit and bounded startup pilot

The immutable `fusion_baseline_maps_20261015_v1` exports for
`20261013143752` (A) and `20261013170722` (B) passed packaged inventory checks:
2,767/2,449 export entries and 2,761/2,443 map entries, respectively (overlapping
inventories, not distinct totals). All 5,178 local PCD headers match declared
point counts. Frame timestamps and body-TUM timestamps match integer
nanoseconds exactly; quaternion squared-norm error is at most 4.44e-16.
Original GNSS-off source dataset inventories, input/odometry bags, runtime
parameters and referenced frontend artifacts were checked. Historical launch
and runner sources have drifted; the user approved retaining these differences
as explicit diagnostic reproducibility limits, without replacing old hashes.

B is an inclusive prefix ending at sensor timestamp
`1791883379499799728`: 8,249 frontend poses retained and 1,138 excluded because
the parent trajectory gate detected a pose freeze. Neither the excluded tail
nor full-record frontend qualification is covered. The separate captures share
calibration and reference dependencies; they are not independent survey truth.

Offline optimized-route XY proximity within 40 m of a target-map keyframe is
844/2,430 (34.73%) for B-on-A and 825/2,748 (30.02%) for A-on-B. This is not a
point-cloud overlap measurement, verified 3D alignment or solver support.
Both maps still declare horizontal-only EPSG:32650 reference, unqualified
physical up and unavailable `T_enu_map`.

The user approved a matcher-only startup pilot with explicitly offline,
future-informed initialization. Each cell uses query frame 0, its optimized
startup pose and the horizontal origin offset as a provisional seed. A
successful startup would establish one frozen map-to-ODOM transform and drive
12 preselected later queries with GNSS-off ODOM; no per-frame optimized seeds
or fallback are permitted. The existing NDT defaults were unchanged.

| Target map / query | NDT convergence | Reported overlap | Match status | Processing time |
| --- | --- | --- | --- | --- |
| A / A | yes | 0.2028 | REJECTED / match_failed | 3,078 ms |
| A / B | yes | 0.2751 | REJECTED / match_failed | 4,184 ms |
| B / B | yes | 0.3126 | REJECTED / match_failed | 3,461 ms |
| B / A | yes | 0.2721 | REJECTED / match_failed | 3,166 ms |

All four startup observations were rejected. Read-only follow-up traced all
four to `MapMatcher::Evidence`'s insufficient-map-overlap gate: nonzero overlap
is assigned there only after NDT convergence, fitness and correction-basin
checks, and matched counts exceed 100 but overlap is below 0.5. Overlap counts
aligned downsampled scan points that have both a nearest target within 1 m and
a valid target normal; it is not pure geometric coverage. NDT does not populate
the ICP source/target overlap fields. The raw local windows contain
242,932–348,968 points. The earlier read-only interpretation of a shared
12,000-point cap was incorrect: that cap is conditional on the ICP backend,
not NDT. Instrumented replay below supersedes the sampling hypothesis.
No
evaluation queries, global fusion, threshold relaxation or backend selection
followed. In-sample controls also failed, so these results do not establish a
cross-capture-specific failure or an incorrect factor graph. Startup support,
scan/map sampling and overlap semantics need a bounded follow-up before full
replay. These four timings are cold-start measurements, not a latency percentile
or a production benchmark.

#### Same-input observed-support accounting

Added diagnostic fields only; matcher parameters and acceptance are unchanged.
Both map test targets passed. Repeating the four frozen startup requests
preserved score, overlap, convergence, iterations, accepted count and rejection
status exactly (timing is not expected to be identical).

| Target/query | Filtered scan | Filtered target | Within 1 m | Valid-normal support | Distance rejects | Normal rejects after distance |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A/A | 1,874 | 69,080 | 1,857 | 380 | 17 | 1,477 |
| A/B | 2,406 | 96,341 | 2,392 | 662 | 14 | 1,730 |
| B/B | 2,406 | 60,055 | 2,401 | 752 | 5 | 1,649 |
| B/A | 1,874 | 60,055 | 1,864 | 510 | 10 | 1,354 |

Thus 99.09–99.79% have geometric nearest support, but normal rejection is the
dominant measured loss in every startup. Exact rejection detail is
`insufficient map overlap`. Target normals use ten nearest neighbors and
reject neighborhoods whose second eigenvalue is near zero or whose smallest
eigenvalue exceeds 0.2 times the second. These results identify the admission
bottleneck, not why the map neighborhoods fail planarity: scene geometry,
voxel scale, sparse/line-like neighborhoods and fused layers remain possible.
No norm/planarity threshold, voxel size or matcher was changed; no fusion ran.
Results and runner identity are frozen under `support_v2/`.

Scripts, audited contracts, preserved provenance failure, coverage, frozen
pilot protocol, exact commands and per-cell results are under the ignored
`.agents/notes/two_capture_validation_20261015/` directory. The existing runner
was rebuilt in the managed non-root test container; no production algorithm
or map source was modified.

### 2026-10-15 software-contract check without new sensor data

Source review found two input-validation gaps and added fail-closed guards:
`DeskewLidarScan` now rejects finite but non-rigid LiDAR extrinsics (scale,
reflection or a malformed homogeneous row), and `GlobalAlignment::AddLocal`
rejects non-unit local quaternions before pose conversion. A zero quaternion
must not become apparently valid identity rotation through conversion.
Regression cases verify that invalid extrinsics do not look up history or
overwrite deskew output, and invalid local orientation revokes global validity
without replacing the retained local snapshot.

A further synthetic regression submits map pose and GNSS antenna-position
observations to one private conservative `GlobalAlignment`. It checks retained
local position, orientation and covariance, both source counters, and unchanged
alignment pose after a rejected GNSS outlier. This is an existing-backend
contract test with synthetic observations, not a matcher-output bridge, public
API integration or a declaration of independent sensor evidence.

The eight-target pre-fix software suite passed in the managed non-root test
container: online contracts, online alignment smoother, online localization,
GTSAM fixed-lag spike, localization algorithms, localization maps, scan-to-map,
and localization health. Four targets were cached; four executed. Third-party
build warnings included Bzlmod version resolution and PCL/Eigen alignment;
these were not treated as calibration, ABI qualification or runtime evidence.

After the two guards and three regression cases were added,
`//:localization_algorithm_tests` was rebuilt and executed again: all 96 tests
in 16 suites passed, with no failures or disabled cases. The four changed C++
files also passed managed-container Clang-Format checking. Commands, source
hashes, source snapshots and test logs/XML are archived under the ignored
`.agents/notes/localization_software_contracts_20261015/` directory.

No new sensor replay, production parameter, global backend selection, frame
leveling transform or baseline change was made. The public map-input gap,
native LIO covariance qualification, physical-frame/calibration contracts and
complete causal chain remain open as described below.

### 2026-10-15 baseline selection and LIO/GNSS/scan-to-map chain audit

The user selected wheel speed + raw gyro as the frozen DR diagnostic baseline.
Use `wheel_raw_gyro`, speed scale 1 and gyro bias 0; do not replace it with the
failed scalar-wheel ESKF, transferred steering model or fixed-bias variant.
The Apollo main-segment endpoint evidence remains 5.943 m over 523.647 m and
1.410 m over 259.210 m. Existing C++ parity and conditional Apollo ESKF
counterexamples remain separate manifests. Exact baseline files/hashes and
limitations are pinned in the ignored
`.agents/notes/ackermann_crosscheck_20261014/baseline_selection_20261015.json`.
This selection is not a production ODOM switch or independent accuracy claim.

Continue the previously selected external GNSS-off FAST-LIO2 frontend for
`20261008141912`. Wheel/raw-gyro is the separate diagnostic control where the
same record contains eligible wheel/IMU inputs, not an injected LIO factor.
The Apollo wheel records and 3dmakerpro LIO record are different captures:
their trajectory metrics cannot be treated as a paired comparison. Do not
invent a wheel channel for the LiDAR record.

Target ownership remains:

```text
LiDAR + IMU -> LIO -> immutable T_odom_base and historical poses
GNSS position -------------------+
deskewed scan + prior map -------+-> one global owner -> T_map_odom
T_map_base = T_map_odom * T_odom_base
```

Read-only source audit identifies distinct, incomplete paths:

| Surface | Verified source behavior | Chain implication |
|---|---|---|
| Native `LocalEstimator` / `LocalLio` | `LocalLio::ApplyUpdate` injects pose/velocity/bias; prior-cloud correlations are unavailable and covariance is inflated rather than independently conditioned. Successful `AddLidar` invalidates `covariance_model_valid` and joint motion export. | Numeric continuous ODOM can exist, but this is not a qualified LIO marginal or motion factor. Do not clear the guard or attach an invented covariance. |
| Private SE(3) `GlobalAlignment` | `AddLocal` rejects invalid covariance models; opt-in graph requires qualified joint motion. `GlobalObservation` supports pose, point position and projected pose. Unanchored point position does not initialize full global pose; graph observations require declared independence from local motion. | Native LIO cannot currently feed a qualified graph through this path. A same-record scan/map observation must not simply set `independent_of_local=true`. |
| Public `LocalizationEngine` | Accepts external ODOM, GNSS fixes and headings; private GTSAM smoother estimates 4DoF alignment. No scan-to-map submission API exists. `OdometrySample` carries pose/epoch/time, not a local covariance or joint motion model. | External FAST-LIO2 + GNSS is the existing diagnostic path, not a full LIO/GNSS/map fusion implementation or qualified local uncertainty propagation. |
| `ScanToMapLocalizer` | Produces timestamped map observations only, with calibration, physical-up, observability and diagnostic flags; never changes frontend ODOM. | A typed adapter connects its result to the diagnostic `GlobalAlignment` owner. `DiagnosticGlobalAlignment` now offers a public diagnostic map/XY interface over that same owner, not a qualified product API; `MATCHED` alone is not admission. |

The source has both a public 4DoF alignment engine and a private SE(3)
candidate. They are not interchangeable and must not become two competing
global publishers. A scan-to-map bridge needs an explicit backend/frame
decision and compatible observation/correlation semantics, not just a call
to a similarly named class.

Existing evidence is component-level only: GNSS-off FAST-LIO2 trajectory
integrity, 148/162 accepted GNSS fixes in an isolated diagnostic, and
same-record scan-to-map matches. No complete causal LIO + GNSS + map replay
is established by combining those results. FAST-LIO `camera_init` +Z is not
qualified as physical up; the current 4DoF backend assumes gravity-level
ODOM. Body/IMU equivalence, GNSS lever arm and uncertainty remain conditional.
The selected map is horizontally georeferenced with unqualified physical up
and uncalibrated matcher covariance. See `plan/g100-rtk-data-validation.md`
and the linked map-assisted plan for provenance and existing gate failures.

Next order:

1. Close external LIO pose/time/epoch/body-frame and gravity-level contracts.
   Establish a consistent LIO/map/GNSS transform chain before any new combined
   replay; do not enable the previously provisional leveling rotation as fact.
2. Keep one global backend and complete the adapter's remaining qualification:
   map/calibration/frame identity, observable projection, covariance
   calibration, local/map/shared-GNSS correlation, and rejected-reason
   accounting. The private diagnostic adapter now enforces budgeted source-time ODOM
   association and map identity, preserves covariance provenance, and never
   claims local independence. Add a public runtime map-submission API only
   after its frame/subspace/correlation contract is defined. Preserve GNSS as
   position evidence rather than invented pose.
3. Add focused synthetic checks for immutable ODOM, delayed GNSS/map inputs,
   null-space preservation, contradictory observations, source outages and
   recovery. Do not call partial position acquisition full-pose certainty.
4. Freeze and approve a bounded causal replay protocol for LIO-only,
   LIO+GNSS, LIO+map and LIO+both. Reuse identical GNSS-off ODOM and map;
   GNSS/map corrections affect only alignment, and scoring dependence must be
   explicit. Production admission remains separate from diagnostic replay.

#### 2026-10-19 implementation progress

The [global integration plan](../../../ad_agents/context/modules/localization/knowledge/openloc-global-integration-plan.md)
is now in staged implementation, not complete. The GNSS quality gate returns
typed accepted/waiting/rejected/invalid decisions, validates its own policy,
and preserves policy/history on invalid configuration. Engine policy validation
reuses that same validator; position accuracy is no longer passed twice, and
recovery no longer inspects private dwell state. Invalid input/order/clock
cannot advance dwell. Waiting still admits no recovery sample.

The map adapter now retains the exact integer measurement timestamp instead of
replacing it with local history time. Explicit `MapObservationTiming` supplies
independent arrival and source sequence; the legacy overload borrows local
arrival/sequence and leaves exact integer arrival unavailable. The owner
rejects inconsistent integer/double timestamps. Its history/frontier remain
double seconds: the conversion budget is not nanosecond resolution. Shared
geometry validation now serves both adapter and owner, including rigid pose,
positive covariance and full-row-rank projection checks. Adapter failures
leave prior output unchanged. The public result also carries its configured
`odom_gravity_contract`.

The CLI regression exposed a missing identity propagation: the runner pinned
map content/frame/base in the owner but omitted them from GNSS observations.
The caller now fills those fields; strict owner identity checks were not
relaxed. All three CLI cases then passed in the managed test container.

Latest software checks: five online targets (contracts, quality, recovery,
smoother, engine) and `scan_to_map_tests` passed; the global algorithm target
passed the selected `GlobalAlignmentTest.*`, `FixedLagGraphTest.*` and
`MapObservationAdapterTest.*` suites (42 cases, including 9 adapter cases).
The quality gate has 8 passing cases. Both replay binaries built. Scoped
Clang-Format and whitespace checks passed. Compilation still emits the
existing PCL/Eigen aligned-allocation warning; these tests do not qualify ABI
compatibility across all external consumers or target hardware.

S1 is complete; S2 records the current diagnostic owner/model decision.
S3's time/geometry boundary and S4's diagnostic integration now include a
PIMPL public `DiagnosticGlobalAlignment` facade over the existing SE(3) owner,
with no PCL/GTSAM/candidate types in its header. It has bounded
measurement-time queues and decision history, retires incompatible queued
epochs, accepts full/projected map and antenna XY, and exposes no alignment
until distinct full-pose observations verify a basin. Five focused facade
cases passed, including startup without an oracle prior, delayed projection
plus GNSS in one revision stream, no invented alignment from XY alone,
epoch retirement and capacity/stale/invalid-input handling. Geodetic
declaration validation is shared with the 4DoF engine; existing contract and
engine regressions passed after extraction.

The source-correlation model and qualified unified runtime
publication/recovery remain open. S5/S6 remain open: both existing
two-capture manifests explicitly use an offline-informed startup prior and
the events have no independent arrival column. No new real-data causal
baseline, qualified joint local-motion model or independent accuracy evidence
is claimed. Frozen data/releases were not modified or relabelled.

Existing contracts remain enforced: failed/ambiguous matches and identity
mismatches are rejected, projected observations do not invent missing
directions, and diagnostic-only evidence cannot enter a trusted owner.
Covariance provenance remains separate from matcher score. The public 4DoF
body roll/pitch regression does not treat body attitude as evidence that
parent ODOM axes are gravity-level.

These are software-contract completions only. No validated `camera_init` to
level transform, qualified product map-submission API, qualified native LIO
covariance, correlated map/GNSS error model, causal recorded-data replay, or
independent truth has been established. Keep the next dataset replay blocked
until exact input exports/provenance and a bounded causal protocol are
available; keep any unresolved gravity declaration diagnostic-only. The old
root `modules/localization` source path is absent and is not a verified
integration entrypoint.

Evidence is software/development evidence, not release or vehicle
qualification. Do not describe an unintegrated candidate as production-ready.

#### 2026-10-20 diagnostic lifecycle and replay

The public SE(3) facade now forwards explicit map ambiguity/recovery to its
existing owner. Contradictory evidence cannot bypass the innovation gate;
recovery temporarily removes the basin, ambiguity restarts verification, and
distinct consistent full poses are required to hand over one correction
revision stream. Decisions distinguish retained position evidence from an
actual alignment update, have monotonic journal IDs across epochs, and report
eviction. Explicit-time result evaluation separates ODOM age from full-pose
evidence age; partial updates do not renew the full-pose age. Stale diagnostic
estimates remain labelled as such, not promoted to trusted output.

`//:diagnostic_global_alignment_test` now has 8 passing cases, covering
contradiction/ambiguous recovery, outage age boundaries and committed-frontier
rejection in addition to the previous cases. `//:online_contracts_test` passed.
Both `//:diagnostic_alignment_replay` and the old
`//:three_source_dataset_replay` built; the old runner's 3 CLI regressions
passed after extracting shared CSV helpers.

The new public prepared-observation runner has no startup-prior input and
does not run scan matching. Eight CLI cases passed in the managed test
container: four-arm ODOM identity, delayed projected updates/null-space,
unresolved pending tail, epoch retirement, ambiguity/recovery, covariance
cross terms, invalid geometry, duplicate/late/stale events, explicit errors,
optional streams and no overwrite. All input events are retained, including
excluded sources; uninitialized global fields are empty. Final decisions and
actual revisions are journalled separately. This verifies software causality
at the public submission boundary, not upstream map/measurement causality.

Synthetic fixture inputs, outputs, command logs, test-source snapshot and
SHA-256 manifest are saved under
`.agents/notes/global_integration_20261020/prepared_replay_v3/`.
The first run's failed epoch fixture is retained in `prepared_replay_v1`:
it put measurement time 25 ms ahead of reception and correctly hit the
existing 20 ms future gate. The fixture alone was corrected to a 15 ms lead;
no implementation threshold was changed. `prepared_replay_v2` records the
intermediate 7-case pass. These archives are not a new recorded-data release.

Scoped Clang-Format and whitespace checks cover the C++ edits. Python
Black/isort/Flake8 and Buildifier were unavailable; no packages were installed
and no pass is claimed for those checks. The existing PCL aligned-allocation
warning remains present in the old matcher-dependent runner.

S5 recorded-data acceptance and S6 qualification remain blocked by the same
external evidence: genuine source-bound arrivals/causal startup and upstream
preparation, independent frame/time/calibration and error/correlation
evidence, suitable independent reference and approved product budgets.
The input/export format is documented in the package README; its existence
does not approve a real-data protocol or qualify production publication.
Frozen previous releases and default backend/admission policy are unchanged.

#### 2026-10-20 source-dependency audit contract

`GlobalObservationContext` now separates diagnostic acquisition/preparation
identities from four tri-state dependencies: ODOM initialization, ODOM deskew,
shared local scans and GNSS map construction. Unknown is the default, not an
independence declaration. Caller-declared absence does not prove independence,
and no provenance tag changes the backend, fusion weight or qualification.
Invalid enum values are rejected before advancing a source frontier.

Final decisions now also retain covariance provenance on application,
rejection and epoch retirement. The prepared replay accepts an optional
six-column provenance suffix; legacy inputs retain unknown dependencies.
Both event and decision logs carry provenance, including excluded sources.
No raw-file provenance or causal timing is inferred from strings or tags.

The facade suite now has 10 passing cases and the prepared CLI suite has
10 passing cases. The old runner's 3 CLI regressions still pass. The new
cases exercise queue completion, early geometry rejection and epoch retirement
without dropping provenance, invalid-tag rejection without consuming the
source frontier, unknown defaults and excluded-source audit retention.
Synthetic replay inputs/outputs/source/manifest are frozen separately in
`.agents/notes/global_integration_20261020/prepared_replay_v4/`.
Earlier archives remain unchanged. The 10-case run is a software-contract
check, not the S5 recorded-data baseline or the missing qualified correlation
model. S5/S6 and qualified production publication/recovery remain open.

#### 2026-10-20 replay measurement and accounting

The prepared replay now records signed arrival delay, queue wait and dispatch
decision time, per-event steady-clock processing duration, and correction
translation/rotation at a common current ODOM body. Initialization, absent
basins and epoch changes have no correction comparison, rather than a fake
zero. Summary records pending peak, maximum wait/correction, nearest-rank
processing P50/P95/max and Linux process peak RSS. Timing/RSS are measured
offline and include documented replay overhead, not product budget evidence.

`GetDecisions` makes pre-ODOM rejections accessible immediately; they are no
longer journalled at the first later ODOM event with a misleading queue wait.
Before successful completion, selected observations must equal updated plus
non-OK plus retained plus pending observations. No tail event is discarded.
The facade has 11 passing cases; the instrumented replay has 12 passing
cases, including exact delayed queue timing, pre-ODOM rejection timing,
ODOM-motion-free correction magnitudes and decision accounting.
Latest synthetic inputs/outputs/command/source/hash evidence is in
`.agents/notes/global_integration_20261020/prepared_replay_v6/`.

No further source of genuine capture arrivals, independent frame/calibration/
truth or approved product requirements was supplied. These changes finish
additional measurable software contracts, not S5 recorded-data or S6
qualification; qualified production publication/recovery and correlated
joint-motion models remain unimplemented. Do not mark the full plan complete
or introduce unvalidated defaults to remove those gates.

#### 2026-10-20 recorded-source availability blocker

A read-only check of the pinned source paths found that both captures'
frontend manifest, source dataset and `odometry.bag` are currently absent at
their recorded locations. The raw LiDAR bags still exist at their recorded
paths; this check did not rehash or decode those bags. An exact-pattern
workspace lookup also found no relocated first-capture frontend manifest.
Absence at the pinned paths does not establish deletion or permanent loss.

The old integrity audit remains historical evidence, not current source
availability. Sparse frozen measurement-time events cannot recover frontend
completion/delivery arrival; raw-bag record time is not a substitute for
frontend-output arrival. No new recorded-data replay was run. Restore or locate
the pinned artifacts, or explicitly freeze and approve a new frontend rebuild
protocol before proceeding. No dependency was installed and no source data,
threshold or default backend was changed.

The exact path-existence results and historical expected hashes are preserved
in `.agents/notes/global_integration_20261020/source_availability_v1.json`.
This is an input blocker, not an algorithm rejection, test pass or completed
S5/S6 qualification.

The subsequent scoped capture-parent check found **new**, not relocated
byte-identical, GNSS-off runs from 2026-10-19:
`fast_lio_session_20261019_full_gnss_off_production` (9,152 indexed ODOM
messages) and `fast_lio_session_20261019_safe_prefix_gnss_off_production`
(8,245). Both ODOM bag SHA-256 values match their new manifests:
`5658c2cd77ea4da75ba30c8ce16b0d22966a630e508d9aba5af0340562dfacdf`
and `aea84d479a7f2f329669530a3c0a90068e8b2a765e2395712aacea7955bd5492`.
Their settings explicitly disable GNSS input and loop closure, use 2x
playback, and the second run declares a 1.75 s start/824.799 s segment.
These are candidate sources for a new protocol, not recovery of the old
release or evidence that its missing artifacts were found.

Bounded decoding of the first 100 ODOM messages per bag preserved recorder
and header times as separate integer nanoseconds. Recorder-minus-header
time ranges were [-8,697,751, 37,175,018] ns and
[-3,351,487, 18,864,297] ns; 39/100 and 18/100 samples respectively had a
recorder timestamp before their measurement header. This prevents treating
recorder time as a proven physical arrival or applying an unexamined common
clock contract. No timestamps were clipped, shifted or replaced.
Existing host `rosbags` was used read-only; no dependency was installed,
no algorithm replay was run and no raw bag was modified.

Exact manifest identities, bag hashes, settings, first-100 time/frame CSVs
and the audit script are in
`.agents/notes/global_integration_20261020/candidate_timing_v1/` and its
parent `audit_candidate_timing.py`. A new approved protocol must declare
clock/delivery semantics, truncated coverage, map construction/startup and
the selected frontend identities. Independent calibration, correlation
and product qualification gates remain unchanged.

| Area | Current evidence | Status |
|---|---|---|
| Package boundary | Openloc is a middleware-neutral localization library; host repositories own sensor adapters, middleware, lifecycle, and actuation. | Documented boundary. |
| Local LIO/DR candidate | Private `LocalEstimator` propagates a 15-state IMU error-state estimate and calls `LocalLio` for deskewed scan-to-local-voxel-plane updates. Scalar signed base-forward wheel speed is the only wheel model. A separate `StationarityDetector` requires a consecutive sample count and dwell (2 s default); a three-axis zero-velocity update is opt-in and requires explicit variance. NHC remains opt-in. Focused local behavior can be tested independently with `//:local_estimator_tests`. | Synthetic behavior passes. A bounded 2026-10-10 chassis-speed + raw-IMU replay now exercises 58.9 s of one trigger-record segment through the OpenLoc estimator; results and the NHC ablation are summarized below. This is same-record receiver RTK/INS consistency only, not independent accuracy, calibrated covariance, or vehicle qualification. The older `20261008141912` LiDAR recording remains unsuitable for wheel-aided DR initialization. |
| FAST-LIO2 | External GNSS-off frontend baseline exists for `20261008141912`; the exact source/config/executable and input hashes are pinned in the run manifest. | Selected frontend for the current OpenLoc global-alignment data diagnostic; this does not integrate FAST-LIO2 into the OpenLoc library or establish target-hardware qualification. |
| GNSS/global alignment | Public `LocalizationEngine` estimates a four-DoF global-frame-to-ODOM transform, exposes `global_alignment_valid` with position-initialization and outage semantics, and requires an explicit gravity-alignment or diagnostic-assumption declaration. Its result includes component-level observability and geodetic-reference declaration fields. | Focused synthetic Bazel tests include nonzero body roll/pitch, but do not prove the external FAST-LIO2 `camera_init` frame is horizontal; its horizontal gates/yaw interpretation remain unvalidated. Neither output field certifies numerical rank, sensor calibration, integrity, or accuracy. No production factor admission or absolute-accuracy qualification. |
| OpenLoc dataset diagnostic | The isolated adapter uses GNSS-off FAST-LIO2 ODOM, payload-time GGA, provisional 1.0 m horizontal sigma, a test-only 120 ms interpolation bound, and a conditional/unverified lever arm. The C++ diagnostic adapter's raw-GGA `H+N` test passed against the mounted source file; this is not independent height truth or production middleware qualification. | The complete 1,637-row export reconstructs 148/162 accepted, 3 stable-start rejects, and 11 distance rejects. A 1,625-row baseline is an exact prefix with 12 omitted tail rows, not an estimator-filtered set. The per-epoch RTCM report matches GPS 1074 time fields for 162/162 epochs (station 271) but does not prove corrections were applied. All 11 causes remain `UNRESOLVED`. Machine-readable artifacts are listed in [`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md). Same-record consistency only; no production admission or accuracy evidence. |
| Phase 0/1 prioritized diagnostics | P0-1 C++ diagnostic-adapter `H+N` check, P0-2 fail-closed reference contract, P0-3 runtime-gravity audit, P0-6 quality/age/RTCM statistics, P1-1 per-rejection neighbor report, ODOM provenance reconciliation, and per-epoch GPS MSM4 association are recorded in [`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md). | P0-1 data-path test and P0-6 descriptive statistics pass; P0-2 software contract passes and the schema-backed diagnostic reference (UTM EPSG:32650, first-GGA origin, ellipsoidal height from GGA `H+N`) is declared explicitly. Its precise source CRS realization/epoch and independent survey provenance remain open. P0-3 remains **UNRESOLVED**; no level transform is enabled. G-P0 is not passed because physical body-frame semantics and surveyed lever arm remain unresolved. G-P1 is inconclusive; all 11 gate rejections remain `UNRESOLVED`. |
| L1 + GNSS dataset evidence | A full-recording GNSS-off FAST-LIO2 baseline and same-record OpenLoc global-alignment diagnostic exist for `20261008141912`. | External FAST-LIO2 frontend plus OpenLoc global-alignment consistency evidence only; OpenLoc native L1, independent accuracy, calibrated uncertainty, and production GNSS admission remain unvalidated. |
| Historical map candidate | PCL `MapMatcher` synthetic known-transform, repeatability, invalid/low-overlap, and degeneracy suite has 25 passing tests. | Historical software evidence only; prior-map matching is outside the current LIO + GNSS scope. |
| Map-assisted localization | The selected georeferenced map's global PCD contains 87,856 points (about 1.4 MB), with no generated tiles. Its horizontal CRS is EPSG:32650; physical up is unqualified and GNSS alignment is diagnostic/non-independent. `ScanToMapLocalizer` retains NDT as its default and now offers an optional custom point-to-plane ICP backend. | Synthetic `scan_to_map_tests` and `localization_map_tests` pass. Earlier NDT ablations and high-error-frame overlays remain in-sample; no geometric/dynamic root cause was established. The 2026-10-12 packager-oriented ICP replay processed all 196 keyframes: 145 solver convergence flags, 19 `MATCHED`, and 177 `REJECTED` (51 non-converged; 126 converged but below the 0.4 target-overlap gate). Overlap is measured against the selected local map window, not the entire global map; the 0.4 bidirectional threshold was not shown suitable for this larger target. Latency was 47.9/190.4/691.1 ms median/P95/max. A deterministic eight-entry window-cache simulation found four cold window builds (517–691 ms); the 691 ms maximum was also a 40-iteration non-convergence. Cached 40-iteration failures had 158 ms median and 206 ms maximum, indicating both window preparation and iteration-limit failures contribute; stage timing is not instrumented. Frames 66–67 and 150–153 converged but were rejected with target overlap 0.279–0.355. Raw-cloud projections show layered planar/vertical structure but do not establish a corridor label or physical degeneracy/dynamic cause; the result remains **UNRESOLVED**. Packager ICP is used for loop-closure candidates, not global scan-to-map against a fused-map window; voxelization also differs. This is operator/config comparison, not full pipeline equivalence or independent accuracy. Covariance remains uncalibrated; see the validation plan and ignored `scan_to_map_point_to_plane_icp_20261012` replay archive. |
| Dataset inputs | `/mnt/synology/3dmakerpro/schema/` documents ROS1 bag v2.0 schemas, Livox LiDAR/IMU topics, embedded UTC/payload timestamps, point fields, Mid-360 profile, and GNSS/RTK evidence for `20261008141912`. Raw NMEA cross-check supports delayed bag publication for a sampled GGA; all 162 GGA payload epochs lie within the FAST-LIO2 ODOM range with 2.450 ms P95 and 11.678 ms maximum nearest scan-end residual, with no fitted offset. PPS/absolute accuracy remains unresolved. | File/schema and sampled-time-axis checks passed; they do not establish clock discipline, independent truth, surveyed lever arm, or calibrated GNSS uncertainty. |
| Absolute accuracy and calibrated uncertainty | No independent-truth qualification evidence is recorded. | Not established. |

The earlier MapMatcher test evidence must not be presented as evidence for
LIO+GNSS. Passing software tests do not establish field accuracy, runtime
performance, calibrated protection levels, safety, or production readiness.

### 1010 chassis-speed + IMU DR diagnostic

The bounded adapter exported only
`2026-10-10-15-00-54/20261010070054.record.00000` (60.0 s; SHA-256
`327f0f54c6a916f77f9a926ef2bad87331c853f6b30e26d0dba75343cf3ca256`).
It required one isolated `.record.00000`, exact indexed channel types/counts,
monotonic sensor times with gaps below 50 ms, automatic DRIVE/no chassis
errors, continuous `SOL_COMPUTED` + `INS_RTKFIXED` BestPose and heading, and
a 2 s joint quiet window. The record passed: each of chassis, raw IMU, BestPose
and heading had 5,895 samples; maximum horizontal receiver-reported RTK sigma
was 0.03 m; the initialization window was 4.967 s/498 IMU samples; maximum
BestPose payload-to-header residual was 11.25 ms. The positive `speed_mps`
direction agreed with RTK heading throughout six forward windows; integrated
chassis distance was 80.72 m versus 79.09 m RTK forward projection (+2.07%).
This is an in-record semantic cross-check, not calibrated wheel scale.

OpenLoc replay used wheel-speed variance `0.01 (m/s)^2`, disabled ZUPT, and
one-time initial yaw/position alignment from the first valid state and the
same-record dual-antenna heading/BestPose. There was no subsequent trajectory
fit. Scores use 5,693 bracketed estimator states from the 58.9 s post-start
window. Integrity was valid for 5,694/5,895 output rows (96.6%; the other 201
rows are startup waiting), but the stricter precision policy was
qualified for only 191/5,895 rows (3.2%) without NHC and 945/5,895 rows
(16.0%) with NHC; remaining post-initialization rows reported
`COVARIANCE_EXCEEDED`.

| Diagnostic configuration | Horizontal error median / P95 / max | Relative translation error at 10 / 30 / 55 s | Precision-qualified rows |
|---|---:|---:|---:|
| Wheel + IMU, NHC off | 25.88 / 146.64 / 182.92 m | 4.62 / 35.61 / 156.66 m | 191/5,895 (3.2%) |
| Wheel + IMU, NHC on (`0.01 (m/s)^2` provisional) | 0.66 / 4.46 / 5.60 m | 0.092 / 0.680 / 4.805 m | 945/5,895 (16.0%) |

NHC substantially reduced this straight-run lateral divergence (NHC-on absolute
lateral P95 4.38 m, longitudinal P95 0.89 m), but accuracy still degraded over
time: the NHC-on horizontal-error P95 grew from 0.11 m in the first 10 s to
5.45 m in the final 8.9 s. This replay therefore **does not pass DR precision
or accuracy qualification**. NHC is an opt-in model experiment, not a promoted
default; the comparison is optimistic and correlated because the receiver RTK/
INS data supplies both the heading used for the single initial yaw alignment
and the scoring reference. Reference-to-base lever arms, sensor-frame
calibration, independent truth, reverse/turn/slope/slip behavior, and
outage/recovery remain unverified.

The 2026-10-13 baseline follow-up froze the selected record, event/reference
CSVs, replay/evaluation sources, relevant estimator source files, lockfile,
and both trajectory/score outputs by SHA-256. The NHC ablation check confirms
the same record/events/reference and common estimator settings; the only
algorithm change is NHC off versus NHC on with its provisional
`0.01 (m/s)^2` variance. The generated overlay and per-state diagnostics show
NHC-off error is predominantly lateral (P95 146.43 m lateral versus 7.87 m
longitudinal). Propagated estimator yaw differs from the same-record
dual-antenna heading by +6.47° at the end with NHC off and -14.31° with NHC
on. Raw IMU `ay` has stronger correlation with the central chassis-speed
derivative than `ax` (Pearson 0.534 versus -0.089 over 5,353 samples). This is
a potential axis/mounting mismatch requiring direct frame verification, not a
calibration result: the record has weak, uncontrolled excitation, and the
heading and RTK are not independent truth. The source files, plots, covariance
diagonals, per-state residuals and deltas, and full freeze manifest are in the
session artifact `files/1010-dr-export-fixed/baseline_diagnostics/`. The
original freeze did not include a replay-binary hash; the controlled
2026-10-13 reruns below identify their executable separately and do not imply
that it is byte-identical to the original baseline executable.

Timestamp gates passed for this one segment (maximum IMU/chassis gaps 16.9/20.0
ms, zero wheel samples behind the IMU processing frontier; BestPose and heading
payload/header maximum absolute residuals 11.25/11.34 ms). These are stream
integrity checks, not proof of PPS lock or calibrated sensor-to-sensor latency.
The estimator consumes the raw IMU axes directly while its wheel update treats
body X as forward and, with NHC, constrains body Y/Z velocity. Until the
physical IMU-to-base rotation is verified, the Y-vs-X acceleration correlation
and NHC-on yaw drift make frame/bias-model mismatch a leading investigation
item, not an established root cause.

#### 2026-10-13 IMU-frame and reverse-speed follow-up

An explicitly selectable `apollo-rfu` adapter hypothesis now rotates IMU
acceleration and angular velocity as `(x_flu, y_flu, z_flu) =
(y_rfu, -x_rfu, z_rfu)`; the default remains unchanged. The same 15:00:54
record, same RTK reference and estimator settings were replayed with the raw
axes and with this candidate transform using replay-binary SHA-256
`95f4d4b9325ce8bfb9f636acb260b8aabdb8811afa50c5cd55d6f0e364fa90c8`. The raw
control retains frozen event hash
`7da60f3a2a847fd4388a0466d70e490e16df288e7b147762b8bc08970051de8e`; both
runs use reference hash
`66b5e880c2bffc049ae569c452dd49c226ac55ec090c2283e6942233b522c080`.
NHC-on horizontal error changed from
`0.66/4.46/5.60 m` median/P95/max to `0.65/3.98/4.84 m`; relative error at
55 s changed from `4.805 m` to `4.237 m`, while precision-qualified states
changed from `945/5,895` to `934/5,895`. The NHC-off P95 increased from
`146.64 m` to `211.87 m`, despite lower median error, while end yaw difference
improved from `+6.47 deg` to `-2.06 deg`. This mixed same-record result does
not establish the frame transform; do not promote it or tune the wheel scale.

The independent 11:01:03 REVERSE segment passed the exporter with gear-signed
speed in `[-2.01, 0] m/s`; its six RTK forward-projection windows also had
negative signed wheel integrals, consistent with reverse direction. The
estimator replay was rejected: 195 chassis samples precede the first IMU
sample by about 1.95 s, exceeding the fixed 64-sample pending-wheel queue
before IMU propagation starts. No samples were trimmed or reordered. The
14:52:02 DRIVE segment had four wheel events behind the IMU frontier, and
other screened candidates failed existing RTK-heading quality or timestamp/
gap gates; none were silently admitted.

Across the screened records, static acceleration remains near `+Z`, and the
multi-record turn-rate check was recomputed with 0.2 s heading windows. In the
11:01:03 REVERSE segment, raw gyro Z versus ENU yaw rate had correlation
`0.9973` and through-origin slope `0.9952` over 4,531 moving-turn samples; in
the 10:54:56 mixed DRIVE/REVERSE segment the corresponding values were
`0.9973` and `1.0012` over 5,245 samples. The 14:57:55 REVERSE segment had
correlation `0.9822` and slope `1.0018`, but its payload/header time residual
reached 2.28 s, so it is not relied on for timing qualification. Raw
`speed_mps` was positive for moving samples in reverse; after gear-signing,
the 11:01:03 six RTK forward-projection windows agreed in sign. Static
acceleration remains near `+Z`, and acceleration versus raw speed-derivative
correlations are weak without a calibrated time/lever-arm model. These
observations support gyro-Z yaw sign and gear-based reverse speed semantics,
but do not determine the full IMU-to-base rotation.

Apollo source paths are not a substitute for the selected record's producer
configuration: the GNSS MSF default `imu_coord_rfu=true` consumes the input
as RFU, while the Huace parser explicitly transforms its output to FLU; the
generic data parser has another transformation path. The source record's IMU
`frame_id` is empty and the active runtime publisher/configuration for these
records remains unknown. Keep RFU-to-FLU as a candidate only. Reproduction
inputs, raw/candidate trajectories, evaluator summaries and hashes are in the ignored
`pkgs/openloc/.agents/notes/chassis_imu_dr_calibration_20261013/` and session
artifacts. The RTK/wheel/DR overlay and error plot is
`files/1010-dr-export-fixed/coordinate_calibration_20261013/forward_150054_raw_vs_rfu_nhc_on.png`;
the source records remain read-only.

The exporter, event CSV, replay trajectories, per-frame scores, and hash-pinned
manifests are kept in the current Copilot session files under
`files/1010-dr-export-fixed/`; the source record remains read-only.

#### 2026-10-13 event-level IMU/wheel trace

An opt-in estimator trace was replayed on the same RFU-converted event CSV
(SHA-256
`9606849d2191c207f5230e77b1c46bd581f11aa01ad3f5b93d1ac6b1901ef943`), with
wheel variance `0.01 (m/s)^2`, ZUPT disabled, and NHC off/on as the only
configuration difference. The replay executable SHA-256 is
`f3751f1bd17cc2cef1ddf17f876b96fc349d9c96fd0069f57b665ff9e33c9505`.
Each run emitted 11,386 midpoint IMU propagation events and 5,693 successful
wheel updates to
`pkgs/openloc/.agents/notes/chassis_imu_dr_calibration_20261013/forward_150054_rfu/trace_20261013/`.
The diagnostics CSV SHA-256 values are `8440c77d42be3f0f2d56919d825fd248bf853b08b8183d7d02df9f9d66c71f4a`
(NHC off) and `c1ba5667c84de7cad7ee2b7f3d4365a6a13907df710bbe6e07caf1bc3f493c64`
(NHC on); the trajectory CSV hashes are `4fa92b5f4c8176708933f9305c87988da467aa4900ab108a84f38e341a9f925c`
and `20f64052269a56f3fb0a4fcce4dcd53850a89baf0027afe55fdea3203517ea8d`,
respectively.
The trace's body acceleration and angular-rate fields are midpoint averaged
inputs after subtracting the current estimated accelerometer and gyro biases;
they are not raw sensor samples.

| Configuration | IMU propagation Δv, local Y | Wheel-update Δv, local Y | Final local-Y velocity |
|---|---:|---:|---:|
| NHC off | +3.6652 m/s | +16.7822 m/s | +20.4474 m/s |
| NHC on (`0.01 (m/s)^2`) | +0.6890 m/s | -1.0030 m/s | -0.3139 m/s |

For both runs, propagation `Δv` agrees with the sum of logged midpoint
acceleration times `dt` to less than `2e-14 m/s`; the discrete propagation
position increments match the implemented `v_before*dt + 0.5*a*dt²` formula,
and consecutive trace states join without a discontinuity. The final trace
state exactly matches the replay trajectory output. This closes the earlier
approximate `~16.78 m/s` NHC-off discrepancy: most of the local-Y velocity
growth is applied during scalar wheel updates, not directly by IMU
propagation. With NHC disabled, the one-dimensional forward-speed Kalman
update can still modify lateral velocity and IMU biases through the full
state covariance cross-terms; the trace establishes where the correction is
applied, not why those cross-covariances become large.

This does not identify a unique frame, bias, or covariance-model root cause,
nor make local-Y identical to instantaneous vehicle lateral velocity as yaw
drifts. It is not a new accuracy score or independent truth result; the trace
does not use the RTK reference. The raw source record remains read-only, and
no estimator defaults or propagation equations were changed.

#### 2026-10-14 wheel observation-model audit and update ablation

The scalar forward-wheel measurement implemented by `UpdateWheel` is
`h(x) = e_x^T R_local_body^T v_local`. With the estimator's right-multiplicative
body attitude error, its Jacobian has velocity block `e_x^T R_local_body^T`,
attitude block `e_x^T [v_body]_x`, and zero direct position/bias blocks. The
logged Jacobian matches these derivatives over all 5,693 accepted updates
(maximum analytic discrepancy `1.4e-17`); a focused finite-difference unit
regression now covers velocity and attitude columns. Direct local-Y velocity
sensitivity `|H_vy|` is `0.0162/0.0256/0.0283` (median/P95/max). The attitude
derivative norm is `2.38/15.64/20.44 m/s/rad`; its increase follows the
estimated body lateral speed, which grows to `20.44 m/s`. This is a
state-dependent sensitivity, not evidence of an incorrect Jacobian by itself.
The model remains conditional on chassis speed referring to the estimator's
motion reference point; the data does not independently verify that point.

The gain and covariance trace shows the large lateral correction is driven by
`P H^T`, not by direct sensitivity to local-Y velocity. Absolute `K_vy` is
`0.315/3.70/37.74` (median/P95/max), while the innovation variance `S` is
`0.0145/1.60/6.44 (m/s)^2` against the configured measurement variance
`0.01 (m/s)^2`. At the largest lateral update (`+1.763 m/s` at 27.89 s),
`K_vy=22.87`, innovation is `+0.0771 m/s`, and `S=0.01323 (m/s)^2`.
At the largest absolute gain (`-37.74` at 44.99 s), the innovation is only
`-0.01073 m/s` and the resulting lateral correction is `+0.405 m/s`.
Across updates, `K_vy = P(v_y,:) H^T / S` closes within `2.0e-12`, and the
logged Joseph covariance update for the `P(v_y,:)` row closes within
`1.2e-10`; the lateral state injection `delta_v_y = K_vy * innovation`
closes within `1.8e-15`. The implementation deliberately zeros position and
attitude gain rows while applying velocity and bias rows.

The covariance growth is primarily in IMU propagation, not the wheel update:
`P(v_y,v_y)` starts at `0.0025 (m/s)^2`, reaches `17,728.6 (m/s)^2` with wheel
updates, and reaches `25,924.7 (m/s)^2` without them. In the wheel-on run,
propagation adds `19,471.5 (m/s)^2` cumulatively and wheel updates remove
`1,742.9 (m/s)^2` cumulatively. At 55 s, the reported local-Y velocity sigma
is `124.69 m/s` with updates and `150.37 m/s` without. The replay uses the
default initial gyro-bias standard deviation `0.01 rad/s`; the scale
`g * sigma_bg * t^2 / 2` is `148.3 m/s` at 55 s, close to the no-wheel
covariance sigma. This suggests the declared gyro-bias prior and gravity
coupling dominate the uncertainty growth; it is not a calibration of that
prior or proof that every covariance-model term is correct.

The explicit ablation used the identical event CSV, wheel variance, NHC-off
setting, and IMU propagation schedule (11,386 intervals) in both runs. Wheel
measurements remained available for the same startup/stationary
initialization; only post-initialization Kalman wheel-velocity updates were
disabled in the ablation. The first valid state and its covariance were
exactly equal in the emitted trajectory (`0` maximum difference), and both
runs consumed 5,895 IMU and 5,895 chassis samples. At 55 s, wheel-on minus
wheel-off local-Y position was
`+217.96 m`; at the end it was `+251.81 m`, and local-Y velocity differed by
`+20.06 m/s` (`20.447` versus `0.389 m/s`). Thus wheel updates causally
produce the divergence in this estimator/record combination, through the
measured state corrections and their effect on subsequent propagation. The
wheel-off replay reports `WHEEL_STALE` precision after the timeout; this
controlled ablation is not a production-mode recommendation or an accuracy
comparison against independent truth.

The executable SHA-256 is
`91bceb566a54c02a4510143b89926d6743057c2355cdeb45ca064609b60f84e1`; both
runs use the event CSV SHA-256
`9606849d2191c207f5230e77b1c46bd581f11aa01ad3f5b93d1ac6b1901ef943`.
The on/off traces and trajectories are preserved in
`pkgs/openloc/.agents/notes/chassis_imu_dr_calibration_20261013/forward_150054_rfu/wheel_model_ablation_20261014_cli_verified/`.
The trace hashes are `26fa6d7672101855d7afc49063de9e8adf919ad1234bec8eaeb5caad0c8dde57`
(wheel on) and `c09007d41c12cd13a5194903045f7a903443f6404e80c8e26f15bed29411611e`
(wheel off); trajectory hashes are `4fa92b5f4c8176708933f9305c87988da467aa4900ab108a84f38e341a9f925c`
(wheel on) and `ab032a79a0fc82f1ccbb51f9d882b26a48db3fed1c93a237a1ad9ec8ed2fdc2b`
(wheel off). No noise, variance, or production defaults were changed.
The corresponding source hashes are `9269be4b172302b1c6d1fb7905678d294531a17121a742bad99f840514d5bffc`
(`common/types.h`), `c4020bbd8b3d05ed1982798944550ef286584fa1312add65f85ef7726085edac`
(`local_estimator.cc`), `aa35aa5a0fc5696c41bd1fedfa50887fcaad6e2c309a57efe9b87b6f330613a0`
(`local_estimator_replay.cc`), and `7fa2adb6a352bf2b282578614d641ddeb3cda91bffc3c814177e35619e921556`
(`local_estimator_test.cc`).

#### 2026-10-14 wheel state-block and IMU F/Q audit

The optional local-estimator trace now records per-wheel innovation, innovation
covariance, NIS, state before/after, and the lateral correction decomposed by
error-state block. On the same RFU-converted 15:00:54 input with NHC disabled,
5,693 wheel updates produced a cumulative local-Y correction of `+16.7822
m/s`. The signed block sums were `-469.7782 m/s` from velocity cross-covariance
and `+486.5604 m/s` from attitude cross-covariance; position, gyro-bias, and
accelerometer-bias blocks contributed zero directly because those columns of
the scalar wheel observation Jacobian are zero. The large opposing sums
nearly cancel, so the net is not attributable to only one block. Per-event
block sums close to the applied lateral correction within `6.1e-14 m/s`.
Wheel updates still change estimated gyro and accelerometer biases through
their gain rows (maximum single-update norm `4.33e-4 rad/s` and
`1.09e-3 m/s²`, respectively), which can affect later propagation. The wheel
update leaves nominal attitude unchanged; its attitude error injection is
exactly zero.

For this scalar, NHC-off replay, NIS is `0.00149/0.02747/0.44931` median/P95/
maximum; no update exceeded the configured `11.345` gate. The largest absolute
lateral update was `+1.7632 m/s` at sensor time `1791615686.257`: innovation
`+0.07711 m/s`, `S=0.01323 (m/s)^2`, NIS `0.44931`, with velocity and attitude
block contributions `+1.0971` and `+0.6662 m/s`. Another large update was
`-1.3530 m/s` at `1791615708.137`, where velocity and attitude contributions
were `-21.3529` and `+19.9999 m/s`; body lateral velocity was already
`11.44 m/s` before that update. These are internal consistency diagnostics,
not evidence that the chassis measurement or its covariance is physically
correct.

The per-propagation trace records the applied 15-state transition `F`, full
discrete `Q`, `FPFᵀ` local-Y covariance row, and local-Y rows of the four
process-noise source contributions. A central finite-difference regression
checks all 15 error-state columns for a nonzero attitude, angular rate,
specific force, gyro bias, and accelerometer bias; the analytic transition
agrees within `2e-5` for that test case. On this recording, transition-block
norm median/P95/max values were `F(v,theta)=0.0664/0.1110/0.1838`,
`F(v,b_g)=0.000171/0.000429/0.001181`, and
`F(theta,b_g)=0.00862/0.01355/0.02226`. This checks the implemented
linearization against the actual discrete nominal mechanization for the
tested case; it does not independently validate all model assumptions.

Across 11,386 propagation intervals, the logged `P_y` row closes against
`FPFᵀ + Q` within `1.8e-10` absolute, process-source rows sum to the logged
`Q_y` row within `4.1e-20`, and `Q` symmetry error is below `2.7e-22`.
`P(v_y,v_y)` starts at `0.0025` and ends at `17,728.5763 (m/s)^2`: cumulative
`FPFᵀ` propagation change is `+19,470.9285`, discrete process noise adds
`+0.5693044`, and wheel updates change it by `-1,742.9240 (m/s)^2`.
The process-noise contribution is almost entirely the configured
accelerometer-white-noise term (`0.5692973`); gyro white noise contributes
`7.09e-6`, and both bias-random-walk terms are negligible on this segment.
The isolated-source tests verify each configured noise PSD square root gives
linear diagonal variance scaling with `dt`; these parameters are therefore
continuous-time noise-density inputs to this implementation, not per-sample
standard deviations. Their physical calibration/provenance is still
unverified. A Monte Carlo calibration of the continuous stochastic model was
not performed; algebraic closure and a unit-scaling test are not a substitute
for such calibration.

Wheel and zero-velocity updates explicitly zero their attitude-gain rows, so
they do not inject an attitude error and require no attitude covariance reset.
The separate local-LiDAR update does inject right-multiplicative attitude
correction; after accepting it, `LocalEstimator` marks the covariance model
invalid, so no qualified covariance claim is made for that path. A covariance
reset Jacobian would be required before treating that corrected covariance as
valid.

Final diagnostic artifacts are in
`pkgs/openloc/.agents/notes/chassis_imu_dr_calibration_20261013/forward_150054_rfu/propagation_covariance_audit_20261014_legacy_expression/`.
They use event CSV hash
`9606849d2191c207f5230e77b1c46bd581f11aa01ad3f5b93d1ac6b1901ef943`; the
trajectory hash is byte-identical to the frozen wheel-on trajectory
(`4fa92b5f4c8176708933f9305c87988da467aa4900ab108a84f38e341a9f925c`).
During instrumentation, an algebraically equivalent refactor of the total
noise-matrix summation changed floating-point rounding enough to diverge late
in this sensitive replay. The original matrix construction and covariance
expression were restored; the final emitted trajectory now matches the
frozen output byte-for-byte. No estimator defaults or model parameters were
changed.
The replay executable hash is
`d0e0e3fef336db866ae30c6913372c8e2956d0590952e1af60ecaa07aba338b5`; the
trace, summary, and top-event CSV hashes are respectively
`06dfc551d753379ee95b36da3b49ee98d742cd2fbb575a3700d5f527c5f1fe81`,
`09d6ee0e78e56c631eabca4bd4ec0be8b143fa8ad93d80101f25155ad9c65e83`, and
`24eaffa28500e6b4956e6168228b42f40aba1930418ba127b09badf556934fbf`.
The updated source hashes are `acfee15df703b485947f41581c9a231a1def29429e3859961fd3dd0cce01a2a8`
(`common/types.h`), `d8437c1609fe69aa808319659e95f214330746b5243148a0c8a11b6077c95189`
(`local_estimator.h`), `d34475c9f98d37ca828f65fbbd41873b3c1b629e0d9a2df3b2325e0bf593e78c`
(`local_estimator.cc`), `69995def953b71b85d3f33d53fe6e1682e19eb349166c5e35d8a02baa69db52f`
(`local_estimator_test.cc`), and `478a157c18d1efc5739777070e9c8965926181bb15a114025550fc0357ba8500`
(`local_estimator_replay.cc`).

#### 2026-10-14 initial-bias and wheel-rate sensitivity

The dataset replay harness has a diagnostic-only
`--initial-gyro-bias-std` override; omitting it preserves the `LocalConfig`
default. No production estimator default was changed. With the frozen RFU
event/reference inputs, wheel variance `0.01 (m/s)^2`, NHC off, and every
other setting held fixed, the default `0.01 rad/s` run reproduced the frozen
wheel-on trajectory byte-for-byte. Reducing only this initial prior standard
deviation by 10x to `0.001 rad/s` changed horizontal error median/P95/max from
`2.053/211.869/260.964 m` to `2.202/115.220/142.745 m`; final local-Y velocity
changed from `20.447` to `10.090 m/s`, and final `P(v_y,v_y)` from
`17,728.576` to `714.735 (m/s)^2`. This establishes strong sensitivity to the
initial gyro-bias covariance, but the candidate still diverges severely and
`0.001 rad/s` is not a calibrated or recommended value.

An event-by-event decomposition of the full frozen covariance ledger
attributed the final `P(v_y,v_y)` of `17,728.576 (m/s)^2` as
`12,518.545` from the initial gyro-bias covariance (about 70.6%), `4,277.542`
from accumulated wheel measurement noise, `862.140` from IMU process noise,
`55.344` from initial attitude covariance, `14.873` from initial
accelerometer-bias covariance, and `0.131` from initial velocity covariance.
The decomposition closes against recorded covariance within
`2.31e-7 (m/s)^2` absolute, approximately `3.4e-14` relative to the largest
matrix element in the ledger. This is a covariance bookkeeping decomposition
under the recorded linearized transitions and gains, not a causal replay of
each isolated source.

The `0.01 rad/s` gyro-bias prior is much broader than the sample-to-sample
scatter in the initial 4.97 s quiet window after the candidate RFU-to-FLU
rotation (axis standard deviations `0.000358/0.000897/0.000741 rad/s`).
That comparison makes the prior a strong review candidate, but a short
stationary sample's standard deviation is not the uncertainty of its mean:
temporal correlation, sensor bandwidth, temperature behavior, and calibration
provenance are unknown. Bias covariance must not be reset to a sample-derived
value without a suitable sensor noise/Allan characterization or independent
validation.

A separate measurement-rate diagnostic retained all 5,895 IMU samples and
all 201 startup wheel samples through initialization, then retained one of
each ten subsequent chassis samples (about 10 Hz rather than 100 Hz). With
the original `0.01 rad/s` gyro prior and the same wheel variance, horizontal
error median/P95/max became `2.312/126.335/161.591 m`; final local-Y velocity
was `12.866 m/s` and cumulative wheel-update local-Y correction was
`+10.598 m/s`. This reduces but does not eliminate divergence. It is
consistent with the previously measured high lag-1 wheel-innovation
correlation (`0.9885`), so treating every 100 Hz sample as independent is a
second model-risk candidate. Decimation is a diagnostic ablation, not a
production update-rate recommendation or proof that the samples are
independent at 10 Hz. The evaluator summary reads its canonical full-input
manifest; `diagnostic_manifest.json` in the sensitivity artifact records the
actual decimated event hash for this run.

Taken together with the wheel-off ablation, the current leading explanation
is an interaction: an NHC-off, weakly constrained lateral state, a broad
initial gyro-bias covariance that grows large lateral cross-covariance, and
repeated temporally correlated scalar wheel updates that feed that covariance
back into nominal lateral velocity and biases. The checked propagation
Jacobian, wheel observation Jacobian, and event-ledger arithmetic close for
their tested cases; this evidence does not establish that real-sensor noise,
frame/reference-point assumptions, or the covariance prior are correct, and
does not rule out all implementation defects. No parameter or production
policy is promoted. Next evidence should characterize the IMU bias prior
(sensor specification/Allan data and longer stationary captures), model or
measure wheel-sample temporal correlation, and repeat on independent straight,
turning, and reverse data with verified sensor-to-base frames and reference
points.

#### 2026-10-14 completed Apollo IMU and conditional ESKF curved-route replay

The user explicitly authorized completing raw gyro X/Y and two test-only
initialization conditions, not altering production defaults. Read both records
only for the raw IMU channel using the already verified reader modes, including
demo's reversed-index read-only workaround. All 11,630 / 4,553 messages have
measured X/Y/Z axes; every previously exported field matches exactly. The
completed CSVs and hashes are in
`.agents/notes/ackermann_crosscheck_20261014/full_imu_20261014/`.
The local matching `wheelos_msgs/sensor_msgs/imu.proto` specifies acceleration
and angular velocity in forward/left/up axes, SI units and GPS payload time.
Actual vehicle calibration, motion-reference equivalence and lever arms remain
conditional; a same-name schema comment is not a physical calibration proof.

With full gyro norms available, neither record has a joint 2 s quiet window
under the original thresholds. Both have 1 s windows. Added
`--diagnostic-stationary-duration` and `--offline-diagnostic-seed` only to
the test executable; no `LocalEstimator` API, propagation, observation model,
noise, initial prior, NHC, ZUPT or production default changed.

- **A, late quiet:** shorten only stationary dwell to 1 s, keeping 50 samples,
  original gyro/acceleration/speed thresholds and subsequent causal propagation.
  sensor_rgb then has only 0.495 s / zero wheel distance, so it supplies no
  moving-route validation. demo has 18.336 s / 86.893 m and approximately
  90.745 degrees of reference heading change.
- **B, offline main segment:** explicitly seed the first real IMU with attitude,
  bias and diagonal prior from A's first initialized state, plus initial
  chassis speed rotated into the local frame. The seed uses future quiet data,
  assumes unchanged tilt/mounting, and is not online initialization or a
  qualified moving-seed covariance. It covers sensor_rgb's 55.175 s /
  523.643 m main segment (about a half turn, max raw yaw rate 0.322 rad/s)
  and demo's 45.320 s / 259.030 m (91.991-degree reference heading change,
  max raw yaw rate 0.250 rad/s).

Each condition compares the ESKF and both planar variants at identical
initialized IMU times, with one scoring position/yaw gauge only. Events are
sorted by measurement time and explicitly use idealized offline receive time
equal to measurement time; arrival/latency behavior is not validated. No
GNSS/pose update enters the filter. All scored endpoints have reference
brackets, without extrapolation. demo's parking gear is retained as a
diagnostic exception, not production admission.

| Condition | Paired scored states | Original ESKF P95 / endpoint | Planar raw gyro P95 / endpoint |
|---|---:|---:|---:|
| sensor_rgb A, stopped tail | 100 | 0.095 / 0.102 m | 0.063 / 0.066 m |
| demo A, causal after 1 s quiet | 1,835 | 50.654 / 59.608 m | 1.169 / 1.174 m |
| sensor_rgb B, offline main route | 10,993 | 3,856.519 / 4,471.004 m | 5.904 / 5.946 m |
| demo B, offline main route | 4,533 | 236.029 / 294.466 m | 1.438 / 1.412 m |

Only 383/10,993 and 190/4,533 B states meet the unchanged ESKF precision
policy. Finite/integrity-valid output is not precision qualification.
Initial sensor-speed sampling differs slightly from the earlier chassis-time
Apollo diagnostic; do not silently replace its 523.647 / 259.210 m distances
with this experiment's IMU-time intervals.

Existing test-only gain policies were then run with identical events and
seeds, without retuning any parameter:

| Condition | Original endpoint | Both biases frozen, full velocity gain | Both biases frozen, forward-projected gain |
|---|---:|---:|---:|
| demo A | 59.608 m | 55.902 m | 24.253 m |
| sensor_rgb B | 4,471.004 m | 139.916 m | 465.682 m |
| demo B | 294.466 m | 167.800 m | 350.995 m |

These are closed-loop changes, not additive attribution or a production fix.
In sensor B the original gyro-Z estimate changes from `+0.0000445` to
`-0.02653 rad/s`, with final pitch `-41.42 deg` and full velocity norm
`244.92 m/s`. In demo A gyro Z changes from `+0.0001284` to
`-0.03312 rad/s`, alongside 28.15-degree endpoint heading disagreement.
Freezing biases limits tilt but does not eliminate drift; forward projection
is not consistently beneficial on curved routes. Acceleration/tilt integration,
unobserved lateral state, physical frame/reference assumptions and coupled
wheel feedback all remain relevant. This does not isolate a unique F/Q bug.

**Decision:** the present scalar-wheel 15-state ESKF does not pass these
conditional Apollo consistency diagnostics. The demo A failure means future
initialization alone cannot explain all failures. The independent planar
wheel/raw-gyro candidate remains a practical reference baseline and is
supported by actual curved-route evidence, but neither route establishes
independent accuracy, calibrated uncertainty, long-horizon behavior, slip/slope
robustness or a user-defined precision requirement. Fixed quiet gyro-Z bias
compensation does not uniformly improve position; do not automatically promote
it or a feedback mask from these results.

Protocol, input/source/binary/output hashes, exact commands, seed files, raw
states and paired scores are frozen under
`.agents/notes/ackermann_crosscheck_20261014/apollo_ekf_20261014_v1/`
and `apollo_feedback_20261014_v1/`. Eight focused Python regressions pass,
including original 2 s default dwell and explicit seed speed/bias behavior;
C++ replay formatting/build passes. A fresh default Zhongji control is
byte-identical to the old baseline, proving these test flags did not change
ordinary replay behavior. Source records and old Apollo exports are untouched.

#### 2026-10-14 C++ parity and frozen Zhongji planar/ESKF comparison

Added a test-only `//:planar_wheel_dataset_replay` runner and
`test/integration/chassis_imu_dr/compare_planar_replay.py`. No production
estimator, wheel feedback policy, noise prior, NHC gate or initialization gate
was changed. The Python and C++ paths share the same bounded interpolation
grid; rerunning the Python control reproduces its frozen trajectory/scoring
bytes.

Apollo parity pairs all 5,832 sensor_rgb and 4,534 demo endpoints. Maximum
C++/Python position difference is `7.11e-15 m` for both; distance difference
is zero, and heading difference is at most `2.45e-16 rad`. Thus the previous
Apollo distance/error tables now also have actual C++ candidate replay
evidence, not merely a matching synthetic implementation.

At this checkpoint this was not an Apollo ESKF comparison. Its frozen IMU
export omitted gyro X/Y,
which cannot be filled with invented zeros. sensor_rgb has only `1.520 s`
of continuous speed-qualified stationarity, below the unchanged 2 s dwell.
demo has a `5.030 s` speed-only quiet window, with earliest possible
wheel-qualified initialization at approximately `26.990 s` after its first
chassis sample; IMU/frame/gear eligibility is not established by that
necessary wheel-only check. No moving-start gate was bypassed.

By separate user approval, replayed the complete frozen 1010 RFU three-axis
IMU/wheel input through the original ESKF and the independent planar C++
candidate. The ESKF output is byte-identical to the prior baseline
(`4fa92b5f4c8176708933f9305c87988da467aa4900ab108a84f38e341a9f925c`).
Both planar variants share its first valid sensor timestamp and local yaw;
one uses raw gyro Z, the other subtracts only the frozen startup gyro-Z bias
(`-9.075712e-5 rad/s`), without a trajectory fit or ongoing bias update.

All 5,694 initialized states are time-paired. There are 5,693 common scored
states over `56.920 s`, integrating `80.713 m` wheel distance. The final IMU
state at `1791615715.294183` has no reference bracket after the last fix at
`1791615715.29` and is explicitly unscored for every candidate.

| Same-input method | Horizontal median / P95 / max | Error at 50.005 m | Scored endpoint error | Endpoint heading difference |
|---|---:|---:|---:|---:|
| Original 15-state ESKF, NHC off | 2.053 / 211.869 / 260.964 m | 42.659 m | 260.964 m | -2.063 deg |
| Planar wheel + raw gyro | 0.637 / 1.519 / 1.633 m | 0.970 m | 1.633 m | -0.344 deg |
| Planar wheel + fixed startup gyro bias | 0.639 / 1.530 / 1.642 m | 0.974 m | 1.642 m | -0.048 deg |

Interpretation: raw gyro integration alone does not account for the original
hundreds-of-meters drift on this input. Startup bias compensation improves
heading consistency but not position here. Combined with the earlier
gain-masking evidence, this reinforces investigation of coupled
acceleration/tilt/cross-covariance/wheel-bias feedback, rather than replacing
gyro integration with an uncalibrated steering-only rate or retuning a prior
to force the result. The planar comparison removes several paths together;
it does not identify a unique faulty state block or certify ESKF statistics.

The forward-only scalar-speed planar model is not a calibrated model for
Zhongji rear-steering reference motion. It does not prove zero rear-axle
lateral velocity, correct dashboard-speed semantics, slip rejection or
long-horizon accuracy. IMU tilt/body-Z assumptions and reference-point/lever-arm
uncertainty remain. GNSS/INS references may share sensors; all numbers are
same-record consistency, not independent truth or production admission.

Protocol, source/input/binary/output hashes, exact commands, generated inputs,
raw trajectories and common per-time scores are archived under the OpenLoc
ignored directory
`.agents/notes/ackermann_crosscheck_20261014/planar_parallel_20261014_v2/`.
The first run is preserved separately in `planar_parallel_20261014_v1/` with
`failure.json`: an overly strict scoring check stopped at the unbracketed
tail; the user approved explicit common-reference scoring before v2. No source
record was altered or re-extracted. Focused C++ core and six Python replay
regressions pass; the C++ replay adapter formatting passes.

#### 2026-10-14 distance checkpoints and independent planar candidate

Implemented the explicitly selected independent `PlanarWheelDr` candidate
without changing or implicitly substituting `LocalEstimator`. Inputs are
time-associated body planar velocity and gyro Z at a declared motion
reference. The candidate uses endpoint-average velocity and bias-compensated
rate, exact constant-twist integration, signed motion, an explicit local
seed, clock checks and a latching input-gap failure. It accepts nonzero
lateral velocity rather than hardcoding rear-axle zero-lateral motion.
Gyro bias is caller-specified, not learned. There is no covariance precision
claim, synchronization adapter, production reset contract or LIO coupling.
It is not wired to production. The follow-up above adds real Apollo C++ replay
and a Zhongji same-input ESKF comparison. The newer completed-IMU experiment
above adds conditional Apollo ESKF replay; production admission remains blocked.

The Python data diagnostic now exports cumulative wheel distance and
distance checkpoints. Reruns in a new artifact directory reproduce every
previous trajectory/scoring field exactly. Main input-continuous segment,
wheel-speed plus raw-gyro horizontal error at the first scored endpoint
reaching each distance:

| Wheel distance | sensor_rgb horizontal error | demo_3.5 horizontal error |
|---|---:|---:|
| 50 m | 0.996 m | 0.055 m |
| 100 m | 1.382 m | 0.039 m |
| 200 m | 3.076 m | 0.658 m |
| 300 m | 4.802 m | not covered |
| 400 m | 5.570 m | not covered |
| 500 m | 5.923 m | not covered |
| Segment endpoint | 5.943 m at 523.647 m | 1.410 m at 259.210 m |

Checkpoint sample overshoot is less than 0.081 m; the exact distance and
elapsed time are recorded, not falsely interpolated as exact threshold truth.
Endpoint horizontal drift divided by wheel path length is about 1.135%
and 0.544%; this ratio is not a constant per-meter error model. demo's
nonmonotonic early errors are normal accumulated-vector cancellation, not
reference feedback.

The favorable result has several bounded explanations: wheel speed supplies
translation directly, avoiding accelerometer double integration and gravity
leakage; gyro heading has only -0.609/+0.285 degree endpoint disagreement;
wheel/reference path length differs by about -1.32%/+0.074%; windows last
only 55/45 s; the initial local gauge is supplied once per segment; and
the localization reference may share the same IMU/GNSS source. It does
not prove superior hardware bias calibration or general accuracy. The
simple candidate also avoids the scalar-wheel cross-covariance/bias
feedback identified in the original 1010 ESKF experiment. These two
datasets are not the same sensor/chassis/turning model as Zhongji.

New C++ tests verify fixed bias compensation, reverse motion, exact turning
with rear-reference lateral velocity, clock/time rejection and gap latching.
`//:planar_wheel_dr_tests` and the existing four-wheel tests pass; changed
C++ formatting passes. Five Python regressions pass and preserve integration
without reference feedback. Updated records are under
`ackermann_crosscheck_20261014/wheel_odometry_distance_20261014/`.
The follow-up above completes C++ candidate replay and the Zhongji ESKF
comparison. The newer completed-IMU experiment adds conditional Apollo ESKF
replay. A qualified Zhongji speed/reference adapter remains unfinished;
do not describe these diagnostics as production
integration or independent accuracy evidence.

#### 2026-10-14 separate Apollo wheel-odometry replays

Replayed `sensor_rgb.record` and `demo_3.5.record` independently from the
audited scalar CSVs, not as concatenated trajectories. Added the standalone
`test/integration/chassis_imu_dr/wheel_odometry.py` diagnostic: integrate
chassis speed with either raw IMU Z or a frozen transferred steering model.
There is no acceleration integration, wheel Kalman update, nominal bias
estimation, yaw-rate fitting on the target record, or subsequent reference
feedback. This is not a `LocalEstimator`/four-wheel-helper replay.
The steering rate model is the exploratory small-angle relation from the
preceding audit, not geometry-calibrated Ackermann kinematics. Unknown
vehicle configurations prevent that stronger claim.

Raw-gyro mode uses IMU GPS payload time converted with `+315964782 s`.
Chassis uses header time and reference uses pose measurement time, not bag
time. The integration uses endpoint-average speed/rate and exact
constant-twist steps no longer than 5 ms. Stream interpolation is bounded
at 50 ms with no extrapolation. Both modes share identical eligible input
intervals. The original direction-positive speed is assumed forward; demo's
invalid parking-gear labels are not used to invent a speed sign.

| Record / main continuous segment | Method | Horizontal error median / P95 / max | Final heading error |
|---|---|---:|---:|
| sensor_rgb, 55.180 s | Wheel speed + raw gyro | 4.563 / 5.899 / 5.949 m | -0.609 deg |
| sensor_rgb, 55.180 s | Wheel speed + transferred steering | 10.938 / 25.545 / 27.737 m | +11.250 deg |
| demo_3.5, 45.340 s | Wheel speed + raw gyro | 0.547 / 1.437 / 1.472 m | +0.285 deg |
| demo_3.5, 45.340 s | Wheel speed + transferred steering | 4.188 / 6.827 / 8.525 m | +9.080 deg |

The sensor_rgb main segment integrates `523.647 m` wheel distance versus
`530.676 m` reference polyline distance (about -1.32%). demo integrates
`259.210 m` versus `259.017 m` (about +0.074%). These are path-length
consistency diagnostics; lever arms, vehicle speed reference, and independent
truth remain unknown. Neither speed scale nor gyro bias was adjusted.
Target sensor_rgb uses the frozen demo coefficient `0.17045098354696603`;
target demo uses the frozen sensor_rgb coefficient `0.17959947347567176`.
There is no target-record steering fit or whole-trajectory alignment.

sensor_rgb includes 80–750 ms chassis/IMU dropouts. It is therefore split
into three explicitly seeded continuous segments (0.600, 55.180 and
2.640 s), with 18 rejected chassis intervals, 58.420 s total eligible
duration, and 5,832 scored endpoint rows. No trajectory is bridged across
the gaps; each segment uses one new reference origin/yaw, so this is not
a full-bag continuous accuracy result. demo has one 45.340 s segment,
one rejected trailing interval outside IMU coverage, and 4,534 scored
endpoints. All integrated endpoints have bracketed references; there are
zero unscored endpoints. sensor_rgb's 188 duplicate pose-time rows have
identical position/heading and are explicitly deduplicated; conflicting
duplicates would fail. All segments and exclusions remain in `summary.json`.

These two records support the practical diagnostic choice of wheel-derived
translation with gyro-derived heading over this *uncalibrated transferred*
steering-only alternative. They do not establish that gyro is universally
more accurate than a calibrated steering model, that its bias can be ignored,
or that Zhongji has the same motion/speed semantics. Published localization
may share the same IMU/GNSS sources, making gyro agreement partly correlated.
Do not extrapolate this result to a production precision or covariance pass.

Five focused Python tests pass in the managed test container, covering
straight/turn integration, bounded gaps/no extrapolation, wrapped headings,
invalid timestamps, and an end-to-end check that changing all reference
poses after the seed cannot alter odometry. Standard Python formatters are
not installed in the checked dev container; no formatter pass is claimed
and no packages were installed. Input CSV hashes match the prior audit;
source/output hashes and scoring counts were verified. Artifacts are
under `ackermann_crosscheck_20261014/wheel_odometry_20261014/`, separately
for each recording. Source records remain unchanged.

#### 2026-10-14 mean-wheel input and Ackermann cross-record audit

The user confirmed Zhongji steering percentage 100 maps to the maximum
angle, initially described speed as a four-wheel mean, then clarified it
as passenger-car-like dashboard speed. Dashboard display semantics alone
do not establish arithmetic raw wheel mean versus ECU vehicle-speed
conversion. Treat the four-wheel mean contract below as an explicit
conditional model, not verified chassis speed semantics.
Added `CounterSteeringMeanWheelKinematics` with an explicit wheel-center
track (not vehicle body width). It maps percent to opposite front/rear
equivalent axle angles and converts the signed mean rolling speed to center
body-X speed using the four wheel-position velocity magnitudes under ideal
rolling. The two-input speed/percent covariance is propagated to the full
reference twist, preserving correlations; its rank is at most two, so it
must not be blindly inverted as an independent three-channel measurement.
Inputs outside +/-100 percent, missing track, non-SPD input uncertainty
and inner-wheel reversal are explicitly rejected. Five focused tests pass,
including rolling-speed reconstruction and covariance finite differences.
This helper does not qualify lateral slip, actual individual steer angles,
actuator delay, or pure in-place rotation. Actual track and measurement
uncertainties remain required before Zhongji record adapter/fusion admission.

Read-only low-bandwidth audit of the requested Ackermann records:

| Record | Chassis / raw IMU / pose count | Max speed | Max abs steering % | Max abs pose body yaw rate |
|---|---:|---:|---:|---:|
| `sensor_rgb.record` | 5,851 / 11,630 / 5,856 | 15.567 m/s | 38.00 | 0.3222 rad/s |
| `demo_3.5.record` | 4,536 / 4,553 / 4,558 | 9.140 m/s | 23.98 | 0.2498 rad/s |

Both provide turning excitation. The exploratory relation
`yaw_rate = c * speed * steering_percent/100`, fitted independently against
same-record pose body-Z rate, gives `c=0.179599` and `0.170451`.
Fitting against raw IMU Z instead gives `0.179670` and `0.170446`.
The raw IMU absolute residual P95 against the pose-fitted model is
`0.02049/0.01012 rad/s`. This supports a compatible sign/basic turning
relationship, not exact Ackermann geometry or independent accuracy.
The real axle/encoder reference, axle distance, steering ratio and vehicle
identity for these old recordings are unknown; no Zhongji geometry is used.

Freezing the sensor_rgb coefficient and applying it to demo without target
refitting yields yaw-rate absolute residual median/P95/max
`0.00659/0.01669/0.02403 rad/s` (1,581 pairs). Reverse transfer gives
`0.01002/0.02503/0.03785 rad/s` (3,169 pairs). This is a cross-record
small-angle consistency diagnostic, not a held-out calibrated steering
model, independent route truth, or a replay of the new four-wheel helper.
Raw IMU and published pose may share GNSS/INS measurements.

Two important data-gate findings:

- Bag times differ from chassis headers by approximately `1,171,626.6 s`
  and `1,792,631.5 s`. Pairing uses payload/header sensor axes, not bag
  arrival time. IMU GPS payload is converted with the existing
  `+315964782 s` convention; no time offset is fitted.
- demo chunk-index labels are reversed: positions labeled CHUNK_HEADER
  contain CHUNK_BODY sections. The standard Python reader silently skips
  all messages at the labeled body indexes. An explicitly approved read-only
  section audit verified the real bodies, then decoded those positions;
  all selected channel counts match metadata. Source files and reader
  installation were not repaired or modified. Additionally, 3,808 moving
  demo chassis samples report GEAR_PARKING, so it cannot be admitted under
  the 1010 gear/status gate without an adapter-specific eligibility decision.
  Neither record has populated per-wheel speed in the decoded chassis.

The reproducible audit, selected scalar CSVs and their hashes are in
`.agents/notes/ackermann_crosscheck_20261014/`. Source sizes/time bounds
are recorded; full source-record SHA hashes were not calculated in this
bounded low-bandwidth audit. No whole-estimator replay or production gyro
bias correction was executed. Exact Ackermann vehicle configuration and
the Zhongji wheel-center track/measurement covariance remain external inputs.

#### 2026-10-14 four-wheel steering observation conversion

Implemented `FourWheelKinematics` as a middleware-neutral conversion layer,
not a new production estimator mode. Inputs are actual equivalent front/rear
axle angles (radians, positive left), signed body-X velocity at the axle
midpoint, and their joint SPD covariance. Outputs are reference-point
`[v_x, v_y, omega_z]` and the full propagated covariance, including
correlations. The reference offset is explicit in FLU; reverse motion,
counter-phase steering, crab motion and rear-axle lateral velocity are
handled without assuming rear lateral velocity is zero.

Read-only configuration evidence comes from
`/mnt/synology/config/vehicle/zhongji-0928/modules/common/data/vehicle_param.pb.txt`:
wheelbase `9.0 m`, maximum equivalent front/rear angle `0.5061 rad`,
wheel rolling radius `0.507 m`, steering ratio `12`, and maximum
steering-wheel angle `6.0732 rad`. The accompanying flagfile declares
reference-point values `2/0` and center-of-mass offset `4.5 m`; those enum
semantics and their actual publishing path have not been verified. The
helper does not parse Apollo configuration or assume these are calibrated
uncertainties. Tests use an explicit rear-axle offset `[-4.5, 0] m`
derived from symmetric axle-midpoint geometry, not from the unresolved enum.

For ideal rolling motion, let
`k=(tan(delta_f)-tan(delta_r))/L` and
`b=(tan(delta_f)+tan(delta_r))/2`. At reference offset `(x,y)`, the model is
`v_x_ref=v_x_center*(1-y*k)`,
`v_y_ref=v_x_center*(b+x*k)`,
`omega_z=v_x_center*k`. The analytic input Jacobian propagates the joint
speed/angle uncertainty. Stopped motion can yield a singular output
covariance; it must not be blindly inverted for a three-component update.
Any actual gyro fusion must account for gyro observation uncertainty and
shared measurement dependencies separately.

Four focused tests passed in the managed non-root test container:
counter-phase steering and rear reference, reverse, crab/stopped motion,
correlated covariance finite differences and invalid-input rejection.
Target: `//:four_wheel_kinematics_tests`. The helper is also wired into the
aggregate localization algorithm test dependencies; that aggregate was not
executed for this slice.

By explicit scope confirmation, this slice does **not** map chassis
steering percentage to radians, identify the chassis speed reference,
feed the derived twist into `LocalEstimator`, or correct gyro bias.
The local checked source tree did not provide a verified Zhongji publishing
path. These adapter/fusion steps remain blocked on that evidence and on
eligible uncertainty inputs. Geometry alone cannot supply them.
The model excludes per-wheel geometry, slip, rollover/grade dynamics,
and in-place rotation requiring steering near 90 degrees; do not use it
to claim a general independent-wheel platform model.

#### 2026-10-14 statistical and vehicle-model review

A 30 s known-truth straight-motion ensemble compares the original wheel
feedback, frozen-bias/body-forward-only feedback, and wheel-disabled
propagation. Each of 64 paired trials samples the declared initial error,
independent wheel noise, IMU white-noise densities, and bias random walks.
The truth moves at `1.6 m/s`, level with identity attitude. The timestep is
`0.05 s`; private propagation receives one independent interval-average
input at both endpoints, with white-noise variance `PSD/dt`, and midpoint
Brownian bias. This deliberately isolates the covariance model from
adjacent-sample averaging correlations; it is not the actual dataset adapter.
Initial position standard deviation is explicitly `0.001 m`, so the
synthetic full-state NEES has a defined initial pose distribution without
post-hoc alignment. All stochastic standard deviations are scaled together
by `0.0001`, `0.01`, or `1`; scale 1 uses the original configured stochastic
parameters. No production parameters changed.

| Noise/prior scale | Strategy | Lateral position RMSE | Mean 15-state NEES | Mean 3-velocity NEES | Raw scalar NIS mean |
|---|---|---:|---:|---:|---:|
| 0.0001 | Original | 0.0534 m | 16.46 | 3.36 | 1.041 |
| 0.0001 | Forward / frozen bias | 0.0534 m | 16.42 | 3.06 | 1.044 |
| 0.0001 | Wheel disabled | 0.0534 m | 16.58 | 3.38 | not applicable |
| 0.01 | Original | 5.349 m | 61.22 | 2.81 | 1.041 |
| 0.01 | Forward / frozen bias | 5.338 m | 34.32 | 2.67 | 1.042 |
| 0.01 | Wheel disabled | 5.338 m | 16.74 | 3.39 | not applicable |
| 1 | Original | 376.569 m | 924.26 | 10.71 | 0.913 |
| 1 | Forward / frozen bias | 509.229 m | 3,101.77 | 26.04 | 0.849 |
| 1 | Wheel disabled | 519.657 m | 1,265.63 | 27.48 | not applicable |

The tiny-error control satisfies the predeclared mean-NEES check:
`15 +/- (4*sqrt(30/64) + 1)`, with the final term allowing discretization.
This is an ensemble check in the linear limit, not a calibration certificate.
At scale 0.01 the wheel-aided joint NEES is already poor despite near-unit
NIS and reasonable velocity-marginal NEES. At scale 1, even wheel-disabled
propagation is inconsistent. These results favor a limited nonlinear/
observability operating envelope over a universally broken Joseph formula,
but do not exclude other implementation defects or establish a unique cause.
The forward-projection candidate's favorable 1010 result does not generalize
to this default-prior ensemble and cannot be promoted as a fix. Full-state
NEES includes cross-state correlations; marginal variances alone conceal
this failure. Raw NIS includes rejected observations, not only the
gate-truncated accepted subset.

An analytic straight-motion observability check is wired to the production
propagation transition and wheel observation. For identity attitude,
gravity-only specific force, scalar forward speed and no NHC, the
first-order observation can see only three combinations:
`v_x`, `g*theta_y - ba_x`, and `bg_y` through its quadratic time effect.
It has no first-order information about local lateral velocity, roll or
gyro X bias. The tilt/accelerometer-bias combination also has an
unobservable direction. Real acceleration and turning may add information;
this particular nullspace is not a universal moving-vehicle rank claim.
A broad uncertainty in those weak directions must not be treated as
well-calibrated wheel evidence simply because the innovation is small.

**Vehicle-reference-point review.** A rear-steering vehicle is not eligible
for zero body-lateral velocity at the rear axle just because it does not
slip. Under a planar rigid-body, ideal rolling model, with `v_x` the rear
axle body-forward velocity, rear angle `delta_r`, front angle `delta_f`
and wheelbase `L`, the rear-axle constraints are
`v_y = v_x*tan(delta_r)` and
`yaw_rate = v_x*(tan(delta_f)-tan(delta_r))/L`.
Thus opposite rear/front steering generally gives nonzero rear-axle
lateral velocity. A synthetic `-5 deg` rear steer at `1.6 m/s` gives
`v_y=-0.13998 m/s`; the current zero-lateral NHC accepts an innovation of
`+0.13998 m/s` and induces `0.04068 m/s` velocity error even from the
correct state. This is a counterexample to NHC applicability, not measured
steering calibration for the 1010 recording. Steering percentage is not
an angle; actual front/rear mapping, reference point, time association,
encoder semantics and slip eligibility must be verified first.
For an offset sensor/reference point the rigid-body velocity term
`omega cross lever_arm` must be included with a declared transform direction.

**Engineering-practice conclusions.**

- Keeping nuisance-state means fixed while retaining their covariance and
  cross-covariance is a legitimate consider-state design choice. Joseph
  covariance using the actual constrained gain is required; deleting the
  nuisance uncertainty to make metrics pass is not equivalent.
- Local wheel feedback and global ODOM ownership are distinct concerns.
  Preserve global isolation, but do not assume it forces local attitude
  gain to zero. Any local attitude injection requires the matching
  error-state reset and an explicit continuity contract.
- Declare an observability/linearization operating envelope and expose
  precision degradation. Do not silently stop covariance growth, lower
  gyro-bias priors, or relax gates to obtain admissible output.
- Do not enable zero-lateral NHC for the declared rear-steering/rear-axle
  reference without the steering-dependent model and eligibility evidence.
  The same-record NHC improvement is not proof that its constraint is true.
- Next algorithm discriminator: trace how longitudinal velocity/tilt/bias
  correlations and nullspace information change at scale 0.01; separately
  test timestep refinement and known turning/rear-steering truth.
  Independent vehicle validation and calibrated sensor priors remain gates.

These conclusions are supported by code, analytic checks and synthetic
counterexamples. External primary-source verification was attempted but
unavailable (search gateway failure and blocked document fetches); no
specific industry implementation or publication was verified during this
review. The focused ensemble/control tests and 41 fast local regressions
passed. "Passed" does not mean the default stochastic model passed
consistency qualification. Logs, source snapshot, hashes and protocol are
in `matched_ensemble_20261014/`; two test targets now have explicit long
timeouts to accommodate the accumulated diagnostics. The aggregate
localization test target was not rerun.

#### 2026-10-14 frozen-record bias-feedback isolation

Private friend-test controls now isolate wheel gyro-bias feedback,
accelerometer-bias feedback, and body-forward-only velocity feedback.
Normal callers cannot select these controls; defaults preserve the original
update. Each diagnostic masks/projects the actual gain before state injection
and Joseph covariance update, rather than freezing a nominal state while
retaining an inconsistent covariance. No pose/attitude feedback, NHC, noise
prior, observation variance, timestamp, or initialization policy was changed.
These are causal closed-loop sensitivity experiments, not production modes.

All five runs use the frozen RFU events and reference, the same executable,
wheel variance `0.01 (m/s)^2`, and NHC off. The default trajectory reproduces
the original baseline byte-for-byte, and all first-valid states are identical.

| Wheel feedback | Horizontal error P95 / max | Final local-Y velocity | Cumulative wheel / IMU local-Y velocity increment |
|---|---:|---:|---:|
| Original velocity + both biases | 211.869 / 260.964 m | 20.447 m/s | +16.782 / +3.665 m/s |
| Velocity + gyro bias only | 241.768 / 291.169 m | 17.236 m/s | +13.231 / +4.005 m/s |
| Velocity + accel bias only | 64.707 / 75.407 m | 3.893 m/s | +3.503 / +0.390 m/s |
| Velocity only, initialized biases frozen | 63.870 / 73.326 m | 3.445 m/s | +3.056 / +0.389 m/s |
| Body-forward velocity only, initialized biases frozen | 8.379 / 9.386 m | 0.392 m/s | +0.003 / +0.389 m/s |

All 5,693 post-initialization wheel updates were accepted in each run.
The velocity event ledger closes within `1e-9 m/s`. Disabled bias gain
rows are exactly zero, and the forward-only gain has body-transverse
components below `1e-10`. The latter still permits a local-Y correction
from a nonzero vehicle heading; it is not a local-axis velocity clamp.

The evidence separates two harmful feedback routes in this recording:
wheel-driven gyro-bias estimation substantially amplifies the closed-loop
error, while direct transverse velocity correction still produces severe
drift when both biases are frozen. The large residual error in the
velocity-only case cannot be explained by bias injection. Projecting that
gain onto body-forward nearly removes the *wheel-induced* lateral velocity
growth, leaving essentially the wheel-off IMU lateral velocity increment.
This narrows the cause to scalar-wheel feedback through poorly constrained
cross-covariance, including its gyro-bias feedback route; it does not identify
a unique faulty Jacobian or calibrated alternative estimator. Changes to
gain also change future covariance and linearization, so differences between
these runs are not an additive source decomposition.

The forward-only result still has `9.386 m` maximum same-record horizontal
error and is not an accuracy pass. Sensor frame/reference-point eligibility,
the gyro-bias prior, wheel sample correlation, and statistical covariance
qualification remain unresolved. Do not promote frozen bias or projected
gain as a production fix without matched-noise long-horizon validation and
independent vehicle evidence.

Two focused gain/Joseph regressions and the fast local-estimator regression
selection passed (39 cases; excluding the long feedback diagnostics and
propagation Monte Carlo). Replay build and changed C++ formatting passed.
The executable hash, source/input/output hashes, identical initialization,
gain checks, scoring, and event ledger are archived in
`bias_feedback_20261014/diagnostic_manifest.json`; `summarize.py` reproduces
the checks. Same-record RTK/INS scoring remains non-independent.

#### 2026-10-14 long-horizon wheel-feedback diagnostic

Two test-only diagnostics compare the production constrained wheel update,
full ESKF feedback, and propagation without wheel updates. All strategies
share the actual `LocalEstimator::Propagate` implementation, initial state,
initial covariance, IMU samples, wheel samples, and unchanged default noise
and innovation-gate settings. The full-feedback reference obtains the
production observation, innovation, and unconstrained gain from the trace,
then injects all state blocks, right-multiplies the attitude correction,
and applies a Joseph covariance update followed by the SO(3) right-Jacobian
reset. Its reset matches central finite differences. This is not a new
production fusion option.

The analytic truth is a 60 s, 1.6 m/s straight line or constant-radius turn
at `0.02 rad/s`, sampled at 100 Hz. A perfect-input control produces less
than `3.4e-7 m` position error for every strategy. The combined disturbance
adds a `0.002/-0.001 rad` roll/pitch initialization error, a
`0.0001/-0.0001 rad/s` initial gyro-bias estimation error, deterministic
`0.01 m/s^2` acceleration harmonics, and a `0.04 m/s` wheel-speed harmonic.
These are sensitivity inputs, not noise drawn from the configured process
model. All rows below report maximum **3D** position error over the full
60 s, not horizontal P95 or same-record RTK error.

| Scenario | Constrained wheel | Full ESKF reference | Wheel disabled |
|---|---:|---:|---:|
| Combined, straight | 5.458 m | 1,470.78 m | 17.488 m |
| Combined, turning | 57.657 m | 134.703 m | 8.808 m |
| Initialization error only, straight | 5.227 m | 5.229 m | 17.652 m |
| Periodic wheel disturbance only, straight | 0.218 m | 164.475 m | effectively zero |
| IID wheel disturbance only, straight | 0.604 m | 410.331 m | effectively zero |
| IMU disturbance only, straight | 0.652 m | 123.116 m | 0.673 m |

The IID wheel case uses a fixed seed `20261014` and standard deviation
`0.04/sqrt(2) m/s`, matching the periodic signal's nominal RMS. Both still
use the unchanged `0.01 (m/s)^2` observation variance, deliberately not a
matched-noise covariance experiment. In the wheel-only straight cases,
local-Y error remains exactly zero; the full-reference 3D errors must not
be mislabeled lateral errors. In contrast, the full-reference combined
straight case ends at local-Y `1,463.68 m` and `74.016 m/s`, and the
IMU-only case at local-Y `-120.984 m` and `-5.945 m/s`.

Every wheel update was accepted. Accepted scalar NIS means are below
`0.0015` in the periodic/combined cases despite large trajectory errors;
the IID-wheel means are approximately `0.071/0.075` for constrained/full
feedback. These are not chi-square calibration results: the deterministic
disturbances, narrow realized initial errors, and mismatched noise variance
do not satisfy that statistical experiment's assumptions. Finite SPD
covariance and a small longitudinal residual are insufficient evidence of
accurate transverse motion.

For perfect straight motion without wheel feedback, an independent
continuous-time small-error calculation gives lateral velocity variance
`32,064.182 (m/s)^2`, or standard deviation `179.064743 m/s`, agreeing with
the propagation output `179.065 m/s`. The leading initial gyro-bias term is
`(g*T^2*sigma_bg/2)^2 = 31,159.204 (m/s)^2`, about 97.2% of that variance.
The calculation also includes initial velocity/roll/accelerometer-bias
uncertainty, acceleration/gyro white noise, and both bias random walks.
This explains the large covariance scale under the declared prior in an
unobserved lateral direction; it does not establish a sensor-calibrated
prior or validate the linearization over such a wide attitude distribution.

The evidence does **not** justify treating the pose/attitude gain truncation
as the unique implementation defect, nor replacing it with unrestricted
ESKF feedback. The full reference can be worse even under independent wheel
disturbances. The current constrained strategy also remains unsuitable for
an accuracy claim: it is substantially worse than wheel-off in the disturbed
turning control. The real 1010 record's unique root cause remains unresolved.
These tests did not replay that record with full feedback. Long-horizon
matched-noise Monte Carlo and a frozen-data feedback comparison remain
separate unfinished discriminators; production priors, NHC, gates, and
feedback policy are unchanged.

All 40 `//:local_estimator_tests` cases passed, including the existing
propagation Monte Carlo. The aggregate `//:localization_algorithm_tests`
timeout class is now medium because it also includes these long-duration
tests; that aggregate target was not rerun for this diagnostic. Logs, the
tested source snapshot, and hashes are stored under the ignored
`feedback_policy_20261014/` directory beside the frozen RFU artifacts.

#### 2026-10-14 covariance validity, NIS, event ledger, and Monte Carlo

**P0 — invalid covariance use after LIO.** A successful `LocalLio::ApplyUpdate`
injects a right-multiplicative attitude correction and inflates covariance
diagonals, but does not apply an attitude-error covariance reset Jacobian.
`LocalEstimator::AddLidar` then sets `covariance_model_valid=false`. This
prevents qualified motion export, but does not disable subsequent wheel
updates: `AddImu` still sends queued wheel samples to `UpdateWheel`, which
forms innovation covariance and gain from the current covariance without
checking that flag. The follow-up fix now rejects a queued wheel update with
`CORRELATION_UNQUALIFIED` before computing `S` or gain whenever the internal
covariance model is invalid. IMU propagation and local pose continuity
continue; wheel health reports the rejection, and no wheel-update trace is
emitted. A regression confirms this after a successful LIO correction. The
estimator does not revalidate covariance after LIO, so wheel updates remain
disabled for that epoch; a stopped reset is required to start a fresh
qualified epoch. The attitude-reset Jacobian is still not applied because
LIO-influenced covariance remains invalid and cannot be used as a qualified
covariance. Repeated LIO geometry updates remain a separate heuristic local
correction path and do not restore covariance validity.

**P1 — NIS audit.** Both replays use the frozen 5,895-sample event stream
(`events.csv` SHA-256
`9606849d2191c207f5230e77b1c46bd581f11aa01ad3f5b93d1ac6b1901ef943`).
The NHC-off scalar update uses \(r^2/S\), one degree of freedom; the NHC-on
update uses \(r^\mathsf{T}S^{-1}r\), three degrees of freedom, recomputed from
the complete \(3\times3\) innovation covariance. Recomputed and logged NIS
agree within `2.8e-17` and `1.8e-15`, respectively.

| Diagnostic | NHC off (1 DoF) | NHC on (3 DoF) |
|---|---:|---:|
| Wheel updates | 5,693 | 5,693 |
| NIS mean | 0.00568 | 0.09507 |
| NIS median / P95 / maximum | 0.001495 / 0.02749 / 0.44931 | 0.01445 / 0.35648 / 4.26632 |
| Samples above chi-square 95% reference | 0 / 5,693 (`3.841`) | 0 / 5,693 (`7.815`) |
| Lag-1 innovation autocorrelation | 0.9885 | 0.8757 / 0.9893 / 0.9780 (x/y/z) |
| Lag-1 NIS autocorrelation | 0.7375 | 0.9784 |

These same-record innovations are strongly serially correlated and are not
independent calibrated measurement residuals. The low NIS values and lack of
95% chi-square exceedances do not establish covariance calibration, model
correctness, or gate validity; no innovation gate or noise parameter was
changed. Strong serial correlation is a property of the observed same-record
residual sequence, not a threshold defect to tune away. Summary JSON and full traces are under
`pkgs/openloc/.agents/notes/chassis_imu_dr_calibration_20261013/forward_150054_rfu/`:
`nis_audit_20261014_summary.json` (SHA-256
`4efff481b0c9e1b1f77513a8767ada92076b8d55d49727d9cb79b6800f89c9de`),
NHC-on trace SHA-256
`7f591f56dec96e0eb5d0e6d1367ec664f7d5c908599b0c3e6442a1eaae07e5e5`.

**P2 — full covariance event ledger.** The NHC-off trace now contains the
full 15×15 covariance after every event, \(FPF^\mathsf{T}-P\), full discrete
`Q`, and each wheel update's \(P_\text{after}-P_\text{before}\). Across 11,386
IMU propagations and 5,693 wheel updates, the maximum per-event full-matrix
closure residual is `2.33e-10`; the reconstructed `FPFᵀ-P` agrees with the
logged prediction increment within `1.78e-15`, and `Q` symmetry error is at
most `2.67e-22`. The compensated full-matrix sum differs from the
start-to-end covariance by `1.295e-7` maximum absolute at `P(1,1)`, or
`1.92e-14` relative to the largest covariance element (`6.7435e6`). For the
previously tracked `P(v_y,v_y)`, start/end are `0.0025` and
`17,728.576289675962`; accumulated `FPFᵀ-P`, `Q`, and wheel-update changes
are `+19,470.928486186916`, `+0.5693044161744887`, and
`-1,742.9240009271798`, respectively. Their sum differs from the endpoint
delta by about `5.5e-11`. The initial absolute `1e-8` script threshold was
not a repository criterion and was too strict for the large full-matrix
entries; the event-wise and scale-relative closure above is the retained
result, with the absolute residual preserved rather than hidden.

The trace and summary are in
`pkgs/openloc/.agents/notes/chassis_imu_dr_calibration_20261013/forward_150054_rfu/covariance_event_ledger_20261014/`
(trace SHA-256
`a1b1fc2d07b3183de5146540906b609a0a2f27e720cc5cf6e0e7cb0c841439c3`,
summary SHA-256
`1b62e6fea11dbaa6bac8bebbd5754f1c78f6c6433afffb96b210d83f93f85997`).
The event hash is unchanged and the trajectory remains byte-identical to the
frozen baseline (`4fa92b5f4c8176708933f9305c87988da467aa4900ab108a84f38e341a9f925c`).

**P3 — bounded synthetic Monte Carlo regression.** A deterministic
2,048-trial test (seed `20261014`) now compares the empirical full 15-state
error covariance against the mean propagated covariance over a 1 s, 100 ×
10 ms synthetic run. Initial errors are sampled from a declared diagonal
covariance; the known truth has fixed body specific force
`(0.15,-0.08,9.80665) m/s²` and zero angular rate. IMU white-noise sample
standard deviations are the configured continuous density divided by
\(\sqrt{\Delta t}\); gyro and accelerometer bias random walks use their
configured density times \(\sqrt{\Delta t}\). All 225 covariance entries
pass the predeclared four-sigma sampling bound plus a 2% finite-step
discretization allowance. The first test implementation exposed a dangling
Eigen expression in the test-only noise sampler; explicitly evaluating the
returned vector fixed the harness, not the estimator. The test is marked
`medium` because this statistical regression takes about 58 s.

This establishes one bounded synthetic consistency case only. It does not
validate all angular-rate/force profiles, sensor bandwidth or correlation,
real sensor noise densities, LIO covariance handling, wheel/NHC statistics,
or real-record accuracy. No parameter calibration or production admission
follows from it. The full `//:local_estimator_tests` target passes with this
regression; the replay executable SHA-256 is
`69ee8a6d88debc7ed4ea02d4d36dd1005bf5f98a2321effbb938bfd169c68770`.

### Map-assisted ICP follow-up

The 2026-10-12 overlap/cache/iteration ablation kept the 0.4 source/target
defaults and tested 0.25 target overlap only as an explicit experiment. The
candidate increased same-record matches from 19/196 to 129/196, but did not
prove those matches correct. Four-window prewarming shifted about 1.94–1.98 s
to startup and reduced the 40-iteration per-frame maximum to 212.57 ms, while
P95 remained 180.23 ms. The best tested latency case (10 iterations with
prewarming) measured 58.02 ms P95 / 71.73 ms max, with convergence reduced
from 145/196 to 136/196. No configuration met both provisional latency goals;
none is promoted to a production default. Full metrics, per-frame outputs, and
hash-pinned provenance are in the
[map-assisted validation plan](../../../ad_agents/context/modules/localization/knowledge/map-assisted-localization-validation-plan.md)
and ignored replay archive
`/home/wfh/01code/map-infra/offline_map/results/20261008141912/scan_to_map_icp_followup_a17c39bf/`.

## Next work: parallel evidence tracks

The immediate goal is the minimum odometry + global-correction algorithm
loop, not production GNSS qualification. The existing `LocalizationEngine`
and GTSAM `AlignmentSmoother` already provide the smallest loop: FAST-LIO2
remains GNSS-off and owns `T_odom_body`; odometry motion connects fixed-lag
four-DoF alignment states; payload-time GGA position factors correct only
`T_enu_odom`; output composition is
`T_enu_body = T_enu_odom * T_odom_body`. Explicit diagnostic-only input and
uncalibrated covariance assumptions are acceptable for this algorithm
milestone. They do not establish truth, accuracy, or production admission.

The raw `LIDAR_20261008141912.bag` was inspected read-only: it has 1,640
`/livox/lidar` messages and 32,799 `/livox/imu` messages (34,439 total), but
no FAST-LIO2 `/Odometry`, registered cloud, health, effective-point-count, or
covariance topics. It can support a new GNSS-off diagnostic replay, but
historical runtime health telemetry cannot be recovered from that input bag.
The current result directory contains the audit JSON/CSV outputs, not the
previous temporary OpenLoc result JSON or a pinned diagnostic runtime
manifest; those artifacts remain unavailable.

1. **Algorithm loop (active delivery):** preserve the existing 4DoF A_k chain;
   complete focused verification of initialization, timestamped interpolation,
   position-factor correction, rejection reporting, outage uncertainty,
   recovery, and ODOM immutability. Add a nonzero-roll/pitch ODOM regression
   for the ENU horizontal distance/direction checks and make the horizontal
   ODOM frame precondition explicit at the adapter boundary; do not enable a
   leveling transform until P0-3 passes. Close the output-contract gap for
   explicit observability and `calibration_state`. Use the existing same-record
   diagnostic result as consistency evidence; only regenerate it if exact
   source exports and provenance are available. Keep diagnostic covariance
   and lever arm explicitly uncalibrated.
2. **Input/frame closure (production qualification, parallel):** the P0-2 API/startup contract now requires full
   declarations for verified references and supports explicit
   `diagnostic_only` references. Per the dataset validation in
   `/mnt/synology/3dmakerpro/schema/rtk-factor-admission-20261008.md`, the
   diagnostic uses UTM EPSG:32650, the first GGA as local origin, and
   ellipsoidal height from GGA `H+N`; those known values are now recorded in
   its diagnostic reference. The source CRS realization/coordinate epoch,
   geoid-model provenance, and independent survey provenance remain
   unconfirmed; do not promote this diagnostic origin to a surveyed
   production reference. Keep P0-3 unresolved pending timestamped runtime
   gravity paired with a physically confirmed stationary interval and
   independent up reference. Keep P0-4/P0-5/P0-7 open pending physical frame
   evidence, measured antenna lever arm, and surveyed station/height
   provenance. No `R_level` activation or production GNSS admission.
3. **Diagnostics and truth (production qualification, parallel):** prepare a bounded GNSS-off instrumentation
   protocol to capture timestamp-aligned FAST-LIO effective point counts,
   exposed health/degeneracy state, and defined covariance, if available.
   Do not launch replay until the protocol and artifacts are frozen and
   explicitly approved. In parallel, prepare static surveyed control-point
   capture and dynamic PPK inputs; use independent truth for later rejection
   attribution and covariance validation.

Do not rerun the completed A–D audit, enable GNSS in the FAST-LIO2 frontend,
retune gates, change the graph, or promote same-record residuals to truth.
Detailed gates and evidence requirements are in
[`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md).

## Dataset allocation and blockers

- `/mnt/synology/3dmakerpro/20261008141912/` is the sole dataset in the
  current validation scope. Its schema/profile and GNSS admission findings
  are under `/mnt/synology/3dmakerpro/schema/`.
- Receiver-reported RTK-fixed quality does not establish calibrated
  uncertainty or independent truth. GNSS remains disabled in the FAST-LIO2
  frontend and is used only as a provisional OpenLoc global-alignment
  diagnostic factor; production factor admission remains disabled.
  Openloc's synthetic global-alignment tests and isolated same-record
  diagnostic pass; independent-truth evaluation does not exist.
- The full baseline's reference-free gate passed over 162.4 seconds at 99.63%
  trajectory coverage. The same-record GNSS comparison is non-independent;
  exact metrics, source hashes, ODOM suffix reconciliation, per-epoch RTCM
  association, and artifact paths are in
  [`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md).
- FAST-LIO2 here is an external baseline, not `pkgs/openloc`'s private
  `LocalEstimator`/`LocalLio`; validating Openloc's own L1 against this data
  still requires an adapter/integration decision and run.
- No prior point-cloud map is required. The thumbnail PLY is not needed for
  the LIO+GNSS target.
- If the GNSS stream is both fused and scored, report consistency rather than
  independent accuracy. Absolute accuracy needs independent truth.

See [`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md)
for the bounded audit and replay plan.

## Milestones

| ID | Scope | Exit gate | Current state |
|---|---|---|---|
| V0 — Input contracts and frontend selection | Reconcile the selected dataset against its schema/profile; pin the GNSS-off FAST-LIO2 frontend; define remaining clock, ENU-origin, lever-arm, and delayed-measurement behavior. | Input/profile/config identity and unresolved assumptions are explicit before replay/fusion. | **Input/init gate passed; external FAST-LIO2 ODOM selected for current global-alignment validation. Actual source/executable are pinned in the run manifest and differ from the older profile diagnostic build. GNSS admission assumptions remain unresolved.** |
| V1 — LIO baseline | Run bounded FAST-LIO2 with GNSS factors disabled; verify initialization, continuous ODOM, timestamps, gaps, resets, and resource use. | Reproducible complete ODOM trajectory without unexplained gaps/resets. | **First 163 s baseline and integrity subgate passed (1,625 poses, 99.63% coverage); runtime resource use and deterministic repeatability remain unqualified.** |
| V2 — Continuous L1 ODOM | Validate OpenLoc `LocalEstimator`/`LocalLio` on LiDAR/IMU with GNSS off, after resolving initialization and adapter requirements. | Synthetic tests and bounded replay pass input, timestamp, continuity, and resource checks. | **Out of current scope and blocked: user selected external FAST-LIO2 ODOM for the current global-alignment validation; no qualifying OpenLoc quiet startup interval was found.** |
| V3 — GNSS consistency diagnostic | Compare LIO ODOM to the same-record GNSS track with explicit time/frame/antenna assumptions; do not score as independent accuracy. | Reproducible, correctly time-associated residual report with limitations. | **Completed for 162 GNSS epochs/161 s using zero offset and a fixed 60 s alignment; reference-relative only, with unresolved PPS and lever arm.** |
| V4 — GNSS factor experiment/admission | Keep production GNSS factors disabled until independent reference or uncertainty calibration plus timing/lever-arm evidence is available. Any new or changed data replay requires an explicit bounded protocol and approval. | Accepted GNSS affects only `T_enu_odom`; no production accuracy claim without admission evidence. | **One isolated OpenLoc same-record diagnostic completed; original temporary JSON is unavailable, while a separate read-only reconstruction reproduces its 148/3/11 aggregate. Production factor admission remains blocked by evidence gaps.** |
| V5 — Runtime integration and performance | Add separate host adapters and measure intended-hardware rate, latency, resource use, GNSS outages, and recovery. | Reviewed runtime budgets pass on target hardware. | **Not started.** |

V0 gates replay configuration; V1 establishes an LIO trajectory; V2 verifies
the selected L1 behavior; V3 provides only GNSS consistency evidence; V4
requires additional admission evidence before production factor use; V5
precedes runtime readiness claims.

## Immediate execution order

1. Supply verified production CRS, coordinate epoch, vertical datum, and ENU
   origin from external provenance, and ensure the host adapter never treats
   `diagnostic_only` as production-admissible. The API validator and focused
   tests are implemented; the dataset remains unresolved.
2. Keep the P0-3, P0-4, P0-5 and P0-7 evidence gates explicit: collect runtime
   gravity with physical quiet-window evidence, establish actual FAST-LIO2
   frame semantics, measure the antenna lever arm, and source survey-grade
   station/height metadata.
3. In parallel, freeze a proposed GNSS-off telemetry capture protocol from
   the confirmed LiDAR/IMU source bag. Capture LIO diagnostics only through a
   separately approved run; do not claim unavailable historical telemetry.
4. Prepare independent static control and dynamic PPK truth inputs. Keep the
   11 historical distance rejections `UNRESOLVED` unless new evidence
   supports a specific cause.
5. Keep GNSS-on FAST-LIO2, graph changes, gate tuning, and production GNSS
   admission out of scope until their stated approval/evidence gates pass.

Do not promote the baseline to Openloc or GNSS-fusion evidence. Keep source
recordings read-only; resolve Openloc integration, timing, calibration, and
GNSS-admission gaps before extending the validation.

## Evidence and promotion rules

For each material change, record the hypothesis, fixed input/configuration,
initialization, thresholds, exact commands/artifacts, result (**pass**, **fail**,
or **inconclusive**), and limitations. Keep negative outcomes. Estimator
agreement is a diagnostic, not accuracy evidence. A successful solver,
smoother trajectory, or passing software test does not establish vehicle
accuracy, calibrated uncertainty, safety, or readiness.
