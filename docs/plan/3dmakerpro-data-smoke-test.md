# 3DMakerPro Data Smoke and Integration Test Plan

**Status:** the data smoke and minimum synthetic MapMatcher gate pass. The
controlled matcher gate does not qualify the algorithm against the external
session. External-session registration remains gated on pose/frame provenance
and a defensible initialization source; recovery remains gated on its public
contracts.

This plan applies Openloc's functional principles: relative localization for
drivable-area use, explicit trust/degradation, and first-class relocalization
and self-recovery. The active architecture and implementation gates remain in
[`../architecture.md`](../architecture.md) and
[`../roadmap.md`](../roadmap.md).
The current sensor-input evidence and unresolved Mid-360/G100 assumptions are
tracked in [`../input-contract.md`](../input-contract.md).

## Goal

Make one explicitly selected 3DMakerPro recording quick to validate from the
managed development environment, without copying the large dataset into the
repository or coupling Openloc's public API to ROS, the scanner SDK, or local
filesystem layout.

The phase-1 test is a **data and adapter smoke test**, not an accuracy or
vehicle qualification test. A synthetic MapMatcher test is a separate,
provenance-independent software gate. External-session registration and
recovery remain gated on verified input semantics and public contracts.

## Verified data shape

The initial recommended session is
`/mnt/synology/3dmakerpro/20260727101936/`:

- `pcd/frames.csv` has 1241 rows and columns `index`, `file`, `seq`,
  `stamp_sec`, `timebase_ns`, `points`, `frame_id`, and `lidar_id`.
- All manifest PCD paths exist; each PCD header's point count matches the
  manifest. `timebase_ns` is strictly increasing, and the session has one
  `frame_id` and one `lidar_id`.
- Individual frame files are binary PCD with `x y z intensity tag line
  timestamp`; the maps use `x y z intensity`. The `amap/amap_5cm.pcd` map has
  684,528 points; the 2 cm map has 3,386,479 points. Start with the 5 cm map
  for lower-cost loading.
- The directory also contains a 405.9 MiB ROS1 bag. Its file header identifies
  ROS bag v2, but `rosbag`/`rostopic` tools were not available during planning.
- `recalculate_trj.txt` exists, but its format and provenance are not
  established for this session; do not treat it as ground truth or a trusted
  initial pose. Vendor documentation identifies the corresponding
  recalculation trajectory as an output, not independent truth.

The G100 is only inferred to contain a Mid-360 from the reported scan and IMU
rates. The available G100 manual is not the Mid-360 manual, and neither the
manual's coordinate/tag figures nor the sensor identity have been verified
for this recording. Keep external registration gated on the frame,
extrinsic, timestamp, map, and initialization evidence in
[`../input-contract.md`](../input-contract.md).

The automated smoke test subsequently verified the full manifest structure
and paths, and decoded bounded point samples from the first, middle, and last
frames and 5 cm map. It did not decode every point, run registration, or
qualify trajectory accuracy.

## Scope and architecture

The test-only adapter is implemented under `test/integration/3dmakerpro/`. It:

1. Reads the caller-selected root from `OPENLOC_3DMAKERPRO_ROOT`; no data path
   is hardcoded in the source or Bazel target.
2. Parses `pcd/frames.csv`, checks all listed paths remain beneath the root,
   and validates required fields, monotonic time, and stable frame/LiDAR IDs.
3. Decodes a bounded deterministic sample of frames and the selected map,
   checking PCD payloads, sampled finite XYZ values, and manifest point counts.
4. Emits actionable failures without logging raw point values.

Keep this adapter outside `include/openloc/`: filesystem parsing and host data
formats are not library contracts. The current public online engine consumes
odometry and GNSS, not LiDAR clouds. An integration with the existing
`unified::MapMatcher` is therefore a candidate-algorithm test, not a test of
the public online engine or a production relocalization system.

## Phases and exit gates

### 1. Data-contract and point-cloud smoke — implemented

`//:openloc_3dmakerpro_smoke_test` uses only the C++ standard library and
GoogleTest, independently of GTSAM and NDT. It validates the manifest, then
decodes bounded point samples from first/middle/last frames and the 5 cm map.
This tests the data adapter, not localization.

Acceptance:

- An unset data-root variable produces an explicit skip with usage guidance;
  a supplied invalid root fails.
- Validate required columns, safe relative file paths, monotonically
  increasing sensor time, stable frame/LiDAR identity, PCD readability, and
  manifest/header point-count agreement for every selected frame.
- Verify finite XYZ values and sufficient points in the selected subset.
- Run from an external data root without modifying, copying, or checking in
  the source recording.

This phase proves adapter/data readability only. The test does not use PCL,
GTSAM, or NDT, and does not claim map alignment, usable localization, trust
calibration, or recovery.

### 2. Minimum testable MapMatcher version — passed

Before interpreting the external recording, establish that the private
candidate matcher can perform a bounded, repeatable registration on controlled
inputs. Extend the existing
`test/localization/candidates/map/map_matcher_test.cc`, which is already
included in `//:localization_map_tests`; do not add a public API or a new
runtime dependency for this gate.

Use a deterministic, asymmetric 3D synthetic map and derive scan points from
a known rigid transform. Keep the fixture and transform generation local to
the test, fix all parameters and inputs, and declare error tolerances before
using the result to tune the matcher. Cover:

- a valid initialization near the known transform converges and returns a
  pose within the declared translation/rotation tolerances;
- invalid input and insufficient overlap are rejected explicitly;
- geometrically degenerate input does not become a trusted full 6-DoF pose;
- repeated execution with the same inputs gives stable status and pose within
  the declared deterministic tolerance.

The decisive command passed on 2026-10-08:
`env -u LD_LIBRARY_PATH bazel test //:localization_map_tests --test_output=errors`
from the standalone package in the managed container. The fixture can use an
exact known transform, so it does not depend on the provenance of
`recalculate_trj.txt` or the 3DMakerPro coordinate frames. A pass establishes
only that the candidate algorithm satisfies this controlled software
contract; it does not promote the candidate to the public API or establish
real-world accuracy, trust calibration, relocalization, or vehicle readiness.
The deterministic full-scan fixture declares a 90% minimum overlap, 0.12 m
translation tolerance, 0.06 rad rotation tolerance, and 1e-6 repeatability
tolerance. The test also checks invalid and non-overlapping inputs and retains
the existing planar-geometry rejection check.

### 3. Candidate registration against the external session — blocked

The session contains `recalculate_trj.txt` with timestamp and translation /
quaternion columns. Its format alone does not establish who produced it, its
coordinate frame, its relationship to the PCD points/map, or whether it is
independent reference data. The PCD header reports identity `VIEWPOINT`, which
also does not establish the frame relationship. Do not use either as a
registration seed or accuracy truth until those semantics are confirmed.

After the synthetic matcher gate passes, confirm the point-cloud coordinate
frame and calibration/extrinsics, the meaning/timebase of the trajectory, its
frame relative to `amap/amap_5cm.pcd`, and an independently defensible initial
pose source or hypothesis generator. Then add a deterministic test-only
adapter for this session. A matcher result alone is not a trust-state
decision. If those data semantics cannot be established, keep this phase
blocked while the synthetic MVP remains testable.

### 4. Trust, degradation, and recovery scenario

Only after the trust/recovery API and pose/reference provenance exist, add
scenarios for loss of match, ambiguous recovery hypotheses, successful
reacquisition, and reset sequencing. Verify that weak/ambiguous evidence stays
degraded or untrusted and that downstream consumers can identify stale state.
Use the external recording to exercise realistic inputs, and a deterministic
controlled fixture with known perturbations for measurable recovery behavior.

If no independent truth is available, label results as software behavior or
relative consistency evidence; do not claim absolute accuracy or calibrated
confidence.

### 5. Optional ROS bag adapter

Inspect ROS1 bag topics, message types, and timestamp semantics only when the
managed environment has an approved reader. Build any bag-to-domain conversion
as a separate test adapter, preserving original sensor time, frame, and
calibration provenance. Keep ROS dependencies out of the library and make bag
parsing optional; phase 1 must remain runnable from the extracted PCD files.

## Repeatable invocation

Run the minimum testable matcher gate independently of the external data:

```bash
env -u LD_LIBRARY_PATH bazel test \
  //:localization_map_tests --test_output=errors
```

Run the external data smoke test with:

```bash
OPENLOC_3DMAKERPRO_ROOT=/path/to/20260727101936 \
  env -u LD_LIBRARY_PATH bazel test \
  --test_env=OPENLOC_3DMAKERPRO_ROOT \
  //:openloc_3dmakerpro_smoke_test --test_output=errors
```

Use the managed non-root test container and confirm the selected external
dataset path is visible there. When running standalone Openloc tests in that
container, unset the Apollo root workspace's `LD_LIBRARY_PATH` as shown above.
The repository's test mode already mounts
`/mnt/synology`; the session was accessed read-only by the test. Do not copy
recordings into the source tree or alter container configuration. Use
`--test_strategy=local` only if Bazel sandboxing cannot access the external
read-only mount.

## Validation evidence

On 2026-10-08, the managed test container ran
`OPENLOC_3DMAKERPRO_ROOT=/mnt/synology/3dmakerpro/20260727101936 bazel test --nocache_test_results --test_env=OPENLOC_3DMAKERPRO_ROOT //:openloc_3dmakerpro_smoke_test --test_output=all`.
All seven tests passed, including the external session case. That case
validated 1241 manifest rows, sampled three frames (`livox_frame`, LiDAR
`192`), and checked the 684,528-point 5 cm map; its recorded test time was
255 ms. The test accessed the already mounted dataset without copying or
modifying it. Bazel reported 3.789 seconds elapsed and the test binary was
already up to date, so this run is not a cold-build measurement or a basis for
a runtime budget.

After the Eigen export fix, the test was run uncached again with
`LD_LIBRARY_PATH` unset. The external session case passed; the test target
completed in 1.4 seconds with a warm build. This confirms the dependency patch
did not regress the data adapter, but remains data-readability evidence only.

The PCL/Eigen build blocker was resolved without changing the Eigen version:
the package's Eigen `single_version_override` applies
`patches/eigen_required_unsupported_modules.patch`, which exports the
`NonLinearOptimization`, `MatrixFunctions`, and `Polynomials` headers already
present in the resolved Eigen source archive. Afterward, `//:map_matcher`
built, all 23 tests in `//:localization_map_tests` passed, and
`//:online_localization_test`, `//:online_gtsam_fixed_lag_spike_test`, and
`//:online_alignment_smoother_test` passed. These results establish software
build/test readiness only; they do not establish registration against the
external recording.

The PCL build also emits its existing warning about potential Eigen aligned
allocation mismatch when AVX or `EIGEN_MAX_ALIGN_BYTES=32` is not enabled. The
tests passing does not resolve that configuration warning; it remains a
separate item to assess before relying on PCL at runtime.

The 2026-10-08 run passed all 25 tests in `//:localization_map_tests`,
including known-transform registration and repeatability. An earlier test
attempt with a 70% overlap threshold accepted a 1 m-offset initialization:
the returned pose was 1.063 m from the known pose, with fitness score 0.0459
and 0.804 overlap. At the final 90% configured threshold, the same case is
rejected for insufficient overlap. This negative result is preserved because
it demonstrates that the acceptance outcome depends on the configured
overlap threshold; it does not define a universal convergence basin. The
controlled success and rejection cases remain software evidence only, not
accuracy or external-session registration evidence.

The first map-test run exited with signal 11. GDB showed that inherited
`LD_LIBRARY_PATH` caused the test to load stale PCL shared libraries from the
Apollo root workspace's different Bazel output tree. Re-running with
`env -u LD_LIBRARY_PATH` passed, including the complete map-test suite. Clear
that variable when running standalone Openloc Bazel tests inside the managed
Apollo container; this is an environment isolation requirement, not a matcher
code workaround.

Measure a cold and warm smoke run before defining a runtime budget. Do not
claim “quick” by skipping validation or by relying only on cached test
results.

## Prerequisites and blockers

- **Resolved for phase 1:** the managed test container can read the selected
  session through its existing `/mnt/synology` mount.
- **Resolved for software-test readiness:** the package-local Eigen export
  patch enables PCL/GTSAM targets, and focused matcher, online, and smoother
  tests pass. Standalone tests in the managed Apollo container must clear its
  inherited `LD_LIBRARY_PATH` to avoid loading shared libraries from another
  Bazel output tree.
- **Blocked for external registration:** no verified initial pose or
  map/trajectory/point-cloud frame relationship is available yet. Build/test
  success does not authorize an assumed identity seed or treating
  `recalculate_trj.txt` as truth.
- **Open data semantics:** confirm frame PCD coordinate frame, calibration and
  extrinsics, `stamp_sec` versus `timebase_ns`, trajectory producer/provenance,
  and trajectory/map frame relationship.
- **Open initialization/truth:** establish a defensible map-relative initial
  pose source or recovery hypothesis generator. Do not infer one from filenames
  or treat `recalculate_trj.txt` as ground truth.
- Current Openloc does not expose a complete trust/degradation and
  relocalization lifecycle through the online public API; phase 3 depends on
  those roadmap contracts and must not be simulated as if already supported.

## Data and result hygiene

Keep the recordings on the mounted data volume. Do not add the bags, maps,
calibration values, trajectory exports, or derived large clouds to Git.
Report the selected session identifier, manifest/map fingerprints, frame
range, configuration, test result, runtime, and limitations. Avoid printing
raw coordinates or dumping point data in routine test output.
