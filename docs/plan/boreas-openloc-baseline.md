# Boreas OpenLoc scan-to-map diagnostic baseline

## Scope

This experiment exercises `ScanToMapLocalizer` with a frozen cross-record
Boreas map and query scans. It is a controlled local-convergence/component
baseline, not independent odometry, global relocalization, calibrated
uncertainty, vehicle accuracy, or production evidence.

The map and query references are the dataset's post-processed Applanix poses.
The query reference trajectory supplies both the matcher initial poses and the
scoring reference. Each initial pose receives a +1 m map-east translation bias
and a +2 degree map-yaw bias. This intentionally measures local correction
around a reference-informed trajectory, not an OpenLoc-created ODOM solution
or a cold-start/relocalization capture range.

## Frozen input

- Dataset: [Boreas](https://boreas.utias.utoronto.ca/), distributed under
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
- Map sequence: `boreas-2020-11-26-13-58`, pose-table indices
  `[5434, 5534)` with step 2 (50 scans).
- Query sequence: `boreas-2021-01-15-12-17`, indices `[5306, 5405)` with
  step 2 (50 expected scans). The scan at timestamp
  `1610731622464417` microseconds is absent; the retained pilot therefore has
  49 query scans. The preparation script fails if any other selected scan is
  missing or if an unexpected scan is present.
- Raw files, pose tables, calibration matrices, route pages, and local run
  outputs are retained under the ignored
  `.agents/notes/boreas_openloc_baseline/` directory. The generated manifest
  records SHA-256 hashes for raw and prepared inputs.

## Frame and time contract

- Boreas scan records contain six little-endian `float32` values per point:
  `x, y, z, intensity, laser_number, time`. The final field is seconds relative
  to the scan-center timestamp encoded in the filename (microseconds).
- `lidar_poses.csv` `GPSTime` and scan filename timestamps are microseconds.
  Pose translation is linearly interpolated; orientation uses quaternion SLERP
  after applying the Boreas `Rx(roll) * Ry(pitch) * Rz(heading)` convention.
- `T_applanix_lidar` maps LiDAR points into the Applanix body frame. Map points
  are transformed at each point time into local ENU, with the first selected
  map-pose translation subtracted. Query points are deskewed into Applanix body
  at scan center. Thus the existing replay runner's `body` frame names denote
  Applanix for this prepared input; no source frame is silently relabeled.
- Boreas poses are post-processed references. They are not replayed as live
  GNSS and do not establish independent truth or calibrated covariance.

## Procedure

Run in the managed development container as the mapped non-root user. From
`/apollo/pkgs/openloc`, use a new output directory for each preparation and
run:

```bash
python3 test/integration/map/boreas_openloc_baseline.py prepare \
  --data-root .agents/notes/boreas_openloc_baseline/data \
  --output-dir .agents/notes/boreas_openloc_baseline/runs/<new-run>/prepared
env -u LD_LIBRARY_PATH bazel build //:scan_to_map_dataset_replay
python3 test/integration/map/boreas_openloc_baseline.py run \
  --prepared-dir .agents/notes/boreas_openloc_baseline/runs/<new-run>/prepared \
  --runner bazel-bin/scan_to_map_dataset_replay \
  --matcher point-to-plane-icp --map-voxel-m 0.5 --icp-max-points 12000 \
  --output-dir .agents/notes/boreas_openloc_baseline/runs/<new-run>/results
```

The preparation stage preserves the full-resolution binary PCD as
`map_raw.pcd` and writes the matcher map as `map.pcd`, formed from 0.5 m XYZ
voxel centroids using `floor(point / leaf_size)` cell assignment. It also writes
49 deskewed query PCDs, `requests.csv`, `reference.csv`, and a hash-pinned
`manifest.json`. The run stage verifies prepared-file hashes, invokes the
existing replay binary with the selected matcher options, then writes raw
matcher results, per-scan evaluation, and a JSON summary. Software-harness
completion requires 49 unique output timestamps; match rate and pose errors
are reported descriptively without inventing a performance acceptance
threshold.

The selected `pilot_v2` run uses the existing point-to-plane ICP diagnostic
backend with a 0.5 m matcher map voxel, 12,000 maximum registration points,
40 iterations, and ICP timing enabled. This is an explicitly configured
component comparison, not a change to the library default NDT backend.

## Recorded pilot result

The 2026-10-19 `pilot_v2` preparation reduced the map from 10,690,417 raw
points to 556,361 0.5 m voxel centroids while preserving both PCDs and their
hashes. All 49 query frames were processed in both point-to-plane ICP arms:

| ICP profile | Accepted | Rejected reasons | Match processing time, median / P95 / max |
| --- | ---: | --- | --- |
| 40 iterations, 12k points | 0/49 | 46 iteration limit; 3 residual gate | 1.382 / 2.157 / 2.225 s |
| 80 iterations, 12k points | 0/49 | 43 iteration limit; 2 insufficient bidirectional overlap; 4 residual gate | 2.739 / 3.552 / 25.200 s |

The 1 m / 2 degree reference-derived initial bias is present on every query.
No ICP output pose was valid, so matched-pose accuracy metrics are unavailable.
The 80-iteration sensitivity did not produce an accepted observation and
increased per-scan processing cost. Keep the 40-iteration result as the
reproducible selected profile; do not tune acceptance gates from this pilot.

Two default-NDT attempts were stopped without a completed scan row: the
full-resolution 10,690,417-point map ran for about 17 minutes, and the
0.5 m/556,361-point map ran for about 14 minutes. They are recorded as
incomplete attempts, not matcher failures or scored runs. `attempts.json`
records the commands, outcomes, and artifact hashes. The selected ICP run's
`results.csv`, `evaluation.csv`, and `summary.json` are under
`.agents/notes/boreas_openloc_baseline/runs/pilot_v2/results_icp_v1/`;
the 80-iteration sensitivity is under `results_icp_80iter/`.

## Root-cause diagnostics

The rejected scan-to-map result was investigated with reference-pose and
same-scan controls. These experiments do not constitute a matcher fix:

| Diagnostic | Outcome |
| --- | --- |
| Verify the 49 oracle requests against `reference.csv` | Maximum translation difference 0 m; maximum quaternion angular difference below `5.2e-8` rad. |
| First query at its reference pose, transformed into the prepared map frame | Nearest-map distance median / P95 `0.302 / 1.144 m`; 97.3% of sampled points are within 1.5 m. |
| Same scan used as map and query, identity seed, 50k registration-point cap | `MATCHED` in 2 iterations; score `0.0338 m`; correction `0.0029 m` and `0.00012 rad`. This checks the real scan format/frame path and a self-match, not cross-record alignment. |
| Cross-record first query at its exact reference pose, 50k cap, RMSE gate relaxed to 0.25 m | `MATCHED`, but the output is `1.962 m` and `0.0393 rad` from the reference. Relaxing the gate therefore admits a poor pose and is not a remedy. |
| All 49 cross-record queries at exact reference poses, 12k cap, 40 iterations, 0.01 m / 0.001 rad step tolerances | 0 accepted: 24 iteration limits, 17 overlap gates, 8 residual gates. |
| Same oracle requests, 200 iterations | 0 accepted: 34 overlap gates, 10 residual gates, 5 iteration limits. |
| Same-scan control with the baseline's +1 m / +2 degree seed and 12k cap | `MATCHED` in 6 iterations; translation / rotation error to identity `0.0053 m / 0.0000734 rad`. |
| Reverse cross-record split (Jan 15 map, Nov 26 query), 49 scans, baseline ICP profile | 0/49 accepted: 47 iteration limits, 1 overlap gate, 1 residual gate. This split is intentionally non-chronological and is diagnostic only. |
| Chronological Jan 15 map / Jan 19 query split, 49 scans, baseline ICP profile | 0/49 accepted: 43 iteration limits, 6 residual gates; processing time median / P95 / max `1.320 / 1.428 / 2.159 s`. |
| First chronological-split query, 50k registration points and 200 iterations, from either biased or exact reference seed | Both runs stopped at the iteration limit; the exact-seed run took `20.38 s`. A larger point/iteration budget did not resolve the failure. |
| Same exact-seed query, 50k points, 200 iterations, relaxed step tolerances `0.01 m / 0.001 rad` | `MATCHED` in 10 iterations, but the result is `0.686 m` and `0.0245 rad` from the exact reference. This is a false positive, not a successful baseline. |

The same-scan control and reference-frame support make a gross point-cloud
frame or pose-table mismatch unlikely. The cross-record matcher nevertheless
moves away from a reference-informed pose and either fails its quality gates
or, if step/residual gates are weakened, returns a substantially worse pose.
The same-scan biased-seed control also shows that the configured ICP can
recover the tested 1 m / 2 degree perturbation when source and target scans
are identical. The unchanged rejection on the reverse cross-record split
points to cross-record scene/registration compatibility, not a gross transform
or initial-bias problem; the current evidence does not establish a defect in
the ICP update equations. Do not relax gates or claim a successful baseline
from these controls. No matcher source change was made because the diagnostics
do not yet justify a safe algorithm change.

Commands, outcomes, and output hashes are recorded in `attempts.json`.
Detailed inputs and outputs are retained in
`.agents/notes/boreas_openloc_baseline/runs/pilot_v2/oracle_control/`,
including `results_loose_tolerance.csv`, `results_200iter.csv`,
`self_match_result.csv`, and `result_first_50k_rmse025.csv`. The reverse split,
its same-scan biased-seed control, and its attempt record are under
`.agents/notes/boreas_openloc_baseline/runs/pilot_v3/`; the unused incomplete
`prepared/` output was removed after verifying its `incomplete_not_used` status
and missing manifest. Its attempt record remains; use `prepared_reverse/` for
the recorded run.
The chronological Jan 15 / Jan 19 split, its selected-scan downloader,
per-object download hashes, preparation wrapper, replay output, and diagnostics
are under `.agents/notes/boreas_openloc_baseline/runs/pilot_v4/`. Only 50
selected LiDAR objects (259,141,584 bytes) were fetched from the public S3
bucket; no full sequence was synced. The query window `[5546,5645)` was chosen
against the map window by trajectory proximity (median / P95 nearest-position
distance `0.372 / 0.745 m`, median heading difference `0.006 rad`). One
available query scan is intentionally excluded to preserve the existing
49-row replay contract, and the map's absent scan is separately recorded in
the prepared manifest. `oracle_first_50k_200iter_loose_step.csv` records the
false-positive exact-seed diagnostic.

Focused script checks can be run from the package root:

```bash
python3 -m unittest discover -s test/integration/map \
  -p 'boreas_openloc_baseline_test.py'
```

## Interpretation

`MATCHED` means only that the standalone matcher accepted that observation
under its current software gates. Its covariance remains uncalibrated, the
map reference is diagnostic-only, and the query prior is reference-derived.
The output can support repeatable matcher/configuration comparisons, but cannot
qualify the map as an independent graph factor or demonstrate online
relocalization. Preserve failed/rejected rows and record the exact binary,
input manifest, command, and output hashes for each run.
