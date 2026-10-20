# Apollo Cyber Record Validation Plan for OpenLoc

> **Historical dataset plan.** This document applies only to
> `/mnt/synology/基础版本/slam_data/apollo/` and is not the active G100/RTK
> development plan. The selected dataset is now
> [`g100-rtk-data-validation.md`](g100-rtk-data-validation.md); retain this
> document as evidence for the earlier Apollo Cyber Record schema inspection.

**Status:** proposed; schema inspection is complete, but input semantics and
algorithm validation against this recording have not been completed.

This plan uses the read-only dataset
`/mnt/synology/基础版本/slam_data/apollo/` and its
[`SCHEMA.md`](/mnt/synology/基础版本/slam_data/apollo/SCHEMA.md). It follows
the package boundaries in [`../architecture.md`](../architecture.md), the
input requirements in [`../input-contract.md`](../input-contract.md), and the
evidence/promotion rules in [`../roadmap.md`](../roadmap.md).

## Goal and evidence boundary

Determine whether the two Apollo Cyber Record segments can provide a
defensible real-data input for a private OpenLoc `MapMatcher` candidate
experiment. Do not couple the Cyber Record reader, Apollo protobufs, or record
filesystem layout to OpenLoc's public API or runtime dependencies.

The records contain 1,071 indexed `apollo.drivers.PointCloud` messages,
10,689 `apollo.localization.CorrectedImu` messages, 10,689
`apollo.localization.Gps` messages on a topic named `gnss/odometry`, 107
`GnssBestPose` messages, and one `/tf_static` message. The embedded descriptors
and bounded samples have been inspected. This establishes schema readability,
not complete data quality, valid registration geometry, an ODOM trajectory,
or localization accuracy.

There is no inventoried standalone global point-cloud map. A local map could
be assembled from scans only if their time, frame, and placement are
established. A map and query scans derived from this same recording are
correlated evidence, not independent truth.

## What can and cannot be validated

| Evidence question | Possible from this dataset? | Boundary |
|---|---|---|
| Decode record metadata, descriptors, and sensor messages | Yes | Schema/readability evidence only. |
| Audit point-cloud shape, finite values, per-point timestamps, and stream coverage | Yes | Requires bounded or streaming inspection of the actual messages; the current schema check sampled payloads only. |
| Exercise `MapMatcher` on real scan geometry | Conditional | Requires a defensible map frame, scan frame, time association, map construction, and initial pose. No global map is supplied. |
| Validate local online estimation using continuous ODOM | Not established | The `Gps`-typed topic is not proof of continuous local ODOM. No verified ODOM source has been identified. |
| Establish absolute accuracy or calibrated trust | No, from this recording alone | No independent truth or independently validated reference map/trajectory is identified. |

GNSS-derived poses may be evaluated as candidate map-relative placement data
only after their producer, coordinate frame, origin, timestamp, and quality
semantics are established. They must not initialize, correct, reset, or
substitute for the local ODOM state.

## Phases and gates

### 0. Preserve the baseline and read-only inputs

- Keep both record segments on the mounted data volume and do not modify or
  copy them into the OpenLoc checkout.
- Use the generic Cyber Record inspection procedure to select an isolated
  `cyber_record` environment. Do not change shared protobuf dependencies to
  make the reader work.
- Keep OpenLoc middleware-independent. Any protobuf conversion or record
  access belongs in an external script or test-only adapter.

**Gate:** record paths are readable, package/test baseline is unchanged, and
all generated or temporary artifacts have an explicit location outside the
source data and Git tree.

### 1. Complete a bounded input audit

Inspect message metadata and payloads across both segments without dumping
raw point values:

- Reconcile actual decoded message counts with record index counts for all
  five channels; report missing, malformed, or undecodable messages.
- For every point cloud, report point count, dimensions, finite/nonfinite XYZ
  counts, zero-range count, field population, and point-timestamp presence and
  range. Do not assign a timestamp unit or epoch from its field name.
- For each message stream, summarize populated header fields, sequence
  continuity, timestamp monotonicity/regressions, duplicate stamps, and
  inter-message interval distributions. Keep header time,
  `measurement_time`, point time, and record/index time as distinct values
  until their clock relationship is proven.
- Preserve `GnssBestPose` solution status/type, uncertainty, solution age,
  and satellite fields in the audit. A populated pose or receiver field alone
  does not establish a usable fix.
- Decode and inventory all seven transforms in the existing `/tf_static`
  sample, including parent/child names, direction, and values. A transform
  list is not calibration evidence until its frame names and producer are
  understood.
- Record raw counts and counts after each explicitly selected quality filter.
  Do not introduce permanent point or GNSS filters during this audit.

**Gate:** produce a concise, reproducible audit summary and identify any
message gaps, timestamp ambiguity, invalid values, or missing fields that
prevent association. Do not proceed to registration if the required input
fields cannot be consistently decoded.

### 2. Resolve frame, time, and pose semantics

Use the record evidence and producer/runtime configuration (where available)
to establish:

1. LiDAR coordinate frame and whether the point coordinates are raw sensor
   values or have already been transformed.
2. Meaning, unit, and clock domain of point `timestamp`,
   `measurement_time`, `Header.timestamp_sec`, sensor timestamp fields, and
   each candidate pose timestamp.
3. Parent/child direction and physical applicability of every transform
   needed to place scans in a common frame.
4. Producer and semantics of `apollo.localization.Gps.localization`, its
   pose frame/origin, reset/session behavior, and whether it is suitable only
   as an external map-relative reference or has any independent provenance.
5. `CorrectedImu` axes, units, frame, and relation to the recorded scans if an
   IMU stream is proposed for any later experiment.

Do not resolve ambiguity by assuming identity transforms, treating an absent
sampled frame ID as known, or assigning UTC/seconds/meters based on
plausibility alone.

**Gate:** document source evidence for every transform and time conversion
used by registration. If pose or scan placement remains unresolved, stop the
real-data MapMatcher phase; retain only schema/data-audit results.

### 3. Select a map and initialization strategy

Only after phase 2 passes, select one reproducible strategy:

- **Preferred, if pose placement is validated:** use a temporally separated
  subset of scans to construct a fixed local map in the validated pose frame;
  reserve distinct scans for queries. Record the exact segment, scan ranges,
  map-build parameters, and the pose source used for map placement.
- **If the pose is not a valid map placement source:** do not assemble a
  multi-scan map by silently integrating it. A pairwise/scan-derived
  exploratory test may be proposed separately, but must be labeled
  self-consistency evidence and must not be presented as external accuracy.
- Use the existing MapMatcher contract (`SetMap`/`Match`) and its current
  private candidate boundary. Keep conversion and external-data handling in
  a test-only adapter; do not add Apollo or Cyber dependencies to library
  targets.
- Declare the query initialization source and perturbation policy before
  running. Do not use the query's own target scan in its map or tune
  initialization against the resulting answer.

**Gate:** every query has an auditable, non-query-derived map and an explicit
initial pose source in the same declared frame. If this cannot be satisfied,
stop without interpreting matcher convergence as a successful validation.

### 4. Run the candidate MapMatcher experiment

Extend the existing `//:localization_map_tests` suite or add a separate
package-local test target only after implementation review. The real-data
adapter should:

- Read selected external data without changing the record.
- Use fixed map/query scan selections, configuration, filtering, and initial
  poses; keep derived PCD/cloud artifacts outside the repository.
- Report match status/reason, overlap, fitness, observable information rank,
  full-pose flag, and correction from the declared initial pose.
- Exercise valid and deliberately invalid/low-overlap inputs. Existing
  synthetic known-transform tests remain the source of known-truth matcher
  accuracy and repeatability checks.
- Preserve run identity, parameters, scan ranges, counts, and failure cases;
  avoid logging raw point clouds.

Declare real-data acceptance thresholds before the run, based on the
experiment's input and intended use. Do not reuse synthetic known-transform
tolerances as real-world accuracy thresholds. A successful optimizer status,
high overlap, or small residual alone is not proof of correct registration.

The existing controlled software gate can be run independently:

```bash
env -u LD_LIBRARY_PATH bazel test \
  //:localization_map_tests --test_output=errors
```

Run it from the standalone `pkgs/openloc` package in the managed test
container. Clear inherited `LD_LIBRARY_PATH` as shown.

**Gate:** fixed real-data cases reproduce status and diagnostics, declared
failure cases are rejected, and results are reported as candidate-algorithm
behavior/relative consistency only. No production promotion follows from
this phase.

### 5. Decide whether online estimator replay is possible

Do not replay the public online estimator using
`/apollo/sensor/gnss/odometry` as its ODOM input solely because of the topic
name. First identify a continuous, locally framed ODOM source with a
measurement-time, frame, epoch/reset, and quality contract that satisfies
OpenLoc's input contract.

If no such ODOM source exists in these records, mark online estimator replay
**blocked for this dataset**. The record can still support a conditional
private MapMatcher experiment after earlier gates pass. Do not use absolute
GNSS pose to bridge the missing local-motion input.

**Gate:** either a verified ODOM stream and adapter contract are documented,
or the limitation is recorded and no online replay is claimed.

### 6. Report evidence and limits

Report each stage as **pass**, **fail**, **blocked**, or **inconclusive**.
Include record/segment identity, topic and message counts, inspected time
range, source/config revision, map/query split, initialization, matcher
configuration, metrics, exact command, and limitations.

Keep these claims separate:

- Schema and message readability.
- Data-quality and time/frame-contract audit.
- Candidate MapMatcher behavior on real scans.
- Online local-ODOM estimator behavior.
- Independent accuracy or calibrated-trust evidence.

With the currently identified dataset, the last two claims are not established.
Do not call same-record map/pose agreement accuracy or ground truth.

## Current starting point

- Embedded schemas and bounded message samples are documented in
  `/mnt/synology/基础版本/slam_data/apollo/SCHEMA.md`.
- Point-cloud messages are available in both record segments; no standalone
  global map file has been inventoried.
- The pose topic is typed `apollo.localization.Gps`; its ODOM semantics are
  unverified.
- One `/tf_static` message exists in segment 1 and none in segment 2.
- The controlled synthetic MapMatcher gate passes, but this is independent of
  record decoding and does not qualify real-data registration.
- The first actionable next step is phase 1's full bounded input audit, not
  an OpenLoc code change or estimator replay.

## Stop conditions

Stop before registration on unresolved measurement-time association, unknown
scan frame/extrinsic, invalid or inconsistent point data, unavailable
defensible initialization, or a map/query split that leaks query scans into
the reference map. Ask before full-record materialization that produces large
derived clouds, changes source data, or requires new shared runtime
dependencies.
