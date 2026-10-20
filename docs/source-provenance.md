# Openloc Source Provenance

**Source:** `wheelos-service/map-infra`'s `apollo-lite` checkout, branch
`local`, commit `837da9f4c595630016a564c6c53aae8c6061870c`.

The online-localization source and reference design documents were copied
from `/home/wfh/01code/map-infra` branch `backend`, checked-out commit
`c4b280f573d88383fdb433159f5b3368c4ae428e`. Those module and design files
were untracked there at migration time; see
[`archive/map-infra/README.md`](archive/map-infra/README.md).

## Migrated from `modules/localization`

Pure algorithms from `core/` and `map/` were copied with their Apache-2.0
notices and tests. The algorithms and their private headers now live under
`src/localization/candidates/`, and tests under
`test/localization/candidates/`, grouped into `common/`, `local/`,
`global/`, `relative/`, and `map/`.

- Runtime DAGs, launch files, Apollo components, message/protobuf contracts,
  and install wiring were not copied. These belong in host adapters.

The algorithm split aligns with the library boundary: local and global
estimators have distinct state ownership, and the map and lane algorithms can
be invoked through ordinary C++ values. The imported code is not assumed to
implement every design requirement. In particular, it is not the specified
FAST-LIO2 runtime integration, and its own tests are not field qualification.
The openloc tests/build must pass before this source is treated as a usable
baseline here.

### Branch review: adopted contracts and exclusions

The `local` branch's explicit local-estimator ownership, timestamp-ordered
inputs, stopped-only reset, source-health separation, and bounded motion
history are represented by the private local candidate and its tests. Its
GNSS adapter's causal heading pairing, receiver-quality checks, antenna
lever-arm handling, and rejection cases are already represented in the
middleware-neutral online engine; Apollo message conversion is not. These
overlapping ideas did not justify copying another estimator or adapter.

The online input contract additionally requires an odometry
producer-session/reset generation and a map-origin/version/calibration
identity. Histories reject cross-epoch samples, and GNSS position/heading
observations reject mismatched map identity. Map identity is immutable for one
configured engine lifecycle. Map-file loading, map-switch requests, Cyber
callbacks, and `LocalizationAssessment` publishing remain host/runtime
responsibilities, not Openloc library features.

## Migrated: `modules/localization_health` policy

The health API, policy, and tests live in
`include/openloc/localization/health/`, `src/health/`, and
`proto/openloc/localization/health/`. The port preserves independent
output checks, capability derivation, reason aggregation, fail-fast/
recover-slow state transitions, directional-constraint validation and recovery,
and producer-session rollover handling.

The package owns the health protobuf value contract. It does not import Apollo
or WheelOS message schemas; topic fields were removed from the library config
because transports and channel names belong to host adapters. The Cyber
component, launch/DAG/configuration files, and publishing bridge were not
migrated. Compatibility between this library schema and Apollo runtime
messages must be implemented and tested in a separate adapter.

The migrated tests are software evidence only; they do not establish
protection-level calibration or vehicle qualification.

## Retirement of the Apollo `localization` branch

The branch's only commit beyond the shared base was
`2093defb2f8fcc3e52d4ee6afee4b2d148bffe36` (`feat: add localization health
module`). It added the health runtime and policy, but also changed the legacy
RTK implementation: bounded/null-safe input handling, synchronized result
access, missing-status handling, and an Apollo `LocalizationAssessment`
publisher. The assessment publisher and its channel wiring are host-runtime
integration, not part of the middleware-neutral library; they have not been
ported to openloc. In particular, openloc health policy does not by itself
provide a producer-side adapter for Apollo RTK.

The health policy in openloc is based on the evolved `local` branch
implementation. Its software tests cover the original branch's core health
scenarios and add producer-session and directional-constraint cases, but its
protobuf contract intentionally differs from Apollo's and does not guarantee
wire compatibility.

The unique branch commit is preserved in this checkout as the annotated tag
`archive/apollo-localization-2093defb`; the tag has not been pushed to the
remote. Before deleting the remote branch, publish and verify this archive tag
or choose another durable archive. The branch may be retired from active
development if the Apollo-specific RTK runtime changes are explicitly out of
scope; deleting the branch does not mean those runtime changes were migrated
or that an Apollo adapter exists. Keep the archive tag until any required host
integration has been separately evaluated.

## Overlapping global alignment implementations

`online_localization` from `map-infra` and Apollo `core/global_alignment`
both implement fixed-lag `map -> odom` estimation with different public types
and receiver/observation policies. Both are retained as source baselines for
now, not as a combined runtime. Unification requires a reviewed common
measurement/result contract and a controlled comparison with identical input
timestamps, frames, initial conditions, and scoring; neither synthetic
agreement nor build success chooses a production backend.
