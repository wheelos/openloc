# Historical Localization Design Note

**Status:** historical input, rewritten to reflect current Openloc direction.
This note records the origin and disposition of the earlier map-infra
localization proposal; it is not a current requirements document.

Use [`../../../principles.md`](../../../principles.md) for functional
requirements, [`../../../architecture.md`](../../../architecture.md) for
current boundaries and contracts, and [`../../../roadmap.md`](../../../roadmap.md) for implementation
status and gates.

## Original proposal

The source design proposed a vehicle localization system combining continuous
local odometry with GNSS and optional map, landmark, and lane observations. It
also proposed explicit trust/degradation reporting, observable decisions,
relocalization, and downstream reset handling. The sensor selection,
middleware messages, supervisor levels, thresholds, and milestone dates were
proposal details, not accepted Openloc contracts.

## Current disposition

- The product goal is reliable relative localization for identifying the
  drivable area. Continuous centimeter-level absolute accuracy is not a
  package-wide requirement.
- Trust assessment and explicit degradation are required behavior, not merely
  diagnostics or downstream policy.
- Relocalization and self-recovery are first-class capabilities. Recovery
  must be evidenced; absent sufficient evidence, output remains degraded or
  untrusted.
- Local estimation owns continuous `odom -> base_link`; global observations
  may estimate `map -> odom` but cannot reset or correct local ODOM.
- Sensor and map observations are admitted through explicit contracts. No
  particular sensor stack or estimator backend is implied as a product
  requirement.
- Vehicle-specific messages, middleware, lifecycle, TF transport, and
  actuation remain host responsibilities.

These decisions replace conflicting assumptions in the source proposal.
Implementation and evidence are tracked in the current roadmap, not here.
