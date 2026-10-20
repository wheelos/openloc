# Historical Map-Infrastructure Review Note

**Review date:** 2026-10-06  
**Status:** historical snapshot, not a current repository status report.

This note replaces the former implementation review with the findings that
remain useful under Openloc's current architecture. The old review described
map-infra at that date; it did not describe the current `pkgs/openloc`
implementation.

## Durable conclusions

- Offline map creation, optimization, product qualification, and rendering
  are distinct from online vehicle localization. `map-infra` remains the
  offline-map owner; Openloc can consume a versioned map contract without
  owning map production.
- An offline batch optimizer or map-product quality gate is not evidence of
  online localization, runtime trust assessment, or vehicle safety.
- Online inputs need explicit time, frame, calibration, identity, and quality
  contracts, and observation decisions need evaluable diagnostics.
- Runtime middleware, message conversion, lifecycle, publishing, and vehicle
  actuation belong in separately reviewed host integrations.
- Candidate estimator code must not become a production default solely
  because it exists or was copied.

The former review's repository inventory, test counts, phase estimates,
ROS-version assumptions, and recommendations were tied to its source checkout
and are intentionally not repeated as current facts. See
[`../../../architecture.md`](../../../architecture.md) for current ownership
and [`../../../roadmap.md`](../../../roadmap.md) for current status and
evidence.
