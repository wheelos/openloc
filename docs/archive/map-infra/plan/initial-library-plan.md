# Initial Online-Library Plan (Historical)

**Status:** superseded planning snapshot; not an active implementation plan.

Current functional priorities are in
[`../../../principles.md`](../../../principles.md), current library contracts
are in [`../../../architecture.md`](../../../architecture.md), and the only
active milestone/evidence tracker is
[`../../../roadmap.md`](../../../roadmap.md).

## What remains relevant

- Keep the online localization library independent of Cyber, ROS, Apollo
  message schemas, and vehicle lifecycle.
- Keep offline mapping and online localization as separate responsibilities;
  use an explicit versioned map interface where they meet.
- Validate causal timestamps, frames, extrinsics, uncertainty, source quality,
  and observation decisions.
- Gate algorithm promotion on focused tests and comparable evidence; preserve
  failed and inconclusive results.
- Keep local ODOM continuous and communicate global-reference resets
  explicitly to consumers.

## Superseded assumptions

- The proposed `online_localization/` repository/module layout was replaced by
  the package layout documented in `architecture.md`.
- The source proposal's sensor stack, GTSAM backend details, supervisor level
  names, numeric thresholds, schedule estimates, and field metrics are not
  normative Openloc requirements.
- Continuous centimeter-level absolute positioning is not the product goal;
  relative localization for drivable-area use is primary.
- Relocalization and self-recovery are required capabilities, not a later
  optional phase. Trust reporting and degradation are user-visible behavior.
- Host adapters remain outside the library; a runtime integration is not
  implied by the existence of a library API.

The previous plan's implementation status and commands are historical and
must not be used to claim current validation. Consult the active roadmap.
