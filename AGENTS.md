# Agent Guide

- Read [`README.md`](README.md) and the relevant source and tests before
  changing code. Public headers are under `include/openloc/`; implementation
  and private candidate headers are under `src/`. Read
  [`docs/principles.md`](docs/principles.md) for functional goals,
  [`docs/architecture.md`](docs/architecture.md) for boundaries, and
  [`docs/roadmap.md`](docs/roadmap.md) for status and evidence.
- Keep changes focused, preserve existing work, and do not move host-runtime
  responsibilities into this middleware-independent library.
- Use the package's verified test targets in [`README.md`](README.md); test
  results do not establish vehicle accuracy or production readiness.
- Keep temporary investigations under [`.agents/notes/`](.agents/notes/).
  Workspace-level durable knowledge belongs to
  [`ad_agents/context/`](../../ad_agents/context/README.md).
- See the [skill index](.agents/skills/README.md) and
  [knowledge index](.agents/knowledge/README.md).

## Fusion validation

- Start with factor availability, observed directions and failure/recovery
  scenarios; derive a test matrix before tuning. Keep algorithms, diagnostics
  and offline evaluation separate.
- Synthetic evidence covers only the declared models, injected faults and
  tested parameter values/ranges, not all environments or real sensor accuracy.
  Report uncovered combinations and never-triggered conditions explicitly.
- Map field evidence to the same matrix using actual factor decisions, times
  and front-end diagnostics, not filenames or presumed indoor/outdoor labels.
  Distinguish matched, partial, missing and contradictory evidence.
- A field mismatch is not automatically bad data. Separate data defects,
  violated/model-unsupported assumptions, implementation defects and unresolved
  causes; attribute only with causal evidence.
- Losing two or all absolute factors can normally reduce global accuracy or
  availability. Accept degradation when it follows the declared contract:
  no invented information/trust, valid local ODOM remains isolated/continuous,
  stale/unavailable output is explicit, and recovery follows its policy.
  Neither degradation nor a large residual alone proves a functional defect;
  silent overconfidence, broken ODOM or a violated recovery contract does.
- Use the [delivery plan and validation matrix](../../ad_agents/context/modules/localization/knowledge/openloc-indoor-outdoor-delivery-plan.md)
  for current coverage, field correspondence and remaining checks. Do not
  require every degraded case to retain nominal accuracy or hide a defect by
  relaxing gates.
