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

## Bottom-up fusion development

- Use [factor-graph-development](.agents/skills/factor-graph-development/SKILL.md)
  for modeling, debugging, ablation and scenario validation. It is an iterative
  engineering method, not a universal industry standard or a strict pipeline.
- Define a small set of requirements, representative scenarios, observable
  directions and normal/degraded failure contracts at the start. Develop
  factors and scenario evidence together; expand coverage as issues emerge.
- Verify residuals, manifold Jacobians, frames, units, time, lever arms and
  source dependencies with known answers before trusting aggregate scores.
  Separate native LIO frontend correctness from graph motion-model correctness.
- Freeze comparable local baselines and add sources incrementally. A LIO
  baseline needs an explicit gauge/anchor and has no long-term absolute
  accuracy guarantee. Diagnostic-owner results do not qualify target FixedLag.
- Adding a factor need not improve every sample or run, even with robustness.
  Investigate persistent material degradation across repeated applicable
  tests against credible references; a single weak-reference discrepancy is
  a conflict signal, not proof of a bug or physical accuracy loss.
- Compare sliding windows with same-prefix, same-factor no-marginalization
  controls separately from full-data offline smoothing using future data.
  Batch is neither truth nor an absolute accuracy upper bound; differences
  alone do not prove a marginalization defect.
- Distinguish pre-update innovation NIS from post-fit whitened residuals.
  NEES requires credible truth and interpretable covariance. Account for
  correlation, robust weighting, non-Gaussian errors and sample selection;
  neither small residuals nor a target statistic certifies calibration.
- Identify shared information without assuming all sources necessarily fail
  together. Explicit approximate models may support functional experiments;
  do not call them calibrated/exact or silently relax production admission.
  Full raw-error fidelity and independent truth are not prerequisites for
  every engineering or functional check.
- Report numerical correctness, functional/degradation behavior, physical
  accuracy, statistical consistency and runtime separately. Do not repair
  unproven physical errors by fitting a correlated weak reference, and do not
  block all functional validation merely because absolute accuracy is unknown.

## Fusion scenario validation

- Start with factor availability, observed directions and failure/recovery
  scenarios; define required coverage early without bypassing the bottom-up
  gates above. Keep algorithms, diagnostics
  and offline evaluation separate. Judge remaining information by observable
  directions, not by the number of failed factors; distinguish local-motion
  failure from loss of absolute references.
- Synthetic evidence covers only the declared models, injected faults and
  tested parameter values/ranges, not all environments or real sensor accuracy.
  Report uncovered combinations and never-triggered conditions explicitly.
- Map field evidence to the same matrix using actual factor decisions, times
  and front-end diagnostics, not filenames or presumed indoor/outdoor labels.
  Distinguish matched, partial, missing and contradictory evidence.
- A field mismatch is not automatically bad data. Separate data defects,
  violated/model-unsupported assumptions, implementation defects and unresolved
  causes; attribute only with causal evidence. Compare the same backend,
  configuration and admission boundary before calling field and synthetic
  results contradictory. Missing scene evidence is uncovered, not failed.
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
