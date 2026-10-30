---
name: factor-graph-development
description: Develop and debug localization factor graphs through early scenario contracts, known-answer factor tests, controlled source ablations, fair window comparisons and evidence-bounded evaluation.
---

# Factor graph development and validation

## Scope and prerequisites

Use for LIO/global-source graph modeling, integration, unexpected fusion
behavior and scenario verification. These are parallel, iterative activities,
not six mandatory serial gates or a universal industry-standard process.
For a small defect, run only the steps needed to establish and verify its cause.

Read the package [principles](../../../AGENTS.md),
[functional goals](../../../docs/principles.md),
[model boundaries](../../../docs/architecture.md), relevant source/tests and
existing evidence. Select executable checks through
[bazel-validation](../bazel-validation/SKILL.md) and the package
[README](../../../README.md); do not invent commands or numeric budgets.
Preserve unrelated work and frozen experiments.

## Procedure

### 1. Start two tracks with a small contract

Record requirements and applicable normal/degraded behavior before tuning.
Use four initial scenarios where relevant: normal operation, GNSS loss and
recovery, abnormal map matching, and GNSS/map disagreement. Specify source
availability, observed directions, output validity/trust, ODOM isolation and
recovery expectations. Derive accuracy, jump, duration and runtime budgets from
requirements; missing budgets remain unspecified, not invented pass criteria.

In parallel, define state variables, factor meanings, gauge/initialization,
frames, transforms, units, event/receive time, lever arms and noise assumptions.
Inventory shared point clouds, seed estimates, map construction and historical
information. Shared inputs can correlate failures without making simultaneous
failure inevitable.

### 2. Establish factor and small-graph correctness

Use simple synthetic trajectories and independently computed known answers:

- Ideal observations give the expected residual, including residual sign and
  response to known translation/rotation/time/lever perturbations.
- Analytic and numerical Jacobians agree under the actual manifold convention.
- Covariance units, frame transformations and projected observed directions
  are correct; missing directions are not filled with invented information.
- Gauge freedoms and degeneracy behave as designed with and without anchors.
- Reused-source and temporal cross terms are tested where claimed by the model.

Keep approximation assumptions explicit. A simpler declared model can establish
functional/numerical behavior without proving native-model fidelity or
calibration. Do not bypass an existing production qualification gate.

### 3. Freeze comparable data and run source ablations

Use LIO, LIO+GNSS, LIO+scan-to-map, then all sources; include marker arms only
when applicable. Keep backend, nominal local inputs, source selection, anchor,
configuration and scoring comparable. Name common absolute anchors explicitly:
an anchored LIO arm is not absolute-free cold start. Relative LIO alone does
not guarantee long-term absolute position.

For source interventions distinguish frozen frontend measurements from a full
pipeline rerun whose matcher seeds may change. Pin code/configuration/data,
record actual admission and source timestamps, and keep tuning separate from
acceptance data.

Use the four scenarios by four arms as a compact diagnostic grid, not sixteen
mandatory accuracy improvements. Mark each cell applicable, control,
uncovered or not run. Natural field events and deliberately injected faults
must be labeled separately; a recorded source outage does not prove a
particular physical environment.

### 4. Diagnose evidence, not scores alone

Compare repeated applicable trials, key trajectory metrics and failure
segments against credible references where available. An added factor may
worsen one realization because of noise, weighting, nonlinearity or local
optima; robust estimation offers no universal guarantee of being no worse
than deleting a factor.

Without truth, inspect pre-gate innovations, accepted/rejected observations,
post-fit residuals, source-wise costs/weights, trajectory differences and
their relation to time, position and motion. A correlated held-out source is
still a weak reference; disagreement does not establish which source is
physically correct. Cost magnitude alone does not establish information
dominance: account for units, dimension, factor count, whitening and Jacobians.

For persistent material degradation or a contract violation, trace the first
wrong boundary: semantics, coordinates, timing, data quality, correlation,
modeling, implementation, then justified tuning. Separate confirmed causes
from hypotheses. Do not modify a physical calibration or covariance merely
to improve a weak-reference score.

### 5. Isolate window and marginalization effects

Use identical available-observation prefixes, initialization, factors and
noise model to compare window lengths and no-eviction controls. Check actual
admissions, convergence, linearization and robust-weight policies; merely
reusing a bag or backend name does not make objectives identical. Examine
anomalous measurements before and after marginalization when relevant.

Separately label full-data offline batch smoothing that can use future
observations. State whether scores use online-time outputs, final endpoints
or retrospectively smoothed history. Batch is an internal comparator, not
truth or an absolute accuracy upper bound. Agreement can coexist with a
wrong model; disagreement alone does not identify a marginalization bug.

### 6. Evaluate consistency and extend scenario regressions

Use NEES only with credible truth, defined error coordinates, gauge treatment
and meaningful covariance. Synthetic NEES validates only the simulated model.
NIS uses the pre-update innovation and its covariance, not simply the squared
post-fit whitened factor residual. Include rejected/pre-gate samples and
account for temporal/source correlation, robust selection and non-Gaussian
errors when interpreting statistical tests.

Do not tune residuals to one, inflate covariance for a pass, or infer calibrated
uncertainty from small residuals. Retain newly reproduced failures as focused
regressions and expand scenarios throughout development, not only after
consistency tuning.

## Acceptance and reporting

Report a compact scenario/arm evidence table with input identity,
availability, applicable expectation, synthetic result, field correspondence
and remaining gaps. Classify conclusions independently:

| Claim | Required evidence boundary |
|---|---|
| Numerical correctness | Known answers, derivatives and actual solver residual/convergence |
| Functional/degraded behavior | Observable-direction, validity/trust, isolation and recovery contracts |
| Physical accuracy or fusion benefit | Credible reference and repeatable applicable comparisons |
| Statistical consistency | Appropriate innovation/error model, covariance and test assumptions |
| Runtime readiness | Actual timing, rate, queue/overload and deadline measurements |

Use passed, failed, unresolved, uncovered and not run accurately. No truth is
not a functional failure; zero process exit is not accuracy acceptance.
An expected loss of precision during information loss is not itself a bug.
Do not claim production or native-model acceptance from an approximate replay.

## Failure handling

Stop the failing action at its first actionable error, preserve inputs and
actual decisions, and choose the smallest discriminating check. Missing truth
or calibration blocks the corresponding claim, not every functional test.
An unresolved weak-reference conflict warrants bounded investigation, not
forced tuning or indefinite matrix expansion. A confirmed implementation
capacity limit remains a separate limitation; a successful approximation
does not repair it.
