# Temporary agent notes

Use this directory only for temporary investigations that are not yet
validated reusable knowledge. Its contents are ignored by Git except for this
README. Do not store credentials, personal data, or raw sensitive logs here.
Promote verified reusable conclusions to their authoritative documentation,
then remove obsolete notes.

## OpenLoc validation entry points

- **ChengTou native four-arm diagnostic (2026-10-25):**
  [retained inputs, commands, results and limits](chengtou_native_fusion_20261025_v1/README.md),
  with the [combined dashboard](chengtou_native_fusion_20261025_v1/diagnostics.html)
  and [equal-scale XY image](chengtou_native_fusion_20261025_v1/diagnostics_xy.png).
  The 30 s replay retains raw-native ODOM, GNSS-only, map-only and map+GNSS
  arms plus their source manifests, map inputs, runtime pins and reports.
  It demonstrates diagnostic corrections and exact local isolation, not
  independent accuracy or target-graph qualification. See the
  [canonical baseline](../../../../ad_agents/context/modules/localization/knowledge/openloc-three-source-dataset-baseline.md).
- **Current raw-native fusion ablation (2026-10-24):**
  [interactive comparison](native_fusion_ablation_20261024_v1/comparison.html)
  overlays single-anchor local-only, map-only, map+GNSS and fixed-GGA
  antenna positions in a common frame; its paired epoch discrepancy plots
  and values are in
  [consistency analysis](native_fusion_ablation_20261024_v1/consistency_analysis.json).
  The underlying runs and provenance are in
  [report.json](native_fusion_ablation_20261024_v1/report.json), with
  [post-hoc source diagnostics](native_fusion_diagnostics_20261024_v1/report.json)
  and the [source-time event dashboard](native_fusion_dashboard_20261024_v1/dashboard.html)
  with its [machine-readable metrics](native_fusion_dashboard_20261024_v1/report.json).
  These are diagnostic comparisons, not independent accuracy or live 100 Hz
  acceptance. See the
  [canonical baseline and metric limits](../../../../ad_agents/context/modules/localization/knowledge/openloc-three-source-dataset-baseline.md).
- **Historical Boreas scan-to-map component baseline:** start at
  [the canonical procedure and results](../../docs/plan/boreas-openloc-baseline.md).
  Raw attempts, including failed/false-positive controls, remain under
  `boreas_openloc_baseline/`; preserve those because they establish failure
  modes. The unused incomplete `pilot_v3/prepared/` output was removed after
  its `incomplete_not_used` status and lack of a manifest were verified;
  `attempts.json` retains that provenance. `prepared_reverse/` is the valid
  preparation for the recorded reverse-split run.

The fusion comparison is regenerable from its frozen runs with
`python3 .agents/notes/native_fusion_ablation_20261024_v1/analyze_consistency.py`
from the `pkgs/openloc` package root. It verifies the pinned evidence before
scoring and rewrites `analysis_verification.json` with the derived-artifact
hashes. Generate the separate read-only time-series dashboard with
`python3 .agents/notes/native_fusion_dashboard_20261024_v1/generate_dashboard.py`;
it verifies the frozen replay and prior post-hoc report hashes, then writes
only to its own directory. Do not change the replay evidence to alter a report.

## Curated and retained evidence

- `real_data_openloc_20261022/` was summarized in the canonical baseline's
  **Historical external-ODOM GNSS diagnostic** section and removed. It did
  not exercise the raw-native frontend or target graph; the useful outcomes
  and limitations are now durable.
- Keep the `native_*` raw-native runs referenced by `pkgs/openloc/README.md`,
  `pkgs/openloc/docs/roadmap.md` or `ad_agents/context/`; they contain distinct
  input/runtime pins and failure evidence. A failed acceptance is not, by
  itself, an invalid experiment.
- Keep Boreas, chassis/IMU and prepared-ODOM evidence while canonical plans
  cite their failure diagnostics or their source data is needed to reproduce
  comparisons. Remove an artifact only when its `incomplete_not_used` or
  exact-duplicate status is established and no retained run depends on it.
