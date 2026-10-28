# Openloc documentation

| Document | Purpose |
| --- | --- |
| [`principles.md`](principles.md) | Functional outcomes and product priorities |
| [`architecture.md`](architecture.md) | Library boundaries, component ownership, and technical contracts |
| [`input-contract.md`](input-contract.md) | Sensor input, provenance, and unresolved 3DMakerPro/Mid-360 semantics |
| [`roadmap.md`](roadmap.md) | Current status, evidence, blockers, and promotion gates |
| [`baseline-20261028.md`](baseline-20261028.md) | Frozen diagnostic functional baseline, goal, 14-scenario field correspondence, performance limits and commit-preparation checks |
| [Indoor/outdoor delivery and scenario checklist](../../../ad_agents/context/modules/localization/knowledge/openloc-indoor-outdoor-delivery-plan.md) | Algorithm/diagnostic/evaluation boundaries, known markers, partial-graph continuity, executable scenarios and remaining field gates |
| [Global integration and interface convergence plan](../../../ad_agents/context/modules/localization/knowledge/openloc-global-integration-plan.md) | Principle review, single-owner decision, responsibility/deduplication boundaries, and staged software/data qualification |
| [`plan/3dmakerpro-data-smoke-test.md`](plan/3dmakerpro-data-smoke-test.md) | Proposed external-data smoke and staged integration-test plan |
| [`plan/native-lio-error-model-inventory.md`](plan/native-lio-error-model-inventory.md) | Native LIO error sources, modeled dependencies, unverified assumptions, and graph trust ceiling |
| [`plan/online-localization-v1-implementation.md`](plan/online-localization-v1-implementation.md) | Phased LiDAR odometry + GNSS global-alignment implementation plan; no prior map required |
| [`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md) | Single-dataset LiDAR ODOM + GNSS validation plan, evidence, and admission gates |
| [`plan/boreas-openloc-baseline.md`](plan/boreas-openloc-baseline.md) | Reproducible cross-record Boreas scan-to-map diagnostic baseline |
| [Map-assisted localization design and validation plan](../../../ad_agents/context/modules/localization/knowledge/map-assisted-localization-validation-plan.md) | Optional scan-to-map path, map artifact assumptions, and separate Mode A/Mode B validation |
| [`plan/apollo-cyber-record-validation.md`](plan/apollo-cyber-record-validation.md) | Historical schema/validation evidence for an earlier Apollo Cyber Record dataset |
| [`source-provenance.md`](source-provenance.md) | Migrated source origins, adopted ideas, and exclusions |
| [`archive/map-infra/`](archive/map-infra/) | Historical source documents; not current implementation guidance |

For feature intent, start with `principles.md`; for implementation boundaries,
read `architecture.md`; before making implementation or validation claims,
check `roadmap.md`. Historical archive material is retained for provenance
and does not override current source, tests, or current package guidance.
