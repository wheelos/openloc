# Temporary agent notes

Use this directory only for temporary investigations that are not yet
validated reusable knowledge. Its contents are ignored by Git except for this
README. Do not store credentials, personal data, or raw sensitive logs here.
Promote verified reusable conclusions to their authoritative documentation,
then remove obsolete notes.

## OpenLoc validation entry points

- **当前LIO局部优化（2026-10-28）：**
  [阶段瓶颈、21次实跑与负面结果](lio_performance_20261028_v1/REPORT.md) /
  [逐次指标及精确对照](lio_performance_20261028_v1/metrics.json)。
  缓存不可变世界点体素索引，城投非限速LIO累计中位数1.605→1.340s，
  P99中位数9.968→8.570ms；三条录制算法结果保持一致。
  总CPU未显示可靠收益，1x CPU与ODOM超期反而增加；
  地图耗时增加尚未定因，不能把局部收益当成整体加速或实时通过。
- **当前本地输入路径优化（2026-10-28）：**
  [分段剖析与串行前后对照](local_performance_20261028_v1/REPORT.md) /
  [逐次指标](local_performance_20261028_v1/metrics.json)。
  优化的是回放适配器CSV分列，不是定位数学模型；启动后点云解码累计耗时
  三次中位数1.031→0.833s，算法输出保持不变。
  ODOM超期和共享调度波动仍保留，不将100Hz快照timer解释为实时达标。
- **当前地图匹配优化（2026-10-28）：**
  [剖析与前后对照](map_optimization_20261028_v1/REPORT.md) /
  [18次新实跑指标](map_optimization_20261028_v1/metrics.json)。
  单线程批量精确搜索保留三条真实录制的算法输出；城投非限速耗时中位数
  12.379→11.259s，地图P99中位数92.869→78.938ms。
  ODOM超期仍波动，100Hz快照timer不等于实时资格。
- **当前真实评测与性能（2026-10-28）：**
  [汇总报告](fleet_results_20261028_v1/REPORT.md) /
  [同源GNSS一致性与场景可视化](fleet_real_evaluation_20261028_v1/index.html) /
  [新实跑性能和频率](fleet_runtime_20261028_v1/index.html)。
  33个共同历元的map/双源水平差P95分别0.323/0.412m，不是独立精度。
  1x异步本地快照timer约99.733Hz，新测量96.059Hz，但155/4071条ODOM超过
  10ms探针期限，仍不具实时资格；全局真实发布频率和目标FixedLag仍未验收。
- **当前统一入口（2026-10-28）：**
  [按需求场景对照的报告](validation_20261028/REPORT.md) /
  [14项场景对照表](validation_20261028/index.html) /
  [结构化矩阵](validation_20261028/scenario_matrix.json)。
  每项列出应有行为、构造数据结果、真实数据结果、未覆盖条件和补充动作；
  14项合成功能通过不等于真实FixedLag验收。
- **当前真实数据证据：**
  [17条独立录制诊断面板](real_breadth_20261028_v1/index.html) /
  [逐臂测试报告](real_breadth_20261028_v1/REPORT.md)。
  首30s广度加城投180s及重复，共36臂，34完成诊断、2原生启动未建立；
  15/17首窗口启动，4条地图修正、3条地图+GNSS修正。
- **存储清理：**
  [清理账本](validation_20261028/cleanup_result.json)保留逐文件hash和去重关系。
  字节相同的冻结CSV/PCD等可共享硬链接，路径与独有失败证据保留；
  后续实验必须新建目录，禁止覆盖冻结文件，编辑副本前先复制到新文件。
  下列旧日期入口均为历史证据，不应作为当前版本的通过列表。

- **Factor-scenario validation principles (2026-10-28):**
  [14-scene matrix](factor_scenario_principles_20261028_v2/index.html) adds
  known ODOM drift during joint absolute-source loss: 0.24 m expected error
  growth is not a functional failure; stale trust, ODOM isolation and
  source reentry are asserted. Per-scene observed parameter/interval coverage
  and explicit uncovered combinations separate synthetic evidence from field
  correspondence. No real-data rerun or covariance qualification is implied.
- **Graph numerical unblock (2026-10-28):**
  [Current graph/scene/raw-prerequisite dashboard](goal_graph_20261028_v1/validation/index.html)
  records 640/640 short-lag and 640/640 no-eviction trials, original
  failure controls, retained-relinearization counts, the 13-scene matrix and
  real native model admission separately. Passing declared synthetic models
  does not qualify real covariance or product readiness.
- **Indoor/outdoor framework follow-up (2026-10-27):**
  [Delivery/diagnostics overview](indoor_outdoor_delivery_20261027_v1/index.html)
  verifies frozen artifacts against the current source/runtime and records
  218 C++ regression cases plus three maintained evaluator cases.
  [13 actual-graph synthetic scenarios](indoor_outdoor_scenarios_20261027_v5/index.html)
  cover no-GNSS map/marker acquisition, complementary/joint degradation,
  source loss/reentry, faults, delayed input and the unanchored cold-start gap.
  [Current-code raw 180 s arms](indoor_outdoor_raw_20261027_v1/summary.json)
  retain 1,702 local scan updates and 407 map + 126 GNSS corrections with
  exact ODOM isolation, using the diagnostic owner. The new
  [canonical delivery/validation list](../../../../ad_agents/context/modules/localization/knowledge/openloc-indoor-outdoor-delivery-plan.md)
  distinguishes selected software completion from missing field/product gates.
- **Long-window goal follow-up (2026-10-27):**
  [trajectory, local support, source-gap/reentry and uncertainty dashboard](goal_continuity_20261027_v1/index.html)
  and [verified metrics](goal_continuity_20261027_v1/goal_analysis.json).
  Geometry-rank separation plus a default-off diagnostic conditional gain
  restores 1,702/1,800 local scans through 180 s; all 1,500 scans after 30 s
  are accepted. Seven absolute arms preserve exact ODOM isolation.
  Map 30/60 s reentry commits, but first global position responses reach
  6.65/22.15 m; 120 s reentry still fails. Four captures are represented,
  only ChengTou has an extended window. No calibrated covariance, independent
  accuracy, real FixedLag or runtime qualification is claimed.
- **当前验证范围与结论（2026-10-27）：** 继续使用
  `/mnt/synology/3dmakerpro` 验证融合功能，NovAtel 待完整数据及标定准备好后再测；
  保留其接入代码，不将其数据准备作为当前迭代阻塞项，不以精度调参为主线。
  当前源码重跑城投首 30 s 四臂，每臂 6,001 IMU / 300 扫描：
  GNSS-only 实际修正 10 次，Map-only 41 次，组合臂 105 次地图 + 20 次 GNSS。
  四臂 ODOM 完全一致（仅忽略 `imu_ms`），扫描决策完全一致（仅忽略
  `processing_ms`）。Map-only 后段平面 ICP 对应不足，GNSS-only 后段仍偏离；
  组合臂在 9 个共同晚段时刻的同源 GNSS 位置差中位数为 1.395 m。
  这些是 `DiagnosticGlobalAlignment` 功能及同源一致性证据，不是独立精度、
  目标图后端、车载双天线或生产资格证明；离线地图 seed 含未来信息，
  协方差未标定。下一步继续检查真实 source correction、丢失/恢复和 ODOM
  隔离，不放宽门限或切换正式默认定位源。
  [详细结论与后续范围](current_fusion_runtime_20261027_v1/README.md)；
  [本次轨迹与诊断报告](current_fusion_runtime_20261027_v1/diagnostics.html)。
- **创新日志与故障注入补测（2026-10-27）：**
  [新四臂重放、输入/runtime pins 与逐臂 NIS/自相关](current_fusion_runtime_20261027_v3_innovation/p0_p2_summary.json)，
  [GNSS 跳变、+50 ms 时间偏移与共同坐标偏移](current_fusion_runtime_20261027_v4_fault_injection/report.json)，
  [IMU 加速度偏置与角速度突变代理](current_fusion_runtime_20261027_v5_imu_fault_injection/report.json)。
  新日志可复算诊断链 NIS 并保留不可用原因；ODOM/scan 隔离保持。
  常量坐标偏移未被 GNSS-only 创新识别，且 GNSS 跳变样本无可用先验/NIS。
  这些仍是 `DiagnosticGlobalAlignment` 的同源功能诊断；目标
  `FixedLagGlobalAlignment` 尚未在该真实输入上完成双链回放，IMU 扰动也不是
  直接 LIO 输出或跨源共同故障注入。不得据此宣称准确度或协方差已校准。
  [全局 owner ODOM 漂移/阶跃及 GNSS+ODOM 同向坐标偏置](current_fusion_runtime_20261027_v6_global_odom_faults/audit.json)
  补充验证了注入日志、原始输入/runtime pins、ODOM/scan 隔离及重算 NIS。
  阶跃显著减少地图匹配接受数；共同平移在 GNSS-only 自由对齐中不可观测。
  该注入位于全局 owner 输入边界，并非修改本地 LIO 输出；仍无独立真值，
  不构成 `FixedLagGlobalAlignment` 双链验收或精度/协方差校准。
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
