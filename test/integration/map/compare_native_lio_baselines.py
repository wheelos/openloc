#!/usr/bin/env python3
"""Read-only, pinned native replay comparison; GNSS consistency, not accuracy."""

import argparse
from bisect import bisect_right
from collections import Counter
import csv
import hashlib
import html
import json
import math
from pathlib import Path
import statistics


PACKAGE = Path(__file__).resolve().parents[3]
TOLERANCE_NS = 10_000_000
COLORS = {"odom": "#555", "map": "#1870ba", "both": "#c54d24",
          "gnss_horizontal": "#8b43a8", "reference": "#167644"}


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def load_json(path):
    return json.loads(path.read_text())


def resolve_path(value):
    path = Path(value)
    prefix = "/apollo/pkgs/openloc/"
    if str(path).startswith(prefix):
        return PACKAGE / str(path)[len(prefix):]
    return path if path.is_absolute() else PACKAGE / path


def rotate(q, vector):
    x, y, z, w = q
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or abs(norm - 1) > 1e-5:
        raise ValueError("invalid pose quaternion")
    cross = (y * vector[2] - z * vector[1],
             z * vector[0] - x * vector[2],
             x * vector[1] - y * vector[0])
    twice = tuple(2 * v for v in cross)
    second = (y * twice[2] - z * twice[1],
              z * twice[0] - x * twice[2],
              x * twice[1] - y * twice[0])
    return tuple(vector[i] + w * twice[i] + second[i] for i in range(3))


def quaternion(row):
    return tuple(float(row["q" + axis]) for axis in "xyzw")


def antenna(row, lever, local=False):
    names = ("px", "py", "pz") if local else ("x", "y", "z")
    offset = rotate(quaternion(row), lever)
    return tuple(float(row[name]) + offset[i] for i, name in enumerate(names))


def horizontal_antenna(row, local, lever):
    offset = rotate(quaternion(local), lever)
    yaw = float(row["alignment_yaw"])
    return (float(row["x"]) + math.cos(yaw) * offset[0] - math.sin(yaw) * offset[1],
            float(row["y"]) + math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1])


def fixed_anchor(local, global_row):
    """One exact epoch anchor, not a trajectory or GNSS fit."""
    qg, ql = quaternion(global_row), quaternion(local)
    inverse = (-ql[0], -ql[1], -ql[2], ql[3])

    def apply(point):
        relative = tuple(point[i] - float(local["p" + axis])
                         for i, axis in enumerate("xyz"))
        mapped = rotate(qg, rotate(inverse, relative))
        return tuple(mapped[i] + float(global_row[axis])
                     for i, axis in enumerate("xyz"))
    return apply


def nearest_causal(epochs, epoch, tolerance=TOLERANCE_NS):
    index = bisect_right(epochs, epoch) - 1
    return epochs[index] if index >= 0 and epoch - epochs[index] <= tolerance else None


def select_revision(records, epoch, receive_ns, decisions):
    """Use only actually emitted, source-causal revisions before reference arrival."""
    usable = []
    for row in records:
        revision = int(row["correction_id"] or 0)
        if (row["alignment_initialized"] == "1"
                and int(row["timestamp_ns"]) == epoch
                and int(row["receive_ns"]) <= receive_ns
                and revision in decisions
                and decisions[revision] <= epoch):
            usable.append(row)
    return usable[-1] if usable else None


def distribution(values):
    values = sorted(values)
    if not values:
        return {"samples": 0, "median_m": None, "p95_m": None,
                "max_m": None, "rmse_m": None}
    position = .95 * (len(values) - 1)
    lo, hi = math.floor(position), math.ceil(position)
    return {"samples": len(values), "median_m": statistics.median(values),
            "p95_m": values[lo] + (values[hi] - values[lo]) * (position - lo),
            "max_m": max(values),
            "rmse_m": math.sqrt(statistics.mean(v * v for v in values))}


def write_csv(path, records):
    if not records:
        path.write_text("")
        return
    with path.open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def load_arm(root, name, pins):
    report = load_json(root / "report.json")
    entry = report["arms"][name]
    arm = resolve_path(entry["reused_from"]) if "reused_from" in entry else root / name
    for filename, expected in entry["artifacts"].items():
        path = arm / "replay" / filename
        if digest(path) != expected:
            raise ValueError("frozen artifact changed: " + str(path))
        pins[path] = expected
    for filename in ("command.json", "summary.json"):
        pins[arm / filename] = digest(arm / filename)
    replay = arm / "replay"
    records = {path.stem: read_csv(path) for path in replay.glob("*.csv")}
    if name == "odom_v2":
        records.setdefault("global_decisions", [])
        records.setdefault("global", [])
    records["_entry"] = entry
    return records


def event_ledger(arm, label):
    final = {(r["source"], r["sequence"]): r for r in arm.get("global_decisions", [])}
    result = []
    for event in arm.get("gnss", []):
        decision = final.get(("raw-gga-horizontal-diagnostic", event["sequence"]))
        result.append(dict(
            arm=label, sequence=event["sequence"], timestamp_ns=event["timestamp_ns"],
            receive_ns=event["receive_ns"], gate=event["gate_outcome"],
            gate_reason=event["gate_reason"], adapter=event["adapter_reason"],
            submission=event["owner_reason"],
            final_owner=(decision["message"] or
                         "retained_without_correction; owner_reason_unavailable")
            if decision else "not_submitted",
            actual_commit=decision["alignment_updated"] if decision else "0",
            correction_id=decision["correction_id"] if decision else ""))
    return result


def arm_summary(arm, start):
    commits = [r for r in arm["global_decisions"] if r["alignment_updated"] == "1"]
    if len({r["correction_id"] for r in commits}) != len(commits):
        raise ValueError("duplicate actual correction IDs")
    valid = [r for r in arm["odometry"] if r["valid"] == "1"]
    full = [r for r in arm["global"] if r["alignment_initialized"] == "1"]
    horizontal = [r for r in arm.get("horizontal", [])
                  if r["alignment_initialized"] == "1"]
    map_rows = arm.get("map", [])
    failures = Counter(r["detail"] or r["reason"] for r in map_rows if r["valid"] != "1")
    return dict(
        imu_rows=len(arm["odometry"]), valid_local_states=len(valid),
        scan_rows=len(arm["lidar"]),
        startup_delay_s=(int(valid[0]["timestamp_ns"]) - start) / 1e9,
        local_capability_reasons=dict(Counter(r["capability_reason"] for r in valid)),
        scan_reasons=dict(Counter(r["reason"] for r in arm["lidar"])),
        actual_commits=dict(Counter(r["source"] for r in commits)),
        distinct_correction_ids=len(commits),
        full_pose_initialized_rows=len(full),
        full_pose_fresh_rows=sum(r["full_pose_fresh"] == "1" for r in arm["global"]),
        horizontal_initialized_rows=len(horizontal),
        horizontal_fresh_rows=sum(r["horizontal_fresh"] == "1"
                                  for r in arm.get("horizontal", [])),
        first_full_pose_s=(int(full[0]["timestamp_ns"]) - start) / 1e9 if full else None,
        first_horizontal_s=(int(horizontal[0]["timestamp_ns"]) - start) / 1e9
        if horizontal else None,
        raw_local_frame_z_range_m=[
            min(float(r["pz"]) for r in valid), max(float(r["pz"]) for r in valid)],
        gravity_projection_status="blocked: no qualified fixed up transform into supplied map",
        map_attempts=len(map_rows),
        map_valid=sum(r["valid"] == "1" for r in map_rows),
        map_failure_details=dict(failures),
        wall_elapsed_s=arm["_entry"]["wall_elapsed_s"],
        reused_control="reused_from" in arm["_entry"],
        wall_time_is_online_latency=False)


def index_epochs(records):
    result = {}
    for row in records:
        result.setdefault(int(row["timestamp_ns"]), []).append(row)
    return result


def score(arms, events, lever, anchor):
    odom = {int(r["timestamp_ns"]): r for r in arms["odom"]["odometry"]
            if r["valid"] == "1"}
    epochs = sorted(odom)
    decision_epochs = {
        name: {int(r["correction_id"]): int(r["timestamp_ns"])
               for r in arm["global_decisions"] if r["alignment_updated"] == "1"}
        for name, arm in arms.items()}
    globals_by_epoch = {name: index_epochs(arm["global"]) for name, arm in arms.items()}
    horizontal = index_epochs(arms["gnss"].get("horizontal", []))
    paired, partial, horizontal_support = [], [], []
    for event in events:
        if event["adapter_reason"] != "ok" or event["quality"] != "4":
            continue
        source_ns, receive = int(event["timestamp_ns"]), int(event["receive_ns"])
        epoch = nearest_causal(epochs, source_ns)
        if epoch is None:
            continue
        points = {"odom": anchor(antenna(odom[epoch], lever, local=True))[:2]}
        revisions = {"odom": ""}
        for name in ("map", "both"):
            row = select_revision(globals_by_epoch[name].get(epoch, []), epoch,
                                  receive, decision_epochs[name])
            if row is not None and row["full_pose_fresh"] == "1":
                points[name] = antenna(row, lever)[:2]
                revisions[name] = row["correction_id"]
        row = select_revision(horizontal.get(epoch, []), epoch, receive,
                              decision_epochs["gnss"])
        if row is not None and row["horizontal_fresh"] == "1":
            points["gnss_horizontal"] = horizontal_antenna(row, odom[epoch], lever)
            revisions["gnss_horizontal"] = row["correction_id"]
            x, y = points["gnss_horizontal"]
            horizontal_support.append(dict(
                sequence=event["sequence"], timestamp_ns=source_ns,
                receive_ns=receive, odom_epoch_ns=epoch,
                association_offset_ns=source_ns - epoch,
                reference_x=float(event["x"]), reference_y=float(event["y"]),
                gnss_horizontal_x=x, gnss_horizontal_y=y,
                gnss_horizontal_revision=row["correction_id"],
                gnss_horizontal_error_m=math.hypot(
                    x - float(event["x"]), y - float(event["y"]))))
        if not {"odom", "map", "both"} <= points.keys():
            continue
        sample = dict(sequence=event["sequence"], timestamp_ns=source_ns,
                      receive_ns=receive, odom_epoch_ns=epoch,
                      association_offset_ns=source_ns - epoch,
                      reference_x=float(event["x"]), reference_y=float(event["y"]))
        for name in ("odom", "map", "both"):
            x, y = points[name]
            sample.update({name + "_x": x, name + "_y": y,
                           name + "_revision": revisions[name],
                           name + "_error_m": math.hypot(
                               x - sample["reference_x"], y - sample["reference_y"])})
        paired.append(sample)
        if "gnss_horizontal" in points:
            extra = dict(sample)
            x, y = points["gnss_horizontal"]
            extra.update(gnss_horizontal_x=x, gnss_horizontal_y=y,
                         gnss_horizontal_revision=revisions["gnss_horizontal"],
                         gnss_horizontal_error_m=math.hypot(
                             x - sample["reference_x"], y - sample["reference_y"]))
            partial.append(extra)
    return paired, partial, horizontal_support


def display_trajectories(arms, lever, anchor):
    """Final emitted per epoch for display only; never used in predictive scores."""
    local = {int(r["timestamp_ns"]): r for r in arms["odom"]["odometry"]
             if r["valid"] == "1"}
    result = []
    for epoch, row in local.items():
        point = anchor(antenna(row, lever, local=True))
        result.append(dict(arm="odom", timestamp_ns=epoch, receive_ns=row["receive_ns"],
                           revision="", x=point[0], y=point[1]))
    for name in ("map", "both", "gnss_horizontal"):
        records = arms["gnss"].get("horizontal", []) if name == "gnss_horizontal" \
            else arms[name]["global"]
        latest = {int(r["timestamp_ns"]): r for r in records
                  if r["alignment_initialized"] == "1"}
        for epoch, row in sorted(latest.items()):
            if name == "gnss_horizontal":
                point = horizontal_antenna(row, local[epoch], lever)
            else:
                point = antenna(row, lever)
            result.append(dict(arm=name, timestamp_ns=epoch, receive_ns=row["receive_ns"],
                               revision=row["correction_id"], x=point[0], y=point[1]))
    return result


def svg(series, equal=False, x_label="source time (s)", y_label="metres"):
    points = [p for values in series.values() for p in values]
    if not points:
        return "<p>No supported samples.</p>"
    xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
    ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
    width, height = 760, 420
    sx, sy = 660 / max(xmax - xmin, 1e-6), 320 / max(ymax - ymin, 1e-6)
    if equal:
        sx = sy = min(sx, sy)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" '
             f'viewBox="0 0 {width} {height}" role="img">',
             '<rect width="760" height="420" fill="#fafafa"/>',
             f'<text x="50" y="20">{html.escape(y_label)}</text>',
             f'<text x="390" y="410">{html.escape(x_label)}</text>']
    for index, (name, values) in enumerate(series.items()):
        color = COLORS.get(name, ("#1870ba", "#c54d24", "#8b43a8")[index % 3])
        coords = " ".join(f"{50+(x-xmin)*sx:.2f},{365-(y-ymin)*sy:.2f}"
                          for x, y in values)
        parts.append(f'<polyline points="{coords}" fill="none" stroke="{color}" '
                     'stroke-width="1.5"/>')
        for x, y in values:
            parts.append(f'<circle cx="{50+(x-xmin)*sx:.2f}" '
                         f'cy="{365-(y-ymin)*sy:.2f}" r="2" fill="{color}"/>')
        parts.append(f'<text x="{55+index*145}" y="390" fill="{color}">'
                     f'{html.escape(name)}</text>')
    parts.append(f'<text x="50" y="40">x [{xmin:.3f}, {xmax:.3f}], '
                 f'y [{ymin:.3f}, {ymax:.3f}]</text></svg>')
    return "".join(parts)


def correction_changes(arm, name):
    rows = arm["global"]
    previous, changes = None, []
    for row in rows:
        if row["alignment_initialized"] != "1":
            continue
        if previous is not None and row["correction_id"] != previous["correction_id"]:
            same = row["timestamp_ns"] == previous["timestamp_ns"]
            changes.append(dict(
                arm=name, timestamp_ns=row["timestamp_ns"],
                correction_id=row["correction_id"], same_epoch_revision=int(same),
                position_jump_m=math.dist(
                    [float(row[k]) for k in "xyz"],
                    [float(previous[k]) for k in "xyz"]) if same else "",
                note="same-epoch emitted pose revision" if same
                else "unavailable: different epochs; do not subtract vehicle motion"))
        previous = row
    return changes


LIMITS = [
    "GNSS consistency, NOT independent absolute ATE: GNSS feeds fusion and same-capture map construction.",
    "Map XY is EPSG:32650 UTM-grid offsets; Z is frontend-relative. T_enu_map is unavailable.",
    "GNSS-only XY/yaw uses an explicit unqualified level-axis assumption; no height/full-SE3 truth.",
    "ODOM display uses exactly the first actual initialized map-only pose at its exact ODOM epoch; no whole-track fit.",
    "Source-causal epoch association: floor ODOM within 10 ms, latest emitted revision at that epoch "
    "before original GNSS receive time, with correction measurement <= ODOM epoch. No interpolation/extrapolation.",
    "Scores are predictive with respect to the current GNSS epoch, not held-out independent truth; "
    "earlier GNSS and future-informed map/seed still leak reference information.",
    "RPE/6DoF ATE, heading accuracy, vertical/gravity qualification, covariance calibration, "
    "full-route coverage and live 100 Hz latency are unavailable, not zero.",
    "4071 finite local states do not mean qualified capability; inspect CORRELATION_UNQUALIFIED "
    "and CORRECTION_BUDGET_EXCEEDED counts.",
]


def generate(baseline, candidate, gnss_input, output):
    baseline, candidate, gnss_input = (p.resolve(strict=True)
                                       for p in (baseline, candidate, gnss_input))
    if output.exists():
        raise ValueError("output must be fresh")
    pins = {}
    for root in (baseline, candidate):
        for path in root.rglob("*"):
            if path.is_file():
                pins[path] = digest(path)
        contract = load_json(root / "contract.json")
        for value, expected in contract["pins"].items():
            if value.startswith("/apollo/pkgs/openloc/.agents/notes/"):
                path = resolve_path(value)
                if digest(path) != expected:
                    raise ValueError("frozen source input changed: " + str(path))
                pins[path] = expected
    for path in gnss_input.glob("*"):
        if path.is_file():
            pins[path] = digest(path)
    declared = load_json(baseline / "contract.json")["pins"]
    declared_paths = {resolve_path(value): expected for value, expected in declared.items()}
    for filename in ("manifest.json", "events.csv", "config.csv", "identity.csv"):
        path = gnss_input / filename
        if declared_paths.get(path) != digest(path):
            raise ValueError("scoring GNSS must match frozen input contract: " + str(path))
    for path in Path(__file__).parent.glob("compare_native_lio_baselines*.py"):
        pins[path] = digest(path)
    manifest = load_json(gnss_input / "manifest.json")
    if manifest["reference"]["T_enu_map"] is not None:
        raise ValueError("report expects explicit horizontal-only map convention")
    raw = next(resolve_path(p) for p in load_json(baseline / "contract.json")["pins"]
               if p.endswith("/raw_30s/manifest.json"))
    if load_json(raw)["external_odometry_used"] is not False:
        raise ValueError("requires actual raw native odometry")
    config = read_csv(gnss_input / "config.csv")[0]
    lever = tuple(float(config["lever_" + axis]) for axis in "xyz")
    events = read_csv(gnss_input / "events.csv")
    versions = {}
    for label, root in (("baseline", baseline), ("candidate", candidate)):
        versions[label] = {name: load_arm(root, key, pins)
                           for name, key in (("odom", "odom_v2"), ("map", "map"),
                                             ("both", "both"), ("gnss", "gnss"))}
    local = {int(r["timestamp_ns"]): r for r in versions["baseline"]["odom"]["odometry"]
             if r["valid"] == "1"}
    first = next(r for r in versions["baseline"]["map"]["global"]
                 if r["alignment_initialized"] == "1")
    anchor_epoch = int(first["timestamp_ns"])
    anchor = fixed_anchor(local[anchor_epoch], first)
    start = int(versions["baseline"]["odom"]["odometry"][0]["timestamp_ns"])
    output.mkdir(parents=True)
    summary = dict(kind="post-hoc frozen functional replay diagnostic",
                   estimator_rerun=False, tolerance_ns=TOLERANCE_NS,
                   lever_arm_imu_to_antenna_m=lever, anchor_epoch_ns=anchor_epoch,
                   anchor_global_pose=first, anchor_local_pose=local[anchor_epoch],
                   coordinate_reference=manifest["reference"], limits=LIMITS,
                   versions={})
    page = ['<!doctype html><meta charset="utf-8"><title>Native baseline diagnostics</title>',
            '<style>body{font:15px sans-serif;max-width:1100px;margin:auto}svg{width:100%}'
            'pre{white-space:pre-wrap}table{border-collapse:collapse}td,th{padding:7px;'
            'border:1px solid #bbb}</style><h1>Native replay: functional evidence and consistency</h1>',
            "<ul>" + "".join("<li>" + html.escape(s) + "</li>" for s in LIMITS) + "</ul>"]
    all_ledgers, all_changes = [], []
    scores = {}
    for label, arms in versions.items():
        for name, arm in arms.items():
            for filename, ignored in (("odometry", {"imu_ms"}), ("lidar", {"processing_ms"})):
                normalized = lambda rows: [{k: v for k, v in r.items() if k not in ignored}
                                           for r in rows]
                if normalized(arm[filename]) != normalized(versions["baseline"]["odom"][filename]):
                    raise ValueError("local isolation mismatch: " + label + "/" + name + "/" + filename)
            all_ledgers.extend(event_ledger(arm, label + "/" + name))
            all_changes.extend(correction_changes(arm, label + "/" + name))
        paired, partial, horizontal_support = score(arms, events, lever, anchor)
        scores[label] = (paired, partial)
        write_csv(output / (label + "_common_support.csv"), paired)
        write_csv(output / (label + "_horizontal_common_support.csv"), partial)
        write_csv(output / (label + "_horizontal_support.csv"), horizontal_support)
        metrics = {name: distribution(r[name + "_error_m"] for r in paired)
                   for name in ("odom", "map", "both")}
        horizontal_metrics = {name: distribution(r[name + "_error_m"] for r in partial)
                              for name in ("odom", "map", "both", "gnss_horizontal")}
        summary["versions"][label] = dict(
            arms={name: arm_summary(arm, start) for name, arm in arms.items()},
            common_support=metrics, horizontal_common_support=horizontal_metrics,
            horizontal_available_support=distribution(
                r["gnss_horizontal_error_m"] for r in horizontal_support),
            coverage=dict(input_gga_epochs=len(events), primary_common_epochs=len(paired),
                          four_arm_common_epochs=len(partial),
                          horizontal_available_epochs=len(horizontal_support),
                          selection="fresh output only; no held/stale extrapolation"),
            exact_local_isolation=True)
        display = display_trajectories(arms, lever, anchor)
        write_csv(output / (label + "_display_trajectories.csv"), display)
        page.append("<h2>" + label + "</h2><h3>Retrospective emitted trajectories "
                    "(equal scale; display only, not the predictive score)</h3><p>"
                    "One final emitted revision per ODOM epoch, including later holdback drains. "
                    "GNSS-only purple is partial XY/yaw, not a full-pose trajectory. "
                    "Plotted every tenth epoch; CSV retains all epochs.</p>")
        overlay = {name: [(r["x"], r["y"]) for r in display if r["arm"] == name][::10]
                   for name in ("odom", "map", "both", "gnss_horizontal")}
        overlay["reference"] = [(float(r["x"]), float(r["y"])) for r in events
                                if r["adapter_reason"] == "ok" and r["quality"] == "4"]
        figure = svg(overlay, equal=True, x_label="map grid X (m)",
                     y_label="map grid Y (m); retrospective display")
        page.append(figure)
        (output / (label + "_trajectory.svg")).write_text(figure + "\n")
        page.append("<h2>" + label + "</h2><h3>Common-support antenna XY (equal scale)</h3>")
        trajectories = {name: [(r[name + "_x"], r[name + "_y"]) for r in paired]
                        for name in ("odom", "map", "both")}
        trajectories["reference"] = [(r["reference_x"], r["reference_y"]) for r in paired]
        if partial:
            trajectories["gnss_horizontal"] = [
                (r["gnss_horizontal_x"], r["gnss_horizontal_y"]) for r in partial]
        figure = svg(trajectories, equal=True, x_label="map grid X (m)",
                     y_label="map grid Y (m); GNSS-only partial distinct")
        page.append(figure)
        (output / (label + "_common_support.svg")).write_text(figure + "\n")
        figure = svg({
            name: [((r["timestamp_ns"] - start) / 1e9, r[name + "_error_m"])
                   for r in paired] for name in ("odom", "map", "both")})
        page.append("<h3>Paired source-epoch residuals</h3>" + figure)
        (output / (label + "_residuals.svg")).write_text(figure + "\n")
        if partial:
            page.append("<h3>GNSS-only partial XY: four-arm common support residuals</h3>" +
                        svg({name: [((r["timestamp_ns"] - start) / 1e9,
                                      r[name + "_error_m"]) for r in partial]
                             for name in ("odom", "map", "both", "gnss_horizontal")}))
        if horizontal_support:
            page.append("<h3>GNSS-only available-support predictive residual "
                        "(different coverage; do not rank against the primary table)</h3>" +
                        svg({"gnss_horizontal": [
                            ((r["timestamp_ns"] - start) / 1e9,
                             r["gnss_horizontal_error_m"]) for r in horizontal_support]}))
        page.append("<h3>Functional / freshness / runtime evidence</h3><pre>" +
                    html.escape(json.dumps(summary["versions"][label], indent=2)) + "</pre>")
    # Keep before/after scores strictly paired even when one version loses coverage.
    before = {r["timestamp_ns"]: r for r in scores["baseline"][0]}
    after = {r["timestamp_ns"]: r for r in scores["candidate"][0]}
    common = sorted(before.keys() & after.keys())
    summary["before_after_common_epochs"] = len(common)
    summary["before_after"] = {
        label: {name: distribution(records[t][name + "_error_m"] for t in common)
                for name in ("odom", "map", "both")}
        for label, records in (("baseline", before), ("candidate", after))}
    summary["combined_outputs_identical"] = all(
        versions["baseline"]["both"][name] == versions["candidate"]["both"][name]
        for name in ("global", "global_decisions"))
    summary["source_event_accounting"] = {
        label: dict(Counter(
            "actual_correction" if r["actual_commit"] == "1" else
            r["final_owner"] if r["final_owner"] != "not_submitted" else
            r["submission"] if r["gate"] == "accepted" else r["gate"]
            for r in all_ledgers if r["arm"] == label))
        for label in sorted({r["arm"] for r in all_ledgers})}
    findings = [
        "Functional GNSS-only acquisition changed from zero to ten actual corrections; "
        "this is partial XY/yaw availability, not full-pose or accuracy qualification.",
        "Both versions retain the same 105 map + 20 GNSS actual corrections; "
        "combined emitted trajectory and decision journal are identical. The bootstrap flag "
        "does not change combined behavior or establish an accuracy improvement.",
        "Map-only versus both is a GNSS-presence ablation, NOT the v1/v2 bootstrap treatment. "
        "Registration success and failure details are observational evidence. "
        "GNSS changing the map prediction/convergence is a plausible explanation, not a proven cause.",
        "Compare median/P95/max on identical epochs; lower inconsistency is not independent accuracy. "
        "Do not hide worsening metrics or the limited GNSS-only support.",
    ]
    for label, (_, partial) in scores.items():
        if partial:
            gnss_median = statistics.median(r["gnss_horizontal_error_m"] for r in partial)
            both_median = statistics.median(r["both_error_m"] for r in partial)
            findings.append(
                f"{label}: four-arm common support is only {len(partial)} epoch(s). "
                f"GNSS-only median {gnss_median:.3f} m versus combined {both_median:.3f} m; "
                "this limited support is not sufficient for a general performance conclusion.")
        available = summary["versions"][label]["horizontal_available_support"]
        if available["samples"]:
            findings.append(
                f"{label}: GNSS-only available-support predictive inconsistency "
                f"({available['samples']} epochs, a different late-window support): "
                f"median {available['median_m']:.3f} m, P95 {available['p95_m']:.3f} m, "
                f"max {available['max_m']:.3f} m, RMSE {available['rmse_m']:.3f} m. "
                "Functional acquisition is therefore not a quality pass. Alignment observability, "
                "motion-derived heading, unqualified level-axis assumption and uncalibrated "
                "uncertainty are unresolved hypotheses, not established causes.")
    summary["interpretation"] = findings
    write_csv(output / "source_events.csv", all_ledgers)
    write_csv(output / "correction_changes.csv", all_changes)
    page.append("<h2>Correction discontinuities</h2>" + svg({
        label: [((int(r["timestamp_ns"]) - start) / 1e9, float(r["position_jump_m"]))
                for r in all_changes if r["arm"] == label and r["position_jump_m"] != ""]
        for label in sorted({r["arm"] for r in all_changes})},
        y_label="same-epoch emitted position revision (m)"))
    page.append("<h2>Interpretation</h2><ul>" +
                "".join("<li>" + html.escape(s) + "</li>" for s in findings) + "</ul>")
    page.append("<h2>Source-time GNSS event accounting</h2><table><tr>" +
                "".join("<th>" + k + "</th>" for k in
                        ("arm", "source s", "gate", "final owner", "actual commit", "revision")) + "</tr>")
    for row in all_ledgers:
        page.append("<tr>" + "".join("<td>" + html.escape(str(v)) + "</td>" for v in
                    (row["arm"], round((int(row["timestamp_ns"]) - start) / 1e9, 3),
                     row["gate"], row["final_owner"], row["actual_commit"],
                     row["correction_id"])) + "</tr>")
    page.append("</table><h2>Method references</h2><p>"
                "evo APE/RPE conventions: https://github.com/MichaelGrupp/evo/wiki/Metrics ; "
                "TUM RGB-D evaluation: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools . "
                "Method pointers only: live source retrieval was blocked in this environment. "
                "These distinguish absolute/aligned and relative errors. Here horizontal dependent "
                "GNSS supports consistency only; no SE3 ground truth or trajectory-fit ATE is invented."
                "</p><p>CSV: baseline_common_support.csv, candidate_common_support.csv, "
                "*_horizontal_common_support.csv, source_events.csv, correction_changes.csv. "
                "JSON: analysis.json, verification.json.</p>")
    (output / "analysis.json").write_text(json.dumps(summary, indent=2) + "\n")
    (output / "diagnostics.html").write_text("\n".join(page) + "\n")
    changed = [str(path) for path, expected in pins.items() if digest(path) != expected]
    if changed:
        raise ValueError("frozen evidence/reporter changed during analysis: " + str(changed))
    verification = dict(inputs_and_reporter_unchanged=True, estimator_rerun=False,
                        pinned_files={str(p): h for p, h in sorted(pins.items())},
                        outputs={p.name: digest(p) for p in output.iterdir() if p.is_file()})
    (output / "verification.json").write_text(json.dumps(verification, indent=2) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("baseline", "candidate", "gnss-input", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    summary = generate(args.baseline, args.candidate, args.gnss_input, args.output)
    print(json.dumps(summary["before_after"], indent=2))


if __name__ == "__main__":
    main()
