"""Frozen two-capture public XY experiment; source-prepared diagnostic only."""

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess


CELLS = (
    "map_20261013170722_query_20261013143752",
    "map_20261013143752_query_20261013170722",
)
SCENARIOS = {
    "nominal": "input.csv",
    "outage": "input_outage.csv",
    "outlier": "input_outlier.csv",
    "omission": "input_outlier_omission.csv",
}
ARMS = ("baseline", "recovery", "tracking")
ODOM_HEADER = "timestamp_ns,px,py,pz,qx,qy,qz,qw"
FIX_HEADER = "arrival_ns,timestamp_ns,gx,gy,sigma,quality"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def write(path, header, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header.split(","))
        writer.writerows(rows)


def multiply(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return [
        w*u + x*s + y*t - z*v,
        w*v - x*t + y*s + z*u,
        w*t + x*v - y*u + z*s,
        w*s - x*u - y*v - z*t,
    ]


def rotate(q, v):
    return multiply(multiply(q, [*v, 0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def quaternion(matrix):
    # Largest-component construction avoids a singular trace near pi.
    r = matrix
    candidates = [
        1 + r[0][0] - r[1][1] - r[2][2],
        1 - r[0][0] + r[1][1] - r[2][2],
        1 - r[0][0] - r[1][1] + r[2][2],
        1 + r[0][0] + r[1][1] + r[2][2],
    ]
    i = max(range(4), key=candidates.__getitem__)
    q = [0.0] * 4
    q[i] = math.sqrt(candidates[i]) / 2
    denominator = 4 * q[i]
    if i == 3:
        q[:3] = [(r[2][1]-r[1][2])/denominator,
                 (r[0][2]-r[2][0])/denominator,
                 (r[1][0]-r[0][1])/denominator]
    else:
        j, k = (i + 1) % 3, (i + 2) % 3
        q[j] = (r[j][i]+r[i][j])/denominator
        q[k] = (r[k][i]+r[i][k])/denominator
        q[3] = (r[k][j]-r[j][k])/denominator
    require(abs(sum(x*x for x in q)-1) < 1e-10, "non-rigid frozen transform")
    return q


def interpolate(left, right, timestamp):
    a = (timestamp-int(left[0]))/(int(right[0])-int(left[0]))
    position = [(1-a)*left[i]+a*right[i] for i in range(1, 4)]
    q, other = left[4:], right[4:]
    dot = sum(x*y for x, y in zip(q, other))
    if dot < 0:
        other, dot = [-x for x in other], -dot
    if dot > 0.9995:
        result = [(1-a)*x+a*y for x, y in zip(q, other)]
    else:
        angle = math.acos(min(1, dot))
        result = [(math.sin((1-a)*angle)*x+math.sin(a*angle)*y)/math.sin(angle)
                  for x, y in zip(q, other)]
    norm = math.sqrt(sum(x*x for x in result))
    return position, [x/norm for x in result]


def prepare(package, cell, destination):
    notes = package / ".agents/notes/two_capture_validation_20261015"
    source = notes / "horizontal_v2" / cell
    manifest_path = source / "input_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    paths = [manifest_path]
    for key, hash_key in (
        ("raw_odom_path", "raw_odom_sha256"),
        ("graph_path", "graph_sha256"),
    ):
        path = Path(manifest[key])
        require(digest(path) == manifest[hash_key], f"source identity conflict: {path}")
        paths.append(path)
    graph = json.loads(Path(manifest["graph_path"]).read_text())
    lever = graph["lever"]
    transform = manifest["offline_initial_transform"]
    rotation = quaternion(transform)
    odom = []
    for row in read(Path(manifest["raw_odom_path"])):
        position = [float(row[key]) for key in ("x", "y", "z")]
        q = [float(row[key]) for key in ("qx", "qy", "qz", "qw")]
        require(abs(sum(x*x for x in q)-1) < 1e-8, "nonunit source ODOM")
        position = [sum(transform[i][j]*position[j] for j in range(3))
                    + transform[i][3] for i in range(3)]
        q = multiply(rotation, q)
        odom.append([int(row["timestamp_ns"]), *position, *q])
    stamps = [row[0] for row in odom]
    require(all(b > a for a, b in zip(stamps, stamps[1:])), "ODOM time conflict")
    nominal = read(source / "input.csv")
    max_body_error = max_antenna_error = 0
    for sample in nominal:
        stamp = int(sample["timestamp_ns"])
        right = bisect.bisect_left(stamps, stamp)
        require(0 < right < len(stamps), "unbracketed prepared fix")
        position, q = interpolate(odom[right-1], odom[right], stamp)
        antenna = [x+y for x, y in zip(position, rotate(q, lever))]
        max_body_error = max(max_body_error, math.dist(
            position, [float(sample[k]) for k in ("px", "py", "pz")]))
        max_antenna_error = max(max_antenna_error, math.dist(
            antenna, [float(sample[k]) for k in ("ax", "ay", "az")]))
    require(max(max_body_error, max_antenna_error) < 1e-8,
            "recovered pose/lever chain disagrees with frozen prepared input")
    write(destination / "odom.csv", ODOM_HEADER, odom)
    write(destination / "lever.csv", "lx,ly,lz", [lever])
    scenario_samples = {}
    for scenario, filename in SCENARIOS.items():
        path = source / filename
        paths.append(path)
        samples = read(path)
        require(len(samples) == len(nominal), "scenario epoch coverage differs")
        for base, sample in zip(nominal, samples):
            for key in ("timestamp_ns", "px", "py", "pz", "ax", "ay", "az"):
                require(base[key] == sample[key], "scenario changes ODOM/time")
        rows = [[sample["timestamp_ns"], sample["timestamp_ns"],
                 sample["gx"], sample["gy"], sample["sigma"], 1]
                for sample in samples if sample["gnss_present"] == "1"]
        write(destination / (scenario + "_fixes.csv"), FIX_HEADER, rows)
        scenario_samples[scenario] = samples
    invalid_quality = [
        [sample["timestamp_ns"], sample["timestamp_ns"],
         sample["gx"], sample["gy"], sample["sigma"], 0 if index == 40 else 1]
        for index, sample in enumerate(nominal)
    ]
    write(destination / "invalid_quality_fixes.csv", FIX_HEADER, invalid_quality)
    write(destination / "prefix_odom.csv", ODOM_HEADER, odom[:64])
    prefix_fixes = [[s["timestamp_ns"], s["timestamp_ns"], s["gx"], s["gy"],
                     s["sigma"], 1]
                    for s in nominal if int(s["timestamp_ns"]) <= odom[63][0]]
    require(prefix_fixes, "prefix must exercise at least one fix")
    write(destination / "prefix_fixes.csv", FIX_HEADER, prefix_fixes)
    return paths, scenario_samples, {
        "query": cell.split("_query_")[1], "raw_keyframes": len(odom),
        "prepared_epochs": len(nominal),
        "max_pose_join_error_m": max_body_error,
        "max_antenna_join_error_m": max_antenna_error,
        "lever_in_body_m": lever, "offline_fixed_transform": transform,
        "max_keyframe_gap_s": max(b-a for a, b in zip(stamps, stamps[1:]))/1e9,
        "speed": "sparse displacement-rate proxy; not certified stationarity",
        "reference": "future-informed provisional tilt; not qualified physical up",
        "hashes": {str(path): digest(path) for path in paths},
    }


def quantiles(values):
    values = sorted(values)
    if not values:
        return None
    result = []
    for fraction in (0.5, 0.95, 1.0):
        offset = (len(values)-1)*fraction
        low = int(offset)
        weight = offset-low
        result.append(values[low]*(1-weight)+values[min(low+1,len(values)-1)]*weight)
    return result


def prepare_controls(root, nominal):
    odom = [
        [int(row["timestamp_ns"])] +
        [float(row[key]) for key in ("px", "py", "pz", "qx", "qy", "qz", "qw")]
        for row in read(root/"odom.csv")
    ]
    stamps = [row[0] for row in odom]
    lever = [float(read(root/"lever.csv")[0][key]) for key in ("lx", "ly", "lz")]
    sampled = []
    for sample in nominal[160:210]:
        stamp = int(sample["timestamp_ns"])
        right = bisect.bisect_left(stamps, stamp)
        position, q = interpolate(odom[right-1], odom[right], stamp)
        sampled.append([stamp, *position, *q])
    for index in range(20):
        sampled.append([sampled[49][0]+(index+1)*10**9, *sampled[49][1:]])
    fixes = []
    for index, row in enumerate(sampled):
        antenna = [x+y for x, y in zip(row[1:4], rotate(row[4:], lever))]
        fixes.append([row[0], row[0], antenna[0]+(100 if index >= 5 else 0),
                      antenna[1], 1, 1])
    write(root/"synthetic_stop_odom.csv", ODOM_HEADER, sampled)
    write(root/"synthetic_stop_fixes.csv", FIX_HEADER, fixes)
    interrupted = [list(row) for row in fixes]
    interrupted[52][-1] = 0
    write(root/"synthetic_quality_fixes.csv", FIX_HEADER, interrupted)
    shifted = [list(row) for row in sampled]
    for row in shifted:
        row[1] -= 1000
    write(root/"synthetic_origin_odom.csv", ODOM_HEADER, shifted)
    return {
        "source": "nominal epochs160..209, exact interpolated source poses",
        "synthetic": "resample at prepared epochs; +100 m GNSS offset from5; "
                     "20 repeated-pose stopped epochs; ideal arrival and quality",
        "interpretation": "coherent bias counterexample, not measured drift/stop",
        "quality_control": "invalid synthetic quality at52",
        "origin_control": "subtract1000 m from ODOM X only; physical GNSS unchanged",
        "stop_start_ns": sampled[50][0],
    }


def state_fields(row):
    return {key: value for key, value in row.items()
            if key.startswith(("backend_", "published_", "cov_", "offset_"))
            or key in ("alignment_revision", "reset_counter", "global_valid",
                       "vertical_initialized", "publication_state", "pending",
                       "mean_matches", "source_fresh", "global_degraded")}


def evaluate(output, samples, inputs):
    rows = read(output)
    summary = read(Path(str(output)+".summary.csv"))[0]
    odom = read(inputs / "odom.csv")
    emitted = [row for row in rows if row["event"] == "odom"]
    require(len(emitted) == len(odom), "missing ODOM output")
    for expected, actual in zip(odom, emitted):
        require(expected["timestamp_ns"] == actual["odom_ns"], "ODOM time changed")
        for key in ("px", "py", "pz", "qx", "qy", "qz", "qw"):
            require(float(expected[key]) == float(actual[key]), "ODOM pose changed")
        require(actual["epoch_generation"] == "1", "ODOM epoch changed")
    require(int(summary["output_events"]) == len(rows), "missing event output")
    require(summary["pending_at_end"] == "0", "unexpected unresolved data fix")
    records = {}
    for record in read(Path(str(output)+".factors.csv")):
        records[record["record_index"]] = record
    reasons = {}
    for record in records.values():
        reasons[record["reason"]] = reasons.get(record["reason"], 0)+1
        require(record["type"] in ("ABS_POS2", "GLOBAL_XY_YAW"), "unexpected factor")
    require(all(row["vertical_initialized"] == "0" for row in rows),
            "XY initialized vertical")
    require(all(abs(float(row["backend_tz"])) < 1e-8 for row in rows),
            "XY changed unobserved Z mean")
    previous_reset = previous_revision = 0
    resets = []
    revisions = []
    holds = []
    for event_index, row in enumerate(rows):
        revision, reset = int(row["alignment_revision"]), int(row["reset_counter"])
        if revision != previous_revision:
            revisions.append({"arrival_ns": int(row["arrival_ns"]),
                              "measurement_ns": int(row["measurement_ns"]),
                              "reset_counter": reset,
                              "pending": row["pending"],
                              "offset_m": math.sqrt(sum(
                                  float(row["offset_"+k])**2 for k in "xyz"))})
        if reset != previous_reset:
            resets.append({"arrival_ns": int(row["arrival_ns"]),
                           "jump_m": float(row["correction_jump_m"]),
                           "speed_proxy_mps": float(row["speed_proxy_mps"])})
        if row["publication_state"] == "held_for_reset":
            holds.append(row)
            if float(row["speed_proxy_mps"]) > 0.2:
                require(float(row["correction_jump_m"]) < 1e-8,
                        "moving held output jumped")
        require(not (row["pending"] == "1" and row["mean_matches"] == "1"),
                "inconsistent mean/covariance applicability flags")
        for i, key in enumerate("xyz"):
            require(abs(float(row["offset_"+key]) -
                        (float(row["backend_"+key]) -
                         float(row["published_"+key]))) < 1e-7,
                    "publication offset differs from composed body output")
        if row["global_valid"] == "1" and row["mean_matches"] == "0":
            require(row["publication_state"] in
                    ("held_for_reset", "rate_limited", "awaiting_policy_update"),
                    "mismatched publication marked aligned")
        if output.stem.endswith("_tracking") and event_index > 0:
            previous = rows[event_index-1]
            if previous["global_valid"] == "1":
                elapsed = (int(row["odom_ns"])-int(previous["odom_ns"]))/1e9
                require(float(row["correction_jump_m"]) <= 0.3*elapsed+1e-7,
                        "tracking exceeded current-epoch position rate")
                require(abs(float(row["correction_yaw_rad"])) <=
                        math.radians(1)*elapsed+1e-8,
                        "tracking exceeded yaw rate")
                require(row["reset_counter"] == previous["reset_counter"],
                        "tracking emitted a post-initialization reset")
            if row["source_fresh"] == "0" and previous["global_valid"] == "1":
                require(float(row["correction_jump_m"]) < 1e-8,
                        "tracking followed stale position evidence")
        previous_reset, previous_revision = reset, revision
    # Last event at each processing time includes same-time fix submission.
    by_time = {}
    for row in rows:
        by_time[int(row["arrival_ns"])] = row
    times = sorted(by_time)
    stamps = [int(row["timestamp_ns"]) for row in odom]
    residuals = {"backend": [], "published": []}
    scores = []
    for sample in samples:
        timestamp = int(sample["timestamp_ns"])
        right = bisect.bisect_left(stamps, timestamp)
        processed = max(timestamp, stamps[right])
        row = by_time[times[bisect.bisect_left(times, processed)]]
        score = {"timestamp_ns": timestamp, "processing_ns": processed}
        ax, ay = float(sample["ax"]), float(sample["ay"])
        for arm in residuals:
            yaw = float(row[arm+"_yaw"])
            x = float(row[arm+"_tx"])+math.cos(yaw)*ax-math.sin(yaw)*ay
            y = float(row[arm+"_ty"])+math.sin(yaw)*ax+math.cos(yaw)*ay
            value = math.hypot(x-float(sample["gx"]), y-float(sample["gy"]))
            residuals[arm].append(value)
            score[arm+"_same_source_xy_m"] = value
        scores.append(score)
    write(Path(str(output)+".scores.csv"), ",".join(scores[0]),
          [list(score.values()) for score in scores])
    accepted = sorted(int(r["timestamp_ns"]) for r in records.values()
                      if r["accepted"] == "1" and r["type"] == "ABS_POS2")
    return rows, {
        "events": len(rows), "odom_epochs": len(emitted),
        "scheduled_scoring_epochs": len(scores),
        "fix_submissions": int(summary["fix_submissions"]),
        "ordinary_admitted": len(accepted), "decision_reasons": reasons,
        "backend_reanchors": revisions, "published_resets": resets,
        "held_event_count": len(holds),
        "pending_event_count": sum(row["pending"] == "1" for row in rows),
        "degraded_event_count": sum(row["global_degraded"] == "1" for row in rows),
        "max_correction_jump_m": max(float(row["correction_jump_m"]) for row in rows),
        "max_pending_offset_m": max(math.sqrt(sum(float(row["offset_"+k])**2
                                                for k in "xyz")) for row in rows
                                    if row["global_valid"] == "1"),
        "max_held_offset_m": max((math.sqrt(sum(float(r["offset_"+k])**2
                                                for k in "xyz"))
                                 for r in holds), default=0),
        "same_source_residuals_median_p95_max_m": {
            key: quantiles(values) for key, values in residuals.items()},
        "first_readmission_after_outage_s": next(
            ((stamp-int(samples[160]["timestamp_ns"]))/1e9
             for stamp in accepted if stamp >= int(samples[160]["timestamp_ns"])),
            None),
        "wall_seconds": float(summary["wall_seconds"]),
        "peak_rss_kb": int(summary["peak_rss_kb"]),
        "final": state_fields(rows[-1]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--container", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    package, output = args.package.resolve(), args.output.resolve()
    require(not output.exists(), "preserve existing experiment output")
    require(output.is_relative_to(package), "output must be inside package")
    output.mkdir(parents=True)
    try:
        execute(package, output, args.container)
    except Exception as error:
        save(output/"failure.json", {"error": str(error), "disposition": "STOP"})
        raise


def execute(package, output, container):
    notes = package/".agents/notes/two_capture_validation_20261015"
    frozen = json.loads((notes/"recovery_v3_hardened/protocol.json").read_text())
    inputs_by_cell = {}
    paths = []
    provenance = {}
    for cell in CELLS:
        destination = output/cell
        destination.mkdir()
        cell_paths, samples, manifest = prepare(package, cell, destination)
        paths.extend(cell_paths)
        inputs_by_cell[cell] = samples
        provenance[cell] = manifest
        for scenario in SCENARIOS.values():
            path = notes/"horizontal_v2"/cell/scenario
            key = "/apollo/pkgs/openloc/"+str(path.relative_to(package))
            require(digest(path) == frozen["hashes_before_execution"][key],
                    "frozen prepared scenario identity changed")
    controls = prepare_controls(output/CELLS[0], inputs_by_cell[CELLS[0]]["nominal"])
    save(output/"input_contract.json", provenance)
    sources = [package/"BUILD", Path(__file__).resolve(),
               package/"test/integration/map/public_xy_replay.cc",
               package/"test/integration/map/public_xy_replay_test.py",
               package/"test/localization/online/localization_engine_test.cc"]
    for root in ("include/openloc/localization/online", "src/localization/online"):
        sources.extend(sorted((package/root).glob("*.*")))
    paths.extend(sources)
    for path in sources:
        target = output/"source_snapshot"/path.relative_to(package)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    for path in output.glob("map_*/*csv"):
        paths.append(path)
    hashes = {str(path): digest(path) for path in set(paths)}
    container_root = Path("/apollo/pkgs/openloc")
    binary = container_root/"bazel-bin/public_xy_dataset_replay"
    prefix = ["docker", "exec", "-u", f"{os.getuid()}:{os.getgid()}", container,
              "env", "-u", "LD_LIBRARY_PATH"]
    binary_hash = subprocess.check_output(
        prefix+["sha256sum", str(binary)], text=True).split()[0]
    protocol = {
        "classification": "source-prepared sparse public XY software diagnostic",
        "production_allowed": False, "sources": provenance,
        "arms": ARMS, "scenarios": SCENARIOS,
        "pair_changes": {
            "baseline_vs_recovery": "enable_gnss_reacquisition only",
            "recovery_vs_tracking": "large_correction_policy only"},
        "alignment_reference": "fixed first ODOM XY; full mean/covariance "
                               "converted back to caller ODOM coordinates",
        "tracking_bounds": {"position_mps": 0.3, "yaw_degps": 1,
                            "accepted_measurement_max_age_ns": 1500000000,
                            "outage_time_credit": False},
        "origin_tolerances": {
            "physical_position_m": 1e-6,
            "covariance": "each entry error / sqrt(max(1,Pii)*max(1,Pjj)) "
                          "< 1e-6; small-variance entries retain 1e-6 absolute",
            "nis": "absolute/relative 1e-6; admission/publication decisions exact"},
        "synthetic_receiver": {"carrier": "fixed", "satellites": 20, "hdop": 0.8,
                               "differential_age_s": 0.2,
                               "horizontal_accuracy_m": 1.0},
        "arrival": "ideal fix arrival at measurement; pending until right ODOM bracket",
        "adapter_config_overrides": {"maximum_interpolation_gap_ns": 2500000000,
                                     "maximum_horizontal_accuracy_m": 2.0,
                                     "maximum_factor_records": "2*submissions+1"},
        "unchanged_gates": "stability, continuity gap, motion consistency, NIS, "
                           "rate and stopped publication defaults retained in "
                           "baseline/recovery; tracking is explicit rate-limited opt-in",
        "noise": "frozen provisional 1 m XY sigma; no vertical observation",
        "faults": "frozen outage indices100..159; +1000 m gx at40; omission40",
        "map": "no map factor; historical rejected-map flag not an observation",
        "scoring": "all scheduled epochs, latest available alignment at right "
                   "bracket composed with measurement-epoch antenna; same-source "
                   "post-update consistency, not prefactor NIS or independent truth",
        "binary_sha256": binary_hash, "hashes_before": hashes,
        "synthetic_controls": controls,
    }
    save(output/"protocol.json", protocol)
    save(output/"data_gaps.json", [
        {"gap": "receiver status/accuracy and arrival unavailable in prepared input",
         "diagnostic": "explicit synthetic receiver profile + idealized arrival",
         "repair": "export source receiver status/accuracy and receive timestamps "
                   "with measurement-time association; rerun real-admission arm"},
        {"gap": "unqualified gravity/vertical datum and manual lever arm",
         "diagnostic": "reuse frozen future-informed fixed transform/declared lever",
         "repair": "independent gravity/frame and antenna extrinsic calibration; "
                   "verify composed antenna positions before physical accuracy claim"},
        {"gap": "sparse keyframe ODOM; no independent truth/calibrated covariance",
         "diagnostic": "2.5 s adapter bracket; sparse speed proxy; same-source scoring",
         "repair": "export full-rate GNSS-off poses and independent reference; "
                   "calibrate uncertainty on held-out data"},
        {"gap": "actual map observations rejected",
         "diagnostic": "continue public GNSS path, no accepted map factors",
         "repair": "separate frozen normal-support/geometry experiment and typed "
                   "observation correlation model decision"},
    ])
    commands = []

    def invoke(cell, name, odom_name, fix_name, arm):
        root = output/cell
        destination = root/(name+".csv")
        arguments = [root/odom_name, root/fix_name, destination]
        mapped = [str(container_root/path.relative_to(package)) for path in arguments]
        command = prefix+[str(binary), *mapped, arm,
                          "--diagnostic-synthetic-quality",
                          str(container_root/(root/"lever.csv").relative_to(package))]
        commands.append(command)
        save(output/"commands.json", commands)
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        (root/(name+".log")).write_text(result.stdout+result.stderr)
        require(result.returncode == 0, f"{cell}/{name}: {result.stderr}")
        return destination

    for cell in CELLS:
        path = invoke(cell, "prefix", "prefix_odom.csv", "prefix_fixes.csv", "recovery")
        stats = read(Path(str(path)+".summary.csv"))[0]
        require(stats["odom_epochs"] == "64" and stats["pending_at_end"] == "0",
                "prefix count/pending contract failed")
    report = []
    outlier_controls = []
    for cell in CELLS:
        results = {}
        for scenario in SCENARIOS:
            for arm in ARMS:
                path = invoke(cell, scenario+"_"+arm, "odom.csv",
                              scenario+"_fixes.csv", arm)
                rows, metrics = evaluate(
                    path, inputs_by_cell[cell][scenario], output/cell)
                if scenario != "outage":
                    metrics.pop("first_readmission_after_outage_s")
                results[(scenario, arm)] = rows
                report.append({"cell": cell, "scenario": scenario, "arm": arm,
                               **metrics})
            for left, right in zip(results[(scenario, "baseline")],
                                   results[(scenario, "recovery")]):
                require(all(left[key] == right[key] for key in
                            ("event", "event_index", "odom_ns", "px", "py", "pz",
                             "qx", "qy", "qz", "qw", "epoch_generation")),
                        "A/B ODOM/event identity differs")
            for left, right in zip(results[(scenario, "recovery")],
                                   results[(scenario, "tracking")]):
                require(all(left[key] == right[key] for key in
                            ("event", "event_index", "odom_ns", "px", "py", "pz",
                             "qx", "qy", "qz", "qw", "epoch_generation")),
                        "publication policy changed ODOM")
                require(all(left[key] == right[key] for key in left
                            if key.startswith(("backend_", "cov_"))),
                        "publication policy fed back to backend")
        for arm in ARMS:
            outlier = {r["odom_ns"]: r for r in results[("outlier", arm)]
                       if r["event"] == "odom"}
            omission = {r["odom_ns"]: r for r in results[("omission", arm)]
                        if r["event"] == "odom"}
            require(outlier.keys() == omission.keys(), "control ODOM epochs differ")
            timestamps = sorted(int(t) for t in outlier)
            rejected_stamp = int(inputs_by_cell[cell]["nominal"][40]["timestamp_ns"])
            processing = str(timestamps[bisect.bisect_left(timestamps, rejected_stamp)])
            require(state_fields(outlier[processing]) ==
                    state_fields(omission[processing]),
                    "outlier changed backend/publication at rejection epoch")
            factors = read(output/cell/(f"outlier_{arm}.csv.factors.csv"))
            record = [r for r in factors if r["timestamp_ns"] == str(rejected_stamp)][-1]
            require(record["accepted"] == "0", "injected outlier admitted")
            different = [t for t in outlier
                         if state_fields(outlier[t]) != state_fields(omission[t])]
            invalid_path = invoke(cell, "invalid_quality_"+arm, "odom.csv",
                                  "invalid_quality_fixes.csv", arm)
            invalid_rows, invalid_metrics = evaluate(
                invalid_path, inputs_by_cell[cell]["nominal"], output/cell)
            invalid_by_time = {r["odom_ns"]: r for r in invalid_rows
                               if r["event"] == "odom"}
            require(state_fields(invalid_by_time[processing]) ==
                    state_fields(omission[processing]),
                    "invalid quality changed state at rejection")
            outlier_controls.append({
                "cell": cell, "arm": arm, "rejected_timestamp_ns": rejected_stamp,
                "processing_ns": int(processing), "outlier_reason": record["reason"],
                "state_parity_at_rejection": True,
                "later_differing_odom_epochs": len(different),
                "first_differing_epoch_ns": int(different[0]) if different else None,
                "interpretation": "fixed-but-motion-rejected observations maintain "
                                  "receiver report continuity; omissions do not. "
                                  "Invalid-quality reports interrupt qualification. "
                                  "Do not require future state parity across these "
                                  "different input streams.",
                "invalid_quality_control": invalid_metrics,
            })
    for path, expected in hashes.items():
        require(digest(Path(path)) == expected, f"frozen file changed: {path}")
    after = subprocess.check_output(
        prefix+["sha256sum", str(binary)], text=True).split()[0]
    require(after == binary_hash, "binary changed during replay")
    control_outputs = {}
    for name, odom_name, fix_name in (
        ("synthetic_stop", "synthetic_stop_odom.csv", "synthetic_stop_fixes.csv"),
        ("synthetic_quality", "synthetic_stop_odom.csv", "synthetic_quality_fixes.csv"),
        ("synthetic_origin", "synthetic_origin_odom.csv", "synthetic_stop_fixes.csv"),
    ):
        path = invoke(CELLS[0], name, odom_name, fix_name, "recovery")
        rows = read(path)
        revision_events = [
            row for previous, row in zip(rows, rows[1:])
            if row["alignment_revision"] != previous["alignment_revision"]]
        reset_events = [
            row for previous, row in zip(rows, rows[1:])
            if previous["global_valid"] == "1" and
            row["reset_counter"] != previous["reset_counter"]]
        startup_reset_counter = next(
            row["reset_counter"] for row in rows if row["global_valid"] == "1")
        if name != "synthetic_origin":
            require(revision_events, "synthetic coherent offset did not reanchor")
            require(all(int(row["arrival_ns"]) < controls["stop_start_ns"] and
                        row["reset_counter"] == startup_reset_counter and
                        row["pending"] == "1"
                        for row in revision_events),
                    "synthetic moving reanchor bypassed publication")
            require(len(reset_events) == 1 and
                    int(reset_events[0]["arrival_ns"]) >= controls["stop_start_ns"],
                    "synthetic stop must allow exactly one later reset")
        control_outputs[name] = {
            "startup_reset_counter": int(startup_reset_counter),
            "reanchors": [{"arrival_ns": row["arrival_ns"],
                           "pending": row["pending"]} for row in revision_events],
            "resets": [{"arrival_ns": row["arrival_ns"],
                        "jump_m": row["correction_jump_m"]}
                       for row in reset_events],
            "final": state_fields(rows[-1]),
        }
    origin_rows = read(output/CELLS[0]/"synthetic_origin.csv")
    stop_rows = read(output/CELLS[0]/"synthetic_stop.csv")
    equivalent = [(a, b) for a, b in zip(origin_rows, stop_rows)
                  if a["global_valid"] == b["global_valid"] == "1"]
    control_outputs["origin_physical_output_comparison"] = {
        "max_backend_position_difference_m": max(
            math.dist([float(a["backend_"+k]) for k in "xyz"],
                      [float(b["backend_"+k]) for k in "xyz"])
            for a, b in equivalent),
        "max_published_position_difference_m": max(
            math.dist([float(a["published_"+k]) for k in "xyz"],
                      [float(b["published_"+k]) for k in "xyz"])
            for a, b in equivalent),
        "max_body_covariance_difference": max(
            abs(float(a[key])-float(b[key])) for a, b in equivalent
            for key in a if key.startswith("body_cov_")),
        "max_xy_covariance_difference": max(
            abs(float(a[f"body_cov_{i}{j}"])-float(b[f"body_cov_{i}{j}"]))
            for a, b in equivalent for i in range(2) for j in range(2)),
        "max_scaled_body_covariance_difference": max(
            abs(float(a[f"body_cov_{i}{j}"])-float(b[f"body_cov_{i}{j}"])) /
            math.sqrt(max(1, float(a[f"body_cov_{i}{i}"])) *
                      max(1, float(a[f"body_cov_{j}{j}"])))
            for a, b in equivalent for i in range(3) for j in range(3)),
        "scope": "translation-only ODOM XY origin change with equivalent priors "
                 "and process noise at fixed first-ODOM reference; not rotation "
                 "or arbitrary reference covariance invariance",
    }
    origin_metrics = control_outputs["origin_physical_output_comparison"]
    save(output/"origin_comparison.json", origin_metrics)
    require(origin_metrics["max_backend_position_difference_m"] < 1e-6 and
            origin_metrics["max_published_position_difference_m"] < 1e-6 and
            origin_metrics["max_scaled_body_covariance_difference"] < 1e-6,
            "equivalent translated ODOM origin changed physical output/covariance")
    for a, b in zip(origin_rows, stop_rows):
        require(all(a[key] == b[key] for key in (
            "global_valid", "publication_state", "pending", "alignment_revision",
            "source_fresh", "global_degraded", "status")),
            "translated origin changed admission/publication decisions")
    origin_factors = read(output/CELLS[0]/"synthetic_origin.csv.factors.csv")
    stop_factors = read(output/CELLS[0]/"synthetic_stop.csv.factors.csv")
    require(len(origin_factors) == len(stop_factors), "origin factor count changed")
    for a, b in zip(origin_factors, stop_factors):
        require(all(a[key] == b[key] for key in
                    ("type", "timestamp_ns", "accepted", "reason")),
                "translated origin changed factor decisions")
        x, y = float(a["nis"]), float(b["nis"])
        require(abs(x-y) <= 1e-6*max(1, abs(x), abs(y)),
                "translated origin changed measurement-space innovation")
    for path, expected in hashes.items():
        require(digest(Path(path)) == expected, f"control changed frozen file: {path}")
    save(output/"evaluation.json", {
        "recovery_pairs": 8, "publication_pairs": 8,
        "full_executions": 24, "prefix_executions": 2,
        "all_output_events": sum(item["events"] for item in report),
        "outlier_controls": outlier_controls,
        "odom_identity": True, "results": report,
        "synthetic_controls": control_outputs,
    })
    save(output/"artifacts.json", {
        str(path.relative_to(output)): digest(path)
        for path in sorted(output.rglob("*")) if path.is_file()
    })
    print(json.dumps({"output": str(output), "pairs": 16,
                      "events": sum(item["events"] for item in report)}))


if __name__ == "__main__":
    main()
