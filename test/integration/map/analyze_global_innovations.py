#!/usr/bin/env python3
"""Verify and summarize pre-update global innovations without truth claims."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys


REQUIRED_FIELDS = {
    "timestamp_ns", "sequence", "status", "message", "source",
    "innovation_available", "innovation_unavailable_reason",
    "observed_dimension", "pre_update_residual", "innovation_covariance",
    "nis",
}


def _matrix(value):
    shape, separator, body = value.partition(":")
    if not separator:
        raise ValueError("matrix encoding is missing ':'")
    dimensions = shape.split("x")
    if len(dimensions) != 2:
        raise ValueError("matrix shape must be rowsxcolumns")
    rows, columns = (int(item) for item in dimensions)
    values = [] if not body else [float(item) for item in body.split(";")]
    if rows < 0 or columns < 0 or len(values) != rows * columns:
        raise ValueError("matrix shape does not match encoded values")
    if not all(math.isfinite(item) for item in values):
        raise ValueError("matrix contains a non-finite value")
    return [values[row * columns:(row + 1) * columns] for row in range(rows)]


def _solve(matrix, vector):
    lower = _cholesky(matrix)
    forward = _forward_substitute(lower, vector)
    solved = [0.0] * len(vector)
    for row in range(len(vector) - 1, -1, -1):
        solved[row] = (
            forward[row] - sum(lower[column][row] * solved[column]
                               for column in range(row + 1, len(vector)))
        ) / lower[row][row]
    return solved


def _cholesky(matrix):
    size = len(matrix)
    if size == 0 or any(len(row) != size for row in matrix):
        raise ValueError("innovation dimensions do not match")
    scale = max(abs(value) for row in matrix for value in row)
    if not math.isfinite(scale) or scale == 0:
        raise ValueError("innovation covariance is singular")
    if any(not math.isclose(matrix[row][column], matrix[column][row],
                            rel_tol=1e-9, abs_tol=scale * 1e-12)
           for row in range(size) for column in range(size)):
        raise ValueError("innovation covariance is not symmetric")
    lower = [[0.0] * size for _ in range(size)]
    for row in range(size):
        for column in range(row + 1):
            value = matrix[row][column] - sum(
                lower[row][index] * lower[column][index]
                for index in range(column)
            )
            if row == column:
                if value <= scale * 1e-14:
                    raise ValueError("innovation covariance is not positive definite")
                lower[row][column] = math.sqrt(value)
            else:
                lower[row][column] = value / lower[column][column]
    return lower


def _forward_substitute(lower, vector):
    size = len(vector)
    if size == 0 or len(lower) != size or any(len(row) != size for row in lower):
        raise ValueError("innovation dimensions do not match")
    values = []
    for row in range(size):
        values.append(
            (vector[row] - sum(lower[row][column] * values[column]
                               for column in range(row))) / lower[row][row]
        )
    return values


def _autocorrelation(values, max_lag):
    if len(values) < 2:
        return []
    mean = statistics.fmean(values)
    denominator = sum((value - mean) ** 2 for value in values)
    result = []
    for lag in range(1, min(max_lag, len(values) - 1) + 1):
        numerator = sum(
            (values[index] - mean) * (values[index - lag] - mean)
            for index in range(lag, len(values))
        )
        result.append({
            "lag": lag,
            "autocorrelation": numerator / denominator if denominator else None,
        })
    return result


def _correlation(left, right):
    if len(left) != len(right) or len(left) < 2:
        return None
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    left_centered = [value - left_mean for value in left]
    right_centered = [value - right_mean for value in right]
    denominator = math.sqrt(
        sum(value * value for value in left_centered) *
        sum(value * value for value in right_centered)
    )
    if denominator == 0:
        return None
    return sum(a * b for a, b in zip(left_centered, right_centered)) / denominator


def _normal_quantile(probability):
    """Acklam's rational approximation to the standard-normal quantile."""
    if not 0 < probability < 1:
        raise ValueError("normal quantile probability must be between zero and one")
    a = (-3.969683028665376e1, 2.209460984245205e2,
         -2.759285104469687e2, 1.383577518672690e2,
         -3.066479806614716e1, 2.506628277459239)
    b = (-5.447609879822406e1, 1.615858368580409e2,
         -1.556989798598866e2, 6.680131188771972e1,
         -1.328068155288572e1)
    c = (-7.784894002430293e-3, -3.223964580411365e-1,
         -2.400758277161838, -2.549732539343734,
         4.374664141464968, 2.938163982698783)
    d = (7.784695709041462e-3, 3.224671290700398e-1,
         2.445134137142996, 3.754408661907416)
    tail = 0.02425
    if probability < tail:
        q = math.sqrt(-2 * math.log(probability))
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / (
            ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q) + 1
        )
    if probability > 1 - tail:
        return -_normal_quantile(1 - probability)
    q = probability - 0.5
    r = q * q
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / (
        ((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1
    )


def _quantile(values, probability):
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1 - fraction) + ordered[upper] * fraction


def _qq_points(values, maximum_points=41):
    if not values:
        return []
    count = min(len(values), maximum_points)
    return [
        {
            "theoretical_normal_quantile": _normal_quantile((index + 0.5) / count),
            "sample_whitened_quantile": _quantile(
                values, (index + 0.5) / count),
        }
        for index in range(count)
    ]


def _cross_source_correlations(samples, tolerance_ns):
    by_source = {}
    for sample in samples:
        by_source.setdefault(sample["source"], []).append(sample)
    for group in by_source.values():
        group.sort(key=lambda sample: (sample["timestamp_ns"], sample["sequence"]))

    results = []
    sources = sorted(by_source)
    for left_index, left_source in enumerate(sources):
        for right_source in sources[left_index + 1:]:
            left_groups = {}
            right_groups = {}
            for sample in by_source[left_source]:
                left_groups.setdefault(
                    (sample["status"], len(sample["residual"])), []).append(sample)
            for sample in by_source[right_source]:
                right_groups.setdefault(
                    (sample["status"], len(sample["residual"])), []).append(sample)
            for (left_status, dimension), left in sorted(left_groups.items()):
                for (right_status, right_dimension), right in sorted(right_groups.items()):
                    if dimension != right_dimension:
                        continue
                    used = set()
                    pairs = []
                    for first in left:
                        candidates = [
                            (abs(first["timestamp_ns"] - second["timestamp_ns"]), index, second)
                            for index, second in enumerate(right)
                            if index not in used and abs(
                                first["timestamp_ns"] - second["timestamp_ns"]
                            ) <= tolerance_ns
                        ]
                        if not candidates:
                            continue
                        distance, index, second = min(candidates, key=lambda item: (item[0], item[1]))
                        used.add(index)
                        pairs.append((first, second, distance))
                    if not pairs:
                        continue
                    if len({first["residual_basis"] for first, _, _ in pairs}) != 1 or not all(
                            first["residual_basis"] and
                            first["residual_basis"] == second["residual_basis"]
                            for first, second, _ in pairs):
                        results.append({
                            "left_source": left_source,
                            "right_source": right_source,
                            "left_status": left_status,
                            "right_status": right_status,
                            "observed_dimension": dimension,
                            "candidate_pair_count": len(pairs),
                            "available": False,
                            "reason": "missing_or_incompatible_residual_basis",
                        })
                        continue
                    results.append({
                        "left_source": left_source,
                        "right_source": right_source,
                        "left_status": left_status,
                        "right_status": right_status,
                        "observed_dimension": dimension,
                        "available": True,
                        "pair_count": len(pairs),
                        "timestamp_tolerance_ns": tolerance_ns,
                        "maximum_pair_offset_ns": max(pair[2] for pair in pairs),
                        "residual_basis": pairs[0][0]["residual_basis"],
                        "residual_component_correlation": [
                            _correlation(
                                [pair[0]["residual"][component] for pair in pairs],
                                [pair[1]["residual"][component] for pair in pairs],
                            )
                            for component in range(dimension)
                        ],
                    })
    return results


def analyze(path, max_lag=10, window_ns=0, pair_tolerance_ns=0):
    if max_lag < 1:
        raise ValueError("max_lag must be positive")
    if window_ns < 0 or pair_tolerance_ns < 0:
        raise ValueError("time windows and pairing tolerance must be non-negative")
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not REQUIRED_FIELDS.issubset(reader.fieldnames):
            missing = sorted(REQUIRED_FIELDS - set(reader.fieldnames or []))
            raise ValueError("innovation journal missing fields: " + ", ".join(missing))
        rows = list(reader)

    samples = []
    unavailable = {}
    gate_diagnostics = []
    for row in rows:
        if row["innovation_available"] not in ("0", "1"):
            raise ValueError("invalid innovation availability")
        if row["innovation_available"] != "1":
            key = row["innovation_unavailable_reason"] or "unspecified"
            unavailable[key] = unavailable.get(key, 0) + 1
            continue
        dimension = int(row["observed_dimension"])
        if dimension < 1:
            raise ValueError("observed dimension must be positive")
        residual_matrix = _matrix(row["pre_update_residual"])
        covariance = _matrix(row["innovation_covariance"])
        if len(residual_matrix) != dimension or any(
                len(values) != 1 for values in residual_matrix):
            raise ValueError("residual dimension disagrees with journal")
        residual = [values[0] for values in residual_matrix]
        if len(covariance) != dimension:
            raise ValueError("covariance dimension disagrees with journal")
        lower = _cholesky(covariance)
        whitened = _forward_substitute(lower, residual)
        solved = _solve(covariance, residual)
        recomputed = sum(left * right for left, right in zip(residual, solved))
        recorded = float(row["nis"])
        if not math.isfinite(recorded) or not math.isclose(
                recorded, recomputed, rel_tol=1e-7, abs_tol=1e-9):
            raise ValueError(
                f"NIS mismatch for decision {row.get('decision_id', '?')}")
        gate = None
        if row.get("gate_threshold", ""):
            gate = float(row["gate_threshold"])
            if not math.isfinite(gate) or gate <= 0:
                raise ValueError("available innovation has invalid gate threshold")
            if int(row["status"]) == 0 and recomputed > gate:
                raise ValueError("accepted innovation exceeds recorded gate")
        decomposition = None
        decomposition_flag = row.get("covariance_decomposition_available", "")
        if decomposition_flag not in ("", "0", "1"):
            raise ValueError("invalid covariance decomposition availability")
        if decomposition_flag == "1":
            decomposition = {
                name: _matrix(row[f"innovation_{name}_contribution"])
                for name in ("prior", "observation", "local")
            }
            if any(len(part) != dimension or any(len(r) != dimension for r in part)
                   for part in decomposition.values()):
                raise ValueError("innovation contribution dimension mismatch")
            scale = max(abs(value) for r in covariance for value in r)
            for i in range(dimension):
                for j in range(dimension):
                    if not math.isclose(
                            sum(part[i][j] for part in decomposition.values()),
                            covariance[i][j], rel_tol=1e-8, abs_tol=scale * 1e-10):
                        raise ValueError("innovation contributions do not sum to covariance")
        gate_diagnostics.append({
            "decision_id": row.get("decision_id"),
            "source": row["source"], "sequence": int(row["sequence"]),
            "timestamp_ns": int(row["timestamp_ns"]),
            "status": int(row["status"]), "nis": recomputed,
            "gate_threshold": gate,
            "gate_available": gate is not None,
            "nis_over_gate": recomputed / gate if gate is not None else None,
            "covariance_model": row.get("covariance_model") or "unavailable_legacy_journal",
            "residual_basis": row.get("residual_basis") or None,
            "residual": residual,
            "innovation_variance_diagonal": [covariance[i][i] for i in range(dimension)],
            "covariance_contribution_diagonals": (
                {name: [part[i][i] for i in range(dimension)]
                 for name, part in decomposition.items()}
                if decomposition is not None else None),
            "decomposition_status": ("available" if decomposition is not None else
                                     "unavailable_not_exported_for_this_model"),
        })
        samples.append({
            "source": row["source"],
            "status": int(row["status"]),
            "timestamp_ns": int(row["timestamp_ns"]),
            "sequence": int(row["sequence"]),
            "residual": residual,
            "whitened_residual": whitened,
            "innovation_sigma": [math.sqrt(covariance[i][i])
                                 for i in range(dimension)],
            # The prior frame alone does not identify a projected residual basis.
            "residual_basis": row.get("residual_basis", ""),
            "nis": recomputed,
        })

    grouped = {}
    for sample in samples:
        key = (sample["source"], sample["status"], len(sample["residual"]))
        grouped.setdefault(key, []).append(sample)

    summaries = []
    time_windows = []
    for (source, status, dimensions), group in sorted(grouped.items()):
        group.sort(key=lambda sample: (sample["timestamp_ns"], sample["sequence"]))
        summaries.append({
            "source": source,
            "status": status,
            "outcome": "accepted" if status == 0 else "rejected",
            "sample_count": len(group),
            "observed_dimension": dimensions,
            "nis_mean": statistics.fmean(sample["nis"] for sample in group),
            "nis_median": statistics.median(sample["nis"] for sample in group),
            "nis_p95": _quantile([sample["nis"] for sample in group], 0.95),
            "nis_p99": _quantile([sample["nis"] for sample in group], 0.99),
            "nis_max": max(sample["nis"] for sample in group),
            "nis_mean_per_dimension": statistics.fmean(
                sample["nis"] for sample in group) / dimensions,
            "innovation_three_sigma_component_median": [
                statistics.median(3 * sample["innovation_sigma"][i] for sample in group)
                for i in range(dimensions)
            ],
            "innovation_three_sigma_component_max": [
                max(3 * sample["innovation_sigma"][i] for sample in group)
                for i in range(dimensions)
            ],
            "uncertainty_scope": (
                "Innovation component scales in journal residual units, not "
                "posterior pose bounds or a calibrated 3-sigma coverage claim."
            ),
            "residual_autocorrelation": [
                _autocorrelation(
                    [sample["residual"][dimension] for sample in group], max_lag)
                for dimension in range(dimensions)
            ],
            "whitened_residual": [
                {
                    "mean": statistics.fmean(
                        sample["whitened_residual"][dimension] for sample in group),
                    "standard_deviation": statistics.pstdev(
                        sample["whitened_residual"][dimension] for sample in group),
                    "qq": _qq_points([
                        sample["whitened_residual"][dimension] for sample in group
                    ]),
                    "autocorrelation": _autocorrelation(
                        [sample["whitened_residual"][dimension] for sample in group],
                        max_lag,
                    ),
                }
                for dimension in range(dimensions)
            ],
        })
        if window_ns:
            windows = {}
            for sample in group:
                start = sample["timestamp_ns"] // window_ns * window_ns
                windows.setdefault(start, []).append(sample["nis"])
            for start, values in sorted(windows.items()):
                time_windows.append({
                    "source": source,
                    "status": status,
                    "observed_dimension": dimensions,
                    "window_start_ns": start,
                    "window_ns": window_ns,
                    "sample_count": len(values),
                    "nis_mean": statistics.fmean(values),
                    "nis_median": statistics.median(values),
                    "nis_p95": _quantile(values, 0.95),
                    "nis_p99": _quantile(values, 0.99),
                    "nis_max": max(values),
                })

    return {
        "decision_count": len(rows),
        "innovation_sample_count": len(samples),
        "unavailable_innovation_reasons": unavailable,
        "groups": summaries,
        "time_windows": time_windows,
        "cross_source_correlations": _cross_source_correlations(
            samples, pair_tolerance_ns),
        "cross_source_pairing_tolerance_ns": pair_tolerance_ns,
        "gate_diagnostics": gate_diagnostics,
        "interpretation": (
            "Pre-update NIS was recomputed from each logged residual and "
            "innovation covariance. Whitening and QQ summaries use the "
            "logged pre-update covariance. Accepted and rejected decisions "
            "are separate; gating/robust selection truncates these samples. "
            "Autocorrelation lags count selected samples, not fixed time steps. "
            "Cross-source pairs require an explicit common residual basis and "
            "use nearest one-to-one timestamps within the declared tolerance. "
            "Basis identifiers must describe frame, projection, component "
            "ordering and units; equal dimension alone is insufficient. "
            "All statistics are descriptive only; no "
            "truth, covariance calibration, or independence claim is implied."
        ),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("decisions", type=Path)
    parser.add_argument("--max-lag", type=int, default=10)
    parser.add_argument("--window-ns", type=int, default=0,
                        help="optional fixed measurement-time window for NIS summaries")
    parser.add_argument("--pair-tolerance-ns", type=int, default=0,
                        help="maximum source timestamp separation for one-to-one pairing")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    result = analyze(args.decisions, args.max_lag, args.window_ns,
                     args.pair_tolerance_ns)
    rendered = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(rendered)
    else:
        sys.stdout.write(rendered)


if __name__ == "__main__":
    main()
