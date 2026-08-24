#!/usr/bin/env python3
"""Offline evidence extractor for E2E01 and later controlled ROS 2 runs.

The script keeps the captured run immutable.  By default, derived files are
written to ~/semantic_ws/analysis/<run-id>.

Run after sourcing ROS 2 Jazzy and the semantic workspace:

    python3 e2e01_offline_analyzer.py RUN_DIR

Only Python's standard library and the ROS 2 Python packages are required.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import re
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Optional


ISO_LINE_RE = re.compile(r"^(?P<iso>[^|]+?)\s*\|\s*(?P<kind>[^|]+?)\s*\|\s*(?P<text>.*)$")
ROS_STAMP_RE = re.compile(r"\[(?P<stamp>\d+(?:\.\d+)?)\]")
CONTROLLER_RE = re.compile(
    r"Control loop missed its desired rate.*?Current loop rate is\s+"
    r"(?P<rate>\d+(?:\.\d+)?)\s*Hz",
    re.IGNORECASE,
)
PLANNER_RE = re.compile(
    r"Failed to create plan|failed to plan|compute_path_to_pose.*Aborting",
    re.IGNORECASE,
)
RECOVERY_RE = re.compile(
    r"Clearing entirely the (?:global|local) costmap|Running (?:wait|spin|backup)|"
    r"Received goal preemption|Begin navigating|goal (?:succeeded|reached)|"
    r"Passing new path to controller",
    re.IGNORECASE,
)
FRAME_RE = re.compile(
    r"Frame\s+(?P<frame>\d+):\s+localized=(?P<localized>\d+).*?"
    r"mapped=(?P<mapped>\d+),\s+fusion_input=(?P<fusion_input>\d+),\s+"
    r"candidates=(?P<candidates>\d+),\s+confirmed=(?P<confirmed>\d+).*?"
    r"sync_span=(?P<sync>[-+\d.eE]+)\s*ms,\s+"
    r"end_to_end=(?P<latency>[-+\d.eE]+)\s*ms"
)
GATE_RE = re.compile(r"FUSION_GATE total:\s+raw=(?P<raw>\d+),\s+outcomes=\[(?P<body>[^]]*)\]")
KEY_VALUE_RE = re.compile(r"(?P<key>[A-Za-z0-9_]+)=(?P<value>\d+)")
VMSTAT_TIME_RE = re.compile(r"(?P<date>\d{4}-\d{2}-\d{2})\s+(?P<time>\d{2}:\d{2}:\d{2})$")
HEALTH_HEADER_RE = re.compile(r"^=====\s+(?P<iso>.+?)\s+=====$")


def percentile(values: list[float], p: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * p
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def describe(values: Iterable[float]) -> dict[str, Optional[float] | int]:
    data = [float(value) for value in values]
    if not data:
        return {"count": 0, "mean": None, "median": None, "p95": None, "min": None, "max": None}
    return {
        "count": len(data),
        "mean": statistics.fmean(data),
        "median": statistics.median(data),
        "p95": percentile(data, 0.95),
        "min": min(data),
        "max": max(data),
    }


def parse_iso(text: str) -> datetime:
    return datetime.fromisoformat(text.strip())


def in_interval(stamp: Optional[float], start: Optional[float], end: Optional[float]) -> bool:
    if stamp is None or start is None or end is None:
        return False
    return start <= stamp <= end


def extract_stamp(line: str) -> Optional[float]:
    match = ROS_STAMP_RE.search(line)
    return float(match.group("stamp")) if match else None


@dataclass
class Timeline:
    explore_start_iso: Optional[str]
    explore_end_iso: Optional[str]
    explore_start_epoch: Optional[float]
    explore_end_epoch: Optional[float]
    explore_duration_s: Optional[float]
    duplicate_end_markers_ignored: int
    marks: list[dict[str, str]]


def parse_timeline(path: Path) -> Timeline:
    marks: list[dict[str, str]] = []
    starts: list[datetime] = []
    ends: list[datetime] = []
    if path.exists():
        for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = ISO_LINE_RE.match(raw)
            if not match:
                continue
            try:
                stamp = parse_iso(match.group("iso"))
            except ValueError:
                continue
            kind = match.group("kind").strip()
            text = match.group("text").strip()
            marks.append({"iso": stamp.isoformat(), "kind": kind, "text": text})
            if "EXPLORE_START" in text:
                starts.append(stamp)
            if "EXPLORE_END" in text:
                ends.append(stamp)

    start = starts[0] if starts else None
    eligible_ends = [value for value in ends if start is None or value >= start]
    end = eligible_ends[0] if eligible_ends else None
    duration = (end - start).total_seconds() if start and end else None
    return Timeline(
        explore_start_iso=start.isoformat() if start else None,
        explore_end_iso=end.isoformat() if end else None,
        explore_start_epoch=start.timestamp() if start else None,
        explore_end_epoch=end.timestamp() if end else None,
        explore_duration_s=duration,
        duplicate_end_markers_ignored=max(0, len(eligible_ends) - 1),
        marks=marks,
    )


def read_log_events(path: Path, pattern: re.Pattern[str]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    if not path.exists():
        return events
    for line_number, raw in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        if pattern.search(raw):
            events.append({"line": line_number, "stamp": extract_stamp(raw), "text": raw.strip()})
    return events


def analyze_navigation(path: Path, timeline: Timeline) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    controller_all: list[dict[str, Any]] = []
    planner_all = read_log_events(path, PLANNER_RE)
    recovery_all = read_log_events(path, RECOVERY_RE)
    if path.exists():
        for line_number, raw in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            match = CONTROLLER_RE.search(raw)
            if match:
                controller_all.append(
                    {
                        "line": line_number,
                        "stamp": extract_stamp(raw),
                        "reported_rate_hz": float(match.group("rate")),
                        "text": raw.strip(),
                    }
                )

    start = timeline.explore_start_epoch
    end = timeline.explore_end_epoch
    controller = [item for item in controller_all if in_interval(item["stamp"], start, end)]
    planner = [item for item in planner_all if in_interval(item["stamp"], start, end)]
    recovery = [item for item in recovery_all if in_interval(item["stamp"], start, end)]
    # Fall back to the complete exploration log only if timestamp filtering is
    # impossible; this is explicitly reported in the metrics.
    used_fallback = False
    if start is not None and end is not None and not controller and controller_all:
        stamps = [item["stamp"] for item in controller_all if item["stamp"] is not None]
        if not stamps or max(stamps) < start - 86400 or min(stamps) > end + 86400:
            controller = controller_all
            planner = planner_all
            recovery = recovery_all
            used_fallback = True

    # One path-planning failure normally emits multiple log lines.  Cluster
    # consecutive lines occurring within five seconds into one failure episode.
    episodes: list[dict[str, Any]] = []
    for item in planner:
        if not episodes or item["stamp"] is None or episodes[-1]["end_stamp"] is None or item["stamp"] - episodes[-1]["end_stamp"] > 5.0:
            episodes.append(
                {
                    "episode": len(episodes) + 1,
                    "start_stamp": item["stamp"],
                    "end_stamp": item["stamp"],
                    "failure_lines": 1,
                    "first_line": item["line"],
                    "last_line": item["line"],
                    "recovery_activity": [],
                }
            )
        else:
            episodes[-1]["end_stamp"] = item["stamp"]
            episodes[-1]["failure_lines"] += 1
            episodes[-1]["last_line"] = item["line"]

    for episode in episodes:
        if episode["start_stamp"] is None:
            continue
        window_end = (episode["end_stamp"] or episode["start_stamp"]) + 20.0
        episode["recovery_activity"] = [
            item["text"]
            for item in recovery
            if item["stamp"] is not None and episode["start_stamp"] <= item["stamp"] <= window_end
        ]

    duration_min = timeline.explore_duration_s / 60.0 if timeline.explore_duration_s else None
    metrics = {
        "log": str(path),
        "timestamp_filter_fallback_to_whole_log": used_fallback,
        "controller_warning_lines_whole_log": len(controller_all),
        "controller_warning_lines_exploration": len(controller),
        "controller_warnings_per_exploration_min": len(controller) / duration_min if duration_min else None,
        "reported_current_rate_hz": describe(item["reported_rate_hz"] for item in controller),
        "planner_failure_lines_whole_log": len(planner_all),
        "planner_failure_lines_exploration": len(planner),
        "planner_failure_episodes_exploration": len(episodes),
        "episodes_with_recovery_activity_within_20s": sum(bool(item["recovery_activity"]) for item in episodes),
        "recovery_activity_lines_exploration": len(recovery),
    }
    return metrics, episodes


def analyze_localizer(path: Path, timeline: Timeline) -> dict[str, Any]:
    frames_all: list[dict[str, Any]] = []
    gates: list[dict[str, Any]] = []
    if path.exists():
        for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
            frame_match = FRAME_RE.search(raw)
            if frame_match:
                record: dict[str, Any] = {key: int(frame_match.group(key)) for key in ("frame", "localized", "mapped", "fusion_input", "candidates", "confirmed")}
                record.update(
                    {
                        "stamp": extract_stamp(raw),
                        "sync_span_ms": float(frame_match.group("sync")),
                        "end_to_end_ms": float(frame_match.group("latency")),
                    }
                )
                frames_all.append(record)
            gate_match = GATE_RE.search(raw)
            if gate_match:
                gates.append(
                    {
                        "stamp": extract_stamp(raw),
                        "raw": int(gate_match.group("raw")),
                        "outcomes": {match.group("key"): int(match.group("value")) for match in KEY_VALUE_RE.finditer(gate_match.group("body"))},
                    }
                )

    start = timeline.explore_start_epoch
    end = timeline.explore_end_epoch
    frames = [item for item in frames_all if in_interval(item["stamp"], start, end)]
    if not frames:
        frames = frames_all
    # Frame 1 is model warm-up and is kept separate from steady-state latency.
    steady = [item for item in frames if item["frame"] != 1]
    eligible_gates = [
        item
        for item in gates
        if item["stamp"] is not None and (end is None or item["stamp"] <= end)
    ]
    gate = eligible_gates[-1] if eligible_gates else (gates[-1] if gates else None)
    return {
        "log": str(path),
        "logged_frame_summaries_whole_log": len(frames_all),
        "logged_frame_summaries_selected": len(frames),
        "selected_scope": "exploration" if frames and frames is not frames_all else "whole_node_log",
        "steady_state_end_to_end_ms": describe(item["end_to_end_ms"] for item in steady),
        "steady_state_sync_span_ms": describe(item["sync_span_ms"] for item in steady),
        "maximum_confirmed_objects_in_logged_frames": max((item["confirmed"] for item in frames), default=None),
        "last_gate_total_in_or_before_exploration_end": gate,
    }


def analyze_drift(path: Path) -> dict[str, Any]:
    rows: list[dict[str, str]] = []
    if path.exists():
        with path.open(newline="", encoding="utf-8", errors="replace") as handle:
            rows = list(csv.DictReader(handle))

    def numbers(column: str) -> list[float]:
        result: list[float] = []
        for row in rows:
            try:
                result.append(float(row[column]))
            except (KeyError, TypeError, ValueError):
                pass
        return result

    correction_m = numbers("correction_step_m")
    correction_deg = numbers("correction_step_deg")
    events = numbers("correction_event")
    return {
        "csv": str(path),
        "scope": "drift diagnostic node lifetime; not wall-clock-filtered",
        "samples": len(rows),
        "correction_events": sum(value != 0 for value in events),
        "max_correction_step_m": max(correction_m, default=None),
        "max_correction_step_deg": max(correction_deg, default=None),
        "cumulative_absolute_correction_m": sum(abs(value) for value in correction_m),
        "cumulative_absolute_correction_deg": sum(abs(value) for value in correction_deg),
    }


def analyze_vmstat(path: Path, timeline: Timeline) -> dict[str, Any]:
    samples_all: list[dict[str, Any]] = []
    if path.exists():
        for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = VMSTAT_TIME_RE.search(raw)
            if not match:
                continue
            prefix = raw[: match.start()].split()
            if len(prefix) < 17:
                continue
            try:
                # vmstat: r b swpd free buff cache si so bi bo in cs us sy id wa st
                us, sy, idle, wait, stolen = map(float, prefix[12:17])
                local_stamp = datetime.fromisoformat(f"{match.group('date')}T{match.group('time')}")
                if timeline.explore_start_iso:
                    local_stamp = local_stamp.replace(tzinfo=parse_iso(timeline.explore_start_iso).tzinfo)
                samples_all.append(
                    {
                        "stamp": local_stamp.timestamp(),
                        "cpu_busy_pct": 100.0 - idle,
                        "user_pct": us,
                        "system_pct": sy,
                        "iowait_pct": wait,
                        "stolen_pct": stolen,
                    }
                )
            except (ValueError, IndexError):
                continue
    samples = [item for item in samples_all if in_interval(item["stamp"], timeline.explore_start_epoch, timeline.explore_end_epoch)]
    return {
        "log": str(path),
        "samples_whole_capture": len(samples_all),
        "samples_exploration": len(samples),
        "cpu_busy_pct_exploration": describe(item["cpu_busy_pct"] for item in samples),
        "cpu_user_pct_exploration": describe(item["user_pct"] for item in samples),
        "cpu_system_pct_exploration": describe(item["system_pct"] for item in samples),
        "cpu_iowait_pct_exploration": describe(item["iowait_pct"] for item in samples),
    }


def analyze_health(path: Path, timeline: Timeline) -> dict[str, Any]:
    memory_samples: list[dict[str, Any]] = []
    process_cpu: dict[str, list[float]] = defaultdict(list)
    current_stamp: Optional[float] = None
    groups = {
        "yolo_localizer": "rgbd_yolo_localizer_node",
        "rtabmap": "/rtabmap_slam/rtabmap",
        "gazebo": "gz sim",
        "controller_server": "controller_server",
        "planner_server": "planner_server",
    }
    if path.exists():
        for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
            header = HEALTH_HEADER_RE.match(raw)
            if header:
                try:
                    current_stamp = parse_iso(header.group("iso")).timestamp()
                except ValueError:
                    current_stamp = None
                continue
            if current_stamp is None or not in_interval(current_stamp, timeline.explore_start_epoch, timeline.explore_end_epoch):
                continue
            if raw.startswith("Mem:"):
                fields = raw.split()
                if len(fields) >= 3:
                    try:
                        memory_samples.append({"stamp": current_stamp, "total_mib": float(fields[1]), "used_mib": float(fields[2])})
                    except ValueError:
                        pass
                continue
            fields = raw.split(maxsplit=6)
            if len(fields) < 7 or not fields[0].isdigit():
                continue
            try:
                cpu = float(fields[1])
            except ValueError:
                continue
            command = fields[6]
            for name, needle in groups.items():
                if needle in command:
                    process_cpu[name].append(cpu)
                    break
    return {
        "log": str(path),
        "memory_used_mib_exploration": describe(item["used_mib"] for item in memory_samples),
        "memory_used_pct_exploration": describe(100.0 * item["used_mib"] / item["total_mib"] for item in memory_samples if item["total_mib"]),
        "sampled_process_cpu_pct_exploration": {name: describe(values) for name, values in process_cpu.items()},
        "note": "ps CPU percentages are sampled cumulative process percentages, not per-second utilization.",
    }


def map_metrics(message: Any) -> dict[str, Any]:
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    data = [int(value) for value in message.data]
    known_indices = [index for index, value in enumerate(data) if value >= 0]
    free = sum(0 <= value < 50 for value in data)
    occupied = sum(value >= 50 for value in data)
    unknown = sum(value < 0 for value in data)
    result: dict[str, Any] = {
        "width_cells": width,
        "height_cells": height,
        "resolution_m_per_cell": resolution,
        "grid_area_m2": width * height * resolution * resolution,
        "known_cells": len(known_indices),
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "known_cell_ratio_within_published_grid": len(known_indices) / len(data) if data else None,
        "known_mapped_area_m2": len(known_indices) * resolution * resolution,
        "free_area_m2": free * resolution * resolution,
        "occupied_area_m2": occupied * resolution * resolution,
        "origin_x_m": float(message.info.origin.position.x),
        "origin_y_m": float(message.info.origin.position.y),
    }
    if known_indices:
        xs = [index % width for index in known_indices]
        ys = [index // width for index in known_indices]
        result.update(
            {
                "known_bbox_width_m": (max(xs) - min(xs) + 1) * resolution,
                "known_bbox_height_m": (max(ys) - min(ys) + 1) * resolution,
                "known_bbox_area_m2": (max(xs) - min(xs) + 1) * (max(ys) - min(ys) + 1) * resolution * resolution,
            }
        )
    return result


def write_map(message: Any, output_dir: Path, stem: str) -> dict[str, Any]:
    metrics = map_metrics(message)
    width = int(message.info.width)
    height = int(message.info.height)
    data = [int(value) for value in message.data]
    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        for column in range(width):
            value = data[row * width + column]
            if value < 0:
                pixels.append(205)
            elif value >= 65:
                pixels.append(0)
            elif value <= 25:
                pixels.append(254)
            else:
                pixels.append(max(0, min(254, round(254 - value * 254 / 100))))
    pgm_path = output_dir / f"{stem}.pgm"
    with pgm_path.open("wb") as handle:
        handle.write(f"P5\n# Derived from captured nav_msgs/OccupancyGrid\n{width} {height}\n255\n".encode("ascii"))
        handle.write(pixels)
    yaml_path = output_dir / f"{stem}.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                f"resolution: {float(message.info.resolution)}",
                f"origin: [{float(message.info.origin.position.x)}, {float(message.info.origin.position.y)}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "mode: trinary",
                "",
            ]
        ),
        encoding="utf-8",
    )
    metrics.update({"pgm": str(pgm_path), "yaml": str(yaml_path)})
    return metrics


def apply_marker_array(state: dict[tuple[str, int], Any], message: Any) -> None:
    for marker in message.markers:
        action = int(marker.action)
        key = (str(marker.ns), int(marker.id))
        if action == 3:  # DELETEALL
            state.clear()
        elif action == 2:  # DELETE
            state.pop(key, None)
        else:  # ADD/MODIFY
            state[key] = copy.deepcopy(marker)


def marker_rows(state: dict[tuple[str, int], Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for (namespace, marker_id), marker in sorted(state.items()):
        rows.append(
            {
                "namespace": namespace,
                "marker_id": marker_id,
                "marker_type": int(marker.type),
                "is_text_label": int(marker.type) == 9,
                "text": str(marker.text),
                "frame_id": str(marker.header.frame_id),
                "x_m": float(marker.pose.position.x),
                "y_m": float(marker.pose.position.y),
                "z_m": float(marker.pose.position.z),
                "scale_x": float(marker.scale.x),
                "scale_y": float(marker.scale.y),
                "scale_z": float(marker.scale.z),
            }
        )
    return rows


def write_marker_csv(rows: list[dict[str, Any]], path: Path) -> dict[str, Any]:
    fieldnames = list(rows[0]) if rows else ["namespace", "marker_id", "marker_type", "is_text_label", "text", "frame_id", "x_m", "y_m", "z_m", "scale_x", "scale_y", "scale_z"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    label_rows = [row for row in rows if row["is_text_label"] and row["text"]]
    return {
        "csv": str(path),
        "active_marker_primitives": len(rows),
        "active_text_labels": len(label_rows),
        "text_label_counts": dict(Counter(str(row["text"]) for row in label_rows)),
        "text_labels": label_rows,
    }


def analyze_bag(bag_dir: Path, output_dir: Path, timeline: Timeline) -> dict[str, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Run: set +u; source /opt/ros/jazzy/setup.bash; "
            "source ~/semantic_ws/install/setup.bash"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = {topic: get_message(topic_types[topic]) for topic in ("/map", "/semantic_map/markers") if topic in topic_types}
    final_map = None
    explore_map = None
    final_state: dict[tuple[str, int], Any] = {}
    explore_state: dict[tuple[str, int], Any] = {}
    count_topics = {"/map", "/semantic_map/markers", "/odom", "/cmd_vel", "/rtabmap/info"}
    counts: Counter[str] = Counter()
    exploration_counts: Counter[str] = Counter()
    start_ns = int(timeline.explore_start_epoch * 1e9) if timeline.explore_start_epoch else None
    cutoff_ns = int(timeline.explore_end_epoch * 1e9) if timeline.explore_end_epoch else None

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic in count_topics:
            counts[topic] += 1
            if start_ns is not None and cutoff_ns is not None and start_ns <= stamp_ns <= cutoff_ns:
                exploration_counts[topic] += 1
        if topic not in selected:
            continue
        message = deserialize_message(data, selected[topic])
        if topic == "/map":
            final_map = message
            if cutoff_ns is not None and stamp_ns <= cutoff_ns:
                explore_map = message
        elif topic == "/semantic_map/markers":
            apply_marker_array(final_state, message)
            if cutoff_ns is not None and stamp_ns <= cutoff_ns:
                apply_marker_array(explore_state, message)

    duration_min = timeline.explore_duration_s / 60.0 if timeline.explore_duration_s else None
    result: dict[str, Any] = {
        "bag_dir": str(bag_dir),
        "topic_message_counts_whole_recording": dict(counts),
        "topic_message_counts_exploration": dict(exploration_counts),
        "topic_message_rates_per_exploration_min": {
            topic: count / duration_min for topic, count in exploration_counts.items()
        }
        if duration_min
        else {},
    }
    if explore_map is not None:
        result["map_at_exploration_end"] = write_map(explore_map, output_dir, "map_at_exploration_end")
    if final_map is not None:
        result["map_at_recording_end"] = write_map(final_map, output_dir, "map_at_recording_end")
    result["markers_at_exploration_end"] = write_marker_csv(marker_rows(explore_state), output_dir / "markers_at_exploration_end.csv")
    result["markers_at_recording_end"] = write_marker_csv(marker_rows(final_state), output_dir / "markers_at_recording_end.csv")
    return result


def write_episodes(path: Path, episodes: list[dict[str, Any]]) -> None:
    fields = ["episode", "start_stamp", "end_stamp", "failure_lines", "first_line", "last_line", "recovery_activity_count", "recovery_activity"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for episode in episodes:
            writer.writerow(
                {
                    **{key: episode[key] for key in fields[:6]},
                    "recovery_activity_count": len(episode["recovery_activity"]),
                    "recovery_activity": " || ".join(episode["recovery_activity"]),
                }
            )


def write_comparison_row(path: Path, metrics: dict[str, Any]) -> None:
    timeline = metrics["timeline"]
    navigation = metrics["navigation"]
    localizer = metrics["localizer"]
    drift = metrics["drift"]
    bag = metrics.get("bag", {})
    map_end = bag.get("map_at_exploration_end", {})
    marker_end = bag.get("markers_at_exploration_end", {})
    rates = bag.get("topic_message_rates_per_exploration_min", {})
    latency = localizer.get("steady_state_end_to_end_ms", {})
    cpu = metrics["vmstat"].get("cpu_busy_pct_exploration", {})
    memory = metrics["system_health"].get("memory_used_mib_exploration", {})
    row = {
        "run_id": metrics["run_id"],
        "exploration_duration_s": timeline.get("explore_duration_s"),
        "known_mapped_area_m2": map_end.get("known_mapped_area_m2"),
        "known_cell_ratio_within_published_grid": map_end.get("known_cell_ratio_within_published_grid"),
        "active_text_labels": marker_end.get("active_text_labels"),
        "map_messages_per_min": rates.get("/map"),
        "rtabmap_info_messages_per_min": rates.get("/rtabmap/info"),
        "odom_messages_per_min": rates.get("/odom"),
        "cmd_vel_messages_per_min": rates.get("/cmd_vel"),
        "latency_median_ms": latency.get("median"),
        "latency_p95_ms": latency.get("p95"),
        "cpu_busy_median_pct": cpu.get("median"),
        "cpu_busy_p95_pct": cpu.get("p95"),
        "memory_used_median_mib": memory.get("median"),
        "memory_used_p95_mib": memory.get("p95"),
        "controller_warnings_per_min": navigation.get("controller_warnings_per_exploration_min"),
        "planner_failure_episodes": navigation.get("planner_failure_episodes_exploration"),
        "slam_correction_events": drift.get("correction_events"),
        "max_slam_correction_m": drift.get("max_correction_step_m"),
        "max_slam_correction_deg": drift.get("max_correction_step_deg"),
    }
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_summary(path: Path, run_id: str, metrics: dict[str, Any]) -> None:
    timeline = metrics["timeline"]
    nav = metrics["navigation"]
    localizer = metrics["localizer"]
    drift = metrics["drift"]
    vmstat = metrics["vmstat"]
    bag = metrics.get("bag", {})
    map_end = bag.get("map_at_exploration_end", {})
    markers_end = bag.get("markers_at_exploration_end", {})
    latency = localizer["steady_state_end_to_end_ms"]
    cpu = vmstat["cpu_busy_pct_exploration"]
    gate = localizer.get("last_gate_total_in_or_before_exploration_end") or {}
    gate_outcomes = gate.get("outcomes", {})
    gate_raw = gate.get("raw")
    accepted = gate_outcomes.get("accepted")
    accepted_pct = 100.0 * accepted / gate_raw if accepted is not None and gate_raw else None
    labels = markers_end.get("text_label_counts", {})

    lines = [
        f"# {run_id} 离线定量摘要",
        "",
        "## 1. 分析范围",
        "",
        f"- 权威探索区间：{timeline['explore_start_iso'] or 'N/A'} 至 {timeline['explore_end_iso'] or 'N/A'}。",
        f"- 探索时长：{fmt(timeline['explore_duration_s'])} s（{fmt((timeline['explore_duration_s'] or 0) / 60)} min）。",
        f"- 忽略探索结束重复标记：{timeline['duplicate_end_markers_ignored']} 条。",
        "- 地图和 Marker 同时给出“探索结束时”与“录包结束时”快照，论文主结果应采用前者。",
        "",
        "## 2. 主要结果",
        "",
        "| 指标 | 结果 | 解释边界 |",
        "|---|---:|---|",
        f"| 已知地图面积 | {fmt(map_end.get('known_mapped_area_m2'))} m² | 最终 OccupancyGrid 的 known cells；不是带真值的环境覆盖率 |",
        f"| published grid 内 known-cell ratio | {fmt(map_end.get('known_cell_ratio_within_published_grid') * 100 if map_end.get('known_cell_ratio_within_published_grid') is not None else None)}% | 会受动态地图边界影响，只作覆盖代理量 |",
        f"| 探索结束 active text labels | {fmt(markers_end.get('active_text_labels'))} | Marker 状态重放结果，不等于检测 precision/recall |",
        f"| 语义标签 | {', '.join(f'{key}: {value}' for key, value in labels.items()) or 'N/A'} | 标签文本与坐标见 CSV |",
        f"| 融合 accepted | {fmt(accepted)}/{fmt(gate_raw)} ({fmt(accepted_pct)}%) | 截止探索结束前最后一条累计 FUSION_GATE |",
        f"| 稳态端到端延迟中位数 / P95 | {fmt(latency.get('median'))} / {fmt(latency.get('p95'))} ms | 排除 Frame 1 模型预热，仅统计已记录 Frame 摘要 |",
        f"| CPU busy 中位数 / P95 | {fmt(cpu.get('median'))}% / {fmt(cpu.get('p95'))}% | vmstat 全机 CPU，探索时间窗 |",
        f"| controller timing warning | {nav['controller_warning_lines_exploration']} ({fmt(nav['controller_warnings_per_exploration_min'])}/min) | 警告行数，不代表同等数量的独立故障 |",
        f"| planner failure lines / episodes | {nav['planner_failure_lines_exploration']} / {nav['planner_failure_episodes_exploration']} | 5 s 内相邻失败行合并为一 episode |",
        f"| 20 s 内含恢复活动的 episodes | {nav['episodes_with_recovery_activity_within_20s']} | 表示日志中出现恢复/重新规划活动，不等于已证明目标成功 |",
        f"| SLAM correction events | {drift['correction_events']} | 仅指诊断阈值以上事件；不能写成“没有 loop closure” |",
        "",
        "## 3. 可直接写入论文的谨慎结论",
        "",
        "- 本轮证明 SLAM、自动探索、RGB-D/YOLO 定位、语义融合和证据采集能够在同一端到端运行中并行工作。",
        "- 自动探索出现 planner failure 与 controller timing warning，但日志显示系统仍执行重新规划/恢复活动；独立 episode 数比原始警告行数更适合作为鲁棒性指标。",
        "- MarkerArray 的 RViz lost-message 警告属于显示链路异常；录包中的 Marker 消息数和重放后的最终状态才是核心数据证据。",
        "- correction event 为 0 只说明未超过当前 0.15 m / 10° 诊断阈值，不能据此否定小幅修正或 loop closure。",
        "",
        "## 4. 尚不能由本轮单独支持的结论",
        "",
        "- 不能把 known-cell ratio 表述为带 Gazebo ground truth 的绝对覆盖率。",
        "- 不能由 active labels 推导 precision、recall 或位置误差；这些需要固定真值标注。",
        "- 不能由一次运行证明跨运行一致性，也不能证明 YOLO 对 SLAM/Nav2 性能的因果影响。",
        "- 不能根据 0 个 correction events 写成没有回环或没有 SLAM 漂移。",
        "",
        "## 5. 生成文件",
        "",
        "- `e2e01_metrics.json`：全部机器可读指标。",
        "- `planner_failure_episodes.csv`：去重后的规划失败 episode。",
        "- `comparison_row.csv`：用于 SLAM-only 与 SLAM+YOLO 对照的单行汇总。",
        "- `map_at_exploration_end.pgm/.yaml`：探索结束时重建地图。",
        "- `markers_at_exploration_end.csv`：探索结束时有效 Marker 状态。",
        "- `map_at_recording_end.*`、`markers_at_recording_end.csv`：用于核查清理阶段是否改变状态。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Captured run directory")
    parser.add_argument("--output", type=Path, help="Derived-output directory; default: <workspace>/analysis/<run-id>")
    parser.add_argument("--skip-bag", action="store_true", help="Analyze logs/CSV only (useful when ROS bag Python modules are unavailable)")
    args = parser.parse_args()

    run_dir = args.run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        parser.error(f"run directory does not exist: {run_dir}")
    output_dir = args.output.expanduser().resolve() if args.output else run_dir.parent.parent / "analysis" / run_dir.name
    if output_dir == run_dir or run_dir in output_dir.parents:
        parser.error("output must be outside the captured run directory so the raw run remains immutable")
    output_dir.mkdir(parents=True, exist_ok=True)

    timeline = parse_timeline(run_dir / "timeline.txt")
    nav, episodes = analyze_navigation(run_dir / "04_autonomous_exploration_console.log", timeline)
    metrics: dict[str, Any] = {
        "schema_version": 1,
        "run_id": run_dir.name,
        "source_run_dir": str(run_dir),
        "analysis_output_dir": str(output_dir),
        "timeline": asdict(timeline),
        "navigation": nav,
        "localizer": analyze_localizer(run_dir / "03_localizer_console.log", timeline),
        "drift": analyze_drift(run_dir / "slam_drift.csv"),
        "vmstat": analyze_vmstat(run_dir / "vmstat.log", timeline),
        "system_health": analyze_health(run_dir / "system_health.log", timeline),
        "interpretation_constraints": [
            "known-cell ratio is a coverage proxy, not ground-truth environment coverage",
            "zero thresholded correction events does not prove zero loop closures or zero drift",
            "RViz lost messages do not imply rosbag loss",
            "single-run metrics do not establish repeatability or causality",
        ],
    }
    if not args.skip_bag:
        bag_dir = run_dir / "rosbag_core"
        if not bag_dir.is_dir():
            raise RuntimeError(f"rosbag directory not found: {bag_dir}")
        metrics["bag"] = analyze_bag(bag_dir, output_dir, timeline)

    write_episodes(output_dir / "planner_failure_episodes.csv", episodes)
    write_comparison_row(output_dir / "comparison_row.csv", metrics)
    metrics_path = output_dir / "e2e01_metrics.json"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_summary(output_dir / "e2e01_summary.md", run_dir.name, metrics)
    print(f"Analysis complete: {output_dir}")
    print(f"Summary: {output_dir / 'e2e01_summary.md'}")
    print(f"Metrics: {metrics_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
