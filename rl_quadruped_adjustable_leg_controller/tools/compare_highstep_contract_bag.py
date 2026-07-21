#!/usr/bin/env python3
"""Compare one strict MuJoCo policy2 contract bag with the approved B300 trace."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


JOINTS = [
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    "FL_box_joint", "FR_box_joint", "RL_box_joint", "RR_box_joint",
]


def vector(row: dict[str, str], prefix: str) -> list[float]:
    return [float(row[f"{prefix}.{name}"]) for name in JOINTS]


def errors(actual: list[list[float]], expected: list[list[float]]) -> dict[str, object]:
    per_joint = [[] for _ in JOINTS]
    for left, right in zip(actual, expected, strict=True):
        for index, (a, b) in enumerate(zip(left, right, strict=True)):
            per_joint[index].append(abs(a - b))
    flat = [value for values in per_joint for value in values]
    return {
        "mae": sum(flat) / len(flat),
        "max_abs": max(flat),
        "per_joint_mae": {
            name: sum(values) / len(values) for name, values in zip(JOINTS, per_joint, strict=True)
        },
    }


def closest(samples: list[tuple[int, list[float]]], timestamp: int) -> list[float]:
    times = [item[0] for item in samples]
    index = bisect.bisect_left(times, timestamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda item: abs(item[0] - timestamp))[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--reference",
        type=Path,
        required=True,
        help="approved 138-row Teacher joint_trace.csv",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.bag / "highstep_contract_comparison.json"

    with args.reference.open(newline="", encoding="utf-8") as handle:
        reference = list(csv.DictReader(handle))
    if len(reference) != 138:
        raise RuntimeError(f"approved reference must contain 138 rows, got {len(reference)}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {"/rl_policy2_contract_trace", "/joint_states", "/mujoco_applied_actuators_cmds"}
    missing = sorted(required - topic_types.keys())
    if missing:
        raise RuntimeError(f"bag is missing required topics: {missing}")
    message_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    traces: dict[int, tuple[int, list[float]]] = {}
    joints: list[tuple[int, list[float]]] = []
    applied: list[tuple[int, list[float]]] = []
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(payload, message_types[topic])
        if topic == "/rl_policy2_contract_trace":
            data = list(message.data)
            if len(data) != 56 or int(round(data[0])) != 1:
                raise RuntimeError(f"unexpected policy2 trace schema/length: {len(data)}")
            step = int(round(data[3]))
            if 0 <= step < 138:
                if step in traces:
                    raise RuntimeError(f"duplicate diagnostic step: {step}")
                traces[step] = (timestamp, data)
        elif topic == "/joint_states":
            by_name = {name: value for name, value in zip(message.name, message.position)}
            if all(name in by_name for name in JOINTS):
                joints.append((timestamp, [float(by_name[name]) for name in JOINTS]))
        else:
            by_name = {name: value for name, value in zip(message.actuators_name, message.pos)}
            if all(name in by_name for name in JOINTS):
                applied.append((timestamp, [float(by_name[name]) for name in JOINTS]))

    expected_steps = set(range(138))
    if set(traces) != expected_steps:
        missing_steps = sorted(expected_steps - set(traces))
        raise RuntimeError(
            f"diagnostic trace is incomplete; missing={missing_steps}"
        )
    if not joints or not applied:
        raise RuntimeError("bag has no complete 16-joint measured/applied samples")

    ordered = [traces[index] for index in range(138)]
    data = [item[1] for item in ordered]
    model_vx = [row[5] for row in data]
    expected_vx = [0.7200000286 if 21 <= index < 80 else 0.0 for index in range(138)]
    raw_output = [row[8:24] for row in data]
    prior_target = [row[24:40] for row in data]
    physical_target = [row[40:56] for row in data]
    measured_q = [closest(joints, timestamp) for timestamp, _ in ordered]
    applied_q = [closest(applied, timestamp) for timestamp, _ in ordered]

    report = {
        "schema_version": 1,
        "bag": str(args.bag.resolve()),
        "reference": str(args.reference.resolve()),
        "sample_count": 138,
        "schedule": {
            "exact": all(abs(a - b) <= 1.0e-6 for a, b in zip(model_vx, expected_vx, strict=True)),
            "model_vx": model_vx,
            "terminal_phase": data[-1][6],
            "terminal_model_vx": model_vx[-1],
        },
        "policy_raw_vs_approved": errors(
            raw_output,
            [vector(row, "policy_raw") for row in reference],
        ),
        "post_prior_target_vs_approved": errors(
            prior_target, [vector(row, "mapped_target") for row in reference]
        ),
        "physical_clamp_target_vs_approved": errors(
            physical_target, [vector(row, "mapped_target") for row in reference]
        ),
        "mujoco_applied_target_vs_approved": errors(
            applied_q, [vector(row, "mapped_target") for row in reference]
        ),
        "measured_joint_pos_vs_approved": errors(
            measured_q, [vector(row, "joint_pos") for row in reference]
        ),
        "finite": all(math.isfinite(value) for row in data for value in row),
    }
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "schedule_exact": report["schedule"]["exact"],
        "terminal_phase": report["schedule"]["terminal_phase"],
        "policy_raw_mae": report["policy_raw_vs_approved"]["mae"],
        "post_prior_target_mae": report["post_prior_target_vs_approved"]["mae"],
        "measured_joint_pos_mae": report["measured_joint_pos_vs_approved"]["mae"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
