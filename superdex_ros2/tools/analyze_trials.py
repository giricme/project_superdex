#!/usr/bin/env python3
# Copyright (c) 2026 Team 6. Licensed under the Apache License, Version 2.0.
"""Compute every reported metric from a recorded trial session.

Nothing here runs live. The trial runner publishes targets and records when
each trial started and finished; this reads that index plus the rosbag and
derives the numbers, so a metric can be redefined -- a different tolerance, a
different settle rule -- without re-running the simulation.

    python3 tools/analyze_trials.py trials/20260928-155846

Reads ``trials.json`` and ``bag/`` from that directory and writes:

    metrics_trials.csv     one row per trial
    metrics_residual.csv   forward-kinematics residual time series
    summary.txt            the numbers to quote

Three groups of metrics, each supporting a different claim:

**Correctness.** The forward-kinematics residual between ``sim_<link>`` (the
engine's own transform, republished by the bridge) and ``<link>``
(``robot_state_publisher``'s, computed from the generated URDF and the
published joint states). These are two independent computations of the same
pose, so agreement tests the URDF conversion, the DOF-to-joint-name mapping and
the message path at once. Reported as max and p99, not just a final value.

**Task.** Final position and orientation error, settle time, path deviation
from the straight line, and success rate -- with success rate treated with
suspicion, since it collapses a continuous error into a threshold. If the error
distribution straddles the tolerance, the success rate measures the threshold
rather than the arm.

**Systems.** Command latency (``/target_pose`` to the first ``/ee_target``
change) and ``/joint_states`` interarrival statistics, which is the only
measurement of what the middleware actually costs end to end.

Requires ``rosbag2_py`` and ``tf2_ros``; run it with the venv interpreter that
can see both ROS and SuperDex, as ``.venv-ros/bin/python``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer
    from rclpy.duration import Duration
    from rclpy.time import Time
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        f"{exc}. Run with the venv interpreter, e.g.\n"
        "  .venv-ros/bin/python tools/analyze_trials.py <dir>"
    ) from exc

# The end-effector link OSC controls, and the frame the bridge republishes it
# as. Everything downstream compares this pair.
EE_LINK = "fr3_link8"
SIM_PREFIX = "sim_"

# The residual is sampled rather than computed at every /tf message: 4000-odd
# TF messages is more resolution than a max-and-p99 needs, and each lookup
# walks the URDF chain.
RESIDUAL_SAMPLE_HZ = 20.0


def stamp_to_sec(stamp) -> float:
    """Convert a ROS time message to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def quat_angle(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Angle in radians between two (x, y, z, w) orientations."""
    dot = abs(float(np.dot(q_a, q_b)))
    return 2.0 * math.acos(min(1.0, dot))


def pose_arrays(msg) -> tuple[np.ndarray, np.ndarray]:
    """Extract (position, orientation) from a PoseStamped."""
    p, o = msg.pose.position, msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])


@dataclass
class Series:
    """A time-ordered pose stream read out of the bag."""

    t: list[float]
    position: list[np.ndarray]
    orientation: list[np.ndarray]

    def slice(self, t0: float, t1: float) -> tuple[np.ndarray, np.ndarray]:
        """Return (times, positions) within a closed time window."""
        idx = [i for i, t in enumerate(self.t) if t0 <= t <= t1]
        if not idx:
            return np.empty(0), np.empty((0, 3))
        return (
            np.array([self.t[i] for i in idx]),
            np.array([self.position[i] for i in idx]),
        )

    def at(self, t_query: float) -> tuple[np.ndarray, np.ndarray] | None:
        """Nearest sample to a time, or None if the series is empty."""
        if not self.t:
            return None
        i = int(np.argmin(np.abs(np.array(self.t) - t_query)))
        return self.position[i], self.orientation[i]


def read_bag(bag_dir: Path) -> tuple[dict[str, Series], Buffer, list[float], list[float]]:
    """Read the bag once, returning pose series, a TF buffer and stamp lists."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    series = {
        name: Series([], [], []) for name in ("/ee_pose", "/ee_target", "/target_pose")
    }
    # A cache long enough to hold the whole session, so lookups never fall off
    # the back of the buffer during a long run.
    tf_buffer = Buffer(cache_time=Duration(seconds=3600))
    joint_stamps: list[float] = []
    target_stamps: list[float] = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))

        if topic in series:
            position, orientation = pose_arrays(msg)
            s = series[topic]
            s.t.append(stamp_to_sec(msg.header.stamp))
            s.position.append(position)
            s.orientation.append(orientation)
            if topic == "/target_pose":
                target_stamps.append(stamp_to_sec(msg.header.stamp))
        elif topic == "/joint_states":
            joint_stamps.append(stamp_to_sec(msg.header.stamp))
        elif topic in ("/tf", "/tf_static"):
            static = topic == "/tf_static"
            for transform in msg.transforms:
                # "bag" is the authority name tf2 records for conflict
                # reporting; it is arbitrary but must be consistent.
                tf_buffer.set_transform_static(transform, "bag") if static else (
                    tf_buffer.set_transform(transform, "bag")
                )

    return series, tf_buffer, joint_stamps, target_stamps


def fk_residual(
    tf_buffer: Buffer, windows: list[tuple[float, float]]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample the sim-vs-URDF transform for the end-effector link.

    Looking the two frames up against each other directly gives the residual as
    a single transform: identity means they agree.

    Sampled only inside the supplied windows, which are the settled portions of
    each trial. The two trees publish at different rates -- 50 Hz for the
    engine's frames, 200 Hz for robot_state_publisher's -- so tf2 interpolates
    across up to 20 ms. While the arm is moving that interpolation alone is
    worth hundreds of micrometres, which measures the rate difference rather
    than the conversion. With the arm at rest it vanishes, leaving the quantity
    actually of interest.
    """
    step = 1.0 / RESIDUAL_SAMPLE_HZ
    times, positions, angles = [], [], []

    for t_start, t_end in windows:
        t = t_start
        while t <= t_end:
            try:
                tf = tf_buffer.lookup_transform(
                    f"{SIM_PREFIX}{EE_LINK}", EE_LINK, Time(seconds=t)
                )
            except Exception:
                # Early samples predate robot_state_publisher's first output,
                # and the last may fall past the final /tf message. Skipping is
                # correct; a failed lookup is not a residual of zero.
                t += step
                continue
            tr = tf.transform.translation
            ro = tf.transform.rotation
            times.append(t)
            positions.append(math.sqrt(tr.x**2 + tr.y**2 + tr.z**2))
            angles.append(quat_angle(np.array([ro.x, ro.y, ro.z, ro.w]),
                                     np.array([0.0, 0.0, 0.0, 1.0])))
            t += step

    return np.array(times), np.array(positions), np.array(angles)


def path_deviation(points: np.ndarray, start: np.ndarray, target: np.ndarray) -> float:
    """Maximum perpendicular distance from the straight line start -> target.

    Impedance control takes whatever path the dynamics produce, with no notion
    of a planned trajectory. This quantifies how far that is from the direct
    route, which is the baseline a planner would later be compared against.
    """
    if len(points) == 0:
        return float("nan")
    direction = target - start
    length = np.linalg.norm(direction)
    if length < 1e-9:
        return 0.0
    direction = direction / length
    offsets = points - start
    along = offsets @ direction
    perpendicular = offsets - np.outer(along, direction)
    return float(np.max(np.linalg.norm(perpendicular, axis=1)))


def command_latency(
    ee_target: Series,
    t_command: float,
    t_limit: float,
    tolerance: float = 1e-4,
) -> float:
    """Seconds from a target being published to /ee_target reflecting it.

    The baseline must come from strictly BEFORE the command. Taking the nearest
    sample instead picks up one that may already carry the new target -- the
    node publishes /ee_target every step and /target_pose lands between two of
    them -- in which case no change is ever detected and the search runs on
    into the next trial, reporting seconds instead of milliseconds.

    Bounded by the trial window for the same reason: a latency that cannot be
    located within its own trial is missing, not large.
    """
    earlier = [i for i, t in enumerate(ee_target.t) if t < t_command]
    if not earlier:
        return float("nan")
    reference = ee_target.position[earlier[-1]]
    for t, position in zip(ee_target.t, ee_target.position):
        if t < t_command:
            continue
        if t > t_limit:
            break
        if np.linalg.norm(position - reference) > tolerance:
            return t - t_command
    return float("nan")


def describe(label: str, values: np.ndarray, scale: float, unit: str) -> str:
    """One summary line: mean, standard deviation, range.

    NaN-aware throughout. A metric that is undefined for one trial -- a window
    with no samples, a latency that could not be located -- must not turn the
    whole row into NaN and hide the nineteen trials that did work. When any
    sample is missing the count is shown, so a silently thin row is visible.
    """
    values = np.asarray(values, dtype=float)
    valid = values[~np.isnan(values)]
    if valid.size == 0:
        return f"  {label:34s} no samples"
    v = valid * scale
    missing = values.size - valid.size
    suffix = f"  [{valid.size}/{values.size}]" if missing else ""
    return (
        f"  {label:34s} mean {v.mean():7.3f}  sd {v.std():6.3f}  "
        f"min {v.min():7.3f}  max {v.max():7.3f}  {unit}{suffix}"
    )


def main() -> int:
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session", type=Path, help="Directory holding trials.json and bag/")
    args = parser.parse_args()

    index_path = args.session / "trials.json"
    bag_dir = args.session / "bag"
    if not index_path.is_file():
        parser.error(f"{index_path} not found")
    if not bag_dir.is_dir():
        parser.error(f"{bag_dir} not found -- was the recorder running?")

    index = json.loads(index_path.read_text())
    trials = index["trials"]
    tolerance = index["tolerance"]

    print(f"Reading {bag_dir} ...")
    series, tf_buffer, joint_stamps, target_stamps = read_bag(bag_dir)
    ee_pose, ee_target = series["/ee_pose"], series["/ee_target"]
    if not ee_pose.t:
        parser.error("no /ee_pose messages in the bag")

    if len(target_stamps) != len(trials):
        print(
            f"  warning: {len(target_stamps)} targets in the bag but "
            f"{len(trials)} trials in the index -- are they from the same run?",
            file=sys.stderr,
        )

    # -- per-trial metrics --------------------------------------------------
    rows = []
    for trial in trials:
        target = np.array(trial["target"])
        target_orientation = np.array(trial["orientation"])
        start = np.array(trial["start_position"])
        t_cmd, t_end = trial["t_command"], trial["t_end"]

        # Average the last 0.2 s rather than taking a single sample: the
        # steady state is a spring-damper at rest, not a fixed point.
        _, tail = ee_pose.slice(max(t_cmd, t_end - 0.2), t_end)
        final = tail.mean(axis=0) if len(tail) else np.array(trial["final_position"])
        final_orientation = ee_pose.at(t_end)[1]

        _, path = ee_pose.slice(t_cmd, t_end)
        rows.append({
            "index": trial["index"],
            "distance_m": trial["distance"],
            "settled": int(trial["settled"]),
            "settle_time_s": (
                trial["t_settle"] - t_cmd if trial["settled"] else float("nan")
            ),
            "position_error_m": float(np.linalg.norm(target - final)),
            "orientation_error_rad": quat_angle(target_orientation, final_orientation),
            "path_deviation_m": path_deviation(path, start, target),
            "command_latency_s": command_latency(ee_target, t_cmd, t_end),
        })

    # -- correctness --------------------------------------------------------
    # Settled windows only: from the settle time to the end of the trial, where
    # the arm is stationary and the two publish rates cannot alias.
    settled_windows = [
        (t["t_settle"], t["t_end"]) for t in trials if t["settled"]
    ]
    t_res, res_position, res_angle = fk_residual(tf_buffer, settled_windows)

    # -- systems ------------------------------------------------------------
    gaps = np.diff(np.array(joint_stamps)) if len(joint_stamps) > 1 else np.empty(0)

    # -- write --------------------------------------------------------------
    trials_csv = args.session / "metrics_trials.csv"
    with trials_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    residual_csv = args.session / "metrics_residual.csv"
    with residual_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "position_residual_m", "rotation_residual_rad"])
        writer.writerows(zip(t_res, res_position, res_angle))

    # -- summarize ----------------------------------------------------------
    position_error = np.array([r["position_error_m"] for r in rows])
    settle_times = np.array(
        [r["settle_time_s"] for r in rows if not math.isnan(r["settle_time_s"])]
    )
    settled = int(sum(r["settled"] for r in rows))

    lines = [
        f"Session: {args.session}",
        f"Trials:  {len(rows)}   tolerance: {tolerance * 1000:.1f} mm",
        "",
        "CORRECTNESS -- bridge vs. URDF forward kinematics (settled windows)",
    ]
    if res_position.size:
        lines += [
            f"  samples                            {res_position.size}",
            f"  position residual   max {res_position.max() * 1e6:8.2f} um"
            f"   p99 {np.percentile(res_position, 99) * 1e6:8.2f} um",
            f"  rotation residual   max {math.degrees(res_angle.max()) * 1e3:8.2f} mdeg"
            f"   p99 {math.degrees(np.percentile(res_angle, 99)) * 1e3:8.2f} mdeg",
        ]
    else:
        lines.append("  no successful lookups -- is /tf_static in the bag?")

    lines += [
        "",
        "TASK -- Cartesian waypoint reaching",
        describe("final position error", position_error, 1e3, "mm"),
        describe(
            "final orientation error",
            np.array([r["orientation_error_rad"] for r in rows]),
            180.0 / math.pi,
            "deg",
        ),
        describe("settle time (settled only)", settle_times, 1.0, "s"),
        describe(
            "path deviation from straight line",
            np.array([r["path_deviation_m"] for r in rows]),
            1e3,
            "mm",
        ),
        f"  settled within tolerance           {settled}/{len(rows)}",
    ]

    # The success rate is only meaningful if the tolerance sits outside the
    # error distribution. Say so when it does not, rather than reporting a
    # number that is really a property of the threshold.
    if position_error.min() < tolerance < position_error.max():
        lines += [
            "",
            f"  NOTE: the tolerance ({tolerance * 1000:.1f} mm) falls INSIDE the error",
            f"  distribution ({position_error.min() * 1000:.1f}-"
            f"{position_error.max() * 1000:.1f} mm). Every trial converged to about the",
            "  same place, so the success rate reflects where the threshold was",
            "  drawn, not whether the arm reached. Report the error distribution;",
            "  set the tolerance above the observed floor if a rate is needed.",
        ]

    correlation = (
        np.corrcoef(
            position_error, np.array([r["orientation_error_rad"] for r in rows])
        )[0, 1]
        if len(rows) > 2
        else float("nan")
    )
    lines += [
        "",
        "  position vs. distance    r = "
        f"{np.corrcoef(position_error, np.array([r['distance_m'] for r in rows]))[0, 1]:+.3f}",
        f"  position vs. orientation r = {correlation:+.3f}",
        "    (near zero rules out the arm trading position error against the",
        "     pinned orientation; a distance correlation instead points at the",
        "     controller's error clamp throttling the approach.)",
        "",
        "SYSTEMS",
        describe(
            "command latency",
            np.array(
                [r["command_latency_s"] for r in rows if not math.isnan(r["command_latency_s"])]
            ),
            1e3,
            "ms",
        ),
        describe("/joint_states interarrival", gaps, 1e3, "ms"),
        f"  /joint_states messages             {len(joint_stamps)}",
    ]

    summary = "\n".join(lines)
    (args.session / "summary.txt").write_text(summary + "\n")
    print()
    print(summary)
    print()
    print(f"Wrote {trials_csv}")
    print(f"Wrote {residual_csv}")
    print(f"Wrote {args.session / 'summary.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())