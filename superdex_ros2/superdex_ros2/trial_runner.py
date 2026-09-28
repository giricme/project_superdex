#!/usr/bin/env python3
# Copyright (c) 2026 Team 6. Licensed under the Apache License, Version 2.0.
"""Drive N Cartesian waypoint-reaching trials and record when each one happened.

This node measures nothing. It publishes targets, decides when each trial has
settled or timed out, and writes a small JSON index of trial windows. Every
metric is computed offline from a rosbag recorded over the same run, so the
numbers can be recomputed -- with a different tolerance, a different definition
of settling -- without re-running the simulation.

Run it through ``trials.launch.py``, which starts the sim node in ``pose`` mode,
starts the bag recorder, and shuts everything down when the trials finish.

Design decisions worth knowing:

* **Orientation is fixed** (hand pointing down) and only position varies. The
  arm is redundant and has no null-space control, so a target that varies in
  both position and orientation produces two coupled error signals that are
  hard to attribute.
* **Targets are sampled from a box known to be reachable.** There is no IK
  available to verify reachability up front, so an unconstrained sampler would
  make "success rate" a property of the sampler rather than of the bridge.
  Trials that never settle are still recorded, and the offline analysis reports
  them separately rather than silently folding them into the success rate.
* **Settling is "inside tolerance continuously for a dwell period"**, not
  "inside tolerance once". The controller is a spring-damper: it crosses the
  tolerance band on the way to overshooting it.
* Timing uses the stamps carried on ``/ee_pose``, not the node clock. Those are
  simulated time straight from the loop, so they stay exact even if the node's
  own callbacks are late.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# Hand pointing straight down: 180 degrees about world X, as (x, y, z, w).
# Matches the orientation the shipped example holds the wrist at.
EE_DOWN_XYZW = (1.0, 0.0, 0.0, 0.0)


def stamp_to_sec(stamp) -> float:
    """Convert a ROS time message to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass
class Trial:
    """One waypoint reach, as recorded for the offline analysis."""

    index: int
    seed: int
    target: list[float]
    orientation: list[float]
    start_position: list[float]
    distance: float
    t_command: float
    t_settle: float | None
    t_end: float
    settled: bool
    # Where the arm actually finished. Without this a 6 mm miss and an 80 mm
    # miss look identical in the index, and a timeout says nothing about
    # whether the controller was close or stuck somewhere else entirely.
    final_position: list[float]
    final_error: float


class TrialRunner(Node):
    """Publish waypoints, watch /ee_pose, record when each trial resolved."""

    def __init__(self) -> None:
        """Declare parameters, set up I/O and seed the sampler."""
        super().__init__("trial_runner")

        self.declare_parameter("n_targets", 20)
        self.declare_parameter("seed", 0)
        self.declare_parameter("tolerance", 0.005)  # [m]
        self.declare_parameter("dwell", 0.5)  # [s] inside tolerance to settle
        self.declare_parameter("timeout", 5.0)  # [s] before giving up
        self.declare_parameter("settle_margin", 0.5)  # [s] held after settling
        # A box inside the region the circle demo already exercises, so targets
        # are reachable without an IK check.
        self.declare_parameter("x_range", [0.35, 0.60])
        self.declare_parameter("y_range", [-0.20, 0.20])
        self.declare_parameter("z_range", [0.30, 0.60])
        self.declare_parameter("output", "trials.json")

        self.n_targets = int(self.get_parameter("n_targets").value)
        self.seed = int(self.get_parameter("seed").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.dwell = float(self.get_parameter("dwell").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.settle_margin = float(self.get_parameter("settle_margin").value)
        self.output = Path(str(self.get_parameter("output").value))
        self.rng = np.random.default_rng(self.seed)

        self.pub_target = self.create_publisher(PoseStamped, "target_pose", 10)
        self.create_subscription(PoseStamped, "ee_pose", self.on_ee_pose, 10)

        # Latest observed end-effector state, and the time it was observed.
        self.ee_position: np.ndarray | None = None
        self.sim_time: float = 0.0

        self.trials: list[Trial] = []
        self.active: Trial | None = None
        self.inside_since: float | None = None
        self.finished = False

        # 50 Hz is well below the 200 Hz simulation loop, so the state machine
        # sees every meaningful change without doing work per physics step.
        self.create_timer(0.02, self.tick)
        self.get_logger().info(
            f"{self.n_targets} trials, seed {self.seed}, "
            f"tolerance {self.tolerance * 1000:.1f} mm, dwell {self.dwell:.2f} s"
        )

    # -- callbacks ----------------------------------------------------------
    def on_ee_pose(self, msg: PoseStamped) -> None:
        """Track the measured end-effector position and simulated time."""
        self.ee_position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        self.sim_time = stamp_to_sec(msg.header.stamp)

    # -- trial lifecycle ----------------------------------------------------
    def sample_target(self) -> np.ndarray:
        """Draw a target position uniformly from the reachable box."""
        ranges = [
            self.get_parameter(name).value for name in ("x_range", "y_range", "z_range")
        ]
        return np.array([self.rng.uniform(lo, hi) for lo, hi in ranges])

    def start_trial(self) -> None:
        """Publish the next target and open a trial record."""
        target = self.sample_target()
        start = self.ee_position.copy()

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = EE_DOWN_XYZW
        self.pub_target.publish(msg)

        self.active = Trial(
            index=len(self.trials),
            seed=self.seed,
            target=target.tolist(),
            orientation=list(EE_DOWN_XYZW),
            start_position=start.tolist(),
            distance=float(np.linalg.norm(target - start)),
            # Simulated time, so it lines up with the bag's /ee_pose stamps.
            # The message header above carries the node clock, which is the
            # same thing only because use_sim_time is set.
            t_command=self.sim_time,
            t_settle=None,
            t_end=self.sim_time,
            settled=False,
            final_position=start.tolist(),
            final_error=float("nan"),
        )
        self.inside_since = None

    def finish_trial(self, settled: bool) -> None:
        """Close the open trial and log a one-line result."""
        trial = self.active
        trial.settled = settled
        trial.t_end = self.sim_time
        trial.final_position = self.ee_position.tolist()
        trial.final_error = float(
            np.linalg.norm(np.array(trial.target) - self.ee_position)
        )
        self.trials.append(trial)
        self.active = None

        if settled:
            self.get_logger().info(
                f"trial {trial.index:3d}  d={trial.distance * 100:5.1f} cm  "
                f"settled in {trial.t_settle - trial.t_command:.2f} s, "
                f"error {trial.final_error * 1000:.2f} mm"
            )
        else:
            self.get_logger().warning(
                f"trial {trial.index:3d}  d={trial.distance * 100:5.1f} cm  "
                f"TIMEOUT after {self.timeout:.1f} s, "
                f"stopped {trial.final_error * 1000:.1f} mm short"
            )

    def tick(self) -> None:
        """Advance the state machine: start, watch, settle, finish, write."""
        if self.finished or self.ee_position is None:
            return  # nothing observed yet; the sim may still be starting

        if self.active is None:
            if len(self.trials) >= self.n_targets:
                self.write_results()
                return
            self.start_trial()
            return

        trial = self.active
        elapsed = self.sim_time - trial.t_command
        error = float(np.linalg.norm(np.array(trial.target) - self.ee_position))

        if error <= self.tolerance:
            if self.inside_since is None:
                self.inside_since = self.sim_time
            elif trial.t_settle is None and self.sim_time - self.inside_since >= self.dwell:
                trial.t_settle = self.inside_since
        else:
            # Leaving the band resets the dwell: an overshoot that passes
            # through tolerance has not settled.
            self.inside_since = None
            trial.t_settle = None

        if trial.t_settle is not None:
            # Hold briefly past settling so the bag contains steady-state
            # samples to average the final error over.
            if self.sim_time - trial.t_settle >= self.dwell + self.settle_margin:
                self.finish_trial(settled=True)
        elif elapsed >= self.timeout:
            self.finish_trial(settled=False)

    def write_results(self) -> None:
        """Write the trial index and ask for shutdown."""
        settled = sum(1 for t in self.trials if t.settled)
        payload = {
            "seed": self.seed,
            "tolerance": self.tolerance,
            "dwell": self.dwell,
            "timeout": self.timeout,
            "trials": [asdict(t) for t in self.trials],
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, indent=2))

        self.get_logger().info(
            f"{settled}/{len(self.trials)} settled; wrote {self.output}"
        )
        self.finished = True
        # Exiting is what tells the launch file to stop the recorder and the
        # simulation; metrics are computed later from the bag.
        raise SystemExit(0)


def main() -> None:
    """Entry point."""
    rclpy.init()
    node = TrialRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()