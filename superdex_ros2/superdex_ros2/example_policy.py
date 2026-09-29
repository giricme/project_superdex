#!/usr/bin/env python3
# Copyright (c) 2026 Team 6. Licensed under the Apache License, Version 2.0.
"""A stand-in policy node: reads /joint_states, writes /joint_command.

The point of this file is the *shape*, not the contents. Observation in, action
out, over standard ROS messages -- which is the same loop a learned policy runs
in deployment, and the reason the bridge grew a joint-space command interface at
all. Replace :func:`Policy.act` with a network forward pass and nothing else
about the wiring changes; point the same node at a real robot's joint-command
topic and nothing about the *node* changes either.

What stands in for the policy here is a phase-shifted sinusoid about whatever
pose the arm was in when the first observation arrived. It is deliberately dull:
the interesting claim is that the loop closes, not that the motion is clever.

    ros2 run superdex_ros2 example_policy

    # alongside the simulator in joint mode
    ros2 launch superdex_ros2 sim.launch.py arm_mode:=joint
    ros2 run superdex_ros2 example_policy

Parameters:
    decimation (int, default 4)
        Act on every Nth observation. The simulation publishes at 200 Hz, so 4
        gives a 50 Hz policy -- deliberately slower than the physics, because
        that is the real situation: a policy runs at tens of Hz while the
        controller underneath it runs at hundreds, and the last action is held
        in between. Set 1 to act on every step.
    amplitude (double, default 0.25)
        Sinusoid amplitude [rad].
    period (double, default 6.0)
        Sinusoid period [s] in simulated time.
    joints (string array)
        Which joints to move. Everything else is commanded to hold its initial
        position -- commanding it explicitly rather than omitting it, so the
        message is a complete action and not a patch.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# A readable default: one arm joint that swings the whole limb, one wrist joint,
# and one knuckle per finger so the hand visibly participates.
DEFAULT_JOINTS = [
    "fr3_joint1",
    "fr3_joint6",
    "dg5f_joint_2_2",
    "dg5f_joint_3_2",
    "dg5f_joint_4_2",
]


def stamp_to_sec(stamp) -> float:
    """Convert a ROS time message to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class ExamplePolicy(Node):
    """Observation in, action out, at a fraction of the simulation rate."""

    def __init__(self) -> None:
        """Declare parameters and wire up the observation/action topics."""
        super().__init__("example_policy")

        self.declare_parameter("decimation", 4)
        self.declare_parameter("amplitude", 0.25)
        self.declare_parameter("period", 6.0)
        self.declare_parameter("joints", DEFAULT_JOINTS)

        self.decimation = max(1, int(self.get_parameter("decimation").value))
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.period = float(self.get_parameter("period").value)
        self.joints = list(self.get_parameter("joints").value)

        self.publisher = self.create_publisher(JointState, "joint_command", 10)
        # Depth 1: a policy wants the newest observation, never a queued one.
        self.create_subscription(JointState, "joint_states", self.on_observation, 1)

        # Captured from the first observation, and used as the pose everything
        # oscillates about. Taking it from the robot rather than hardcoding it
        # means this node needs no knowledge of the model.
        self.home: dict[str, float] | None = None
        self.t0: float = 0.0
        self.count = 0

        self.get_logger().info(
            f"acting on every {self.decimation} observation(s), "
            f"moving {len(self.joints)} joint(s); waiting for /joint_states"
        )

    def act(self, observation: dict[str, float], t: float) -> dict[str, float]:
        """Map an observation to an action. Replace this with a real policy.

        Both sides are dictionaries keyed by joint name, so a policy that
        reorders or omits joints cannot silently corrupt the command -- the
        bridge resolves names, not array positions.
        """
        action = dict(self.home)  # hold everything by default
        phase = 2.0 * math.pi * (t - self.t0) / self.period
        for i, name in enumerate(self.joints):
            if name not in action:
                continue
            offset = i * 2.0 * math.pi / max(len(self.joints), 1)
            action[name] = self.home[name] + self.amplitude * math.sin(phase + offset)
        return action

    def on_observation(self, msg: JointState) -> None:
        """Act on a decimated subset of incoming states."""
        if not msg.name or len(msg.position) < len(msg.name):
            return

        observation = dict(zip(msg.name, msg.position))
        t = stamp_to_sec(msg.header.stamp)

        if self.home is None:
            self.home = dict(observation)
            self.t0 = t
            missing = [j for j in self.joints if j not in self.home]
            if missing:
                self.get_logger().warning(f"joints not on this robot: {missing}")
            self.get_logger().info(
                f"captured home pose from {len(self.home)} joints at t={t:.3f}"
            )

        self.count += 1
        if self.count % self.decimation:
            return

        action = self.act(observation, t)

        command = JointState()
        # Stamp with the observation's time, not the wall clock: this action is
        # a response to that state, and keeping the two aligned is what makes a
        # recorded run analysable afterwards.
        command.header.stamp = msg.header.stamp
        command.name = list(action.keys())
        command.position = [float(v) for v in action.values()]
        self.publisher.publish(command)


def main() -> None:
    """Entry point."""
    rclpy.init()
    node = ExamplePolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()