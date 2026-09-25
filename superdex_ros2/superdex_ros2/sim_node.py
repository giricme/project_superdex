# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example: Combining OSC and JSC control on a single bot

Runs the OSC and JSC examples at the same time on one arm-hand combo: an FR3 arm
with a DG5F short hand at its tip. OSC tracks a planar circle with the wrist
while JSC waves the hand's knuckles.

Two controllers on one actor means two torque vectors on one actor. Both
controllers return a vector sized to the *whole* actor, so combining them is a
sum -- but only if each contributes zero on the DOFs it does not own. OSC does
that for free: it zeros every DOF outside the base-to-end-effector chain. JSC
does not; it spans every DOF, so we zero the arm entries of its output by hand
before summing. Each step we build both targets, compute both torques, mask and
sum them, apply the result, and step the simulation.

MODIFIED for superdex_ros2: the loop no longer terminates when the Physics
Debugger disconnects. It runs for a fixed amount of *simulated* time whether or
not a debugger is attached, so the step loop can be driven from a ROS 2 node.
Attaching the debugger is still supported and purely optional.

The loop publishes /joint_states (position, velocity, effort), /clock, and the
commanded vs. achieved end-effector pose each step, and accepts commands on
/target_pose and /cmd_vel, and broadcasts every link frame on /tf.
Timestamps are simulated
time, not wall time, so downstream nodes must run with use_sim_time:=true.
Everything runs on one thread for now -- the sim loop *is* the node's loop, and
there is no executor. That is fine while the node only publishes; it has to
change when we subscribe to a command topic.

The loop is paced against a wall clock so simulated time tracks real time,
which is what makes published timestamps meaningful. Set `realtime = False` to
let it free-run. The overrun counter reports how many steps missed their wall
deadline -- with publishers added later, that is the direct readout of whether
ROS serialization has eaten the per-step budget.

Usage:
    # run until Ctrl-C
    .venv-ros/bin/python superdex_ros2/superdex_ros2/sim_node.py

    # run for 10 s of simulated time, then exit
    .venv-ros/bin/python superdex_ros2/superdex_ros2/sim_node.py \
        --ros-args -p duration:=10.0

    # free-run as fast as the CPU allows (timestamps become meaningless)
    .venv-ros/bin/python superdex_ros2/superdex_ros2/sim_node.py \
        --ros-args -p realtime:=false

    # launch the SuperDex Physics Debugger alongside the node
    .venv-ros/bin/python superdex_ros2/superdex_ros2/sim_node.py \
        --ros-args -p gui:=true

Parameters:
    duration (double, default 0.0)
        Simulated seconds to run. Zero or negative runs indefinitely until
        Ctrl-C or ROS shutdown.
    realtime (bool, default true)
        Pace the loop against the wall clock.
    gui (bool, default false)
        Launch and attach the SuperDex Physics Debugger. Off by default: attach()
        starts the debugger application, and a ROS node should not open a window
        unasked. Closing the debugger no longer stops the run.
    arm_mode (string, default "circle")
        Where the arm's Cartesian target comes from.
          circle -- the upstream demo trajectory, self-driving, no commander needed
          pose   -- latest /target_pose (geometry_msgs/PoseStamped)
          twist  -- integrate /cmd_vel (geometry_msgs/Twist), e.g. from
                    teleop_twist_keyboard
          hold   -- hold the pose the end effector started in
    cmd_timeout (double, default 0.5)
        Simulated seconds of /cmd_vel silence after which the twist is treated as
        zero. A dropped teleop connection must not leave the arm drifting.
    workspace_radius (double, default 0.8) / workspace_z_min (double, default 0.05)
        Clamp on the commanded target, measured from the arm root. A held teleop
        key would otherwise walk the target out past the arm's reach, where the
        controller stalls with no error signal.
"""

import os
import time
from pathlib import Path

import numpy as np
import rclpy
import superdex.physics as sdp
import superdex.robotics as sdr
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# The build's `real` type. A pose handed to a controller Target is copied into the
# Target's own storage, so matching the dtype here keeps that a straight copy rather
# than an element-by-element conversion.
np_real = np.float64 if sdp.uses_double_precision() else np.float32
from superdex.physics.paths import resolve_asset

# The OSC controller acts on the chain of joints between these two links. A
# bot's link actors are named "<bot_name>/<link_name>", so we prefix these at
# runtime with bot.get_name(). fr3_link8 is the arm's tool flange, which is
# where the hand is attached.
ARM_BASE_LINK = "fr3_link0"
ARM_EE_LINK = "fr3_link8"

# The arm's joints all share this prefix, which is how we tell arm DOFs (owned
# by OSC) from hand DOFs (owned by JSC).
ARM_JOINT_PREFIX = "fr3_joint"

# The non-thumb knuckle joints (finger 1 is the thumb).
KNUCKLE_JOINTS = (
    "dg5f_joint_2_2",
    "dg5f_joint_3_2",
    "dg5f_joint_4_2",
    "dg5f_joint_5_3",
)


def sim_time_to_msg(t: float) -> TimeMsg:
    """Convert simulated seconds to a ROS time message.

    Done with integer nanoseconds rather than float sec/nanosec fields so the
    split can never produce nanosec == 1e9, which is out of range and which
    naive rounding of (t - int(t)) hits at values just below a whole second.
    """
    ns_total = int(round(t * 1e9))
    stamp = TimeMsg()
    stamp.sec = ns_total // 1_000_000_000
    stamp.nanosec = ns_total % 1_000_000_000
    return stamp


def pose_msg_to_transform(msg: PoseStamped) -> sdp.TransformRT:
    """geometry_msgs/PoseStamped -> SuperDex TransformRT (world frame).

    Both use (x, y, z, w) quaternion order and metres, and SuperDex declares an
    X-forward / Y-left / Z-up convention, which is REP-103. So this is a field
    copy, not a frame conversion.
    """
    tr = sdp.TransformRT()
    p, q = msg.pose.position, msg.pose.orientation
    tr.translation = [p.x, p.y, p.z]
    tr.rotation = sdp.Quaternion(q.x, q.y, q.z, q.w)
    return tr


def transform_to_pose_msg(tr: sdp.TransformRT, stamp: TimeMsg) -> PoseStamped:
    """SuperDex TransformRT -> geometry_msgs/PoseStamped stamped in sim time."""
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = "world"
    t = np.asarray(tr.translation, dtype=float)
    q = tr.rotation
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
        float(t[0]),
        float(t[1]),
        float(t[2]),
    )
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


def clamp_to_workspace(
    xyz: np.ndarray, root_xyz: np.ndarray, radius: float, z_min: float
) -> np.ndarray:
    """Keep a commanded point within reach of the arm root and above the ground."""
    out = np.array(xyz, dtype=float)
    offset = out - root_xyz
    dist = float(np.linalg.norm(offset))
    if dist > radius:
        out = root_xyz + offset * (radius / dist)
    out[2] = max(out[2], z_min)
    return out


def ensure_assets_path() -> str | None:
    """Point SuperDex at its assets root, inferring it if necessary.

    The pip wheel does not bundle robot assets; resolve_asset() finds them by
    looking for an assets root, which in practice means running from inside a
    SuperDex source checkout. A ROS node is launched from the colcon workspace
    instead, so that search fails and every asset lookup raises.

    This file lives at <checkout>/superdex_ros2/superdex_ros2/sim_node.py, so
    the checkout is an ancestor of it. Path.resolve() follows the symlink that
    `colcon build --symlink-install` leaves in install/, which is what makes
    the walk land in the source tree rather than the install tree.

    An explicit SUPERDEX_ASSETS_PATH always wins; returns whatever is in force,
    or None if no root was found.
    """
    existing = os.environ.get("SUPERDEX_ASSETS_PATH")
    if existing:
        return existing

    for parent in Path(__file__).resolve().parents:
        candidate = parent / "assets"
        if (candidate / "bots").is_dir():
            os.environ["SUPERDEX_ASSETS_PATH"] = str(candidate)
            return str(candidate)
    return None


def get_default_bot_path() -> str:
    """Resolve path to the default FR3 + DG5F short (right) .superdex_bot file."""
    return str(
        resolve_asset(
            "bots/arm_hand_combos/fr3_dg5f_short/right/fr3_dg5f_short_right.superdex_bot"
        )
    )


def main() -> None:
    """Load an arm-hand combo and drive the arm with OSC and the hand with JSC."""
    assets_path = ensure_assets_path()
    if assets_path is None:
        raise RuntimeError(
            "No SuperDex assets root found. Set SUPERDEX_ASSETS_PATH to the "
            "'assets' directory of a SuperDex checkout, or run from inside one."
        )
    bot_path = get_default_bot_path()

    # Initialize the physics engine before creating scenes or actors.
    # num_worker_threads=0 runs single-threaded; pass -1 to auto-select.
    sdp.initialize(num_worker_threads=0)

    # Create an empty scene. SuperDex robots use a Z-up convention, so gravity
    # points down the -Z axis.
    scene = sdp.create_scene("OSC + JSC Control Example")
    scene.set_gravity([0, 0, -9.81])

    bot_prefab = sdr.load_bot_prefab_from_file(bot_path)

    # Cheap "gravity compensation": disable gravity on every link before
    # spawning. Neither BASIC_OSC_PD nor BASIC_JSC_PD has a gravity term, so
    # both the arm and the fingers would otherwise sag off their targets.
    for i in range(len(bot_prefab.links)):
        bot_prefab.links[i].has_gravity = False

    # Split the DOFs into the two disjoint sets the controllers own. This numbers them
    # in bot DOF space: the prefab's joint order, skipping the root joint, and every
    # moving joint on this bot is a 1-DoF revolute joint, so the indices run
    # consecutively.
    arm_dofs = []
    joint_name_to_dof = {}
    dof = 0
    for i in range(len(bot_prefab.joints)):
        joint = bot_prefab.joints[i]
        if joint.type != sdp.ArticulatedJointType.REVOLUTE:
            continue
        joint_name_to_dof[joint.name] = dof
        if joint.name.startswith(ARM_JOINT_PREFIX):
            arm_dofs.append(dof)
        dof += 1

    # Instantiate the prefab as a live Bot. The robotics context tracks every
    # bot and controller you create.
    robotics_context = sdr.create_context()
    bot = sdr.create_bot(scene, bot_prefab, robotics_context)
    bot_actor = bot.get_articulated_actor()

    # Add a static ground plane for the robot to rest on (normal points up, +Z).
    plane_shape = sdp.create_plane_shape(normal=[0, 0, 1], distance=0)
    scene.create_rigid_actor(name="ground", shape=plane_shape, is_static=True)

    num_dofs = bot_actor.get_num_dofs()
    all_dof_indices = np.arange(num_dofs, dtype=np.int32)

    # The loop above numbered the joints in bot DOF space, which never includes the
    # root joint's DOFs. Everything from here on indexes the actor, where the root's
    # DOFs come first, so shift the indices across that gap. This arm is welded to the
    # world and contributes none, but reading the count off the actor keeps the mapping
    # correct for a bot on a free-floating base, which contributes six.
    num_root_dofs = bot_actor.get_articulated_shape_info().dof_info[0].get_size()
    arm_dof_indices = np.array(arm_dofs, dtype=np.int32) + num_root_dofs
    knuckle_dofs = [joint_name_to_dof[name] + num_root_dofs for name in KNUCKLE_JOINTS]

    # sensor_msgs/JointState is name-indexed; the engine is DOF-index-indexed.
    # Build the ordered name list once. Verified on this bot: 27 DOFs, all named,
    # fr3_joint1..7 at indices 0-6, num_root_dofs == 0 because the FR3 base is
    # welded. The num_root_dofs shift is UNVERIFIED for a floating-base bot --
    # none ships with the examples.
    dof_to_joint_name = {v + num_root_dofs: k for k, v in joint_name_to_dof.items()}
    joint_names = [dof_to_joint_name.get(i, f"<unmapped_{i}>") for i in range(num_dofs)]
    unmapped = [n for n in joint_names if n.startswith("<unmapped_")]
    if unmapped:
        raise RuntimeError(f"DOFs with no joint name: {unmapped}")

    # --- OSC controller on the arm ---------------------------------------------
    # initialize() resolves the base/end-effector links by name and figures out
    # which DOFs lie between them, i.e. the arm joints.
    osc = bot.create_controller("BASIC_OSC_PD")
    bot_name = bot.get_name()
    osc.initialize(f"{bot_name}/{ARM_BASE_LINK}", f"{bot_name}/{ARM_EE_LINK}")

    osc_params = osc.get_params()
    osc_params.kp_p = 900.0
    osc_params.kd_p = 75.0
    osc_params.kp_r = 30.0
    osc_params.kd_r = 3.0
    osc_params.max_translation_error = 0.05  # [m]
    osc_params.max_rotation_error = 0.4  # [rad]
    osc_params.b_apply_max_osc_torque_normalization = True
    osc.set_params(osc_params)

    # --- JSC controller on the hand --------------------------------------------
    # JSC has no notion of a sub-chain: its gains, its target pose and its output
    # are all sized to the full actor, arm DOFs included.
    jsc = bot.create_controller("BASIC_JSC_PD")
    jsc_params = sdr.ControllerBasicJscPdParams()
    jsc_params.kp = np.full(num_dofs, 3.0, dtype=np.float32)  # position gain [Nm/rad]
    jsc_params.kd = np.full(num_dofs, 0.2, dtype=np.float32)  # damping gain [Nms/rad]
    jsc_params.saturation = np.full(num_dofs, 2.0, dtype=np.float32)  # torque clamp
    jsc_params.deadband = np.zeros(num_dofs, dtype=np.float32)
    jsc.set_params(jsc_params)

    # The default pose is the JSC hold target; only the knuckles move off it.
    default_pose = sdp.DynamicArrayReal(num_dofs)
    bot_actor.get_articulated_pose(default_pose)
    hold_pose = np.array(default_pose, dtype=np_real)

    # --- Targets ---------------------------------------------------------------
    # The FR3 base is an intrinsically fixed Hard weld, so world_from_root is
    # constant and can be captured once to convert world-frame targets into the
    # root frame OSC expects.
    world_from_root = osc.get_current_observations_from_mochi().world_from_root

    # Circle lies in a horizontal plane 0.45 m above the ground and 0.5 m in
    # front of the robot base along +X.
    root_pos = np.asarray(world_from_root.translation, dtype=float)
    circle_center = np.array([root_pos[0] + 0.5, root_pos[1], 0.45])
    circle_radius = 0.12  # [m]
    circle_period = 4.0  # [s] per revolution

    # Keep the hand pointing straight down (its z-axis into the ground): a
    # 180-degree rotation about world X flips local +Z to world -Z.
    ee_down = sdp.Quaternion.rotation_x(np.pi)

    # Knuckle sweep: each knuckle oscillates between 0 and 60 degrees, with a
    # 30-degree phase offset between fingers and a 2 s period.
    sweep_period = 2.0  # [s]
    sweep_mid = np.radians(30.0)  # midpoint so the swing spans 0..60 deg
    sweep_amplitude = np.radians(30.0)  # [rad]
    finger_phase_offset = np.radians(30.0)  # [rad] between successive fingers

    # Simulation time step in seconds.
    time_step = 1.0 / 200.0

    # `duration` and `realtime` are ROS parameters, declared below once the node
    # exists.

    # Declare the scene's coordinate convention so the debugger renders it the
    # right way up: SuperDex is X-forward, Y-left, Z-up (FLU). Must come before
    # attach(), which starts the server.
    sdp.get_debug_server().set_coordinate_space(
        sdp.CoordinateSpace(axes=sdp.CoordinateSpaceAxes.FLU)
    )

    # Upstream gated the whole loop on the Physics Debugger being attached, which
    # made a GUI mandatory. We attach if we can and ignore the result, then run on
    # simulated time so the loop is headless-capable.
    #
    # The `if True:` is deliberate and temporary: it preserves the original two
    # levels of indentation so the loop body below stays byte-identical to
    # upstream, keeping this diff reviewable. The real refactor extracts a
    # step() function and drops it.
    # --- ROS 2 -----------------------------------------------------------------
    # Reusable scratch buffers and messages: the loop runs at 200 Hz, so
    # allocating a fresh DynamicArrayReal or JointState per step is avoidable
    # garbage. `name` never changes, so it is set once here.
    rclpy.init()
    node = Node("superdex_sim")

    # Simulated seconds to run. Zero or negative means run indefinitely, which is
    # the default: a ROS node that exits on its own after ten seconds is a demo,
    # not a simulator. A bounded run stays available for reproducible timing
    # measurements, where a fixed step count is what makes the numbers
    # comparable.
    node.declare_parameter("duration", 0.0)
    node.declare_parameter("realtime", True)
    node.declare_parameter("gui", False)
    node.declare_parameter("arm_mode", "circle")
    node.declare_parameter("cmd_timeout", 0.5)
    node.declare_parameter("workspace_radius", 0.8)
    node.declare_parameter("workspace_z_min", 0.05)
    node.declare_parameter("publish_tf", True)
    node.declare_parameter("tf_rate", 50.0)
    node.declare_parameter("tf_prefix", "sim_")
    node.declare_parameter("debug_links", False)
    duration = float(node.get_parameter("duration").value)
    realtime = bool(node.get_parameter("realtime").value)
    gui = bool(node.get_parameter("gui").value)
    arm_mode = str(node.get_parameter("arm_mode").value).lower()
    cmd_timeout = float(node.get_parameter("cmd_timeout").value)
    workspace_radius = float(node.get_parameter("workspace_radius").value)
    workspace_z_min = float(node.get_parameter("workspace_z_min").value)
    publish_tf = bool(node.get_parameter("publish_tf").value)
    tf_rate = float(node.get_parameter("tf_rate").value)
    tf_prefix = str(node.get_parameter("tf_prefix").value)
    debug_links = bool(node.get_parameter("debug_links").value)
    run_forever = duration <= 0.0

    valid_modes = ("circle", "pose", "twist", "hold")
    if arm_mode not in valid_modes:
        raise RuntimeError(f"arm_mode must be one of {valid_modes}, got '{arm_mode}'")

    pub_joint_states = node.create_publisher(JointState, "joint_states", 10)
    pub_clock = node.create_publisher(Clock, "clock", 10)
    # Commanded vs. achieved end-effector pose. Publishing both makes tracking
    # error directly plottable (rqt_plot) and is what the task-level metrics are
    # computed from.
    pub_ee_target = node.create_publisher(PoseStamped, "ee_target", 10)
    pub_ee_pose = node.create_publisher(PoseStamped, "ee_pose", 10)

    # Latest command. Depth 1 on both subscriptions: for a live command stream a
    # stale message is worse than a dropped one, so we never queue.
    # `last_twist_time` is simulated time, so the timeout below is unaffected by
    # the realtime flag.
    cmd = {"pose": None, "twist": np.zeros(6), "last_twist_time": -1e9}

    def on_target_pose(msg: PoseStamped) -> None:
        cmd["pose"] = pose_msg_to_transform(msg)

    def on_cmd_vel(msg: Twist) -> None:
        cmd["twist"] = np.array(
            [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ],
            dtype=float,
        )
        cmd["last_twist_time"] = scene.get_total_simulation_time()

    node.create_subscription(PoseStamped, "target_pose", on_target_pose, 1)
    node.create_subscription(Twist, "cmd_vel", on_cmd_vel, 1)

    pose_out = sdp.DynamicArrayReal(num_dofs)
    vel_out = sdp.DynamicArrayReal(num_dofs)

    # Link transforms, for the /tf publisher that comes next. Sized to the link
    # count, not the DOF count -- they differ. get_articulated_link_transforms()
    # writes into this span rather than returning, hence the preallocation.
    link_handles = bot_actor.get_nested_link_actors()
    num_links = len(link_handles)
    link_transforms_out = sdp.DynamicArrayTransformRT(num_links, sdp.TransformRT())
    js_msg = JointState()
    js_msg.name = joint_names
    clock_msg = Clock()

    # --- /tf ---------------------------------------------------------------
    # Engine link transforms are world-from-link, so this is a flat broadcast:
    # every link is a direct child of "world", with no composition. Link actors
    # are named "<bot_name>/<link_name>"; we strip the prefix and add our own,
    # default "sim_". That keeps these frames distinct from the ones
    # robot_state_publisher will emit from a URDF under the bare names, so the
    # two trees can coexist and be differenced rather than fight over parents.
    tf_broadcaster = TransformBroadcaster(node)
    link_frame_names = []
    for i in range(num_links):
        raw = scene.get_actor(link_handles[i]).get_name()
        link_frame_names.append(tf_prefix + raw.split("/")[-1])

    # 38 transforms at 200 Hz is 7600 messages/s, far more than any consumer
    # needs. Decimate to tf_rate; /joint_states stays at full rate.
    tf_decim = max(1, int(round((1.0 / time_step) / max(tf_rate, 1e-6))))
    tf_msgs = []
    for name in link_frame_names:
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = name
        tf_msgs.append(tf_msg)

    node.get_logger().info(
        f"publishing {num_dofs} DOFs on /joint_states at {1.0 / time_step:.0f} Hz "
        f"(sim time; set use_sim_time:=true downstream)"
    )
    node.get_logger().info(
        ("running until Ctrl-C" if run_forever else f"running for {duration:.3f} s sim")
        + (", realtime paced" if realtime else ", free-running")
        + (", debugger GUI on" if gui else ", headless")
        + f", arm_mode={arm_mode}"
    )
    if publish_tf:
        node.get_logger().info(
            f"broadcasting {num_links} link frames on /tf at "
            f"{(1.0 / time_step) / tf_decim:.0f} Hz, parent 'world', "
            f"prefix '{tf_prefix}'"
        )

    # --- one-shot TF reconnaissance --------------------------------------------
    # Print what get_articulated_link_transforms() actually gives us BEFORE
    # writing any /tf code. Two things decide the shape of that publisher:
    #   1. Are these world-from-link (flat, broadcast each against "world") or
    #      parent-relative (must be composed)? The stub says world-from-local,
    #      so we expect distinct translations that are NOT all near the origin.
    #   2. Link actors are named "<bot_name>/<link_name>", but
    #      robot_state_publisher expects bare URDF link names, so the prefix
    #      almost certainly has to be stripped.
    # A wrong guess here yields a robot that looks *almost* right in RViz, which
    # is harder to debug than one that is obviously broken.
    if debug_links:
        bot_actor.get_articulated_link_transforms(link_transforms_out)
        print(f"\n--- link transforms: {num_links} links, {num_dofs} DOFs ---")
        for i in range(num_links):
            link_actor = scene.get_actor(link_handles[i])
            tr = link_transforms_out[i]
            t_xyz = np.asarray(tr.translation, dtype=float)
            print(f"  {i:3d} {link_actor.get_name():40s} "
                  f"-> {link_frame_names[i]:28s} "
                  f"t=[{t_xyz[0]: .4f} {t_xyz[1]: .4f} {t_xyz[2]: .4f}]")
        print("--- end link transforms ---\n")

    # Index of the link OSC actually controls. The controller's end effector is
    # `<bot>/fr3_link8` (the wrist flange) unless the EELinkFromEE parameter is
    # set, which we leave at identity. We need the index to report the achieved
    # pose alongside the commanded one.
    ee_link_index = next(
        i
        for i in range(num_links)
        if scene.get_actor(link_handles[i]).get_name().endswith("/" + ARM_EE_LINK)
    )

    def read_ee_world() -> sdp.TransformRT:
        """Current world-frame pose of the controlled end-effector link."""
        bot_actor.get_articulated_link_transforms(link_transforms_out)
        return link_transforms_out[ee_link_index]

    # Seed the target with where the end effector actually is, so "hold" and
    # "twist" start from a zero-error condition and the arm does not jump on the
    # first step.
    start_ee = read_ee_world()
    cmd_xyz = np.asarray(start_ee.translation, dtype=float).copy()
    cmd_rot = start_ee.rotation

    try:
        if gui:
            # attach() launches the debugger application and returns False if it
            # cannot connect. Unlike upstream, a failed or later-closed debugger
            # is not fatal: the loop is driven by simulated time, not by the
            # viewer's connection state.
            if sdp.debugger.attach():
                node.get_logger().info("physics debugger attached")
            else:
                node.get_logger().warning(
                    "physics debugger failed to attach; continuing headless"
                )

        wall_start = time.perf_counter()
        n_steps = 0
        n_overruns = 0
        # rclpy.ok() first so an external shutdown ends the run even in the
        # indefinite case.
        while rclpy.ok() and (
            run_forever or scene.get_total_simulation_time() < duration
        ):
            t = scene.get_total_simulation_time()

            # Heartbeat: one line per simulated second. Confirms the loop is
            # advancing sim time at time_step per step with no debugger present.
            if t % 1.0 < time_step:
                print(f"t={t:.3f}")

            # Service ROS callbacks without blocking. One work item per step is
            # ample: commands arrive at teleop rates (tens of Hz) against a
            # 200 Hz loop, and depth-1 queues mean nothing accumulates. Keeping
            # this in the sim thread avoids an executor thread contending for the
            # GIL with the step loop.
            rclpy.spin_once(node, timeout_sec=0.0)

            # --- where the arm's Cartesian target comes from -------------------
            world_from_target_ee = sdp.TransformRT()
            if arm_mode == "circle":
                # Upstream demo: a point on a circle, hand oriented into the ground.
                theta = 2.0 * np.pi * t / circle_period
                world_from_target_ee.translation = [
                    circle_center[0] + circle_radius * np.cos(theta),
                    circle_center[1] + circle_radius * np.sin(theta),
                    circle_center[2],
                ]
                world_from_target_ee.rotation = ee_down
            else:
                if arm_mode == "pose" and cmd["pose"] is not None:
                    cmd_xyz = np.asarray(cmd["pose"].translation, dtype=float).copy()
                    cmd_rot = cmd["pose"].rotation
                elif arm_mode == "twist":
                    # Integrate only while commands are fresh: a dead teleop
                    # publisher must not leave the target drifting.
                    if t - cmd["last_twist_time"] <= cmd_timeout:
                        v = cmd["twist"][:3]
                        w = cmd["twist"][3:]
                        cmd_xyz = cmd_xyz + v * time_step
                        if np.any(w):
                            # World-frame angular velocity, so the incremental
                            # rotation pre-multiplies the current orientation.
                            dq = sdp.Quaternion.from_rotation_vector(
                                (w * time_step).tolist()
                            )
                            cmd_rot = dq * cmd_rot
                # "hold" and the not-yet-commanded cases fall through with
                # cmd_xyz / cmd_rot unchanged.
                cmd_xyz = clamp_to_workspace(
                    cmd_xyz, root_pos, workspace_radius, workspace_z_min
                )
                world_from_target_ee.translation = cmd_xyz.tolist()
                world_from_target_ee.rotation = cmd_rot
            # OSC targets are expressed in the actor root frame.
            target_root_from_ee = world_from_root.inverse() * world_from_target_ee

            # JSC target: the default pose everywhere, with the four knuckles
            # driven by a phase-shifted sine.
            target_pose = np.array(hold_pose, dtype=np_real)
            for finger, knuckle_dof in enumerate(knuckle_dofs):
                target_pose[knuckle_dof] = sweep_mid + sweep_amplitude * np.sin(
                    2.0 * np.pi * t / sweep_period + finger * finger_phase_offset
                )

            # np.array (not np.asarray) because the spans the controllers return
            # are read-only views onto their internal buffers.
            # Each controller reads its own observations off the simulation; the
            # JSC additionally needs the control period, which cannot be harvested.
            osc_obsv = osc.get_current_observations_from_mochi()
            arm_tau = np.array(
                osc.compute_output(
                    osc_obsv,
                    sdr.ControllerBasicOscPdTarget(
                        root_from_target_ee=target_root_from_ee
                    ),
                ),
                dtype=np.float32,
            )
            jsc_obsv = jsc.get_current_observations_from_mochi()
            jsc_obsv.dt = time_step
            hand_tau = np.array(
                jsc.compute_output(
                    jsc_obsv,
                    sdr.ControllerBasicJscPdTarget(target_pose=target_pose),
                ),
                dtype=np.float32,
            )

            # Both torque vectors span the whole actor. OSC already zeros
            # everything outside its arm chain, but JSC does not, so we zero its
            # arm entries here -- otherwise it would fight OSC over those DOFs.
            # With the two now disjoint, the combined torque is just their sum.
            hand_tau[arm_dof_indices] = 0.0
            total_tau = arm_tau + hand_tau
            bot_actor.set_external_forces_on_dofs(
                dof_indices=all_dof_indices,
                force_values=total_tau,
            )
            scene.step(time_step)
            n_steps += 1

            # Publish AFTER the step, and re-read the clock: the state now on
            # the wire is the post-step state at t + time_step, not the `t`
            # captured at the top of the loop that the targets were built from.
            t_pub = scene.get_total_simulation_time()
            stamp = sim_time_to_msg(t_pub)

            bot_actor.get_articulated_pose(pose_out)
            bot_actor.get_articulated_joint_velocities(vel_out)
            js_msg.header.stamp = stamp
            js_msg.position = np.asarray(pose_out, dtype=np.float64).tolist()
            js_msg.velocity = np.asarray(vel_out, dtype=np.float64).tolist()
            # effort is the torque WE commanded this step, which is unambiguous.
            # get_articulated_controller_force() reports only what the engine's
            # own pose controller applied, and these torques are applied as
            # external DOF forces instead, so it would read zero here.
            js_msg.effort = total_tau.astype(np.float64).tolist()
            pub_joint_states.publish(js_msg)

            clock_msg.clock = stamp
            pub_clock.publish(clock_msg)

            pub_ee_target.publish(transform_to_pose_msg(world_from_target_ee, stamp))
            pub_ee_pose.publish(transform_to_pose_msg(read_ee_world(), stamp))

            if publish_tf and n_steps % tf_decim == 0:
                # Refresh in place; the span is reused every publish.
                bot_actor.get_articulated_link_transforms(link_transforms_out)
                for i in range(num_links):
                    tr = link_transforms_out[i]
                    tf_msg = tf_msgs[i]
                    tf_msg.header.stamp = stamp
                    txyz = tr.translation
                    tf_msg.transform.translation.x = float(txyz[0])
                    tf_msg.transform.translation.y = float(txyz[1])
                    tf_msg.transform.translation.z = float(txyz[2])
                    # Quaternion storage order is (x, y, z, w), the same order
                    # geometry_msgs uses -- confirmed against the pybind stub's
                    # __init__(x, y, z, w) signature.
                    q = tr.rotation
                    tf_msg.transform.rotation.x = float(q[0])
                    tf_msg.transform.rotation.y = float(q[1])
                    tf_msg.transform.rotation.z = float(q[2])
                    tf_msg.transform.rotation.w = float(q[3])
                tf_broadcaster.sendTransform(tf_msgs)

            if realtime:
                # Absolute schedule, not incremental sleeps: each sleep
                # overshoots slightly, and incremental sleeps accumulate that
                # overshoot into unbounded drift. Deadline for step n is
                # wall_start + n * time_step, computed fresh every step.
                lag = (wall_start + n_steps * time_step) - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
                else:
                    n_overruns += 1

    except KeyboardInterrupt:
        print()  # break the line the ^C landed on
        node.get_logger().info("interrupted")

    # Summary lives outside the try so it prints on Ctrl-C too. RTF is measured
    # against simulated time elapsed rather than `duration`, which is unset when
    # running indefinitely and wrong whenever the run is cut short.
    wall = time.perf_counter() - wall_start
    sim_elapsed = n_steps * time_step
    if n_steps > 0 and wall > 0.0:
        print(
            f"loop {wall:.3f}s  steps {n_steps}  "
            f"{1000.0 * wall / n_steps:.3f} ms/step  "
            f"RTF {sim_elapsed / wall:.2f}  "
            f"overruns {n_overruns} ({100.0 * n_overruns / n_steps:.1f}%)"
        )

    # Tear down: ROS first, then destroy the bot and shut the engine down cleanly.
    node.destroy_node()
    rclpy.shutdown()
    sdr.destroy_bot(scene, bot)
    sdp.shutdown()
    print("Simulation complete.")


if __name__ == "__main__":
    main()