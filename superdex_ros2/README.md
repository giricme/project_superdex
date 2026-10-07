# superdex_ros2

A ROS 2 interface to [Project SuperDex](https://github.com/facebookresearch/project_superdex).

SuperDex ships a contact-first physics engine, robot assets and controllers, but
no ROS integration of any kind. This package adds the missing seam: simulated
robot state flows out as standard ROS messages, and commands flow in — end-effector
poses and twists for Cartesian control, joint targets and torques for policies —
so unmodified ROS 2 tooling can observe and drive a SuperDex simulation.

Demonstrated two ways: `teleop_twist_keyboard`, a node written for mobile bases,
driving a 27-DOF FR3 arm with a DG5F dexterous hand without modification; and a
policy node that reads `/joint_states` and writes `/joint_command`, which is the
same loop a learned policy runs against real hardware.

**In a hurry?** [QUICKSTART.md](QUICKSTART.md) is the commands with no prose.

## Workspace layout

The packages live inside the SuperDex fork so they stay committable; the colcon
workspace lives outside it, so build artifacts never touch the repo and colcon
crawls only `src/` instead of the whole SuperDex tree.

```
16-709/project/
├── project_superdex/                 <- fork (git)
│   ├── superdex_ros2/                <- the node (ament_python)
│   │   ├── superdex_ros2/sim_node.py <- the bridge
│   │   ├── superdex_ros2/trial_runner.py  <- waypoint trials
│   │   ├── superdex_ros2/example_policy.py <- stand-in policy node
│   │   ├── tools/bot_to_urdf.py      <- .superdex_bot -> URDF converter
│   │   ├── tools/analyze_trials.py   <- offline metrics from a recorded session
│   │   ├── launch/                   <- sim.launch.py, display.launch.py
│   │   ├── urdf/  meshes/            <- generated, committed
│   │   └── package.xml  setup.py
│   └── superdex_ros2_msgs/           <- the messages (ament_cmake)
│       ├── msg/ContactPoint.msg  msg/ContactArray.msg
│       └── package.xml  CMakeLists.txt
└── ros2_ws/                          <- workspace (not under git)
    ├── .venv-ros/
    ├── src/superdex_ros2      -> ../../project_superdex/superdex_ros2
    ├── src/superdex_ros2_msgs -> ../../project_superdex/superdex_ros2_msgs
    ├── build/  install/  log/
```

Symlinking the packages into `src/` is a normal ROS practice and keeps one copy
of the source under version control.

Two packages rather than one because message generation forces it: `ament_python`
cannot run `rosidl`. See [contacts](#contacts-the-one-custom-message).

## The interpreter problem — read this first

SuperDex is a pip package; ROS 2 is an apt install. They must be importable by
the *same* interpreter, and that is the single most common way to get this
package not working.

```bash
source /opt/ros/jazzy/setup.bash          # BEFORE creating the venv
cd ros2_ws
uv venv --system-site-packages .venv-ros  # --system-site-packages is required
uv pip install --python .venv-ros/bin/python superdex
.venv-ros/bin/python -c "import rclpy, yaml, superdex.physics; print('ok')"
```

`--system-site-packages` is not optional. Sourcing ROS puts `rclpy` on
`PYTHONPATH`, but `rclpy` imports `yaml`, `lark` and others that live in
`/usr/lib/python3/dist-packages` as apt packages. A sealed venv finds `rclpy`
and then fails on its dependencies.

A venv stores absolute paths, so it cannot be moved — recreate it rather than
relocating it.

Requires Python 3.12 (SuperDex ships wheels for 3.12 only) and therefore ROS 2
Jazzy on Ubuntu 24.04, whose system Python is 3.12.

## Building and running

Build colcon *with the venv's interpreter*. This is the whole trick:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
.venv-ros/bin/python -m colcon build --symlink-install
source install/setup.bash

ros2 launch superdex_ros2 sim.launch.py arm_mode:=twist gui:=true
```

colcon writes a console script whose shebang is the interpreter that *ran*
colcon. Invoke the `colcon` command directly and that is `/usr/bin/python3`,
which cannot import SuperDex — the build succeeds and `ros2 run` then dies with
`ModuleNotFoundError: No module named 'superdex'`. Running it as
`.venv-ros/bin/python -m colcon` makes the venv the recorded interpreter and the
problem disappears, with no `PYTHONPATH` juggling.

`--symlink-install` means edits to `sim_node.py` take effect without rebuilding.
It also matters for asset lookup: the node finds SuperDex's `assets/` directory
by walking up from its own file, and the symlink is what makes that walk land in
the source checkout instead of the install tree.

### Launch files

| File | Brings up |
|---|---|
| `sim.launch.py` | The sim node alone. Every node parameter is a launch argument. |
| `display.launch.py` | Sim node + `robot_state_publisher` + RViz, with `use_sim_time` set on both consumers. Arguments: `arm_mode`, `gui`, `rviz`, `rviz_gl`, `arm_kp`, `arm_kd`. |
| `example_policy` (a node, not a launch file) | Stand-in policy: `/joint_states` in, `/joint_command` out. Run alongside `arm_mode:=joint`. |
| `trials.launch.py` | A headless waypoint-trial session: sim node in `pose` mode, `robot_state_publisher`, a bag recorder and the trial runner. Shuts everything down when the trials finish. Arguments: `seed`, `n_targets`, `tolerance`, `tf_rate`, `rsp_rate`, `output_dir`. |

### Assets

The pip wheel does not bundle robot assets — SuperDex normally finds them by
running from inside a source checkout. Launched from a colcon workspace it
cannot, and fails with `no assets root found`. The node infers the location from
its own path, so this usually resolves itself. If the package is installed
somewhere outside the checkout, set it explicitly:

```bash
export SUPERDEX_ASSETS_PATH=/path/to/project_superdex/assets
```

If `-m colcon` reports `No module named colcon`, colcon was installed somewhere
the venv cannot see:

```bash
sudo apt install python3-colcon-common-extensions
```

Direct invocation still works and bypasses all of the above:

```bash
.venv-ros/bin/python src/superdex_ros2/superdex_ros2/sim_node.py \
    --ros-args -p arm_mode:=twist
```

## Background: how control actually happens

Worth understanding before reading `sim_node.py`, because the division of labour
is not what the names suggest.

**The controllers never touch the robot.** `BASIC_OSC_PD` and `BASIC_JSC_PD` are
pure functions: observations and a target in, a torque vector out. Actuation is a
separate engine call that the *caller* makes. That is why the node can swap
where the arm's target comes from — a circle, a topic, an integrated twist —
without touching the controllers at all.

Each step does four things:

```python
# 1. Controllers read state (the engine computes FK and the Jacobian here)
osc_obsv = osc.get_current_observations_from_mochi()
jsc_obsv = jsc.get_current_observations_from_mochi()
jsc_obsv.dt = time_step

# 2. Controllers compute torques -- nothing is applied yet
arm_tau  = osc.compute_output(osc_obsv, sdr.ControllerBasicOscPdTarget(
               root_from_target_ee=target_root_from_ee))
hand_tau = jsc.compute_output(jsc_obsv, sdr.ControllerBasicJscPdTarget(
               target_pose=target_pose))

# 3. YOU apply them, as generalized forces on the articulated actor's DOFs
bot_actor.set_external_forces_on_dofs(
    dof_indices=all_dof_indices, force_values=arm_tau + hand_tau)

# 4. Integrate
scene.step(time_step)
```

### Reading state (step 1)

`get_current_observations_from_mochi()` harvests everything the controller needs
straight from the live simulation — current pose, velocities, the end-effector
transform and its Jacobian. Forward kinematics is done by the *engine*, not by
the controller and not by us.

One thing cannot be harvested: JSC needs the control period, so `jsc_obsv.dt` is
assigned by hand. Miss it and the derivative term is wrong.

### Computing torques (step 2)

**OSC — Operational Space Control.** Despite the name it is not Khatib's
formulation: there is no task-space inertia matrix. The source computes a 6-D
pose error between the current and target end-effector transforms, applies PD
gains, and maps the resulting wrench to joint torques with the Jacobian
*transpose* — deliberately avoiding any Jacobian inversion. It is Cartesian
impedance control.

Consequences that matter in practice:

- The target is the pose of `fr3_link8`, the wrist flange, expressed in the arm
  root frame. Not the palm, not a fingertip. The `EELinkFromEE` parameter
  (identity by default) can shift the controlled point.
- No null-space term. A 7-DOF arm tracking a 6-DOF goal has one redundant
  degree of freedom, and nothing controls it — the elbow settles wherever the
  dynamics leave it, and need not repeat between runs that reach the same pose.
- Error is clamped (5 cm, 0.4 rad by default) so distant targets are approached
  with bounded effort. Settle time therefore scales with distance.
- Deadbands of 0.1 mm and 5 mrad floor the achievable tracking error.
- An unreachable target produces no failure signal. The arm simply stalls short,
  so any success criterion has to be imposed by the caller.

**JSC — Joint Space Control.** Per-joint PD on joint angles, no kinematics
involved. Its output spans the *whole* actor, arm DOFs included.

Both torque vectors are actor-sized, so combining them is a sum — but only once
each contributes zero on the DOFs it does not own. OSC zeroes everything outside
its base-to-end-effector chain for free; JSC does not, so the node zeroes JSC's
arm entries by hand before summing. Skip that and the two controllers fight over
the arm.

### Applying torques (step 3)

`set_external_forces_on_dofs()` is the actuation call. For an articulated actor
the values are generalized forces in joint-DOF coordinates — N·m for revolute
joints — so it is effectively a direct joint-torque command.

Two semantics to know. Each call *replaces* every previously set external force,
including DOFs not named in `dof_indices`; and forces persist across steps until
replaced or cleared with `clear_external_forces()`. Harmless while the loop
recomputes every step, but if command rate and step rate ever decouple, the last
torque keeps being applied.

This is also why `/joint_states` reports commanded torque in its `effort` field
rather than calling `get_articulated_controller_force()` — that method reports
what the engine's own pose controller applied, and these torques arrive as
external forces instead, so it would read zero.

### A second command path, not used here

Articulated actors also expose an engine-native pose controller —
`set_articulated_target_pose()`, `set_articulated_target_velocity()` and
`set_articulated_pose_controller_params()` — which tracks joint targets as
constraints inside the solver rather than as torques computed in Python. That
maps closely onto a real robot's low-level command (position, velocity,
stiffness, damping per joint), and is the natural backend for a joint-command
topic. The node does not use it.

## The robot model: converting .superdex_bot to URDF

SuperDex describes robots in its own JSON format and ships no URDF for the
arm-hand combos. Nearly every stock ROS 2 tool needs one — `robot_state_publisher`
to compute forward kinematics, RViz's RobotModel display to draw meshes, MoveIt
to plan at all — so `tools/bot_to_urdf.py` generates it.

The converter is standalone: no ROS, no SuperDex engine, no simulation. Pure
JSON to XML. Run it once and commit the result.

```bash
# from the fork root
python3 superdex_ros2/tools/bot_to_urdf.py \
  --bot assets/bots/arm_hand_combos/fr3_dg5f_short/right/fr3_dg5f_short_right.superdex_bot \
  --output-dir superdex_ros2/urdf
```

Writes `urdf/fr3_dg5f_short_right.urdf` and copies 36 link meshes into
`meshes/`. Both directories are installed into the package share directory by
`setup.py`; without that, `package://` URIs do not resolve and RViz reports the
model as missing rather than broken.

Options: `--mesh-format {glb,dae,stl}` (`glb` copies as-is; the others convert
via trimesh, which lives in the venv — use `.venv-ros/bin/python` to run the
converter if you need them), `--mesh-up {y,z}`, `--no-world-link`,
`--assets-root`, `--package`.

### Format notes

None of this is documented upstream; it was established by reading the shipped
assets.

- **`links` and `joints` are parallel arrays.** Joint *i* connects
  `links[links[i]["parentLink"]]` to `links[i]`. The root link has no
  `parentLink`, and its joint entry is a placeholder.
- **Limits are 3-vectors indexed by the joint axis.** A joint with
  `axis = [0,1,0]` carries its limits in component 1. `fr3_joint1` has
  `axis = [0,0,1]` and `±2.7437` in component 2, which is the correct FR3 value.
- **`momentOfInertia` is `(ixx, ixy, ixz, iyy, iyz, izz)`** — determined by
  elimination, since the diagonal-first alternative would give `iyy = 0` for
  FR3 link 0.
- **Quaternions are `(x, y, z, w)`**, matching `geometry_msgs`. URDF wants
  roll-pitch-yaw, so the converter converts.
- **Joint `type`** is `Revolute`, `Hard` (fixed) or `Free` (a floating root,
  replaced by the attach joint when the bot is composed into a combo).
- **No effort or velocity limits exist in the source.** URDF requires both on
  revolute joints, so the converter supplies placeholders (100 N·m, 2 rad/s).
  These matter to MoveIt and not to visualization, and they are not
  manufacturer data.
- **Combos are composition.** The combo file names a `base` bot plus
  `modifications` entries, each an `AttachBot` giving a parent link, a child bot
  and the fixed joint between them. Here the DG5F hand attaches to `fr3_link8`
  through a 180° rotation about Z.

### Meshes are Y-up

glTF is a Y-up format; URDF link frames here are Z-up. Neither assimp (which
RViz loads meshes through) nor trimesh rotates on import, so the converter
emits `rpy="1.5708 0 0"` on every visual origin.

Without it, link *frames* are still correct and only the geometry is rotated —
so the robot renders as disconnected pieces floating near, but not on, their
joints. That failure mode is worth recognizing: it looks like a kinematics bug
and is not one.

Evidence for the convention: `fr3_link0` spans mesh-Y 0 → 0.14 and `fr3_link1`
spans mesh-Y −0.192 → 0.055, which are those links' Z extents on the real robot.
Use `--mesh-up z` to disable the correction.

### What is not converted

Collision geometry. SuperDex stores it in a proprietary `.mochi.h5` format,
unusable by ROS. RViz does not need it; MoveIt does, so planning work will need
collision meshes generated from the render meshes (convex hulls via trimesh are
the obvious route).

## Visualizing in RViz

```bash
cd ../ros2_ws
.venv-ros/bin/python -m colcon build --symlink-install
source install/setup.bash
ros2 launch superdex_ros2 display.launch.py arm_mode:=circle
```

This brings up the sim node, `robot_state_publisher` and RViz together, with
`use_sim_time` set on both consumers.

RViz starts from its default configuration, which is a navigation layout — Grid,
Map, LaserScan, fixed frame `map` — and shows nothing useful. Set it up once:

1. **Fixed Frame** → `world` (not `map`).
2. **Add** → **RobotModel**, then set its *Description Topic* to
   `/robot_description`.
3. **Add** → **TF** to see the frame tree.

Then **File → Save Config As** so the layout persists.

### Two frame trees

Both run at once, deliberately:

```
world -> sim_fr3_link0, sim_fr3_link1, ...    sim node: flat, straight from the engine
world -> base -> fr3_link0 -> fr3_link1 ...   robot_state_publisher: kinematic, from the URDF
```

They share the `world` root and never collide, because the sim node prefixes its
frames with `sim_`. The sim node publishes what the engine reports; RViz draws
what the URDF computes from `/joint_states`. Differencing a pair therefore tests
the URDF conversion, the DOF-to-joint-name mapping and the message path at once:

```bash
ros2 run tf2_ros tf2_echo sim_fr3_link8 fr3_link8 --ros-args -p use_sim_time:=true
```

Measured over 20 waypoint trials: **0.21 um** maximum position residual and
**0.02 mdeg** maximum rotation residual. That is the float32 floor — the engine
computes in single precision, whose relative epsilon at the ~1 m scale of these
transforms is about 0.12 um — so the two computations agree exactly and what is
left is rounding.

### The 20 Hz trap

Reaching that number required one non-obvious setting. `robot_state_publisher`
defaults to `publish_frequency: 20.0`, so out of the box it is the *coarser* of
the two trees by a factor of ten. tf2 interpolates across whichever tree is
coarser, and a 50 ms gap on a moving arm is worth hundreds of micrometres:

| `robot_state_publisher` | sim node `/tf` | Max residual |
|---|---|---|
| 20 Hz (default) | 50 Hz | 610 um |
| 20 Hz (default) | 200 Hz | 605 um |
| 200 Hz | 200 Hz | **0.21 um** |

The middle row is the instructive one: raising the *simulator's* rate changed
nothing, because it was never the limiter. Both launch files now set
`publish_frequency` to 200 Hz.

Worth knowing beyond this package. Anyone cross-validating a simulator against
`robot_state_publisher` will measure sub-millimetre "disagreement" in the default
configuration, see a plausible number, and attribute it to their model. It is a
sampling artifact with a one-line fix.

### Graphics on Pop!_OS

RViz renders through Ogre, which has no Wayland backend, so Qt must go via
XWayland: `QT_QPA_PLATFORM=xcb` is required or no window appears. On hybrid
NVIDIA graphics the GL context may also fail, in which case software
rasterization works but is slow with 38 meshes.

The launch file handles this per-node, so `LIBGL_ALWAYS_SOFTWARE` does not leak
into the sim node and force the SuperDex debugger onto CPU rendering too.
`rviz_gl:=software` is the default because it always works; try hardware first
and keep it if the window appears:

```bash
ros2 launch superdex_ros2 display.launch.py rviz_gl:=hardware
```

Standalone, the equivalent is:

```bash
QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
  ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

### If the robot does not appear

In order of likelihood:

- **Default RViz config.** No RobotModel display, fixed frame `map`. See above.
- **`urdf/` and `meshes/` not installed.** `ls install/superdex_ros2/share/superdex_ros2/`
  should list both. If not, `setup.py` is missing its `data_files` entries, or
  setuptools cached an old file list — `rm -rf build install log` and rebuild.
- **"Two or more unconnected trees."** `world -> base` is a fixed joint, so it
  is published once on `/tf_static`. If the TF buffer was cleared — see the next
  item — restarting the *consumer* re-subscribes and transient-local QoS
  redelivers it.
- **"Detected jump back in time."** Sim time restarts at zero on every run, so
  relaunching the sim node moves the clock backwards and TF clears its buffer.
  Harmless in itself, but two sim nodes running at once will do it repeatedly:
  `pkill -f sim_node` before relaunching.
- **Meshes render as nothing.** RViz's assimp build may not read glTF.
  Regenerate with `--mesh-format dae`.
- **Pieces float near their joints.** The Y-up correction is missing — see
  above.

## Measuring: trials and offline analysis

Two pieces, deliberately separated. `trial_runner` publishes waypoints and
records *when* each trial started, settled and ended; `tools/analyze_trials.py`
reads that index plus a rosbag of the same session and derives every number.
Nothing is measured live, so a metric can be redefined without re-running the
simulation.

```bash
ros2 launch superdex_ros2 trials.launch.py \
    seed:=0 n_targets:=20 tolerance:=0.010 tf_rate:=200.0 rsp_rate:=200.0

.venv-ros/bin/python tools/analyze_trials.py trials/<timestamp>
```

Output lands in `trials/<timestamp>/`: `trials.json` (the index),
`bag/`, `metrics_trials.csv` (one row per trial), `metrics_residual.csv` (the
forward-kinematics residual time series) and `summary.txt`.

Notes on the setup:

- `tf_rate` and `rsp_rate` both default to 200 Hz here, unlike the sim node's own
  50 Hz default, because the forward-kinematics residual is only meaningful when
  neither tree is being interpolated. See [the 20 Hz trap](#the-20-hz-trap).
- `output_dir` is timestamped. `ros2 bag record -o` refuses to write into an
  existing directory and dies on the spot while the rest of the session carries
  on, which leaves a `trials.json` paired to the *previous* run's bag.
- Targets are sampled from a box known to be reachable: there is no IK available
  to verify reachability up front, so an unconstrained sampler would make the
  success rate a property of the sampler.
- Settling means "inside tolerance continuously for a dwell period", not "inside
  tolerance once" — the controller is a spring-damper and crosses the band on
  its way to overshooting it.

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | out | 200 Hz |
| `/tf` | `tf2_msgs/TFMessage` | out | `tf_rate` (50 Hz) |
| `/clock` | `rosgraph_msgs/Clock` | out | 200 Hz |
| `/ee_pose` | `geometry_msgs/PoseStamped` | out | 200 Hz |
| `/ee_target` | `geometry_msgs/PoseStamped` | out | 200 Hz |
| `/target_pose` | `geometry_msgs/PoseStamped` | in | — |
| `/cmd_vel` | `geometry_msgs/Twist` | in | — |
| `/joint_command` | `sensor_msgs/JointState` | in | — |
| `/contacts` | `superdex_ros2_msgs/ContactArray` | out | `contact_rate` (28.6 Hz), off by default |

Every type here is a standard ROS type except `/contacts`, which is the one
piece of SuperDex's state that has no standard equivalent — see
[contacts](#contacts-the-one-custom-message) below.

`/joint_states` carries position, velocity and commanded effort for all 27 DOFs.
`/tf` broadcasts all 38 link frames as direct children of `world`, prefixed
`sim_` so they can coexist with frames a `robot_state_publisher` would emit
under the bare URDF names.

**Timestamps are simulated time, starting near zero.** Anything doing TF lookups
needs `use_sim_time:=true`, or it will report extrapolation errors against a
perfectly good transform:

```bash
ros2 run tf2_ros tf2_echo world sim_fr3_link8 --ros-args -p use_sim_time:=true
```

`/joint_command` is the mirror of `/joint_states` and is described under
[joint-space control](#joint-space-control-policy-rollout).

`/ee_pose` and `/ee_target` are diagnostics: actual versus commanded
end-effector pose, for plotting tracking error without a TF lookup.

```bash
rqt_plot /ee_target/pose/position/x /ee_pose/pose/position/x
```

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `duration` | `0.0` | Simulated seconds to run; 0 or less runs until Ctrl-C. |
| `realtime` | `true` | Pace the loop to the wall clock. False free-runs and makes timestamps meaningless. |
| `gui` | `false` | Launch and attach the SuperDex Physics Debugger. |
| `arm_mode` | `circle` | Where the arm's Cartesian target comes from. |
| `cmd_timeout` | `0.5` | Simulated seconds of `/cmd_vel` silence before the twist is dropped. |
| `workspace_radius` | `0.8` | Maximum target distance from the arm base [m]. |
| `workspace_z_min` | `0.05` | Minimum target height [m]. |
| `publish_tf` | `true` | Broadcast link frames on `/tf`. |
| `tf_rate` | `50.0` | `/tf` rate [Hz]. 38 frames at 200 Hz is 7600 msg/s for no benefit. |
| `tf_prefix` | `sim_` | Prefix for broadcast frame names. |
| `debug_links` | `false` | Print the link and DOF tables once at startup. |
| `publish_contacts` | `false` | Publish `/contacts`. Needs `superdex_ros2_msgs` built. |
| `contact_rate` | `30.0` | `/contacts` rate [Hz], quantized to 200/n — see below. |
| `arm_kp` | `150.0` | Arm joint-space position gain [N·m/rad]. Joint mode only. |
| `arm_kd` | `15.0` | Arm joint-space damping gain [N·m·s/rad]. Joint mode only. |
| `arm_saturation` | `80.0` | Arm per-joint torque clamp [N·m]. Joint mode only. |

### `arm_mode`

- `circle` — the upstream example's hardcoded trajectory. Commands ignored.
- `pose` — track the latest `/target_pose`. Holds the startup pose until the
  first message arrives.
- `twist` — integrate `/cmd_vel` into the target. For keyboard and gamepad teleop.
- `hold` — stay at the startup pose. Useful as a control condition when timing.
- `joint` — drive every DOF from `/joint_command`. Cartesian control is off.

In the first four modes the hand is unaffected by `arm_mode` and always runs the
example's knuckle sweep. `joint` is the exception: it hands the whole robot over
to the command topic.

## Joint-space control (policy rollout)

The Cartesian modes above are the wrong shape for a learned policy. Policies —
and real hardware, a Unitree arm for instance — work in joint space: the policy
emits joint targets or torques and something underneath closes the loop at a
higher rate. `arm_mode:=joint` gives the bridge that interface, so the same
policy node can drive the simulator and the real robot without changes.

```bash
ros2 launch superdex_ros2 display.launch.py arm_mode:=joint
ros2 run superdex_ros2 example_policy
```

What changes in this mode:

- The Cartesian controller is switched off, not merely ignored — computing a
  Jacobian and discarding the result every step would be waste.
- The joint-space controller owns all 27 DOFs instead of just the hand, so the
  masking that normally keeps the two controllers off each other's DOFs is
  skipped.
- The knuckle sweep stops. A policy driving joint space owns the hand too.

### The command

`/joint_command` is `sensor_msgs/JointState`, the mirror of what the bridge
publishes. It is **name-indexed**, which matters: a bare float array invites a
publisher to transpose two joints silently, and nothing downstream would catch
it. Unknown names are ignored with a one-time warning.

- **`position`** is a standing target. Uncommanded joints hold the startup pose,
  and a message naming only some joints leaves the rest where they were.
- **`effort`** is feedforward torque, for a policy that outputs torques. It is
  applied only while commands are fresh (`cmd_timeout`), and a message carrying
  no effort array clears it. Stale *position* is standing still; stale *torque*
  is how a robot runs away.

### Gains

The shipped example tunes one set of joint-space gains for the fingers
(`kp = 3.0`), which a seven-link arm sags straight through. In every other mode
the arm entries of that controller's output are masked away and never applied,
so it does not matter; in joint mode they are what moves the arm. Hence
`arm_kp`, `arm_kd` and `arm_saturation`, applied to the arm DOFs only.

The defaults (150 / 15 / 80) were chosen by reasoning, not measurement, and are
worth re-checking on any other robot. Gravity is disabled on every link — see
the shipped example's "cheap gravity compensation" — so they only have to track,
not hold weight. Measured on the FR3: **~1 mrad of steady-state error** on a
held joint, which is tight enough to leave alone.

If the arm sags, raise `arm_kp` (300, then 600). If it rings, raise `arm_kd`.

### The example policy

`example_policy` is a stand-in: it reads `/joint_states`, computes an action,
publishes `/joint_command`, and runs at 50 Hz against the simulation's 200 Hz —
deliberately, because that is the real situation. A policy runs at tens of Hz
while the controller under it runs at hundreds, and the last action is held in
between.

The interesting part is the shape, not the contents. `Policy.act()` maps an
observation dictionary to an action dictionary; replacing it with a network
forward pass changes nothing else, and pointing the same node at a real robot's
joint-command topic changes nothing about the node.

Parameters: `decimation` (default 4), `amplitude` (0.25 rad), `period` (6.0 s),
`joints` (which joints to move; everything else is commanded to hold).

## Contacts: the one custom message

Everything else on this interface is a standard ROS type, which is what lets
stock tooling consume it. Contacts are the exception, and they are worth
describing because the exception is instructive.

SuperDex is a *contact-first* engine: the thing it does better than its
alternatives is resolve rich contact between deformable surfaces. It reports, per
step, a set of contact points — each with a position on both bodies, a normal, a
force, a signed separation distance, and a quadrature weight giving the surface
area that point stands for. That last field is the one with no analogue in a
point-contact engine, and it is what you integrate over to get pressure or
contact area.

ROS has no message for that. The near misses:

| Candidate | What it loses |
|---|---|
| `geometry_msgs/WrenchStamped` | Collapses the whole manifold to one resultant — exactly the information a contact-first engine exists to produce. |
| `sensor_msgs/PointCloud2` | Keeps the geometry, drops force and area, and buries the field layout in a runtime-described struct. |
| `visualization_msgs/MarkerArray` | Renders in RViz for free, but a marker array is a *drawing*. Subscribing to it to recover numbers is reading pixels. |

So we define one. `superdex_ros2_msgs/ContactArray` is a stamped batch of
`ContactPoint`, one message per published step, carrying all seven quantities the
engine reports.

### Why it is a second package

**An `ament_python` package cannot generate messages.** Message generation runs
through `rosidl`, which is driven from CMake; `ament_python` has no CMake step.
This is not a policy we could argue with — there is no way to put a `.msg` file
in `superdex_ros2` and have it build.

So `superdex_ros2_msgs` is a separate `ament_cmake` package next to it. That
split is the ROS convention for interfaces anyway, for a better reason than ours:
a node that only wants to *subscribe* to contacts can depend on the messages
without pulling in the simulator, the `superdex` pip wheel, or the venv
gymnastics in [the interpreter problem](#the-interpreter-problem--read-this-first).

```
src/
├── superdex_ros2        -> ament_python, the node
└── superdex_ros2_msgs   -> ament_cmake,  the messages
```

```bash
.venv-ros/bin/python -m colcon build --symlink-install
source install/setup.bash     # required before the new messages are importable
```

`colcon` orders them correctly on its own: `superdex_ros2` has no build
dependency on the messages, because the import is optional.

### Why the import is optional

```python
try:
    from superdex_ros2_msgs.msg import ContactArray
    HAVE_CONTACT_MSGS = True
except ImportError:
    HAVE_CONTACT_MSGS = False
```

A workspace that built only `superdex_ros2` — which is every workspace that
existed before this was added — still runs the node unchanged. Requesting
contacts there logs a warning naming the build command and continues without
them. A missing optional message package should not take a simulator down.

### Running it

```bash
ros2 launch superdex_ros2 display.launch.py publish_contacts:=true
ros2 topic echo /contacts --once
```

Off by default, for two independent reasons. The `CONTACT_POINTS` query makes the
engine do work on every step, so leaving it on would quietly change every
throughput number in [measured performance](#measured-performance) — all of which
were taken with it off. And RViz has no display for this type, so nothing is
watching unless you arranged for something to be.

**Expect `/contacts` to be empty in the stock scene.** The only other body is the
ground plane, and the arm does not reach it. Contacts appear when the hand closes
on something, which means adding an object to the scene — not done here.
Publishing an empty array every step is the honest behaviour and is what you
will see.

### Implementation notes

Three things about the engine side are worth knowing before touching this code.

**The query has to be registered before the step that produces the data.**
`CONTACT_POINTS` explicitly does not support `register_query_and_compute`, so
there is no asking after the fact — the data only exists for a step taken with
the query already live. The node registers once at startup and cancels at
teardown.

**Register on the links, not on the articulation.** This is the one that cost
real time. Contact sample points belong to whichever actor owns a collidable
surface, and for an articulated actor that depends on how the robot is built:

| Robot | Surface lives on | Register on |
|---|---|---|
| Skinned articulation | one skin covering the articulation actor | the articulation |
| Rigid-link bot (the arm-hand combos) | each link's own mesh | each link actor |

The arm-hand combos are the second kind. Registering on the articulated actor —
which is what the engine's own skinned-pendulum example does, and which is the
natural first guess — fails as `is_query_supported() == False`, not as an error.
The node probes the articulation, falls back to the nested link actors, and logs
which topology it found, so a bot built the other way works without a code
change.

**A contact between two queried links appears twice.** The pair is symmetric; the
engine does not designate one body as canonical, and which one lands in `link_a`
depends on the direction of the contact test. Two fingers touching each other are
reported from both sides, with `a` and `b` swapped. We publish both rather than
guess: `sample_index` is relative to `actor_a`, so there is no stable key to
deduplicate on. Filter on `link_a < link_b` if you want each pair once.
Hand-object contact is unaffected, since the object is not a queried actor.

**Handles are not names.** The engine identifies the two bodies by `ActorHandle`,
a raw 64-bit id that means nothing outside the process that issued it and that
can be recycled after an actor is destroyed. The node resolves each to the bare
link name, memoized on first lookup. Those are the same names used in the URDF
and in the `/tf` frame ids, so a subscriber can look a contact's body up in tf2
and get its pose without asking us anything.

**This publisher allocates, and the others do not.** Every other message in the
loop is built once and refilled, because its size is fixed. The contact count
changes every step, so there is nothing to reuse. That, plus the per-point
nesting, is why the default rate is 30 Hz and not 200.

**Rates are quantized to 200/n.** Both `contact_rate` and `tf_rate` decimate an
integer number of simulation steps, so the achievable rates are 200 Hz divided by
a whole number: 200, 100, 66.7, 50, 40, 33.3, 28.6, 25. Asking for 30 Hz gets you
`round(200/30) = 7` and therefore 28.6 Hz, which is what `ros2 topic hz` reports.
Nothing is wrong when it reads 28.6. `tf_rate:=50.0` happens to land exactly
(200/4), which is why this is easy to miss.

## Teleop

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Motion keys are `u i o / j k l / m , .`, with `t` and `b` for up and down. `q w e
z x c` only change the speed scale — they publish a zero twist, which looks like
a broken pipeline if you are watching `ros2 topic echo /cmd_vel` and wondering
why everything reads 0.0.

Hold a key: the node publishes on keypress, not continuously, and `cmd_timeout`
zeroes a stale twist after 0.5 s.

Being a mobile-base tool, `j` and `l` send *angular* z, so the arm twists in
place rather than moving sideways. Lateral motion is `J` and `L` (shifted).

## Notes

- **With `gui:=true` the simulation is paused until you press play in the
  debugger.** Sim time will sit at `t=0.000` and no topics will carry data. This
  also means wall-clock timings from a GUI run are meaningless; measure headless.
- SuperDex link actors are named `<bot>/<link>`; the bot prefix is stripped for
  frame names.
- Quaternion storage order is `(x, y, z, w)`, the same as `geometry_msgs`, so
  no reordering is needed.
- The generated URDF and its meshes are committed, so a fresh clone does not
  need the converter — but regenerating requires a SuperDex asset tree, and
  `setup.py` must install `urdf/` and `meshes/` for `package://` to resolve.
- `robot_state_publisher` subscribes to `/joint_states`, so the URDF's joint
  names must match what the sim node publishes exactly. They do, because both
  derive from the same `.superdex_bot` joint list.
- The node runs single-threaded: `rclpy.spin_once(timeout_sec=0.0)` once per
  step, inside the sim loop. Commands arrive at teleop rates against a 200 Hz
  loop and queues are depth 1, so nothing accumulates, and no executor thread
  contends for the GIL with the physics step.

## Measured performance

All numbers from a laptop with default power management, real-time factor 1.00
throughout. Nothing here is estimated.

### Throughput and timing

Headless and unpaced, the simulation runs at roughly **5.5x real time**
(~0.9 ms/step), leaving about 4 ms per step of budget at 200 Hz.

Paced, over 30 s runs of 6001 steps:

| Configuration | Steps missing their deadline |
|---|---|
| `hold`, TF off | 18 (0.3%) |
| `twist`, TF off | 16 (0.3%) |
| `twist` + TF at 50 Hz + teleop | 23 (0.4%) |

The spread is within run-to-run noise, so neither ROS publishing nor arm motion
measurably affects deadline adherence at 200 Hz. The residual is the host's
scheduling floor — `time.sleep()` granularity on a non-realtime kernel.

Recording changes this: a `ros2 bag record` over seven topics costs about 2% of
deadlines on its own. The measurement configuration is more expensive than the
configuration being measured.

### Bridge

| Metric | Result |
|---|---|
| `/joint_states` interarrival | 5.000 ms, sd 0.000, over 7095 messages — no drops, no jitter |
| Command latency, `/target_pose` to `/ee_target` | 6.6 ± 2.3 ms (range 5–10) |
| Command latency, fully instrumented | 9.8 ± 2.5 ms (range 5–15) |

Latency is one to two simulation steps. The two rows bracket it: the first is
the bridge's own cost, the second is with `/tf` at 200 Hz from both trees and a
recorder running, which is the worst case rather than the normal one.

### Correctness

| Check | Result |
|---|---|
| Link and joint counts vs. the engine | 38 links, 27 revolute joints — exact |
| Joint names vs. `/joint_states` | exact, including the actor/bot DOF index offset |
| URDF forward kinematics vs. engine link transforms, default pose | ≤ 0.07 mm, limited by the precision of the reference values |
| `sim_fr3_link8` vs. `fr3_link8`, 20 trials, both trees at 200 Hz | **0.21 um** position, **0.02 mdeg** rotation (max) |

The last row is the end-to-end check: the engine's own link transform against
one computed independently by `robot_state_publisher` from the generated URDF
and the published joint states. Agreement at the float32 floor means the URDF
conversion, the DOF-to-joint-name mapping and the message path are all exact.
See [the 20 Hz trap](#the-20-hz-trap) for why the default configuration reports
600 um instead.

### Task: Cartesian waypoint reaching

20 targets sampled from a reachable box, orientation pinned hand-down, driven
through `/target_pose` and tracked by the shipped Cartesian impedance
controller. Reproduced across four runs:

| Metric | Result |
|---|---|
| Final position error | 4.2 ± 1.3 mm (range 2.2–6.6) |
| Final orientation error | 1.9 ± 0.5 deg |
| Settle time | 0.75 ± 0.24 s (range 0.24–1.21) |
| Path deviation from the straight line | 16.8 ± 6.1 mm (max 30.4) |
| Settled within 10 mm | 20/20 |

**The error floor is the finding, not the success rate.** An earlier run at a
5 mm tolerance reported 15/20 — but every trial in that run, pass and fail
alike, landed between 2.3 mm and 5.7 mm. The threshold sat inside the error
distribution, so the "success rate" measured where the line was drawn rather
than whether the arm reached. One trial flipped between settling and timing out
across two runs of the same seed with 4.9 mm error both times. The tolerance was
raised to 10 mm *after* measuring the floor, and the analysis script now flags
this condition automatically.

Position error correlates with travel distance (r = +0.37) but not with
orientation error (r = +0.10), which rules out the arm trading position against
the pinned orientation and points instead at the controller's 5 cm error clamp
throttling the approach.

Path deviation is worth keeping as a baseline: this is Cartesian impedance
control with no path planning, so 17 mm of wander from the direct route is what
a planner would later be compared against.

### Task: joint-space policy rollout

`example_policy` driving all 27 DOFs at 50 Hz against the 200 Hz simulation,
with the default arm gains (`arm_kp = 150`):

| Metric | Result |
|---|---|
| Steady-state error, held joint | ~1 mrad |
| Driven joints | track a 0.25 rad, 6 s sinusoid |

A per-joint tracking error over several cycles — the joint-space counterpart to
the 4.2 mm Cartesian figure — is still to do; it is a join of `/joint_command`
and `/joint_states` on joint name and timestamp, both of which a recorded
session already contains.

### Reproducing

```bash
ros2 launch superdex_ros2 trials.launch.py \
    seed:=0 n_targets:=20 tolerance:=0.010 tf_rate:=200.0 rsp_rate:=200.0

.venv-ros/bin/python tools/analyze_trials.py trials/<timestamp>
```

The trial runner records only *when* each trial happened; every metric is
computed offline from the bag, so a definition can be changed — a different
tolerance, a different settle rule — without re-running the simulation.

## License

Apache-2.0, matching upstream. `sim_node.py` derives from
`superdex_robotics/examples/control/example_osc_jsc_control.py`.