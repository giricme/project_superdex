# superdex_ros2

A ROS 2 interface to [Project SuperDex](https://github.com/facebookresearch/project_superdex).

SuperDex ships a contact-first physics engine, robot assets and controllers, but
no ROS integration of any kind. This package adds the missing seam: simulated
robot state flows out as standard ROS messages, and Cartesian commands flow in,
so unmodified ROS 2 tooling can observe and drive a SuperDex simulation.

Demonstrated with `teleop_twist_keyboard` — a node written for mobile bases,
used unchanged to drive a 27-DOF FR3 arm with a DG5F dexterous hand.

## Workspace layout

The package lives inside the SuperDex fork so it stays committable; the colcon
workspace lives outside it, so build artifacts never touch the repo and colcon
crawls only `src/` instead of the whole SuperDex tree.

```
16-709/project/
├── project_superdex/                 <- fork (git)
│   └── superdex_ros2/                <- this package
│       ├── superdex_ros2/sim_node.py <- the bridge
│       ├── tools/bot_to_urdf.py      <- .superdex_bot -> URDF converter
│       ├── launch/                   <- sim.launch.py, display.launch.py
│       ├── urdf/  meshes/            <- generated, committed
│       └── package.xml  setup.py
└── ros2_ws/                          <- workspace (not under git)
    ├── .venv-ros/
    ├── src/superdex_ros2 -> ../../project_superdex/superdex_ros2
    ├── build/  install/  log/
```

Symlinking the package into `src/` is a normal ROS practice and keeps one copy
of the source under version control.

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
| `display.launch.py` | Sim node + `robot_state_publisher` + RViz, with `use_sim_time` set on both consumers. Arguments: `arm_mode`, `gui`, `rviz`, `rviz_gl`. |

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

Measured: identity to the tool's printed precision (three decimals) throughout a
full `arm_mode:=circle` sweep. An independent forward-kinematics walk over the
generated URDF at the bot's default pose agrees with engine-reported link
positions to within 0.07 mm, which is the rounding in the comparison rather than
converter error.

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

### `arm_mode`

- `circle` — the upstream example's hardcoded trajectory. Commands ignored.
- `pose` — track the latest `/target_pose`. Holds the startup pose until the
  first message arrives.
- `twist` — integrate `/cmd_vel` into the target. For keyboard and gamepad teleop.
- `hold` — stay at the startup pose. Useful as a control condition when timing.

The hand is unaffected by `arm_mode`; it always runs the example's knuckle sweep.

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

30 s runs, 6001 steps, laptop with default power management, real-time factor
1.00 throughout:

| Configuration | Steps missing their deadline |
|---|---|
| `hold`, TF off | 18 (0.3%) |
| `twist`, TF off | 16 (0.3%) |
| `twist` + TF at 50 Hz + teleop | 23 (0.4%) |

The spread is within run-to-run noise, so neither ROS publishing nor arm motion
measurably affects deadline adherence at 200 Hz. The residual is the host's
scheduling floor — `time.sleep()` granularity on a non-realtime kernel.

Headless and unpaced, the simulation runs at roughly 5.5x real time
(~0.9 ms/step), leaving about 4 ms per step of budget at 200 Hz.

### Correctness

| Check | Result |
|---|---|
| Link and joint counts vs. the engine | 38 links, 27 revolute joints — exact |
| Joint names vs. `/joint_states` | exact, including the actor/bot DOF index offset |
| URDF forward kinematics vs. engine link transforms, default pose | ≤ 0.07 mm, limited by the precision of the reference values |
| `sim_fr3_link8` vs. `fr3_link8` during an `arm_mode:=circle` sweep | identity to `tf2_echo`'s three printed decimals |

The last row is the end-to-end check: the engine's own link transform against
one computed independently by `robot_state_publisher` from the generated URDF
and the published joint states. A logged residual with a stated bound is still
to do — `tf2_echo` rounds too early to quote a number from.

## License

Apache-2.0, matching upstream. `sim_node.py` derives from
`superdex_robotics/examples/control/example_osc_jsc_control.py`.