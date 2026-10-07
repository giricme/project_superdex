# Quickstart

Commands only. Everything is explained in [README.md](README.md).

Paths assume `project_superdex/` (the fork) and `ros2_ws/` are siblings.

## One-time setup

```bash
source /opt/ros/jazzy/setup.bash          # BEFORE creating the venv
cd ros2_ws
uv venv --system-site-packages .venv-ros  # --system-site-packages is required
uv pip install --python .venv-ros/bin/python superdex
.venv-ros/bin/python -c "import rclpy, yaml, superdex.physics; print('ok')"

mkdir -p src
ln -sfn ../../project_superdex/superdex_ros2 src/superdex_ros2
ln -sfn ../../project_superdex/superdex_ros2_msgs src/superdex_ros2_msgs
```

Generate the robot model (once; the output is committed):

```bash
cd ../project_superdex
python3 superdex_ros2/tools/bot_to_urdf.py \
  --bot assets/bots/arm_hand_combos/fr3_dg5f_short/right/fr3_dg5f_short_right.superdex_bot \
  --output-dir superdex_ros2/urdf
```

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
.venv-ros/bin/python -m colcon build --symlink-install   # -m colcon, not colcon
source install/setup.bash
```

Rebuild after adding a **new** file; `--symlink-install` covers edits to existing ones.

## Run

```bash
# simulator only
ros2 launch superdex_ros2 sim.launch.py arm_mode:=circle

# simulator + robot_state_publisher + RViz
ros2 launch superdex_ros2 display.launch.py arm_mode:=circle

# with the SuperDex debugger (press play in it, or nothing steps)
ros2 launch superdex_ros2 display.launch.py arm_mode:=circle gui:=true
```

`arm_mode`: `circle` (shipped demo) · `pose` (`/target_pose`) · `twist`
(`/cmd_vel`) · `hold` (stay put) · `joint` (`/joint_command`).

In RViz, first time only: Fixed Frame → `world`, **Add** → RobotModel →
Description Topic `/robot_description`. Then File → Save Config As.

## Drive it

```bash
# keyboard teleop -- hold a key; q/w/e only change speed
ros2 launch superdex_ros2 display.launch.py arm_mode:=twist
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# single Cartesian target
ros2 launch superdex_ros2 display.launch.py arm_mode:=pose
ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.45, y: 0.0, z: 0.5},
    orientation: {x: 1.0, w: 0.0}}}'

# policy rollout over all 27 DOFs
ros2 launch superdex_ros2 display.launch.py arm_mode:=joint
ros2 run superdex_ros2 example_policy

# contacts (off by default; empty in the stock scene -- nothing to touch)
ros2 launch superdex_ros2 display.launch.py publish_contacts:=true
ros2 topic echo /contacts --once
```

## Measure

```bash
cd ros2_ws
for s in 0 1 2; do
  ros2 launch superdex_ros2 trials.launch.py \
      seed:=$s n_targets:=20 tolerance:=0.010 tf_rate:=200.0 rsp_rate:=200.0
done

.venv-ros/bin/python ../project_superdex/superdex_ros2/tools/analyze_trials.py \
    trials/<timestamp>          # once per session

# pool every seed into one figure set
python3 ../project_superdex/superdex_ros2/tools/plot_metrics.py \
    trials/<ts-seed0> trials/<ts-seed1> trials/<ts-seed2> \
    --outdir ../project_superdex/report/figures
```

Results land in `trials/<timestamp>/`: `summary.txt`, `metrics_trials.csv`,
`metrics_residual.csv`, `figures/`.

## Checks

```bash
ros2 topic hz /joint_states          # ~200
ros2 topic hz /tf                    # ~50, or tf_rate
ros2 topic echo /ee_pose --once

# TF lookups need sim time, or they fail as extrapolation errors
ros2 run tf2_ros tf2_echo world fr3_link8 --ros-args -p use_sim_time:=true

# bridge vs. URDF forward kinematics -- should read identity
ros2 run tf2_ros tf2_echo sim_fr3_link8 fr3_link8 --ros-args -p use_sim_time:=true
```

## If it misbehaves

| Symptom | Fix |
|---|---|
| `No module named 'superdex'` | Built with the wrong interpreter — use `.venv-ros/bin/python -m colcon build` |
| `No module named 'yaml'` | venv missing `--system-site-packages`; recreate it |
| Sim time stuck at `t=0.000` | `gui:=true` — press play in the debugger |
| `no assets root found` | `export SUPERDEX_ASSETS_PATH=<fork>/assets` |
| Nothing in RViz | Default config: set Fixed Frame and add RobotModel (above) |
| "unconnected trees" / "jump back in time" | `pkill -f sim_node`, relaunch, restart RViz |
| RViz window never opens | `rviz_gl:=hardware`, or see README for the Pop!\_OS notes |
| Launch file not found | New file — rebuild |
| `superdex_ros2_msgs is not importable` | `colcon build --packages-select superdex_ros2_msgs`, then re-source `install/setup.bash` |