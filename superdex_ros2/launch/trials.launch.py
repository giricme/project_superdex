"""Run a headless waypoint-trial session and record everything to a rosbag.

    ros2 launch superdex_ros2 trials.launch.py
    ros2 launch superdex_ros2 trials.launch.py seed:=1 n_targets:=50

Headless and unattended by design: no debugger (which would pause the loop
waiting for a play button) and no RViz (which competes for the GPU and adds
nothing a bag cannot reconstruct).

Everything needed for the offline analysis goes into the bag, including both
frame trees, so the forward-kinematics residual can be computed after the fact
with no live TF listener. The trial runner writes only a small JSON index
saying when each trial started and finished.

When the runner exits, the whole session shuts down -- which is what closes the
bag cleanly.
"""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

URDF_NAME = "fr3_dg5f_short_right.urdf"

# /tf_static carries world -> base, published once with transient-local
# durability. Without it in the bag the URDF tree is detached from world and
# every offline lookup fails.
RECORD_TOPICS = [
    "/clock",
    "/tf",
    "/tf_static",
    "/joint_states",
    "/ee_pose",
    "/ee_target",
    "/target_pose",
]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share = Path(get_package_share_directory("superdex_ros2"))
    urdf_path = share / "urdf" / URDF_NAME
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"{urdf_path} not found; generate it with tools/bot_to_urdf.py "
            "and rebuild."
        )

    output_dir = LaunchConfiguration("output_dir")
    seed = LaunchConfiguration("seed")
    n_targets = LaunchConfiguration("n_targets")
    tolerance = LaunchConfiguration("tolerance")
    tf_rate = LaunchConfiguration("tf_rate")
    rsp_rate = LaunchConfiguration("rsp_rate")

    trial_runner = Node(
        package="superdex_ros2",
        executable="trial_runner",
        name="trial_runner",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "seed": seed,
                "n_targets": n_targets,
                "tolerance": tolerance,
                "use_sim_time": True,
                "output": PathJoinSubstitution([output_dir, "trials.json"]),
            }
        ],
    )

    return LaunchDescription([
        # `ros2 bag record -o` refuses to write into a directory that already
        # exists: the recorder dies on the spot, the rest of the session runs
        # on regardless, and you are left with a trials.json paired to the
        # PREVIOUS run's bag. Timestamping the default makes that impossible.
        DeclareLaunchArgument(
            "output_dir",
            default_value="trials/" + datetime.now().strftime("%Y%m%d-%H%M%S"),
        ),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("n_targets", default_value="20"),
        DeclareLaunchArgument("tolerance", default_value="0.005"),
        DeclareLaunchArgument("tf_rate", default_value="50.0"),
        DeclareLaunchArgument("rsp_rate", default_value="200.0"),

        Node(
            package="superdex_ros2",
            executable="sim_node",
            name="superdex_sim",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "arm_mode": "pose",
                    "gui": False,
                    "realtime": True,
                    "tf_rate": tf_rate,
                }
            ],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="log",
            parameters=[
                {"robot_description": urdf_path.read_text()},
                {"use_sim_time": True},
                # robot_state_publisher defaults to 20 Hz, which makes IT the
                # coarser of the two frame trees no matter what the sim node
                # does. tf2 interpolates across whichever tree is coarser, so a
                # 50 ms gap lands squarely in the forward-kinematics residual
                # whenever the arm is still moving. Match the simulation rate
                # and there is nothing left to interpolate.
                {"publish_frequency": rsp_rate},
            ],
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "-o", PathJoinSubstitution([output_dir, "bag"]),
                *RECORD_TOPICS,
            ],
            output="log",
        ),

        trial_runner,

        # The runner finishing is the end of the session: stopping here is what
        # closes the bag cleanly rather than leaving it mid-write.
        RegisterEventHandler(
            OnProcessExit(
                target_action=trial_runner,
                on_exit=[EmitEvent(event=Shutdown(reason="trials complete"))],
            )
        ),
    ])