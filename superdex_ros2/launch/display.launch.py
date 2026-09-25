"""Launch the sim node alongside robot_state_publisher, for RViz display.

    ros2 launch superdex_ros2 display.launch.py
    ros2 launch superdex_ros2 display.launch.py arm_mode:=twist rviz:=false

Two frame trees run at once, deliberately:

  world -> sim_fr3_link0, sim_fr3_link1, ...   from the sim node, flat,
                                               straight out of the engine
  world -> base -> fr3_link0 -> fr3_link1 ...  from robot_state_publisher,
                                               kinematic, computed from the
                                               URDF and /joint_states

They share the `world` root and never collide, because the sim node prefixes
its frames. Differencing a pair -- `sim_fr3_link8` against `fr3_link8` --
measures the URDF conversion and the DOF-to-joint-name mapping together, which
is the point of running both.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

URDF_NAME = "fr3_dg5f_short_right.urdf"


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share = Path(get_package_share_directory("superdex_ros2"))
    urdf_path = share / "urdf" / URDF_NAME
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"{urdf_path} not found. Generate it first:\n"
            "  python3 tools/bot_to_urdf.py --bot <...>.superdex_bot "
            "--output-dir superdex_ros2/urdf\n"
            "then rebuild so it is installed into the package share directory."
        )
    robot_description = urdf_path.read_text()

    arm_mode = LaunchConfiguration("arm_mode")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    software_gl = PythonExpression([
        "'", LaunchConfiguration("rviz"), "' == 'true' and '",
        LaunchConfiguration("rviz_gl"), "' == 'software'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("arm_mode", default_value="circle"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_gl",
            default_value="software",
            choices=["software", "hardware"],
            description="software is slow but reliable; try hardware first.",
        ),

        Node(
            package="superdex_ros2",
            executable="sim_node",
            name="superdex_sim",
            output="screen",
            emulate_tty=True,
            parameters=[{"arm_mode": arm_mode, "gui": gui}],
        ),

        # Everything downstream of the sim node needs use_sim_time: its stamps
        # are simulated seconds counting from zero, and against wall clock every
        # TF lookup fails as an extrapolation error on a perfectly good
        # transform.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": True},
            ],
        ),

        # RViz on Pop!_OS 24.04 (COSMIC/Wayland, hybrid NVIDIA graphics) needs
        # QT_QPA_PLATFORM=xcb to get a window at all -- Ogre, which RViz renders
        # through, has no Wayland backend, so Qt must go via XWayland.
        #
        # Software GL is the safe default because it always works, but it is
        # CPU rasterization: 38 meshes will feel sluggish. Try
        # rviz_gl:=hardware first and keep it if the window appears; the
        # NVIDIA/XWayland path is far faster when it cooperates.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(software_gl),
            parameters=[{"use_sim_time": True}],
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
                "LIBGL_ALWAYS_SOFTWARE": "1",
            },
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("rviz"), "' == 'true' and '",
                LaunchConfiguration("rviz_gl"), "' == 'hardware'",
            ])),
            parameters=[{"use_sim_time": True}],
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
                # Route GL to the discrete GPU rather than the integrated one.
                # Harmless on a single-GPU machine.
                "__NV_PRIME_RENDER_OFFLOAD": "1",
                "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
            },
        ),
    ])