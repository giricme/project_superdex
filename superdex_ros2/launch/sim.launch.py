"""Launch the SuperDex simulation node.

Every node parameter is exposed as a launch argument, so the usual run modes
are one command:

    ros2 launch superdex_ros2 sim.launch.py
    ros2 launch superdex_ros2 sim.launch.py arm_mode:=twist gui:=true
    ros2 launch superdex_ros2 sim.launch.py duration:=30.0 publish_tf:=false

The node publishes simulated time on /clock, so anything consuming its
timestamps -- RViz, robot_state_publisher, tf2_echo -- needs use_sim_time:=true.
This file does not set it globally, because it only applies to nodes launched
here and silently does nothing for tools started in another terminal; setting it
per tool is less surprising than setting it halfway.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default, description) -- kept in the same order as the node declares
# them, so a parameter added there is easy to mirror here.
ARGS = [
    ("duration", "0.0", "Simulated seconds to run; 0 or less runs until Ctrl-C."),
    ("realtime", "true", "Pace the loop against the wall clock."),
    ("gui", "false", "Launch and attach the SuperDex Physics Debugger."),
    ("arm_mode", "circle", "Arm target source: circle | pose | twist | hold."),
    ("cmd_timeout", "0.5", "Simulated seconds of /cmd_vel silence before the twist is dropped."),
    ("workspace_radius", "0.8", "Maximum target distance from the arm base [m]."),
    ("workspace_z_min", "0.05", "Minimum target height [m]."),
    ("publish_tf", "true", "Broadcast link frames on /tf."),
    ("tf_rate", "50.0", "/tf broadcast rate [Hz]; /joint_states stays at full rate."),
    ("tf_prefix", "sim_", "Prefix for broadcast frame names."),
    ("debug_links", "false", "Print the link/DOF table once at startup."),
]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGS
    ]

    sim_node = Node(
        package="superdex_ros2",
        executable="sim_node",
        name="superdex_sim",
        output="screen",
        emulate_tty=True,  # otherwise Python buffers stdout and the heartbeat
        # stalls in the launch log until the process exits
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in ARGS}],
    )

    return LaunchDescription([*declarations, sim_node])