from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("trajectory_bridge")
    config_file = os.path.join(pkg_share, "config", "trajectory_bridge.yaml")

    return LaunchDescription([
        # `publish_rate` 还保留为 CLI 覆盖,常用快速调试。`joint_names`
        # 和 `home_positions_rad` 由 yaml 给,要改请直接编辑 config。
        DeclareLaunchArgument(
            "publish_rate",
            default_value="50.0",
            description="Servo command publish rate (Hz)"
        ),

        Node(
            package="trajectory_bridge",
            executable="trajectory_bridge_node",
            name="trajectory_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                config_file,
                {"publish_rate": LaunchConfiguration("publish_rate")},
            ],
        ),
    ])