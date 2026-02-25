import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    decision_maker_cfg_dir = LaunchConfiguration("decision_maker_cfg_dir")

    decision_maker_dir = get_package_share_directory("decision_maker")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_decision_maker_cfg_dir = DeclareLaunchArgument(
        "decision_maker_cfg_dir",
        default_value=PathJoinSubstitution(
            [decision_maker_dir, "config", "params.yaml"]
        ),
        description="Path to the decision maker config file",
    )

    start_decision_maker_node = Node(
        package="decision_maker",
        executable="decision_maker_node",
        namespace=namespace,
        parameters=[decision_maker_cfg_dir],
        remappings=remappings,
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_decision_maker_cfg_dir)
    ld.add_action(start_decision_maker_node)

    return ld
