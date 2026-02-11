# Copyright 2025 Lihan Chen
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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import os


def generate_launch_description():
    config_filepath = LaunchConfiguration("config_filepath")
    declare_config_filepath_cmd = DeclareLaunchArgument(
        "config_filepath",
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    get_package_share_directory("autopilot_server"), "config", ""
                )
            ),
            TextSubstitution(text="params.yaml"),
        ],
    )
    autopilot_server_node = Node(
        package="autopilot_server",
        executable="autopilot_server_node",
        output="screen",
        parameters=[config_filepath],
    )

    ld = LaunchDescription()

    ld.add_action(declare_config_filepath_cmd)
    ld.add_action(autopilot_server_node)

    return ld
