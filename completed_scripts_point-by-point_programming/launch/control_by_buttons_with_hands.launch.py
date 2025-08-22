# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="buttons_server",
                executable="server_stand_up_with_hands",
            ),

            Node(
                package="buttons_server",
                executable="hear_server",
            ),
            
            Node(
                package="button_analyzer",
                executable="button_analyzer_with_hands",
            ),
        ]
    )
