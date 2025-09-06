# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # get value param 'mode'
    mode = LaunchConfiguration('mode').perform(context)
    
    hear_server_node = Node(
        package="buttons_server",
        executable="hear_server",
        namespace='web_face',
    )
    
    # Choose config depended by 'mode'
    if mode == 'with_hands':
        button_analyzer_node = Node(
                package="button_analyzer",
                executable="button_analyzer_with_hands",
                namespace='web_face',
                name='button_analyzer_with_hands',
                remappings=[('positions_to_unitree', '/positions_to_unitree')],
        )
        button_server_node = Node(
                package="buttons_server",
                executable="server_stand_up_with_hands",
                namespace='web_face',
                name='server_stand_up_with_hands',
        )

    elif mode == 'without_hands':
        button_analyzer_node = Node(
                package="button_analyzer",
                executable="button_analyzer_without_hands",
                name='button_analyzer_without_hands',
                namespace='web_face',
                remappings=[('positions_to_unitree', '/positions_to_unitree')],
        )
        button_server_node = Node(
                package="buttons_server",
                executable="server_stand_up_without_hands",
                name='server_stand_up_without_hands',
                namespace='web_face',
        )

    else:
        raise ValueError(f"Unknown mode: {mode}. Use 'with_hands' or 'without_hands'")
    

    return [hear_server_node, button_analyzer_node, button_server_node]


def generate_launch_description():
    # declare param for choice mode
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='with_hands',
        description='Режим работы: with_hands или without_hands.',
        choices=['with_hands', 'without_hands']
    )
    
    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=launch_setup)
    ])