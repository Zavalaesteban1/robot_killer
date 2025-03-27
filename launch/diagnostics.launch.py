#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Diagnostics node
    diagnostics_node = Node(
        package='robot_killer',
        executable='hardware_diagnostics_node.py',
        name='hardware_diagnostics_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Optional: Add a terminal node to display diagnostics in console
    diagnostics_display_node = Node(
        package='topic_tools',
        executable='echo',
        name='diagnostics_display',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=['/diagnostics_summary']
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Nodes
        diagnostics_node,
        diagnostics_display_node
    ]) 