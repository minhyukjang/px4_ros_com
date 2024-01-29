#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions.execute_process import ExecuteProcess
import os
import signal

def generate_launch_description():
    if os.environ['ROS_DISTRO'] != "galactic" and os.environ['ROS_DISTRO'] != "rolling":
        return LaunchDescription([
            ExecuteProcess(
                cmd=['micrortps_agent', '-t', 'UDP'],
                output='screen',
                name='micrortps_agent'
            ),
            OpaqueFunction(function=launch_subscribers_and_publishers),
        ])
    else:
        return LaunchDescription([
            ExecuteProcess(
                cmd=['micrortps_agent', '-t', 'UDP'],
                output='screen',
                name='micrortps_agent'
            ),
            OpaqueFunction(function=launch_subscribers_and_publishers),
        ])

def launch_subscribers_and_publishers(context, *args, **kwargs):
    micrortps_agent_process = context['micrortps_agent']
    micrortps_agent_process.wait_for_shutdown()

    if micrortps_agent_process.is_shutdown:
        print("MicroRTPS Agent has terminated. Launching px4_ros_com node.")
        return [
            Node(
                package='px4_ros_com',
                executable='offboard_control',
                output='screen',
                name='offboard_control'
            )
        ]
    else:
        print("Unexpected error: MicroRTPS Agent did not terminate as expected.")
        return []