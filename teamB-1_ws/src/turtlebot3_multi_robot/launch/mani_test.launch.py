#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# def is_valid_to_launch():
#     # Path includes model name of Raspberry Pi series
#     path = '/sys/firmware/devicetree/base/model'
#     if os.path.exists(path):
#         return False
#     else:
#         return True

def generate_launch_description():
    # if not is_valid_to_launch():
    #     print('Cannot launch fake robot in Raspberry Pi')
    #     return LaunchDescription([])

    # Launch configurations
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')

    # Gazebo world configuration
    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [
                FindPackageShare('turtlebot3_manipulation_bringup'),
                'worlds',
                'turtlebot3_world.model'
            ]
        )
    )

    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # URDF/Xacro configurations for OpenManipulator-X
    manipulator_urdf = PathJoinSubstitution([
        FindPackageShare('turtlebot3_manipulation_description'),
        'urdf',
        'open_manipulator_x.urdf.xacro'
    ])

    manipulator_pose = {'x': LaunchConfiguration('manip_x', default='0.0'),
                        'y': LaunchConfiguration('manip_y', default='0.0'),
                        'z': LaunchConfiguration('manip_z', default='0.01'),
                        'R': LaunchConfiguration('manip_roll', default='0.00'),
                        'P': LaunchConfiguration('manip_pitch', default='0.00'),
                        'Y': LaunchConfiguration('manip_yaw', default='1.57')}  # 90 degrees rotation

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to execute rviz2'),

        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.'),

        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Directory of Gazebo world file'),

        DeclareLaunchArgument(
            'x_pose',
            default_value=pose['x'],
            description='Position of turtlebot3'),

        DeclareLaunchArgument(
            'y_pose',
            default_value=pose['y'],
            description='Position of turtlebot3'),

        DeclareLaunchArgument(
            'z_pose',
            default_value=pose['z'],
            description='Position of turtlebot3'),

        DeclareLaunchArgument(
            'roll',
            default_value=pose['R'],
            description='Orientation of turtlebot3'),

        DeclareLaunchArgument(
            'pitch',
            default_value=pose['P'],
            description='Orientation of turtlebot3'),

        DeclareLaunchArgument(
            'yaw',
            default_value=pose['Y'],
            description='Orientation of turtlebot3'),

        DeclareLaunchArgument(
            'manip_x',
            default_value=manipulator_pose['x'],
            description='Position of open_manipulator_x'),

        DeclareLaunchArgument(
            'manip_y',
            default_value=manipulator_pose['y'],
            description='Position of open_manipulator_x'),

        DeclareLaunchArgument(
            'manip_z',
            default_value=manipulator_pose['z'],
            description='Position of open_manipulator_x'),

        DeclareLaunchArgument(
            'manip_roll',
            default_value=manipulator_pose['R'],
            description='Orientation of open_manipulator_x'),

        DeclareLaunchArgument(
            'manip_pitch',
            default_value=manipulator_pose['P'],
            description='Orientation of open_manipulator_x'),

        DeclareLaunchArgument(
            'manip_yaw',
            default_value=manipulator_pose['Y'],
            description='Orientation of open_manipulator_x'),

        # Include base launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/base.launch.py']),
            launch_arguments={
                'start_rviz': start_rviz,
                'prefix': prefix,
                'use_sim': use_sim,
            }.items(),
        ),

        # Include Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('gazebo_ros'),
                            'launch',
                            'gazebo.launch.py'
                        ]
                    )
                ]
            ),
            launch_arguments={
                'verbose': 'false',
                'world': world,
            }.items(),
        ),

        # Spawn TurtleBot3 Manipulation System
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'turtlebot3_manipulation_system',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
            ],
            output='screen',
        ),
    ])
