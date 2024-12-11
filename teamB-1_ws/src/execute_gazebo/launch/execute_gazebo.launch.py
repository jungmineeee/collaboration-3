from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    # Launch Gazebo with a specified world file
    gazebo_world = ExecuteProcess(
        cmd=['gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so','../project_world'],
        output='screen'
    )
    
    object_spawner_node = TimerAction(
        period = 4.0,  # Delay in seconds
        actions = [
            Node(
                package='execute_gazebo',  # Replace with your package name
                executable='spawn_boxes',  # Replace with your executable name
                name='object_spawner',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_world,
        object_spawner_node,
    ])
