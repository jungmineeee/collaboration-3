import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.actions import Shutdown
from launch.event_handlers import OnShutdown
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare
from urdf_parser_py.urdf import URDF
import launch.logging


"""
base2.launch.py 연결 -> namespace 있는 버전
모든 연결에 namespace 추가.
=> 이렇게 하니까 lifecycle에서 변경된 namespace 인식하지 못해서 걔를 건드려야하는 상황
그냥 namespace 없애는게 나을듯.
"""

def generate_launch_description():
    ld = LaunchDescription()

    TURTLEBOT3_MODEL = 'waffle'


    # Names and poses of the robots
    robots = [
        {'name': 'turtle1', 'x_pose': '0.000753', 'y_pose': '-7.913351', 'z_pose': 0.01},
        {'name': '', 'x_pose': '7', 'y_pose': '0', 'z_pose': 0.01},
        ]


    # shutdown 시 실행할 커맨드들을 하나의 스크립트로 합치기
    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[[
                        FindExecutable(name='bash'), '-c',
                        'ros2 node kill /map_server /lifecycle_manager_map_server /turtle1/robot_state_publisher /turtle2/robot_state_publisher /turtle1/rviz /turtle2/rviz; '
                        'ros2 daemon stop; '
                        'pkill -f rviz2; '
                        'pkill -f gazebo; '
                        'pkill -f robot_state_publisher'
                    ]],
                    name='cleanup_process',
                    output='screen'
                )
            ]
        )
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )
    
    # 패키지 경로 설정
    turtlebot3_manipulation_description = get_package_share_directory('turtlebot3_manipulation_description')
    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    # URDF 파일 경로 설정
    urdf_manipulation = os.path.join(
        turtlebot3_manipulation_description,
        'urdf',
        'turtlebot3_manipulation.urdf.xacro'
    )

    package_dir = get_package_share_directory('turtlebot3_multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    world = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        'worlds', 'project_world3.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
     
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)


    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # HOME 밑에 있는 내 yaml 파일로 변경
    map_server=Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(os.path.expanduser('~'), 'map.yaml'),
        }],
        remappings=remappings
    )

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)
    
    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # xacro 파일을 URDF로 변환
    # robot_description = Command([
    #             FindExecutable(name='xacro'), ' ',
    #             urdf_manipulation,
    #             'prefix:=/turtle2',  # 네임스페이스 또는 프리픽스 전달
    #             'use_sim:=true'  # Gazebo 시뮬레이션 사용
    #             ])

    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        if robot['name'] == '':
            # turtle2는 turtlebot3_manipulation 모델 사용
            # robot_description = Command([
            #     FindExecutable(name='xacro'), ' ',
            #     urdf_manipulation  # TurtleBot3 Manipulation URDF/Xacro 파일
            # ])

            # State publisher for turtle2
            # turtlebot_state_publisher = Node(
            #     package='robot_state_publisher',
            #     namespace=namespace,
            #     executable='robot_state_publisher',
            #     output='screen',
            #     parameters=[{
            #         'robot_description': robot_description,
            #         'use_sim_time': use_sim_time,
            #         'publish_frequency': 10.0
            #     }],
            #     remappings=remappings
            # )
            # turtle2의 경우 base.launch_2.py 포함
            base_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('turtlebot3_manipulation_bringup'),
                    '/launch/base_2.launch.py'
                ]),
                launch_arguments={
                    'start_rviz': 'false',
                    'prefix': '""',
                    'use_sim': 'true',
                    'use_fake_hardware': 'false',
                    'fake_sensor_commands': 'false'
                }.items()
            )
            # Spawn node
            yaw = '3.14159'  # 180도 회전
            spawn_turtlebot = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', robot['name'],
                    '-robot_namespace', namespace,
                    '-x', robot['x_pose'],
                    '-y', robot['y_pose'],
                    '-z', '0.01',
                    '-Y', yaw,
                    '-unpause',
                ],
                output='screen',
            )

            #ld.add_action(base_launch)
            #model_file = urdf_manipulation  # URDF/Xacro 경로
        else:
            # turtle1용 일반 설정
            turtlebot_state_publisher = Node(
                package='robot_state_publisher',
                namespace=namespace,
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 10.0
                }],
                remappings=remappings,
                arguments=[urdf, '-robot_namespace', namespace,],
            )
            yaw = '1.5708'  # 90도 회전
            spawn_turtlebot = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', os.path.join(turtlebot3_multi_robot, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                    '-entity', robot['name'],
                    '-robot_namespace', namespace,
                    '-x', robot['x_pose'],
                    '-y', robot['y_pose'],
                    '-z', '0.01',
                    '-Y', yaw,
                    '-unpause',
                ],
                output='screen',
            )

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': os.path.join(os.path.expanduser('~'), 'map.yaml'),
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                        )

        if last_action is None:
            # 첫 번째 로봇은 바로 실행
            #ld.add_action(turtlebot_state_publisher)
            if robot['name'] == 'turtle2':
                ld.add_action(base_launch)  # base_launch를 먼저 실행
            elif robot['name'] == 'turtle1':
                ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot)
            ld.add_action(bringup_cmd)
            last_action = spawn_turtlebot
        else:
            # 두 번째 로봇의 경우
            if robot['name'] == 'turtle2':
                spawn_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        #target_action=base_launch,
                        on_exit=[
                            #turtlebot_state_publisher,
                            base_launch,  # base_launch를 spawn 전에 실행
                            spawn_turtlebot,
                            bringup_cmd
                        ],
                    )
                )
            else:
                spawn_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[
                            turtlebot_state_publisher,
                            spawn_turtlebot,
                            bringup_cmd
                        ],
                    )
                )
            ld.add_action(spawn_event)
            last_action = spawn_turtlebot
        # Save last instance for next RegisterEventHandler
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned

    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # 추가 - 각 로봇별로 다른 initial pose 설정
        if robot['name'] == 'turtle1':
            message = '''{
                header: {frame_id: map},
                pose: {
                    pose: {
                        position: {x: 0.5349944829940796, y: -8.263012886047363, z: 0.0},
                        orientation: {x: 0.0, y: 0.0, z: -0.002112606714249749, w: 0.9999977684439455}
                    }
                }
            }'''
        elif robot['name'] == 'turtle2':
            message = '''{
                header: {frame_id: map},
                pose: {
                    pose: {
                        position: {x: 8.358158111572266, y: -15.332627296447754, z: 0.0},
                        orientation: {x: 0.0, y: 0.0, z: 0.7071812807060384, w: 0.707032273817092}
                    }
                }
            }'''

        # Create a initial pose topic publish call
        # message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
        #     robot['x_pose'] + ', y: ' + robot['y_pose'] + \
        #     ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )


        # Use RegisterEventHandler to ensure next robot rviz launch happens 
        # only after all robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        ld.add_action(post_spawn_event)
        last_action = initial_pose_cmd
        #last_action = post_spawn_event
        
    # 마지막 initial_pose_cmd 이후에 box spawner 실행
    random_box_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_action,  # 마지막 initial_pose_cmd// 이제 post_spawn_event를 참조
            on_exit=[
                Node(
                    package='execute_gazebo',
                    executable='spawn_boxes',
                    name='object_spawner',
                    output='screen'
                    )
                ]
            )
        )

    
    ld.add_action(random_box_spawner)

    ######################
    # 기존 LaunchDescription에 Shutdown 핸들러 추가
    ld.add_action(shutdown_handler)
    return ld
