from launch import LaunchDescription                                            # 실행할 것들을 모두 리스트에 담
from launch_ros.actions import Node                                             # ROS2 노드를 실행시키는 객체  , ros2 run 처럼 노드 실행
from launch.actions import IncludeLaunchDescription, TimerAction                # 다른 launch 파일을 불러와서 같이 실행 ,TimerAction : 노드 실행 순서를 조절할 때 사용
from launch.launch_description_sources import PythonLaunchDescriptionSource     # IncludeLaunchDescription에서 “python launch 파일임”을 알려주는 객체
from ament_index_python.packages import get_package_share_directory             # 설치된 ROS2 패키지의 share 폴더 경로를 가져옴
import os                                                                       # 운영체제(OS)와 관련된 작업을 하기 위한 모듈 ,파일 경로 만들기,환경 변수 읽기 , 디렉토리 확인, 파일 존재 여부 확인

from launch_ros.actions import PushRosNamespace                      # 실행되는 노드들을 특정 namespace 안에 넣음, 멀티 로봇 /robot1/.../robot2/. , Nav2 멀티 인스턴스 /robot1/map/robot2/map

from launch.actions import LogInfo
#자체 수정용 파일
#LaunchDescription
#  ├── Node              (노드 실행)
#  ├── IncludeLaunchDescription  (다른 launch 실행)
#  │       └── PythonLaunchDescriptionSource
#  ├── TimerAction       (지연 실행)
#  └── DeclareLaunchArgument

# 실행 순서
# map_server → amcl → planner → controller → bt_navigator

def generate_launch_description():

    #params_file = '/home/neorobot/my_nav2_params.yaml'
    # params_file = '/home/neorobot/ros2_ws/src/neorobot_bringup/config/nav2.yaml'

    params_file = os.path.join(
        get_package_share_directory('neorobot_bringup'),
        'config',
        'nav2.yaml'
    )
    map_file = os.path.join(
        get_package_share_directory('neorobot_bringup'),
        'maps',
        # 'my_map_gimp.yaml'
        # 'my_map_new.yaml'
        'empty_map.yaml'
    )
    
    return LaunchDescription([

        # map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                # 'yaml_filename': '/home/neorobot/empty_map.yaml',
                'use_sim_time': False
            }]
        ),

         # 라이다
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_node',
            output='screen',
            parameters=[{
                'port': '/dev/ydlidar',
                'baudrate': 230400,
                'frame_id': 'laser_frame',
                'scan_frequency': 10.0
            }]
        ),

        Node(
            package='neo_robot_safety',
            executable='lidar_human_leg_detect',
            name='lidar_human_leg_detect',
            output='screen'
        ),

       
        # ===== Motion Arbiter =====
        Node(
            package='neo_robot_safety',
            executable='motion_arbiter',
            name='motion_arbiter',
            output='screen'
        ),

       
        # SOUND
        Node(
            package='sound_control',
            executable='sound_play',
            name='sound_play',
            output='screen'
        ),
        # 오도메트리 & 모터 제어
        Node(
            package='uart_driver',
            executable='uart_node',
            name='uart_node',
            output='screen'
        ),

        # 도킹 매니저
        Node(
            package='robot_docking',
            executable='dock_manager',
            name='dock_manager',
            output='screen'
        ),

        # 게임패드 입력
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 조이스틱 → cmd_vel 변환
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=['/home/neorobot/ros2_ws/src/neorobot_bringup/config/teleop_speed.yaml'],
            remappings=[
                ('cmd_vel', '/cmd_vel_joy')
            ],
        ),  

        # base_footprint → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'base_footprint',
                'base_link'
            ]
        ),

        # ================= TF base_link -> laser =================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_tf',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.10',
                '--roll', '3.141592',  # 좌우반전
                '--pitch', '0',
                # '--yaw','0.32', #약 18도
                '--yaw','0.27', #약 15.14도
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame'
            ]
        ),
        

        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="🔥 TIMER STARTED 🔥"),
                # amcl
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[params_file]
                ),

                # --------------planner
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[params_file]
                ),

                #--------------controller
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[params_file],
                    remappings=[('/cmd_vel', '/cmd_vel_nav')]
                ),

                Node(
                    package='neo_robot_controller',
                    executable='lookahead_controller',
                    name='lookahead_controller',
                    output='screen',
                    remappings=[
                        ('/cmd_vel','/cmd_vel_lookahead')
                    ]
                ),

                # ---------------- VELOCITY SMOOTHER ----------------
                # Node( 
                #     package='nav2_velocity_smoother', 
                #     executable='velocity_smoother', 
                #     name='velocity_smoother', 
                #     output='screen', 
                #     parameters=[params_file],
                #     remappings=[ ('cmd_vel', '/cmd_vel_nav'), 
                #         ('cmd_vel_smoothed', '/cmd_vel') 
                #     ] ),

                # # ---------------- COLLISION MONITOR ---------------- 
                # Node( 
                #     package='nav2_collision_monitor',
                #     executable='collision_monitor', 
                #     name='collision_monitor', 
                #     output='screen', 
                #     parameters=[params_file] 
                #     ),




                # bt_navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[params_file]
                ),

                # behavior server (필수!)
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[params_file],
                    remappings=[('/cmd_vel', '/cmd_vel_nav')]
                ),

                # lifecycle manager
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': [
                            'map_server',
                            'amcl',
                            'planner_server',
                            'controller_server',
                            # 'velocity_smoother', #
                            # 'collision_monitor', #
                            'bt_navigator',
                            'behavior_server'
                        ]
                    }]
                )
            ]
        )            
    ])

# view_frames Result PDF 저장 하는법
# ros2 run tf2_tools view_frames

# 전체 노드 어떤 값을 가지고 있는지 알아내는 방법
# for n in $(ros2 node list); do echo "=== $n ==="; ros2 param get $n use_sim_time; done     

# 노드 하나씩 알아내는 법
# ros2 param get /amcl use_sim_time