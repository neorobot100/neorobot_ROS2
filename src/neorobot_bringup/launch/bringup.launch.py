from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    return LaunchDescription([

        # 1. 라이다
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_node',
            output='screen',
            parameters=[{
                'port': '/dev/ydlidar',
                'baudrate': 230400,
                'frame_id': 'laser_frame',
                'fixed_resolution': True,
                #'fixed_point_count': 934,
                'scan_frequency': 8.0   # G4 안정 주파수
            }]
        ),
        

        # 2. 오도메트리 노드
        # 오도메트리 & 모터 제어
        Node(
            package='uart_driver',
            executable='uart_node',
            name='uart_node',
            output='screen'
        ),

        # 3. LiDAR 위치 TF (18도 회전 적용)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.10',
                '--roll', '3.141592',  # 좌우반전
                '--pitch', '0',
                '--yaw','0.27', #약 15.14도
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame'
            ]
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
            parameters=['/home/neorobot/teleop_speed.yaml']
        ),


        # 4. SLAM
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'slam_mode': 'mapping',
        #         'scan_topic': '/scan',

        #         'odom_frame': 'odom',
        #         'base_frame': 'base_link',
        #         'map_frame': 'map',

        #         'provide_odom_frame': True,
        #         'resolution': 0.05,
        #         # 'minimum_travel_distance': 0.002,
        #         # 'minimum_travel_heading': 0.002
        #         'minimum_travel_distance': 0.0,
        #         'minimum_travel_heading': 0.0
        #     }]
        # ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[{
                        '/home/neorobot/ros2_ws/src/ap_uart_rx/config/slam.yaml'
                        'use_sim_time': False,
                        'slam_mode': 'mapping',

                        'scan_topic': '/scan',
                        

                        'odom_frame': 'odom',
                        'base_frame': 'base_link',
                        'map_frame': 'map',

                        # 해상도
                        'resolution': 0.05,

                        # 이동 조건 완전 제거
                        # 'minimum_travel_distance': 0.0,
                        # 'minimum_travel_heading': 0.0,
                        'minimum_travel_distance': 0.05,
                        'minimum_travel_heading': 0.05,


                        # 맵 갱신 강제
                        # 'map_update_interval': 0.2,
                        'map_update_interval': 0.5,

                        # scan 매칭 약화 (거의 무시)
                        'use_scan_matching': True,
                        'use_scan_barycenter': True,
                        # loop closure 비활성화
                        #'do_loop_closing': False,

                        # TF 강제 출력
                        #'transform_publish_period': 0.02,

                        # 노이즈 감소
                        'throttle_scans': 2,
                        'scan_buffer_size': 10,


                        # localization 방지
                        'mode': 'mapping'
                    }]
                )
                
            ]
        ),

        # 5. lifecycle manager
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_slam',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['slam_toolbox']
                    }]
                )
            ]
        ),
                    
    ])
