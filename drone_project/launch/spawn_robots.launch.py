import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')

    # Gazebo와 드론 실행
    drone_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drone_pkg, 'launch', 'sjtu_drone_bringup.launch.py')
        ),
    )

    # TurtleBot3 스폰
    tb3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'TURTLEBOT3_MODEL': 'waffle_pi',
            'x_pose': '0.5',
            'y_pose': '0.5'
            }.items(),
    )

    # 드론 이륙
    takeoff_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/simple_drone/takeoff', 'std_msgs/msg/Empty', '{}'],
        shell=True,
        output="screen"
    )

    # Gazebo 초기화 후 실행하도록 지연 추가
    nodes_to_run = [
        drone_bringup,  # 먼저 Gazebo 실행
        TimerAction(
            period=8.0,  # 8초 대기 (필요 시 조정)
            actions=[
                takeoff_cmd,  # 드론 이륙
                tb3_bringup   # TurtleBot3 스폰
            ]
        )
    ]

    return LaunchDescription(nodes_to_run)