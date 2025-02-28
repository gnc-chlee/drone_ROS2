from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sjcu_package',
            executable='timer_node',
            name='sjcu_timer_node_test'
        ),
        Node(
            package='sjcu_package',
            executable='fast_timer_node',
            name='sjcu_fast_timer_node_test',
        )
    ])