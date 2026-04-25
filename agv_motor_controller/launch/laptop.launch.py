from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_motor_controller',
            executable='camera_test',
            name='camera_test',
            output='screen',
        ),
        Node(
            package='agv_motor_controller',
            executable='grid_path_planner_node',
            name='grid_path_planner_node',
            output='screen',
        ),
        Node(
            package='agv_motor_controller',
            executable='realtime_grid_visualizer_node',
            name='realtime_grid_visualizer_node',
            output='screen',
        ),
    ])
