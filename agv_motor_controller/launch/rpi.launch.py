from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_motor_controller',
            executable='ina219_battery_node',
            name='ina219_battery_node',
            output='screen',
        ),
        Node(
            package='agv_motor_controller',
            executable='tof_test_2',
            name='tof_test_2',
            output='screen',
        ),
        Node(
            package='agv_motor_controller',
            executable='motor_command_serial_bridge',
            name='motor_command_serial_bridge',
            output='screen',
        ),
        Node(
            package='agv_motor_controller',
            executable='imu_yaw_node',
            name='imu_yaw_node',
            output='screen',
        ),
    ])
