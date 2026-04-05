from setuptools import find_packages, setup

package_name = 'agv_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'adafruit-circuitpython-mpu6050',
        'adafruit-circuitpython-busdevice',
        'adafruit-circuitpython-register',
        'adafruit-circuitpython-vl53l0x',
        'adafruit-circuitpython-ads1x15',
        'adafruit-circuitpython-pca9685',
        'opencv-python',
        'pyzbar',
    ],
    zip_safe=True,
    maintainer='gaurav',
    maintainer_email='gaurav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'motor_test = agv_motor_controller.motor_test_node:main',
        'encoder_test = agv_motor_controller.encoder_test_node:main',
        'imu6050_node = agv_motor_controller.imu6050_node:main',
        'imu_node = agv_motor_controller.imu_node:main',
        'tof_test = agv_motor_controller.tof_test_node:main',
        'tof_test_2 = agv_motor_controller.tof_test_node_2:main',
        'line_follower = agv_motor_controller.line_follower_node:main',
        'servo_control = agv_motor_controller.servo_control_node:main',
        'camera_node = agv_motor_controller.camera_node:main',
        'camera_test = agv_motor_controller.camera_test:main',
        'grid_path_planner_node = agv_motor_controller.grid_path_planner_node:main',
        'path_result_monitor = agv_motor_controller.path_result_monitor:main',
        'serial_node = agv_motor_controller.serial_node:main',
    ],
},
)
