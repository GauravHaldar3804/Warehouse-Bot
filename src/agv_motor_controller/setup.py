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
    install_requires=['setuptools', 'adafruit-circuitpython-mpu6050', 'adafruit-circuitpython-busdevice', 'adafruit-circuitpython-register'],
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
    ],
},
)
