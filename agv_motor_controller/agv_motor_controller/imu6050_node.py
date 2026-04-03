import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import board
import busio
import adafruit_mpu6050
import math
import time


class IMU6050Node(Node):
    """
    ROS 2 Node for publishing MPU6050 (IMU6050) sensor data.
    Publishes accelerometer, gyroscope, and calculated angles to 'imu/data' topic.
    """

    def __init__(self):
        super().__init__('imu6050_node')

        # Get parameters
        self.publish_rate = self.declare_parameter('publish_rate', 50).value  # Hz
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value

        # MPU6050 setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_mpu6050.MPU6050(self.i2c)

        # Variables for angle calculation
        self.prev_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Calibration offsets (set to 0 initially, adjust if needed)
        self.gyro_x_offset = 0.0
        self.gyro_y_offset = 0.0
        self.gyro_z_offset = 0.0

        # Publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)

        # Create timer for publishing
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_imu_data
        )

        self.get_logger().info(
            f'IMU6050 Node started at {self.publish_rate} Hz'
        )

    def calculate_angles(self, accel_x, accel_y, accel_z):
        """
        Calculate roll and pitch angles from accelerometer data.
        Uses complementary filter combining accelerometer and gyroscope data.
        """
        # Calculate angles from accelerometer (in degrees)
        accel_roll = math.atan2(accel_y, accel_z) * 180 / math.pi
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi

        # Get gyroscope data (in rad/s, convert to deg/s)
        gyro_x, gyro_y, gyro_z = self.sensor.gyro
        gyro_x_deg = (gyro_x - self.gyro_x_offset) * 180 / math.pi
        gyro_y_deg = (gyro_y - self.gyro_y_offset) * 180 / math.pi
        gyro_z_deg = (gyro_z - self.gyro_z_offset) * 180 / math.pi

        # Time delta
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt > 0:
            # Complementary filter (0.96 gyro, 0.04 accel)
            alpha = 0.96
            self.roll = alpha * (self.roll + gyro_x_deg * dt) + (1 - alpha) * accel_roll
            self.pitch = alpha * (self.pitch + gyro_y_deg * dt) + (1 - alpha) * accel_pitch
            self.yaw += gyro_z_deg * dt

        return self.roll, self.pitch, self.yaw, gyro_x_deg, gyro_y_deg, gyro_z_deg

    def publish_imu_data(self):
        """Publish IMU data to the 'imu/data' topic."""
        try:
            # Read accelerometer data (in m/s^2)
            accel_data = self.sensor.acceleration
            accel_x, accel_y, accel_z = accel_data

            # Calculate angles (also returns converted gyro values)
            roll, pitch, yaw, gyro_x_deg, gyro_y_deg, gyro_z_deg = self.calculate_angles(accel_x, accel_y, accel_z)
            
            # Get raw gyroscope data for publishing (in rad/s)
            gyro_x, gyro_y, gyro_z = self.sensor.gyro

            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Linear acceleration
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            # Angular velocity (gyroscope data in rad/s)
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # Orientation (converted to quaternion from angles)
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            # Covariance matrices (initialized with reasonable defaults)
            # Linear acceleration covariance
            imu_msg.linear_acceleration_covariance = [
                0.001, 0.0,    0.0,
                0.0,    0.001, 0.0,
                0.0,    0.0,    0.001
            ]

            # Angular velocity covariance
            imu_msg.angular_velocity_covariance = [
                0.001, 0.0,    0.0,
                0.0,    0.001, 0.0,
                0.0,    0.0,    0.001
            ]

            # Orientation covariance
            imu_msg.orientation_covariance = [
                0.01, 0.0,   0.0,
                0.0,   0.01, 0.0,
                0.0,   0.0,   0.01
            ]

            # Publish message
            self.imu_publisher.publish(imu_msg)

            # Print values to terminal
            print(f'\n--- IMU6050 Data ---')
            print(f'Angles: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°')
            print(f'Accel: X={accel_x:.3f} m/s², Y={accel_y:.3f} m/s², Z={accel_z:.3f} m/s²')
            print(f'Gyro (rad/s): X={gyro_x:.4f}, Y={gyro_y:.4f}, Z={gyro_z:.4f}')
            print(f'Gyro (deg/s): X={gyro_x_deg:.2f}, Y={gyro_y_deg:.2f}, Z={gyro_z_deg:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {str(e)}')

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles (in degrees) to quaternion.
        Returns: qx, qy, qz, qw
        """
        # Convert degrees to radians
        roll_rad = roll * math.pi / 180
        pitch_rad = pitch * math.pi / 180
        yaw_rad = yaw * math.pi / 180

        # Calculate quaternion components
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = IMU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU6050 Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
