import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import socket
import numpy as np
import cv2
import struct
import time
from threading import Thread

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 1234)

        self.udp_ip = self.get_parameter('udp_ip').value
        self.udp_port = self.get_parameter('udp_port').value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', qos_profile)
        self.qr_code_publisher = self.create_publisher(String, 'camera/qr_code', qos_profile)

        self.bridge = CvBridge()

        # FPS
        self.prev_time = time.time()
        self.fps = 0.0

        # OpenCV QR detector
        self.qr_detector = cv2.QRCodeDetector()

        # UDP
        self.setup_socket()

        self.running = True
        self.thread = Thread(target=self.receive_frames, daemon=True)
        self.thread.start()

        self.get_logger().info(f"Listening on {self.udp_ip}:{self.udp_port}")

    def setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(2)

    def receive_frames(self):
        MARKER = 0xDEADBEEF

        while self.running:
            try:
                # Wait for marker
                while True:
                    data, _ = self.sock.recvfrom(4096)
                    if len(data) == 4 and struct.unpack("I", data)[0] == MARKER:
                        break

                # Frame size
                data, _ = self.sock.recvfrom(4096)
                frame_size = struct.unpack("I", data)[0]

                # Frame data
                frame_data = bytearray()
                while len(frame_data) < frame_size:
                    packet, _ = self.sock.recvfrom(4096)
                    frame_data.extend(packet)

                self.process_frame(frame_data)

            except Exception as e:
                self.get_logger().error(f"UDP Error: {e}")

    def process_frame(self, frame_data):
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        img = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        if img is None:
            return

        # FPS calculation
        now = time.time()
        self.fps = 1.0 / (now - self.prev_time)
        self.prev_time = now

        # Convert to grayscale (better decoding)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Multi QR detection
        retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(gray)

        if retval and points is not None:
            for data, pts in zip(decoded_info, points):
                if not data:
                    continue

                pts = pts.astype(int)

                # Draw QR box
                cv2.polylines(img, [pts], True, (0, 255, 0), 2)

                # Compute orientation
                angle = self.calculate_orientation(pts)

                # Draw arrow direction
                center = np.mean(pts, axis=0).astype(int)
                direction = pts[1] - pts[0]
                end_point = center + direction

                cv2.arrowedLine(img, tuple(center), tuple(end_point), (255, 0, 0), 2)

                # Draw readable text background
                x, y = pts[0]
                text = f"QR: {data}"

                (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(img, (x, y - 30), (x + w, y), (0, 0, 0), -1)

                # Draw text
                cv2.putText(img, text, (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Angle text
                cv2.putText(img, f"Angle: {angle:.1f}", (x, y + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # PRINT TO TERMINAL (IMPORTANT)
                print(f"\n===== QR DETECTED =====")
                print(f"Data: {data}")
                print(f"Angle: {angle:.2f} degrees")
                print("=======================\n")

                # Publish QR
                msg = String()
                msg.data = f"{data} | Angle: {angle:.2f}"
                self.qr_code_publisher.publish(msg)

        # FPS display
        cv2.putText(img, f"FPS: {self.fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Publish image
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_publisher.publish(ros_img)

        # Show window
        cv2.imshow("ESP32 Camera Stream", img)
        cv2.waitKey(1)

    def calculate_orientation(self, pts):
        """
        Calculate QR orientation using top-left → top-right vector
        """
        pt1 = pts[0]
        pt2 = pts[1]

        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]

        angle = np.degrees(np.arctan2(dy, dx))
        return angle

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=2.0)
        self.sock.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()