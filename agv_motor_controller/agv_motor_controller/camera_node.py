import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import socket
import numpy as np
import cv2
import struct
import time
from threading import Thread
import json

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

        # Subscriber for path results from grid path planner
        self.path_subscription = self.create_subscription(
            String,
            'path_result',
            self.path_result_callback,
            10
        )

        # Path tracking
        self.path_nodes = []  # Current path nodes
        self.current_node_index = 0  # Index of current node in path
        self.detected_nodes = []  # Track detected QR codes

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

    def path_result_callback(self, msg: String):
        """
        Callback for path_result topic - updates the current path nodes
        """
        try:
            result = json.loads(msg.data)
            
            if result.get('status') == 'success':
                # Extract path nodes from the result
                self.path_nodes = result.get('path', [])
                self.current_node_index = 0
                self.detected_nodes = []
                
                self.get_logger().info(f"Path received: {' → '.join(self.path_nodes)}")
                self.get_logger().info(f"Total nodes in path: {len(self.path_nodes)}")
            else:
                self.get_logger().warn(f"Path query error: {result.get('message', 'Unknown error')}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse path result: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in path_result_callback: {e}")

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

    def preprocess_frame(self, gray):
        """
        Preprocess grayscale image for better QR code detection
        Conservative approach: enhance without destroying QR patterns
        """
        # Step 1: Upscale if image is too small (helps with small QR codes)
        height, width = gray.shape
        if height < 480 or width < 640:
            scale_factor = 2
            gray = cv2.resize(gray, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
        
        # Step 2: Denoise with bilateral filtering (preserves edges, removes noise)
        denoised = cv2.bilateralFilter(gray, 9, 75, 75)
        
        # Step 3: Enhance contrast with CLAHE (adaptive histogram equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(denoised)
        
        # Step 4: Light sharpening with unsharp masking
        gaussian = cv2.GaussianBlur(enhanced, (3, 3), 0)
        sharpened = cv2.addWeighted(enhanced, 1.3, gaussian, -0.3, 0)
        
        result = np.clip(sharpened, 0, 255).astype(np.uint8)
        return result

    def process_frame(self, frame_data):
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        img = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        if img is None:
            return

        # FPS calculation
        now = time.time()
        self.fps = 1.0 / (now - self.prev_time)
        self.prev_time = now

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Preprocess frame for better QR detection
        gray_processed = self.preprocess_frame(gray)

        # Try detection on both raw and processed images (use whichever works better)
        retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(gray_processed)
        
        # If not detected on processed image, try raw image
        if not retval or decoded_info is None or len([d for d in decoded_info if d]) == 0:
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

                # Check if QR code matches current path node
                if self.path_nodes:
                    self.check_path_node(data.upper())

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

        # Create side-by-side comparison (Original vs Preprocessed)
        gray_processed_colored = cv2.cvtColor(gray_processed, cv2.COLOR_GRAY2BGR)
        
        # Add labels to the images
        cv2.putText(gray_processed_colored, "PREPROCESSED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, "ORIGINAL + DETECTION", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Resize to same height if needed
        h_orig = img.shape[0]
        h_proc = gray_processed_colored.shape[0]
        if h_orig != h_proc:
            gray_processed_colored = cv2.resize(gray_processed_colored, (gray_processed_colored.shape[1], h_orig))
        
        # Create horizontal stack
        comparison = cv2.hconcat([gray_processed_colored, img])
        
        # Show windows
        cv2.imshow("QR Detection: Preprocessed vs Original", comparison)
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

    def check_path_node(self, qr_code_data: str):
        """
        Check if detected QR code matches the expected path node
        """
        if not self.path_nodes:
            self.get_logger().warn("No path loaded")
            return

        expected_node = self.path_nodes[self.current_node_index]

        if qr_code_data == expected_node:
            # QR code matches expected node
            self.detected_nodes.append(qr_code_data)
            self.get_logger().info(f"✓ PATH NODE FOUND: {qr_code_data} (Node {self.current_node_index + 1}/{len(self.path_nodes)})")
            
            # Move to next node
            if self.current_node_index < len(self.path_nodes) - 1:
                self.current_node_index += 1
                next_node = self.path_nodes[self.current_node_index]
                self.get_logger().info(f"Next expected node: {next_node}")
            else:
                self.get_logger().info("✓ ALL NODES DETECTED - PATH COMPLETE!")
        else:
            # QR code doesn't match expected node
            self.get_logger().warn(
                f"✗ UNEXPECTED NODE: Expected {expected_node}, but detected {qr_code_data}"
            )

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