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

try:
    from pyzbar.pyzbar import decode
    QR_CODE_AVAILABLE = True
    PYZBAR_ERROR = None
except ImportError as e:
    QR_CODE_AVAILABLE = False
    PYZBAR_ERROR = str(e)
except Exception as e:
    QR_CODE_AVAILABLE = False
    PYZBAR_ERROR = str(e)


class CameraNode(Node):
    """ROS2 Node for receiving camera frames via UDP and detecting QR codes."""

    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 1234)
        self.declare_parameter('enable_qr_detection', True)
        self.declare_parameter('enable_visualization', False)
        self.declare_parameter('save_frames', False)
        self.declare_parameter('frame_save_dir', '/tmp/camera_frames')
        
        # Get parameters
        self.udp_ip = self.get_parameter('udp_ip').value
        self.udp_port = self.get_parameter('udp_port').value
        self.enable_qr = self.get_parameter('enable_qr_detection').value
        self.enable_vis = self.get_parameter('enable_visualization').value
        self.save_frames = self.get_parameter('save_frames').value
        self.frame_save_dir = self.get_parameter('frame_save_dir').value
        
        # Create frame save directory if needed
        if self.save_frames:
            import os
            os.makedirs(self.frame_save_dir, exist_ok=True)
        
        if not QR_CODE_AVAILABLE and self.enable_qr:
            self.get_logger().warn(
                f'pyzbar not installed. QR code detection disabled. '
                f'Error: {PYZBAR_ERROR} | '
                f'Install with: pip install pyzbar'
            )
            self.enable_qr = False
        
        # QoS profile for reliable image streaming
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # Publishers
        self.image_publisher = self.create_publisher(
            Image, 'camera/image_raw', qos_profile
        )
        self.qr_code_publisher = self.create_publisher(
            String, 'camera/qr_code', qos_profile
        )
        
        # CvBridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()
        
        # Camera frame counter and timing
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0.0
        
        # UDP socket setup
        self.setup_socket()
        
        # Start UDP receiving thread
        self.running = True
        self.receiver_thread = Thread(target=self.receive_frames, daemon=True)
        self.receiver_thread.start()
        
        self.get_logger().info(
            f'Camera node started - UDP {self.udp_ip}:{self.udp_port}'
        )
        self.get_logger().info(f'QR Detection: {"Enabled" if self.enable_qr else "Disabled"}')
        self.get_logger().info(f'Visualization: {"Enabled" if self.enable_vis else "Disabled"}')
        self.get_logger().info(f'Frame Saving: {"Enabled" if self.save_frames else "Disabled"}')
        if self.save_frames:
            self.get_logger().info(f'Saving frames to: {self.frame_save_dir}')
    
    def setup_socket(self):
        """Initialize UDP socket for receiving frames."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(2)
    
    def receive_frames(self):
        """Thread function to receive UDP frames continuously."""
        MARKER = 0xDEADBEEF
        
        while self.running:
            try:
                # Wait for marker
                marker_received = False
                while not marker_received and self.running:
                    try:
                        data, _ = self.sock.recvfrom(4096)
                    except socket.timeout:
                        continue
                    
                    if len(data) == 4:
                        val = struct.unpack("I", data)[0]
                        if val == MARKER:
                            marker_received = True
                            break
                
                if not self.running:
                    break
                
                # Receive frame size
                frame_size_received = False
                while not frame_size_received and self.running:
                    try:
                        data, _ = self.sock.recvfrom(4096)
                    except socket.timeout:
                        continue
                    
                    if len(data) == 4:
                        frame_size = struct.unpack("I", data)[0]
                        frame_size_received = True
                        break
                
                if not self.running:
                    break
                
                # Receive frame data
                frame_data = bytearray()
                while len(frame_data) < frame_size and self.running:
                    try:
                        packet, _ = self.sock.recvfrom(4096)
                    except socket.timeout:
                        continue
                    
                    if len(packet) + len(frame_data) > frame_size:
                        packet = packet[:frame_size - len(frame_data)]
                    
                    frame_data.extend(packet)
                
                if not self.running:
                    break
                
                # Decode and process frame
                self.process_frame(frame_data)
                
            except Exception as e:
                self.get_logger().error(f'Error in frame reception: {e}')
    
    def process_frame(self, frame_data):
        """Decode and process received frame data."""
        try:
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            img = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            if img is None:
                return
            
            # Calculate FPS
            current_time = time.time()
            delta_time = current_time - self.prev_time
            if delta_time > 0:
                self.fps = 1.0 / delta_time
            self.prev_time = current_time
            self.frame_count += 1
            
            # Detect QR codes
            qr_data = None
            if self.enable_qr:
                qr_data = self.detect_qr_codes(img)
            
            # Add FPS to image
            if self.enable_vis:
                cv2.putText(
                    img, f"FPS: {self.fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
                )
                
                # Add QR detection status
                if qr_data:
                    cv2.putText(
                        img, f"QR Detected: {len(qr_data)}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
                    )
            
            # Publish image
            try:
                ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                ros_image.header.frame_id = "camera"
                ros_image.header.stamp = self.get_clock().now().to_msg()
                self.image_publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {e}')
            
            # Display raw image
            if self.enable_vis:
                try:
                    cv2.imshow("Camera - ESP32 Stream", img)
                    if cv2.waitKey(1) == 27:  # ESC key to exit
                        self.running = False
                except Exception as e:
                    # If display fails, log it once and disable visualization
                    if self.enable_vis:
                        self.get_logger().warn(
                            f'Cannot display window (no X11 server?): {e}. '
                            'Continuing without visualization.'
                        )
                        self.enable_vis = False
            
            # Save frames to disk if enabled
            if self.save_frames:
                try:
                    frame_filename = f"{self.frame_save_dir}/frame_{self.frame_count:06d}.jpg"
                    cv2.imwrite(frame_filename, img)
                except Exception as e:
                    self.get_logger().error(f'Error saving frame: {e}')
            
            # Publish QR codes if detected
            if qr_data:
                for qr in qr_data:
                    msg = String()
                    msg.data = qr
                    self.qr_code_publisher.publish(msg)
        
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
    
    def detect_qr_codes(self, image):
        """Detect QR codes in the image and return their data."""
        try:
            qr_codes = decode(image)
            
            if not qr_codes:
                return None
            
            detected_data = []
            for idx, qr_code in enumerate(qr_codes):
                # Extract QR code data
                data = qr_code.data.decode('utf-8')
                detected_data.append(data)
                
                # Draw QR code bounding box on image (if visualization enabled)
                if self.enable_vis:
                    (x, y, w, h) = qr_code.rect
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Display decoded data on image
                    cv2.putText(
                        image, f"QR: {data[:20]}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                    )
                
                # Print QR code data to console
                print(f"\n{'='*60}")
                print(f"QR Code #{idx + 1} Detected!")
                print(f"{'='*60}")
                print(f"Data: {data}")
                print(f"Type: {qr_code.type}")
                print(f"Position: x={qr_code.rect.left}, y={qr_code.rect.top}")
                print(f"Size: width={qr_code.rect.width}, height={qr_code.rect.height}")
                print(f"{'='*60}\n")
                
                self.get_logger().info(f'QR Code #{idx + 1} detected: {data}')
            
            return detected_data
        
        except Exception as e:
            self.get_logger().error(f'QR detection error: {e}')
            return None
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.running = False
        self.receiver_thread.join(timeout=2.0)
        if self.sock:
            self.sock.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main entry point for the camera node."""
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info('Camera node shutting down...')
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()