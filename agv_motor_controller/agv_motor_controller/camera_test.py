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
import re

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
        self.intersection_publisher = self.create_publisher(String, 'camera/intersection', 10)
        self.motor_command_publisher = self.create_publisher(String, 'motor_command', 10)

        # Subscriber
        self.path_subscription = self.create_subscription(
            String,
            'path_result',
            self.path_result_callback,
            10
        )

        # Path tracking
        self.path_nodes = []
        self.current_node_index = 0
        self.visited_nodes = []
        self.navigation_actions = []
        self.last_node_event_time = 0.0
        self.node_event_cooldown_sec = 1.0

        # Heading estimation config
        self.declare_parameter('agv_heading_offset_deg', 0.0)
        self.declare_parameter('invert_east_west', False)
        self.agv_heading_offset_deg = float(self.get_parameter('agv_heading_offset_deg').value)
        self.invert_east_west = bool(self.get_parameter('invert_east_west').value)
        self.current_agv_heading = 'UNKNOWN'
        self.pending_expected_heading = 'UNKNOWN'
        self.pending_command = None
        self.pending_command_source_heading = None
        self.last_motor_command = 'NONE'
        self.command_sent_this_frame = False

        self.bridge = CvBridge()

        self.prev_time = time.time()
        self.fps = 0.0

        self.qr_detector = cv2.QRCodeDetector()

        self.setup_socket()

        self.running = True
        self.thread = Thread(target=self.receive_frames, daemon=True)
        self.thread.start()

        self.get_logger().info(f"Listening on {self.udp_ip}:{self.udp_port}")
        self.get_logger().info(
            f"Heading config: offset={self.agv_heading_offset_deg:.1f} deg, invert_east_west={self.invert_east_west}"
        )

    def setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(2)

    def path_result_callback(self, msg: String):
        try:
            result = json.loads(msg.data)
            if result.get('status') == 'success':
                self.path_nodes = result.get('path', [])
                self.navigation_actions = [item.get('action', '').upper() for item in result.get('navigation', [])]
                self.current_node_index = 0
                self.visited_nodes = []
                if len(self.path_nodes) > 1:
                    first_heading = self.get_heading_between_nodes(self.path_nodes[0], self.path_nodes[1])
                    self.pending_expected_heading = first_heading if first_heading else 'UNKNOWN'
                else:
                    self.pending_expected_heading = 'DONE'
                self.pending_command = None
                self.pending_command_source_heading = None
                self.get_logger().info(f"🗺️  PATH RECEIVED: {' → '.join(self.path_nodes)}")
                self.get_logger().info(f"Total nodes: {len(self.path_nodes)}")
                if self.navigation_actions:
                    self.get_logger().info(f"Planner actions: {' | '.join(self.navigation_actions)}")
        except Exception as e:
            self.get_logger().error(f"Path parse error: {e}")

    def receive_frames(self):
        MARKER = 0xDEADBEEF

        while self.running:
            try:
                while True:
                    data, _ = self.sock.recvfrom(4096)
                    if len(data) == 4 and struct.unpack("I", data)[0] == MARKER:
                        break

                data, _ = self.sock.recvfrom(4096)
                frame_size = struct.unpack("I", data)[0]

                frame_data = bytearray()
                while len(frame_data) < frame_size:
                    packet, _ = self.sock.recvfrom(4096)
                    frame_data.extend(packet)

                self.process_frame(frame_data)

            except Exception as e:
                self.get_logger().error(f"UDP Error: {e}")

    # =========================
    # � PATH & NAVIGATION
    # =========================
    def normalize_angle(self, angle_deg):
        """Normalize angle to [-180, 180)."""
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def node_to_xy(self, node_label):
        """Convert planner node labels to grid coordinates used by the planner."""
        if not node_label:
            return None

        m = re.match(r'^([A-D])(\d)$', node_label)
        if m:
            col = ord(m.group(1)) - ord('A')
            row = int(m.group(2))
            return float(col), float(4 - row)

        m = re.match(r'^([A-D])([A-D])(\d)$', node_label)
        if m:
            c1 = ord(m.group(1)) - ord('A')
            c2 = ord(m.group(2)) - ord('A')
            row = int(m.group(3))
            return (c1 + c2) / 2.0, float(4 - row)

        m = re.match(r'^([A-D])(\d)(\d)$', node_label)
        if m:
            col = ord(m.group(1)) - ord('A')
            r1 = int(m.group(2))
            r2 = int(m.group(3))
            return float(col), (4 - (r1 + r2) / 2.0)

        m = re.match(r'^DOC-([A-D])([A-D])(\d)$', node_label)
        if m:
            c1 = ord(m.group(1)) - ord('A')
            c2 = ord(m.group(2)) - ord('A')
            row = int(m.group(3))
            return (c1 + c2) / 2.0, float(4 - row) + 0.5

        m = re.match(r'^HOME-(\d)$', node_label)
        if m:
            row = int(m.group(1))
            return 3.5, float(4 - row)

        return None

    def get_heading_between_nodes(self, from_node, to_node):
        """Get absolute heading from one path node to the next node."""
        p1 = self.node_to_xy(from_node)
        p2 = self.node_to_xy(to_node)
        if p1 is None or p2 is None:
            return None

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None

        if abs(dx) >= abs(dy):
            return "EAST" if dx > 0 else "WEST"
        return "NORTH" if dy > 0 else "SOUTH"

    def get_agv_heading_from_qr(self, qr_edge_angle_deg):
        """
        Estimate AGV heading from detected QR orientation.
        Heading convention:
        - angle around   0 deg -> NORTH
        - angle around -90 deg -> EAST
        - angle around +90 deg -> WEST
        - angle around +/-180 deg -> SOUTH
        """
        agv_heading_angle = self.normalize_angle(qr_edge_angle_deg + self.agv_heading_offset_deg)

        if -45.0 <= agv_heading_angle < 45.0:
            heading = "NORTH"
        elif -135.0 <= agv_heading_angle < -45.0:
            heading = "EAST"
        elif 45.0 <= agv_heading_angle < 135.0:
            heading = "WEST"
        else:
            heading = "SOUTH"

        # Use this when camera feed is mirrored left-right.
        if self.invert_east_west:
            if heading == "EAST":
                return "WEST"
            if heading == "WEST":
                return "EAST"
        return heading

    def apply_relative_command_to_heading(self, current_heading, command):
        """Return expected absolute heading after applying planner command."""
        heading_order = ["NORTH", "EAST", "SOUTH", "WEST"]
        if current_heading not in heading_order or not command:
            return None

        command = command.upper()
        current_idx = heading_order.index(current_heading)

        if command == "STRAIGHT":
            delta = 0
        elif command == "RIGHT":
            delta = 1
        elif command == "UTURN":
            delta = 2
        elif command == "LEFT":
            delta = 3
        else:
            return None

        return heading_order[(current_idx + delta) % 4]

    def publish_motor_command(self, direction):
        """Publish motor command to navigate in given direction."""
        msg = String()
        msg.data = direction
        self.motor_command_publisher.publish(msg)
        self.last_motor_command = direction
        self.command_sent_this_frame = True
        self.get_logger().info(f"🎯 MOTOR COMMAND: {direction}")

    # =========================
    # �🚀 IMPROVED INTERSECTION DETECTION
    # =========================
    def detect_intersection(self, gray):
        _, thresh = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)

        h, w = thresh.shape
        cx, cy = w // 2, h // 2

        offset = 80
        region_w = 60
        region_h = 40

        def check_region(x1, y1, x2, y2):
            region = thresh[y1:y2, x1:x2]
            if region.size == 0:
                return False, 0.0

            white_ratio = np.sum(region) / (255 * region.size)
            return white_ratio > 0.15, white_ratio

        # Regions
        up = (cx - region_w//2, cy - offset - region_h,
              cx + region_w//2, cy - offset)

        down = (cx - region_w//2, cy + offset,
                cx + region_w//2, cy + offset + region_h)

        left = (cx - offset - region_w, cy - region_h//2,
                cx - offset, cy + region_h//2)

        right = (cx + offset, cy - region_h//2,
                 cx + offset + region_w, cy + region_h//2)

        # Detection
        up_det, up_ratio = check_region(*up)
        down_det, down_ratio = check_region(*down)
        left_det, left_ratio = check_region(*left)
        right_det, right_ratio = check_region(*right)

        count = sum([up_det, down_det, left_det, right_det])

        if count == 4:
            intersection = "CROSS"
        elif count == 3:
            intersection = "T-JUNCTION"
        elif count == 2:
            if (up_det and down_det) or (left_det and right_det):
                intersection = "STRAIGHT"
            else:
                intersection = "TURN"
        elif count == 1:
            intersection = "DEAD END"
        else:
            intersection = "NO LINE"

        debug = {
            "regions": {
                "up": (up, up_det, up_ratio),
                "down": (down, down_det, down_ratio),
                "left": (left, left_det, left_ratio),
                "right": (right, right_det, right_ratio),
            },
            "thresh": thresh
        }

        return intersection, debug

    def process_frame(self, frame_data):
        self.command_sent_this_frame = False

        frame = np.frombuffer(frame_data, dtype=np.uint8)
        img = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        if img is None:
            return

        now = time.time()
        self.fps = 1.0 / (now - self.prev_time)
        self.prev_time = now

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 🚀 INTERSECTION DETECTION
        intersection_type, debug = self.detect_intersection(gray)
        intersection_detected = intersection_type not in ["NO LINE", "DEAD END"]

        msg_int = String()
        msg_int.data = intersection_type
        self.intersection_publisher.publish(msg_int)

        # 🔍 DRAW DEBUG REGIONS
        for key, (rect, detected, ratio) in debug["regions"].items():
            x1, y1, x2, y2 = rect
            color = (0, 255, 0) if detected else (0, 0, 255)

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, f"{key}:{ratio:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # 🔥 QR DETECTION
        qr_detected = False
        detected_qr_data = None
        detected_qr_angle = None
        
        retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(gray)

        if retval and points is not None:
            for data, pts in zip(decoded_info, points):
                if not data:
                    continue

                qr_detected = True
                detected_qr_data = data.upper()

                pts = pts.astype(int)
                cv2.polylines(img, [pts], True, (0, 255, 0), 2)

                angle = self.calculate_orientation(pts)
                detected_qr_angle = angle

                x, y = pts[0]
                cv2.putText(img, f"QR: {data}", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(img, f"Angle: {angle:.1f}", (x, y + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                msg = String()
                msg.data = f"{data} | Angle: {angle:.2f}"
                self.qr_code_publisher.publish(msg)

        # ✅ PATH TRACKING: NODE + CORRECT HEADING
        heading_ok_for_node = False
        if qr_detected and detected_qr_angle is not None:
            current_heading_for_gate = self.get_agv_heading_from_qr(detected_qr_angle)
            heading_ok_for_node = (
                self.pending_expected_heading in ["UNKNOWN", "DONE"]
                or current_heading_for_gate == self.pending_expected_heading
            )

        node_event_detected = qr_detected and heading_ok_for_node
        if node_event_detected:
            self.handle_node_detection(detected_qr_data, detected_qr_angle)

        # Update heading overlay from latest QR orientation
        if qr_detected and detected_qr_angle is not None:
            self.current_agv_heading = self.get_agv_heading_from_qr(detected_qr_angle)
        if self.pending_command and self.pending_command_source_heading in ["NORTH", "EAST", "SOUTH", "WEST"]:
            exp = self.apply_relative_command_to_heading(self.pending_command_source_heading, self.pending_command)
            self.pending_expected_heading = exp if exp else "UNKNOWN"
        elif self.path_nodes and self.current_node_index == 0 and len(self.path_nodes) > 1 and self.pending_command is None:
            start_heading = self.get_heading_between_nodes(self.path_nodes[0], self.path_nodes[1])
            self.pending_expected_heading = start_heading if start_heading else "UNKNOWN"
        elif self.current_node_index >= len(self.path_nodes) - 1 and self.path_nodes:
            self.pending_expected_heading = "DONE"
        else:
            self.pending_expected_heading = "UNKNOWN"

        # DISPLAY
        cv2.putText(img, f"FPS: {self.fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(img, f"Intersection: {intersection_type}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        node_event_text = "YES" if node_event_detected else "NO"
        node_event_color = (0, 255, 0) if node_event_detected else (0, 0, 255)
        cv2.putText(img, f"Node+CorrectHeading: {node_event_text}", (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, node_event_color, 2)

        # Display path progress
        if self.path_nodes:
            path_str = " → ".join(self.path_nodes)
            visited_str = " → ".join(self.visited_nodes) if self.visited_nodes else "None"
            cv2.putText(img, f"Path: {path_str}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            cv2.putText(img, f"Visited: {visited_str}", (10, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            
            expected = self.path_nodes[self.current_node_index] if self.current_node_index < len(self.path_nodes) else "DONE"
            cv2.putText(img, f"Expected: {expected}", (10, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 1)
            cv2.putText(img, f"AGV Heading: {self.current_agv_heading}", (10, 185),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
            cv2.putText(img, f"Expected Heading: {self.pending_expected_heading}", (10, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 1)
            cv2.putText(img, f"Last Motor Cmd: {self.last_motor_command}", (10, 235),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cmd_sent_text = "YES" if self.command_sent_this_frame else "NO"
            cmd_sent_color = (0, 255, 0) if self.command_sent_this_frame else (0, 0, 255)
            cv2.putText(img, f"Cmd Sent This Frame: {cmd_sent_text}", (10, 260),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, cmd_sent_color, 1)
            mirror_text = "ON" if self.invert_east_west else "OFF"
            cv2.putText(img, f"Invert E/W: {mirror_text}", (10, 285),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 1)

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_publisher.publish(ros_img)

        cv2.imshow("Camera View", img)
        cv2.imshow("Threshold Debug", debug["thresh"])

        cv2.waitKey(1)

    def handle_node_detection(self, detected_qr, detected_qr_angle):
        """
        Handle detection of a node (intersection + QR code).
        Verify it matches expected node and update path progress.
        """
        now = time.time()
        if now - self.last_node_event_time < self.node_event_cooldown_sec:
            return

        if not self.path_nodes:
            self.get_logger().warn("❌ No path loaded!")
            return
        
        if self.current_node_index >= len(self.path_nodes):
            self.get_logger().info("✅ PATH COMPLETE! All nodes visited.")
            return
        
        expected_node = self.path_nodes[self.current_node_index]
        
        if detected_qr == expected_node:
            # ✅ CORRECT NODE DETECTED
            self.visited_nodes.append(detected_qr)
            self.get_logger().info(f"✅ NODE FOUND: {detected_qr} at position {self.current_node_index + 1}/{len(self.path_nodes)}")
            self.last_node_event_time = now
            
            current_heading = self.get_agv_heading_from_qr(detected_qr_angle) if detected_qr_angle is not None else None
            if current_heading:
                self.current_agv_heading = current_heading

            # Validate previously issued planner command at arrival to this node.
            if self.pending_command and self.pending_command_source_heading and current_heading:
                expected_heading = self.apply_relative_command_to_heading(
                    self.pending_command_source_heading,
                    self.pending_command
                )
                self.pending_expected_heading = expected_heading if expected_heading else "UNKNOWN"

                if expected_heading == current_heading:
                    self.get_logger().info(
                        f"✅ HEADING VALID after {self.pending_command}: expected={expected_heading}, current={current_heading}"
                    )
                else:
                    self.get_logger().warn(
                        f"⚠️ HEADING MISMATCH after {self.pending_command}: expected={expected_heading}, current={current_heading}"
                    )

            # Use planner-provided action for this node (no node-difference turn calculation here).
            if self.current_node_index < len(self.path_nodes) - 1:
                planner_action = None
                if self.current_node_index < len(self.navigation_actions):
                    planner_action = self.navigation_actions[self.current_node_index]

                if planner_action in ["LEFT", "RIGHT", "STRAIGHT", "UTURN"]:
                    self.publish_motor_command(planner_action)
                    self.pending_command = planner_action
                    self.pending_command_source_heading = current_heading
                    exp = self.apply_relative_command_to_heading(current_heading, planner_action) if current_heading else None
                    self.pending_expected_heading = exp if exp else "UNKNOWN"
                elif planner_action == "START":
                    self.pending_command = None
                    self.pending_command_source_heading = None
                    self.pending_expected_heading = "UNKNOWN"
                elif planner_action == "STOP":
                    self.publish_motor_command("STOP")
                    self.pending_command = None
                    self.pending_command_source_heading = None
                    self.pending_expected_heading = "DONE"
                
                # Move to next node
                self.current_node_index += 1
                next_node = self.path_nodes[self.current_node_index]
                self.get_logger().info(f"➡️  Next target: {next_node}")
            else:
                self.get_logger().info("🎉 DESTINATION REACHED!")
                self.publish_motor_command("STOP")
                self.pending_command = None
                self.pending_command_source_heading = None
                self.pending_expected_heading = "DONE"
        else:
            # ❌ WRONG NODE DETECTED
            self.get_logger().warn(f"❌ WRONG NODE! Expected {expected_node}, but detected {detected_qr}")

    def calculate_orientation(self, pts):
        pt1 = pts[0]
        pt2 = pts[1]

        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]

        return np.degrees(np.arctan2(dy, dx))

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