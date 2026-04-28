# Warehouse Bot - Complete System Architecture & Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Hardware Architecture](#hardware-architecture)
3. [Software Layers](#software-layers)
4. [ROS2 Nodes & Topics](#ros2-nodes--topics)
5. [Data Flow & Communication](#data-flow--communication)
6. [GUI Application Architecture](#gui-application-architecture)
7. [Launch Configurations](#launch-configurations)
8. [Message Formats](#message-formats)
9. [Development & Testing](#development--testing)

---

## System Overview

The Warehouse Bot is a complete autonomous ground vehicle (AGV) system designed for warehouse automation. It consists of three integrated layers:

```
┌─────────────────────────────────────────────────────────────┐
│                   DESKTOP GUI (PyQt5)                       │
│  [Home] [Tasks] [Camera] [Map] [AGV Status] [System] [Log]  │
└─────────────────────────────────────────────────────────────┘
                            ↕
                       ROS2 Topics
                            ↕
┌─────────────────────────────────────────────────────────────┐
│                 ROS2 CONTROL LAYER (Laptop/RPi)             │
│  Path Planner | Camera | Battery | Motor Bridge | IMU Node  │
└─────────────────────────────────────────────────────────────┘
                            ↕
                       Serial Bridge
                            ↕
┌─────────────────────────────────────────────────────────────┐
│              HARDWARE (Arduino & Sensors)                    │
│  Motors | Encoders | IMU | Battery Monitor | TOF Sensors    │
└─────────────────────────────────────────────────────────────┘
```

**Key Metrics:**
- Grid Size: 4 columns × 3 rows (12 main intersections + 40+ intermediate nodes)
- Nodes Include: Shelf locations (A1-D3), Home stations (HOME-1 to HOME-4), Dock nodes
- Communication: ROS2 with QoS BEST_EFFORT for real-time topics

---

## Hardware Architecture

### Physical Robot
- **Platform**: Differential drive AGV with encoders
- **Sensors**:
  - MPU6050 IMU (Inertial Measurement Unit) - Yaw/heading detection
  - VL53L0X ToF (Time of Flight) Sensor(s) - Obstacle detection
  - INA219 - Battery voltage/current monitoring
  - Line followers - Path tracking
  - Wheel encoders - Odometry
  
- **Actuators**:
  - DC Motors (2) with PWM control
  - Servo motors for PAN/TILT camera control
  
- **Optional Camera**: ESP32-CAM for QR code detection and grid localization

### Communication
- **Raspberry Pi** (Robot Side): Runs ROS2 nodes, serial communication with Arduino
- **Laptop** (Supervisor): Runs GUI + camera processing, desktop ROS2 nodes
- **Arduino**: Motor control, sensor reading, encoder processing
- **Protocol**: Serial UART (115200 baud) between RPi/Laptop ↔ Arduino

---

## Software Layers

### Layer 1: Firmware (Arduino Sketch)

**Active Sketch:**
- `arduino_line_follower_with_servos.ino` - Integrated motor, servo, and sensor control

**Responsibilities:**
- Read encoders @ 100 Hz and format: `ENC,rf,rb,lb,lf`
- Execute motor commands received via serial (LEFT, RIGHT, STRAIGHT, UTURN, etc.)
- Control servo motors (PAN, TILT for camera gimbal)
- Read line follower sensors for path tracking
- Implement low-level motion control and sensor integration

---

### Layer 2: ROS2 Control Nodes (on RPi or Laptop)

#### A. **Path Planning Node** (`grid_path_planner_node.py`)
```
Subscribes to:  path_query (String, JSON)
Publishes to:   path_result (String, JSON)
Algorithm:      Dijkstra's shortest path
Grid Model:     4×3 intersections + intermediate nodes
```

**Input Format (JSON):**
```json
{"start": "HOME-1", "goal": "A1"}
```

**Output Format (JSON):**
```json
{
  "status": "success",
  "start": "HOME-1",
  "goal": "A1",
  "total_steps": 4,
  "path": ["HOME-1", "A12", "A1"],
  "navigation": [
    {"step": 1, "node": "HOME-1", "action": "START"},
    {"step": 2, "node": "A12", "action": "LEFT"},
    {"step": 3, "node": "A1", "action": "STOP"}
  ]
}
```

#### B. **Camera Node** (`camera_test.py`)
```
Publishes to:   camera/image_raw (sensor_msgs/Image)
Publishes to:   camera/qr_code (std_msgs/String)
Subscribes to:  path_result (String)
Capabilities:   UDP frame reception, QR code detection, heading estimation
QoS:            BEST_EFFORT (real-time video)
```

**Key Functions:**
- Receives video frames via UDP from ESP32-CAM
- Detects QR codes on the warehouse grid
- Extracts node labels and heading angles
- Publishes in format: `"A1 | Angle: 90.5"`
- Executes navigation commands from path planner

#### C. **Motor Command Serial Bridge** (`motor_command_serial_bridge.py`)
```
Subscribes to:  motor_command (String)
Publishes to:   encoder_counts (Int32MultiArray)
Serial:         /dev/ttyUSB0 @ 115200 baud
Functions:      Command routing to Arduino, encoder reading, obstacle detection
```

**Commands Supported:**
```
Motion:     LEFT, RIGHT, STRAIGHT, UTURN, FORWARD, TURN
Special:    STRAIGHT4, FORWARD4, RIGHT6, SPIN360
Control:    START, STOP, OBSTACLE, CLEAR
Camera:     PAN+, PAN-, PANCENTER, TILT+, TILT-, TILTCENTER
```

#### D. **IMU Yaw Node** (`imu_yaw_node.py`)
```
Publishes to:   imu/yaw (Float32)
Publishes to:   imu/yaw_rate (Float32)
Rate:           50 Hz (configurable)
Hardware:       MPU6050 via I2C
```

**Purpose:** Provides heading angle for navigation validation

#### E. **Battery Node** (`ina219_battery_node.py`)
```
Publishes to:   battery_state (BatteryState)
Publishes to:   battery_metrics (String, JSON)
Publishes to:   battery_status_text (String)
Rate:           10 Hz (configurable)
Hardware:       INA219 via I2C
```

**Output Example:**
```json
{
  "percentage": 85.5,
  "percentage_text": "85%",
  "voltage": 11.8,
  "current": 2.3
}
```

#### F. **Grid Visualizer Node** (`realtime_grid_visualizer_node.py`)
```
Subscribes to:  camera/qr_code (String)
Renders to:     OpenCV window (standalone) OR framebuffer (headless)
Grid Model:     4×3 with intermediate nodes & dock areas
Modes:          Window display OR headless frame generation
```

**Features:**
- Real-time grid rendering
- AGV position tracking via QR code
- Node labeling (A1-D3, HOME-1-4, DOC-* etc.)
- Heading visualization with directional arrow
- Dock station display

---

### Layer 3: Desktop GUI Application (PyQt5)

#### Application Structure

```
agv_gui/
├── main.py                          # QApplication entry point, MainWindow controller
├── ros_interface/
│   └── ros_node.py                 # DashboardRosNode: topic subscriptions/publishers
├── pages/
│   ├── home.py                     # Dashboard with status cards & navigation buttons
│   ├── task_management.py          # Mission planning & execution
│   ├── agv_status.py               # Real-time AGV state monitoring
│   ├── system_status.py            # Hardware health (battery, IMU, camera, motors)
│   ├── activity_log.py             # Event logging with timestamps
│   ├── warehouse_map.py            # Embedded realtime_grid_visualizer rendering
│   └── camera.py                   # Live camera feed display
├── widgets/
│   └── map_widget.py               # Custom map rendering (legacy)
```

#### GUI Pages & Features

**1. Home Page** (Dashboard)
- Real-time AGV status indicator
- Battery level progress bar (%)
- Position, state, task status cards
- Navigation buttons to other pages

**2. Task Management Page**
- Dropdown selectors for start rack & drop location
- "Send Mission" button → publishes to `path_query`
- Receives from `path_result` → displays path & instructions
- Mission state tracking

**3. Camera Page** (NEW)
- Live camera feed from `camera/image_raw`
- Embedded ROS node (`embedded_camera_viewer_node`)
- ~15 FPS refresh rate
- Connection status indicator

**4. Warehouse Map Page** (NEW - Embedded Visualizer)
- Runs `RealtimeGridVisualizerNode` in headless mode
- Displays OpenCV frame rendered as Qt pixmap
- Shows AGV position, grid nodes, heading angle
- Real-time updates from camera QR code topic

**5. AGV Status Page**
- Position (from `camera/qr_code`)
- Heading angle (from IMU or camera)
- Motor commands
- Obstacle status (from `/tof/obstacle_detected`)
- State machine (Idle, Navigating, Stopped, etc.)

**6. System Status Page**
- Battery: voltage, current, percentage
- Camera: active/inactive
- IMU: yaw angle, yaw rate
- Motors: last command
- ToF: obstacle detection status

**7. Activity Log Page**
- Timestamped events
- Mission updates
- Error messages
- Motor commands
- Real-time scrollable list

---

## ROS2 Nodes & Topics

### Node Topology (Laptop)

| Node | Type | Publisher | Subscriber |
|------|------|-----------|-----------|
| `grid_path_planner_node` | Planning | `path_result` | `path_query` |
| `camera_test` | Sensing | `camera/image_raw`, `camera/qr_code` | `path_result` |
| `realtime_grid_visualizer_node` | Viz | — | `camera/qr_code` |
| `agv_dashboard_node` | GUI | `path_query`, `motor_command` | `path_result`, `motor_command`, `camera/qr_code`, `battery_state`, `/tof/obstacle_detected` |

### Node Topology (RPi)

| Node | Type | Publisher | Subscriber |
|------|------|-----------|-----------|
| `motor_command_serial_bridge` | Control | `encoder_counts`, `encoder_counts_text` | `motor_command` |
| `imu_yaw_node` | Sensing | `imu/yaw`, `imu/yaw_rate` | — |
| `ina219_battery_node` | Sensing | `battery_state`, `battery_metrics` | — |
| `tof_test_2` | Sensing | `/tof/obstacle_detected` | — |
### ROS Topics Reference

| Topic | Message Type | Direction | Producer | Consumer | Rate |
|-------|--------------|-----------|----------|----------|------|
| `path_query` | String (JSON) | → | GUI TaskPage | grid_path_planner | On demand |
| `path_result` | String (JSON) | ← | grid_path_planner | GUI + camera_test | On demand |
| `motor_command` | String | → | GUI + camera_test | motor_command_serial_bridge | Variable |
| `camera/image_raw` | sensor_msgs/Image | ← | camera_test | GUI CameraPage | ~30 FPS |
| `camera/qr_code` | String | ← | camera_test | GUI, grid_visualizer | ~10 Hz |
| `camera/intersection` | String | ← | camera_test | — | ~5 Hz |
| `battery_state` | BatteryState | ← | ina219_battery_node | GUI | 10 Hz |
| `battery_metrics` | String (JSON) | ← | ina219_battery_node | GUI | 10 Hz |
| `imu/yaw` | Float32 | ← | imu_yaw_node | camera_test | 50 Hz |
| `imu/yaw_rate` | Float32 | ← | imu_yaw_node | — | 50 Hz |
| `encoder_counts` | Int32MultiArray | ← | motor_command_serial_bridge | — | 200 Hz |
| `encoder_counts_text` | String | ← | motor_command_serial_bridge | — | 200 Hz |
| `/tof/obstacle_detected` | Bool | ← | tof_test_2 | GUI, motor_bridge | On event |

---

## Data Flow & Communication

### Mission Execution Flow

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. USER INITIATES MISSION                                       │
│    - Selects start rack (e.g., A1) and drop location (e.g., HOME-1)
│    - Clicks "Send Mission" button in Task Page               │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 2. TASK PAGE → PATH QUERY                                      │
│    Publishes JSON to path_query topic:                         │
│    {"start": "A1", "goal": "HOME-1"}                           │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 3. PATH PLANNER COMPUTES PATH                                  │
│    - Runs Dijkstra on 4×3 grid                                │
│    - Generates direction commands (LEFT, RIGHT, STRAIGHT, etc.) │
│    - Publishes to path_result                                 │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 4. TASK PAGE + CAMERA_TEST RECEIVE PATH                        │
│    - Task page displays path on GUI                            │
│    - camera_test stores path & begins execution               │
│    - Publishes START command                                  │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 5. MOTOR BRIDGE RECEIVES COMMANDS                              │
│    - Routes commands to Arduino via serial                    │
│    - Arduino executes motor movements                         │
│    - Encoders return encoder data                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 6. CAMERA DETECTS QR CODE AT EACH NODE                         │
│    - Reads QR code containing node label                      │
│    - Publishes node label to camera/qr_code                  │
│    - Sends next navigation command                           │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ 7. GUI UPDATES IN REAL-TIME                                    │
│    - AGV Status page shows current position                   │
│    - Warehouse Map page visualizes AGV on grid                │
│    - Activity Log records events                              │
└─────────────────────────────────────────────────────────────────┘
```

### Sensor Monitoring Flow (Parallel)

```
┌──────────────────────────────┐         ┌────────────────┐
│    Battery Monitor (INA219)  │         │   IMU (MPU)    │
└──────────────────────────────┘         └────────────────┘
         10 Hz publish                        50 Hz publish
              ↓                                    ↓
┌──────────────────────────────┐         ┌────────────────┐
│   battery_state topic        │         │  imu/yaw topic │
└──────────────────────────────┘         └────────────────┘
              ↓                                    ↓
┌──────────────────────────────────────────────────────────┐
│  DashboardRosNode (GUI) receives all status topics     │
│  - Updates internal _status dict                       │
│  - GUI pages read _status via get_status_snapshot()   │
└──────────────────────────────────────────────────────────┘
```

---

## GUI Application Architecture

### Main Application Flow

**File: `agv_gui/main.py`**
```python
def main():
    rclpy.init()  # Initialize ROS2
    ros_node = DashboardRosNode()  # Create dashboard bridge
    
    app = QApplication()  # Create Qt application
    window = MainWindow(ros_node)  # Create main window with all pages
    window.show()
    
    # Timer-based ROS spin (non-blocking)
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    spin_timer.start(30)  # 30 ms = ~33 Hz
    
    app.exec_()  # Run event loop
```

### ROS Bridge Node (`ros_interface/ros_node.py`)

The `DashboardRosNode` acts as the central ROS interface for the GUI:

**Subscriptions (Input from ROS):**
- `battery_state` → updates battery percentage, voltage, current
- `battery_metrics` → JSON battery data
- `camera/qr_code` → AGV position via QR code label
- `path_result` → Path planning results
- `motor_command` → Echo of sent commands
- `/tof/obstacle_detected` → Obstacle status

**Publications (Output to ROS):**
- `path_query` → Send path planning requests
- `motor_command` → Send motor commands

**Status Dictionary:**
```python
self._status = {
    'connection': 'ROS Node Active',
    'battery': '85%',
    'position': 'A1',
    'target': 'HOME-1',
    'state': 'Navigating',
    'last_motor_command': 'LEFT',
    'obstacle_detected': False,
    'system_battery': 'Active',
    'system_camera': 'Active',
    'system_motor': 'Active',
}
```

---

## Launch Configurations

### Laptop Launch (`launch/laptop.launch.py`)

Starts vision and path planning nodes:

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_motor_controller',
            executable='camera_test',
            name='camera_test',
        ),
        Node(
            package='agv_motor_controller',
            executable='grid_path_planner_node',
            name='grid_path_planner_node',
        ),
        Node(
            package='agv_motor_controller',
            executable='realtime_grid_visualizer_node',
            name='realtime_grid_visualizer_node',
        ),
    ])
```

**Start with:**
```bash
ros2 launch agv_motor_controller laptop.launch.py
```

### RPi Launch (`launch/rpi.launch.py`)

Starts hardware control and sensor nodes:

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_motor_controller',
            executable='ina219_battery_node',
            name='ina219_battery_node',
        ),
        Node(
            package='agv_motor_controller',
            executable='tof_test_2',
            name='tof_test_2',
        ),
        Node(
            package='agv_motor_controller',
            executable='motor_command_serial_bridge',
            name='motor_command_serial_bridge',
        ),
        Node(
            package='agv_motor_controller',
            executable='imu_yaw_node',
            name='imu_yaw_node',
        ),
    ])
```

**Start with:**
```bash
ros2 launch agv_motor_controller rpi.launch.py
```

---

## Message Formats

### Path Query (JSON)
```json
{
  "start": "A1",
  "goal": "HOME-1"
}
```

### Path Result (JSON)
```json
{
  "status": "success",
  "start": "A1",
  "goal": "HOME-1",
  "total_steps": 5,
  "path": ["A1", "A12", "A2", "A23", "HOME-1"],
  "navigation": [
    {"step": 1, "node": "A1", "action": "START"},
    {"step": 2, "node": "A12", "action": "STRAIGHT"},
    {"step": 3, "node": "A2", "action": "RIGHT"},
    {"step": 4, "node": "A23", "action": "LEFT"},
    {"step": 5, "node": "HOME-1", "action": "STOP"}
  ]
}
```

### QR Code Detection (String)
```
"A1 | Angle: 90.5"
```
- Node label: `A1` to `D3`, `HOME-1` to `HOME-4`, `DOC-*`
- Heading angle in degrees (0-360)

### Battery State (std_msgs/BatteryState)
- `percentage`: 0-100 (float)
- `voltage`: volts (float)
- `current`: amps (float)
- `charge`: coulombs (optional)

### Motor Commands (String)
One of: `LEFT`, `RIGHT`, `STRAIGHT`, `UTURN`, `START`, `STOP`, `OBSTACLE`, `CLEAR`, `FORWARD`, `TURN`, etc.

### Encoder Counts (Int32MultiArray)
```
[rf_ticks, rb_ticks, lb_ticks, lf_ticks]
```
(Right Front, Right Back, Left Back, Left Front)

---

## Development & Testing

### Testing Path Planning
```bash
# Start path planner node
ros2 run agv_motor_controller grid_path_planner_node

# Publish a test query
ros2 topic pub -1 /path_query std_msgs/String "{data: '{\"start\": \"A1\", \"goal\": \"HOME-1\"}'}"

# Monitor results
ros2 topic echo /path_result
```

### Testing Motor Commands
```bash
# Start motor bridge
ros2 run agv_motor_controller motor_command_serial_bridge

# Send a motor command
ros2 topic pub -1 /motor_command std_msgs/String "{data: 'FORWARD'}"

# Monitor encoder feedback
ros2 topic echo /encoder_counts
```

### Testing Camera Feed
```bash
# Publish QR code detections manually
ros2 topic pub -1 /camera/qr_code std_msgs/String "{data: 'A1 | Angle: 45.5'}"

# Monitor visualizer window
ros2 run agv_motor_controller realtime_grid_visualizer_node
```

### Running the GUI
```bash
cd /home/gaurav/warehouse_ws/src/Warehouse-Bot/agv_gui
python3 main.py
```

**Prerequisites:**
- ROS2 running on network
- Nodes publishing to expected topics
- Camera_test node running for live feed

---

## Key Design Patterns

### 1. QoS Profiles
**Real-time Topics** (camera, IMU):
```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

**Reliable Topics** (commands, queries):
```python
QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
```

### 2. Lazy Imports
OpenCV and CvBridge are imported inside methods (not at module level) to avoid Qt plugin conflicts:

```python
def init_camera_node(self):
    from cv_bridge import CvBridge  # Import when needed
    self.bridge = CvBridge()
```

### 3. Embedded Nodes in GUI
The GUI can spawn ROS nodes directly:
- `embedded_realtime_grid_visualizer_node` - Renders frames for display
- `embedded_camera_viewer_node` - Subscribes to image_raw
- All run in headless mode without OpenCV windows

### 4. JSON for Complex Messages
Paths and results use JSON strings instead of custom message types for flexibility:
```python
msg = String(data=json.dumps({...}))
self.publisher.publish(msg)
```

---

## Future Extensions

1. **Obstacle Avoidance**: Integrate ToF sensor data into path planner
2. **Multi-AGV Coordination**: Add fleet management UI
3. **Persistent Logging**: Database storage of mission history
4. **Machine Learning**: QR code detection via ML instead of pyzbar
5. **Web Interface**: ROS2 bridge + React frontend for remote monitoring
6. **Auto-calibration**: Encoder correction and IMU drift compensation

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| GUI crashes on startup | cv2 Qt plugin conflict | Already fixed - lazy imports handle this |
| No camera frames | camera_test not running | Start `camera_test` node on laptop |
| Path planner timeout | Grid not initialized | Ensure grid_path_planner_node is active |
| Motor commands ignored | Serial port closed | Check `/dev/ttyUSB0` permissions; Arduino not responding |
| QR codes not detected | Wrong camera orientation | Adjust camera/servo angles; check QR lighting |
| Visualizer window blank | No QR codes published | Ensure camera_test is publishing to `camera/qr_code` |
| Obstacle detection not working | tof_test_2 not running | Ensure tof_test_2 is launched on RPi; check ToF sensor I2C connection |
| Servo commands not executing | Motor bridge command format wrong | Verify command format (PAN+, PAN-, TILT+, etc.) |
| Encoders returning zeros | Arduino not connected | Check serial cable; verify baud rate 115200 |

---

**Document Version**: 1.0  
**Last Updated**: April 28, 2026  
**Maintainer**: Development Team
