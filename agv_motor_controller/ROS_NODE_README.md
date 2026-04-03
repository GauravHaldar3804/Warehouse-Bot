# Grid Path Planner ROS Node

A ROS2 node that provides path planning for a line-follower AGV on a 4x4 grid with enhanced node types (intermediate nodes, dock stations, home positions).

## Features

- **Path Planning**: Uses Dijkstra's algorithm to find optimal paths
- **Direction Tracking**: Provides turn-by-turn navigation instructions (START, LEFT, RIGHT, STRAIGHT, UTURN, STOP)
- **Multiple Node Types**: Supports 53 nodes across the grid:
  - Main intersection nodes (16): A1-D4
  - Vertical intermediate nodes (12): A12, A23, etc.
  - Horizontal intermediate nodes (12): AB1, AB2, etc.
  - Dock nodes (9): DOC-AB2 through DOC-CD4
  - Home nodes (4): HOME-1 through HOME-4

## Installation

1. Build the package:
```bash
cd ~/warehouse_ws
colcon build --packages-select agv_motor_controller
```

2. Source the workspace:
```bash
source install/setup.bash
```

## Running the Node

### Start the Path Planner Node
```bash
ros2 run agv_motor_controller grid_path_planner_node
```

Expected output:
```
[INFO] Grid Path Planner Node initialized
[INFO] Grid size: 4x4 | Total nodes: 53
[INFO] Subscribed to: path_query
[INFO] Publishing to: path_result
```

### Publish Path Queries

In another terminal:

```bash
source ~/warehouse_ws/install/setup.bash
ros2 topic pub /path_query std_msgs/String "{data: '{\"start\": \"A1\", \"goal\": \"D4\"}'}"
```

## Message Formats

### Query Message (Subscribe to `/path_query`)

**Format**: JSON string

```json
{
  "start": "A1",
  "goal": "D4"
}
```

**Examples**:
- `{"start": "A1", "goal": "D4"}` - Grid intersection to grid intersection
- `{"start": "B2", "goal": "HOME-3"}` - Grid to home location
- `{"start": "C3", "goal": "DOC-BC3"}` - Grid to dock station

### Result Message (Publish to `/path_result`)

**Success Response**:
```json
{
  "status": "success",
  "start": "A1",
  "goal": "D4",
  "total_steps": 12,
  "path": ["A1", "A12", "A2", "A23", "A3", "A34", "A4", "AB4", "B4", "BC4", "C4", "CD4", "D4"],
  "navigation": [
    {
      "step": 1,
      "node": "A1",
      "action": "START"
    },
    {
      "step": 2,
      "node": "A12",
      "action": "STRAIGHT"
    },
    {
      "step": 3,
      "node": "A2",
      "action": "STRAIGHT"
    },
    ...
    {
      "step": 13,
      "node": "D4",
      "action": "STOP"
    }
  ]
}
```

**Error Response**:
```json
{
  "status": "error",
  "message": "Start node A99 not found"
}
```

## Action Types

| Action | Meaning |
|--------|---------|
| `START` | Initial node - bot starts here |
| `STRAIGHT` | Continue in the same direction |
| `LEFT` | Turn left at this node |
| `RIGHT` | Turn right at this node |
| `UTURN` | Make a 180-degree turn |
| `STOP` | Final destination - bot stops here |

## Usage Examples

### Example 1: Using ros2 topic command

```bash
# Query path from A1 to D4
ros2 topic pub /path_query std_msgs/String "{data: '{\"start\": \"A1\", \"goal\": \"D4\"}'}"

# Query path from B2 to HOME-3
ros2 topic pub /path_query std_msgs/String "{data: '{\"start\": \"B2\", \"goal\": \"HOME-3\"}'}"

# Query path from C3 to DOC-BC3
ros2 topic pub /path_query std_msgs/String "{data: '{\"start\": \"C3\", \"goal\": \"DOC-BC3\"}'}"
```

### Example 2: Using Python script

See `grid_path_planner_test_publisher.py` for a complete example.

```bash
ros2 run agv_motor_controller grid_path_planner_test_publisher
```

### Example 3: Subscribing to results

```bash
ros2 topic echo /path_result
```

## Grid Structure

### Main Grid (4x4)

```
Row 1: A1  AB1  B1  BC1  C1  CD1  D1
Row 2: A2  AB2  B2  BC2  C2  CD2  D2  HOME-2
Row 3: A3  AB3  B3  BC3  C3  CD3  D3  HOME-3
Row 4: A4  AB4  B4  BC4  C4  CD4  D4  HOME-4
```

### Vertical Intermediate Nodes

Between each row: A12, A23, A34, B12, B23, B34, C12, C23, C34, D12, D23, D34

### Dock Nodes

From horizontal intermediate nodes (rows 2-4):
- DOC-AB2, DOC-AB3, DOC-AB4
- DOC-BC2, DOC-BC3, DOC-BC4
- DOC-CD2, DOC-CD3, DOC-CD4

### Home Nodes

Extending right from column D:
- HOME-1, HOME-2, HOME-3, HOME-4

## Integration with AGV

1. **Subscribe** to `/path_result` in your AGV controller
2. **Parse** the navigation instructions
3. **Execute** each action in sequence:
   - Use `START` to initialize movement
   - Use direction actions (LEFT, RIGHT, STRAIGHT) at each node
   - Use `STOP` to halt at destination

## Troubleshooting

### Node not found error
```json
{"status": "error", "message": "Start node A99 not found"}
```
Check the node name spelling. Valid node types:
- Main nodes: A1-D4
- Vertical nodes: A12, A23, A34, etc.
- Horizontal nodes: AB1, AB2, etc.
- Dock nodes: DOC-AB2, DOC-AB3, etc.
- Home nodes: HOME-1, HOME-2, etc.

### No path found
```json
{"status": "error", "message": "No path found from A1 to Z99"}
```
The start or goal node may not exist or be valid.

### JSON Parse Error
```json
{"status": "error", "message": "Invalid JSON format in query"}
```
Ensure the query message is valid JSON. Example:
```bash
ros2 topic pub /path_query std_msgs/String "{data: '{\"start\": \"A1\", \"goal\": \"D4\"}'}"
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/path_query` | `std_msgs/String` | Subscribe | Receives path query requests (JSON) |
| `/path_result` | `std_msgs/String` | Publish | Publishes path results with navigation directions (JSON) |

## Parameters

Currently, the node uses a fixed 4x4 grid. To modify:
- Edit `grid_size=4` in `GridPathPlannerNode.__init__()`
- Rebuild the package: `colcon build --packages-select agv_motor_controller`

## Files

- `grid_path_planner_node.py`: Main ROS2 node
- `grid_path_planner_test_publisher.py`: Test publisher script
- `grid_path_planner.py`: Core path planning library
- `setup.py`: Package configuration (includes node entry point)

## Dependencies

- ROS2 (Foxy, Galactic, Humble, or Jazzy)
- Python 3.7+
- Standard ROS2 messages (std_msgs)
- NumPy
- Matplotlib (optional, for visualization)
