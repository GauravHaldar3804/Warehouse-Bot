#!/bin/bash
# Grid Path Planner ROS2 Node - Setup and Test Commands

echo "Step 1: Building the package..."
cd ~/warehouse_ws
colcon build --packages-select agv_motor_controller
source install/setup.bash
echo "Build complete!"

echo ""
echo "==============================================="
echo "Now open 4 terminals and run these commands:"
echo "==============================================="
echo ""
echo "TERMINAL 1 - Start the Node:"
echo "source ~/warehouse_ws/install/setup.bash && ros2 run agv_motor_controller grid_path_planner_node"
echo ""
echo "TERMINAL 2 - Monitor Results:"
echo "source ~/warehouse_ws/install/setup.bash && ros2 topic echo /path_result"
echo ""
echo "TERMINAL 3 - Send Test Query (A1 to D4):"
echo "source ~/warehouse_ws/install/setup.bash && ros2 topic pub /path_query std_msgs/String '{data: \"{\\\"start\\\": \\\"A1\\\", \\\"goal\\\": \\\"D4\\\"}\"}'"
echo ""
echo "TERMINAL 4 - Send Other Queries:"
echo "B2 to HOME-3:"
echo "source ~/warehouse_ws/install/setup.bash && ros2 topic pub /path_query std_msgs/String '{data: \"{\\\"start\\\": \\\"B2\\\", \\\"goal\\\": \\\"HOME-3\\\"}\"}'"
echo ""
echo "C3 to DOC-BC3:"
echo "source ~/warehouse_ws/install/setup.bash && ros2 topic pub /path_query std_msgs/String '{data: \"{\\\"start\\\": \\\"C3\\\", \\\"goal\\\": \\\"DOC-BC3\\\"}\"}'"
echo ""
