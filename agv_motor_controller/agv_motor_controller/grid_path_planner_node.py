#!/usr/bin/env python3
"""
Grid Path Planner ROS2 Node
Subscribes to path queries and publishes planned paths with navigation directions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from .grid_path_planner import GridPathPlanner


class GridPathPlannerNode(Node):
    """ROS2 Node for grid-based path planning"""
    
    def __init__(self):
        super().__init__('grid_path_planner_node')
        
        # Initialize the grid path planner
        self.planner = GridPathPlanner(num_cols=4, num_rows=3)
        
        # Create subscriber for path queries
        self.query_subscription = self.create_subscription(
            String,
            'path_query',
            self.path_query_callback,
            10
        )
        
        # Create publisher for path results
        self.result_publisher = self.create_publisher(
            String,
            'path_result',
            10
        )
        
        self.get_logger().info('Grid Path Planner Node initialized')
        self.get_logger().info(
            f'Grid size: {self.planner.num_cols}x{self.planner.num_rows} | Total nodes: {len(self.planner.nodes)}'
        )
        self.get_logger().info('Subscribed to: path_query')
        self.get_logger().info('Publishing to: path_result')
    
    def path_query_callback(self, msg: String):
        """Handle incoming path query requests"""
        try:
            # Parse JSON message
            query = json.loads(msg.data)
            start_node = query.get('start', '').upper()
            goal_node = query.get('goal', '').upper()
            
            self.get_logger().info(f'Path query received: {start_node} → {goal_node}')
            
            # Validate nodes
            if start_node not in self.planner.nodes:
                error_msg = {'status': 'error', 'message': f'Start node {start_node} not found'}
                self.result_publisher.publish(String(data=json.dumps(error_msg)))
                self.get_logger().error(f'Start node {start_node} not found')
                return
            
            if goal_node not in self.planner.nodes:
                error_msg = {'status': 'error', 'message': f'Goal node {goal_node} not found'}
                self.result_publisher.publish(String(data=json.dumps(error_msg)))
                self.get_logger().error(f'Goal node {goal_node} not found')
                return
            
            if start_node == goal_node:
                error_msg = {'status': 'error', 'message': 'Start and goal nodes must be different'}
                self.result_publisher.publish(String(data=json.dumps(error_msg)))
                self.get_logger().error('Start and goal nodes are the same')
                return
            
            # Compute path
            path = self.planner.dijkstra_path(start_node, goal_node)
            
            if not path:
                error_msg = {'status': 'error', 'message': f'No path found from {start_node} to {goal_node}'}
                self.result_publisher.publish(String(data=json.dumps(error_msg)))
                self.get_logger().error(f'No path found from {start_node} to {goal_node}')
                return
            
            # Get path with directions
            path_with_directions = self.planner.get_path_with_directions(path)
            
            # Format result
            result = {
                'status': 'success',
                'start': start_node,
                'goal': goal_node,
                'total_steps': len(path) - 1,
                'path': path,
                'navigation': [
                    {
                        'step': i + 1,
                        'node': node,
                        'action': direction
                    }
                    for i, (node, direction) in enumerate(path_with_directions)
                ]
            }
            
            # Publish result
            self.result_publisher.publish(String(data=json.dumps(result)))
            
            self.get_logger().info(f'Path published: {len(path_with_directions)} nodes, {len(path)-1} steps')
            self.get_logger().info(f'Path: {" → ".join(path)}')
            
        except json.JSONDecodeError:
            error_msg = {'status': 'error', 'message': 'Invalid JSON format in query'}
            self.result_publisher.publish(String(data=json.dumps(error_msg)))
            self.get_logger().error('Invalid JSON format in query')
        except Exception as e:
            error_msg = {'status': 'error', 'message': str(e)}
            self.result_publisher.publish(String(data=json.dumps(error_msg)))
            self.get_logger().error(f'Error processing query: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = GridPathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
