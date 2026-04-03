#!/usr/bin/env python3
"""
Grid Path Planner Test Publisher
This script demonstrates how to use the grid_path_planner_node by publishing
path queries and subscribing to results
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class PathQueryPublisher(Node):
    """Test publisher for path queries"""
    
    def __init__(self):
        super().__init__('path_query_publisher')
        
        # Create publisher for queries
        self.query_publisher = self.create_publisher(
            String,
            'path_query',
            10
        )
        
        # Create subscriber for results
        self.result_subscription = self.create_subscription(
            String,
            'path_result',
            self.result_callback,
            10
        )
        
        self.get_logger().info('Path Query Publisher initialized')
        self.get_logger().info('Publishing queries to: path_query')
        self.get_logger().info('Subscribing to results from: path_result')
        
        # Store results
        self.results = []
    
    def result_callback(self, msg: String):
        """Handle incoming results"""
        try:
            result = json.loads(msg.data)
            self.results.append(result)
            
            if result['status'] == 'success':
                self.get_logger().info('=' * 60)
                self.get_logger().info(f"✅ Path Found: {result['start']} → {result['goal']}")
                self.get_logger().info(f"   Total Steps: {result['total_steps']}")
                self.get_logger().info(f"   Path: {' → '.join(result['path'])}")
                self.get_logger().info('   Navigation Instructions:')
                for nav in result['navigation']:
                    self.get_logger().info(
                        f"     {nav['step']:2d}. Node: {nav['node']:10s} | Action: {nav['action']}"
                    )
                self.get_logger().info('=' * 60)
            else:
                self.get_logger().error(f"❌ Error: {result['message']}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse result: {str(e)}")
    
    def publish_query(self, start: str, goal: str):
        """Publish a path query"""
        query = {
            'start': start.upper(),
            'goal': goal.upper()
        }
        
        self.get_logger().info(f'Publishing query: {start} → {goal}')
        self.query_publisher.publish(String(data=json.dumps(query)))
        
        # Wait for result
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    publisher = PathQueryPublisher()
    
    # Test queries
    test_queries = [
        ('A1', 'D4'),
        ('B2', 'HOME-3'),
        ('C3', 'DOC-BC3'),
        ('A1', 'HOME-1'),
    ]
    
    try:
        publisher.get_logger().info('Starting path queries...')
        
        for start, goal in test_queries:
            publisher.publish_query(start, goal)
            rclpy.spin_once(publisher, timeout_sec=1)
        
        publisher.get_logger().info('\nAll queries completed!')
        
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
