#!/usr/bin/env python3
"""
Better monitoring script to view path results with formatted output
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class PathResultMonitor(Node):
    """Monitor and display path results with nice formatting"""
    
    def __init__(self):
        super().__init__('path_result_monitor')
        
        self.subscription = self.create_subscription(
            String,
            'path_result',
            self.result_callback,
            10
        )
        
        self.get_logger().info('Path Result Monitor Started')
        self.get_logger().info('Listening on /path_result topic...\n')
    
    def result_callback(self, msg: String):
        """Display results in a readable format"""
        try:
            result = json.loads(msg.data)
            
            # Clear screen and print header
            print("\n" + "=" * 70)
            
            if result['status'] == 'success':
                print(f"✅ PATH FOUND: {result['start']} → {result['goal']}")
                print("=" * 70)
                print(f"Total Steps: {result['total_steps']}")
                print(f"Total Nodes: {len(result['path'])}")
                print(f"Full Path: {' → '.join(result['path'])}\n")
                
                print("📍 NAVIGATION INSTRUCTIONS:")
                print("-" * 70)
                for nav in result['navigation']:
                    step = nav['step']
                    node = nav['node']
                    action = nav['action']
                    
                    # Format with better spacing
                    print(f"  Step {step:2d}: Node {node:10s} | Action: {action:8s}")
                
                print("-" * 70)
            else:
                print(f"❌ ERROR: {result['message']}")
                print("=" * 70)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    monitor = PathResultMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
