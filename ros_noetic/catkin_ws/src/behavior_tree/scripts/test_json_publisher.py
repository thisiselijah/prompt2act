#!/usr/bin/env python3

"""
Test script to demonstrate the JSON-based behavior tree visualization system.
This script subscribes to the behavior_tree_status topic and prints the JSON data.
"""

import rospy
import json
from std_msgs.msg import String

def json_callback(msg):
    """
    Callback function to handle received JSON messages
    Args:
        msg: std_msgs/String message containing JSON data
    """
    try:
        # Parse the JSON data
        data = json.loads(msg.data)
        
        # Print formatted information
        print("=" * 60)
        print(f"Received behavior tree data at {data.get('timestamp', 'unknown')}")
        print(f"Frame count: {data.get('frame_count', 'unknown')}")
        print(f"Message type: {data.get('type', 'unknown')}")
        
        # Print status information if available
        if 'status' in data:
            status_data = data['status']
            print(f"Tree status: {status_data.get('tree_status', 'unknown')}")
            print(f"Number of nodes: {len(status_data.get('nodes', []))}")
            
            # Print individual node statuses
            print("\nNode statuses:")
            for node in status_data.get('nodes', []):
                print(f"  - {node.get('name', 'unnamed')}: {node.get('status', 'unknown')}")
        
        # Print structure information if available
        if 'structure' in data:
            print("\nTree structure included in this message")
            structure = data['structure']
            print(f"Root node: {structure.get('name', 'unknown')} ({structure.get('type', 'unknown')})")
            print(f"Child count: {len(structure.get('children', []))}")
        
        print("=" * 60)
        
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON: {e}")
    except Exception as e:
        print(f"Error processing message: {e}")

def main():
    """
    Main function to initialize ROS node and subscribe to JSON topic
    """
    # Initialize the ROS node
    rospy.init_node('behavior_tree_json_listener', anonymous=True)
    rospy.loginfo("Behavior Tree JSON Listener started")
    
    # Subscribe to the behavior tree status topic
    rospy.Subscriber('behavior_tree_status', String, json_callback)
    
    rospy.loginfo("Listening for behavior tree JSON data on topic: behavior_tree_status")
    rospy.loginfo("Press Ctrl+C to stop")
    
    try:
        # Keep the node running
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("JSON Listener shutting down")

if __name__ == '__main__':
    main()
