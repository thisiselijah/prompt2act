#!/usr/bin/env python3

"""
Example behavior tree configurations for the enhanced robot control system.
This file contains JSON configurations that can be used with the behavior tree assembly service.
"""

import json

# Example 1: Simple pick and place sequence
PICK_AND_PLACE_SEQUENCE = {
    "type": "sequence",
    "name": "Pick and Place Task",
    "children": [
        {
            "type": "detect_objects",
            "name": "Detect Objects"
        },
        {
            "type": "pick_up", 
            "name": "Pick Up Object"
        },
        {
            "type": "place_down",
            "name": "Place Object",
            "place_x": 0.15,
            "place_y": -0.15,
            "place_z": 0.18
        },
        {
            "type": "move_to_home",
            "name": "Return Home"
        }
    ]
}

# Example 2: Object sorting with selector
OBJECT_SORTING_TASK = {
    "type": "sequence",
    "name": "Object Sorting Task", 
    "children": [
        {
            "type": "detect_objects",
            "name": "Scan for Objects"
        },
        {
            "type": "selector",
            "name": "Sort Objects",
            "children": [
                {
                    "type": "sequence",
                    "name": "Process Red Objects",
                    "children": [
                        {"type": "pick_up", "name": "Pick Red Object"},
                        {"type": "place_down", "name": "Place in Red Zone", "place_x": 0.12, "place_y": -0.18, "place_z": 0.18}
                    ]
                },
                {
                    "type": "sequence", 
                    "name": "Process Blue Objects",
                    "children": [
                        {"type": "pick_up", "name": "Pick Blue Object"},
                        {"type": "place_down", "name": "Place in Blue Zone", "place_x": 0.18, "place_y": -0.12, "place_z": 0.18}
                    ]
                }
            ]
        },
        {
            "type": "move_to_home",
            "name": "Finish and Return Home"
        }
    ]
}

# Example 3: Gripper test sequence
GRIPPER_TEST = {
    "type": "sequence",
    "name": "Gripper Test",
    "children": [
        {
            "type": "open_gripper",
            "name": "Open Gripper"
        },
        {
            "type": "close_gripper", 
            "name": "Close Gripper"
        },
        {
            "type": "open_gripper",
            "name": "Open Gripper Again"
        }
    ]
}

def print_example_configs():
    """Print example configurations in JSON format"""
    print("=== BEHAVIOR TREE CONFIGURATION EXAMPLES ===\n")
    
    print("1. PICK AND PLACE SEQUENCE:")
    print(json.dumps(PICK_AND_PLACE_SEQUENCE, indent=2))
    print("\n" + "="*50 + "\n")
    
    print("2. OBJECT SORTING TASK:")
    print(json.dumps(OBJECT_SORTING_TASK, indent=2))
    print("\n" + "="*50 + "\n")
    
    print("3. GRIPPER TEST:")
    print(json.dumps(GRIPPER_TEST, indent=2))
    print("\n" + "="*50 + "\n")
    
    print("USAGE:")
    print("Copy any of the above JSON configurations and use them with:")
    print("rosservice call /assemble_behavior_tree \"behavior_tree_json: 'PASTE_JSON_HERE'\"")

if __name__ == "__main__":
    print_example_configs()
