# Behavior Tree Package

This ROS package provides a behavior tree execution system with JSON-based visualization and status reporting.

## Overview

The behavior tree system allows for dynamic assembly and execution of behavior trees through ROS services, with real-time status reporting via JSON messages published to ROS topics.

## Features

- **Dynamic Tree Assembly**: Create behavior trees from JSON configurations via ROS service calls
- **JSON-based Visualization**: Real-time status and structure publishing through ROS topics
- **Modular Architecture**: Separate visualization rendering from tree execution
- **Custom Behaviors**: Extensible behavior library for robotic tasks

## Architecture

### Core Components

1. **BehaviorTreeNode** (`behavior_tree_node.py`)
   - Main execution node for behavior trees
   - Provides assembly service interface
   - Publishes JSON status data

2. **BehaviorTreeJSONPublisher** (within `behavior_tree_node.py`)
   - Handles JSON serialization of tree structure and status
   - Publishes data to ROS topics for visualization
   - Manages snapshot visitors for status tracking

3. **Custom Service Messages**
   - `AssembleBehaviorTree.srv`: Service for tree assembly from JSON

### ROS Topics

- **`behavior_tree_status`** (std_msgs/String)
  - Published after each tree tick
  - Contains JSON-serialized tree status and optionally structure
  - Used by visualization tools and monitoring systems

### ROS Services

- **`assemble_behavior_tree`** (behavior_tree/AssembleBehaviorTree)
  - Input: JSON string defining tree structure
  - Output: Success status and message
  - Assembles and loads a new behavior tree for execution

- **`hello_world`** (robot_control/helloworld)
  - Simple test service returning "Hello World"

## Usage

### Starting the Behavior Tree Node

```bash
# In ROS environment
rosrun behavior_tree behavior_tree_node.py
```

### Assembling a Behavior Tree

Use the ROS service to load a behavior tree from JSON:

```bash
# Example service call
rosservice call /assemble_behavior_tree "behavior_tree_json: '{
  \"type\": \"sequence\",
  \"name\": \"Main Sequence\",
  \"children\": [
    {\"type\": \"detect_objects\", \"name\": \"Detect Objects\"},
    {\"type\": \"pick_up\", \"name\": \"Pick Up Object\"},
    {\"type\": \"place_down\", \"name\": \"Place Object\"}
  ]
}'"
```

### Monitoring Tree Status

Subscribe to the JSON status topic:

```bash
# Listen to status updates
rostopic echo /behavior_tree_status
```

Or use the provided test listener:

```bash
rosrun behavior_tree test_json_publisher.py
```

## JSON Message Format

### Status Message Structure

```json
{
  "type": "behavior_tree_status",
  "timestamp": 1642681234.567,
  "frame_count": 42,
  "status": {
    "timestamp": 1642681234.567,
    "frame_count": 42,
    "tree_status": "RUNNING",
    "nodes": [
      {
        "id": "uuid-string",
        "name": "Node Name",
        "status": "SUCCESS",
        "type": "DetectObjects"
      }
    ]
  },
  "structure": {
    "id": "root-uuid",
    "name": "Root Node",
    "type": "Sequence",
    "composite_type": "sequence",
    "children": [...]
  }
}
```

### Tree Configuration Format

```json
{
  "type": "sequence|selector",
  "name": "Node Name",
  "children": [
    {
      "type": "detect_objects|pick_up|place_down|sequence|selector",
      "name": "Child Node Name",
      "children": [...]
    }
  ]
}
```

## Available Behaviors

- **DetectObjects**: Object detection behavior
- **PickUp**: Object pickup behavior  
- **PlaceDown**: Object placement behavior
- **Sequence**: Execute children in order until one fails
- **Selector**: Execute children until one succeeds

## Configuration

Edit the configuration constants in `behavior_tree_node.py`:

```python
TICK_FREQUENCY_HZ = 2.0     # Tree execution frequency
ENABLE_DETAILED_LOGGING = False  # Detailed logging level
```

## Development

### Adding New Behaviors

1. Create a new behavior class inheriting from `py_trees.behaviour.Behaviour`
2. Implement the `update()` method returning appropriate status
3. Add the behavior type to `create_behavior_from_config()`
4. Update this documentation with the new behavior

### Visualization Tools

The JSON publishing system enables modular visualization:

1. **Real-time Monitoring**: Subscribe to `behavior_tree_status` topic
2. **Graphical Visualization**: Create separate nodes that render tree diagrams
3. **Data Logging**: Record JSON streams for analysis and playback
4. **Web Interfaces**: Use JSON data in web-based dashboards

## Dependencies

- ROS Noetic
- py_trees (Python behavior tree library)
- Python 3.8+
- Standard ROS message packages (std_msgs, std_srvs)

## Files

```
behavior_tree/
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # Package dependencies
├── srv/
│   └── AssembleBehaviorTree.srv  # Custom service definition
└── scripts/
    ├── behavior_tree_node.py     # Main behavior tree node
    └── test_json_publisher.py    # JSON listener test tool
```

## Migration Notes

This package has been redesigned to use JSON-based publishing instead of direct image generation:

- **Removed Dependencies**: No longer requires pydot/graphviz for core functionality
- **Modular Design**: Visualization is now separated from execution
- **Enhanced Monitoring**: JSON format enables programmatic analysis
- **ROS Integration**: Better integration with ROS ecosystem through standard message types
