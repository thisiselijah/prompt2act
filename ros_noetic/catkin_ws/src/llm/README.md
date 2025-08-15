# LLM Node Usage Guide

This guide explains how to use the LLM (Large Language Model) node in ROS for text generation and behavior tree configuration.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Starting the LLM Node](#starting-the-llm-node)
- [ROS Topics](#ros-topics)
- [ROS Services](#ros-services)
- [Usage Examples](#### Available Behavior Types
- **detect_objects**: For detecting objects in the environment using YOLO vision system
- **pick_up**: For picking up objects using robot arm and gripper coordination
- **place_down**: For placing objects at specified coordinates (supports place_x, place_y, place_z parameters)
- **open_gripper**: For opening the robot gripper mechanism
- **close_gripper**: For closing the robot gripper mechanism
- **move_to_home**: For moving robot to home/rest position safely
- **sequence**: Execute children in order (all must succeed for sequence success)
- **selector**: Try children until one succeeds (first success completes selector)

### Enhanced Behavior Tree Examples

#### Basic Pick and Place with Gripper Control
```json
{
  "type": "sequence",
  "name": "Basic Pick and Place",
  "children": [
    {"type": "detect_objects", "name": "Scan Environment"},
    {"type": "open_gripper", "name": "Prepare Gripper"},
    {"type": "pick_up", "name": "Grasp Object"},
    {"type": "place_down", "name": "Place Object", "place_x": 0.15, "place_y": -0.15, "place_z": 0.18},
    {"type": "move_to_home", "name": "Return to Rest"}
  ]
}
```

#### Object Sorting with Multi-Location Placement
```json
{
  "type": "sequence",
  "name": "Object Sorting System",
  "children": [
    {"type": "detect_objects", "name": "Scan Workspace"},
    {"type": "selector", "name": "Sort By Color", "children": [
      {"type": "sequence", "name": "Red Object Processing", "children": [
        {"type": "pick_up", "name": "Pick Red Object"},
        {"type": "place_down", "name": "Red Zone", "place_x": 0.12, "place_y": -0.18, "place_z": 0.18}
      ]},
      {"type": "sequence", "name": "Blue Object Processing", "children": [
        {"type": "pick_up", "name": "Pick Blue Object"},
        {"type": "place_down", "name": "Blue Zone", "place_x": 0.18, "place_y": -0.12, "place_z": 0.18}
      ]}
    ]},
    {"type": "move_to_home", "name": "Task Complete"}
  ]
}
```

#### Gripper Test Sequence
```json
{
  "type": "sequence",
  "name": "Gripper Function Test",
  "children": [
    {"type": "open_gripper", "name": "Test Open"},
    {"type": "close_gripper", "name": "Test Close"},
    {"type": "open_gripper", "name": "Reset Open"},
    {"type": "move_to_home", "name": "Return Home"}
  ]
}
```examples)
- [Behavior Tree Integration](#behavior-tree-integration)
- [Troubleshooting](#troubleshooting)

## Prerequisites

### 1. Environment Setup
Before using the LLM node, ensure your ROS environment is properly configured:

```bash
# Inside the Docker container
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
```

### 2. API Keys Configuration
Set up your API keys for the LLM providers:

```bash
# For Google Gemini (recommended)
export GEMINI_API_KEY="your_gemini_api_key_here"

# Alternative: Google API Key
export GOOGLE_API_KEY="your_google_api_key_here"

# For OpenAI (optional)
export OPENAI_API_KEY="your_openai_api_key_here"
```

### 3. Build the Workspace
Make sure the workspace is built:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

## Starting the LLM Node

### Option 1: Direct Execution
```bash
# Start with default Gemini provider
rosrun llm llm_node.py

# Start with specific provider
rosrun llm llm_node.py _provider:=gemini
rosrun llm llm_node.py _provider:=openai
```

### Option 2: Using Launch File
```bash
# Launch all nodes including LLM node
roslaunch launcher all_nodes.launch llm_provider:=gemini

# Or with OpenAI
roslaunch launcher all_nodes.launch llm_provider:=openai
```

### Option 3: Individual Node Launch
```bash
# Create a simple launch file or run directly
roscore &
rosrun llm llm_node.py _provider:=gemini
```

## ROS Topics

### Publishers
The LLM node publishes responses on these topics:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/llm_response` | `std_msgs/String` | Text responses from LLM |
| `/llm_json_response` | `std_msgs/String` | JSON responses from LLM |

### Subscribers
The LLM node listens to these topics:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/llm_prompt` | `std_msgs/String` | Text prompts to LLM |
| `/llm_json_prompt` | `std_msgs/String` | JSON-specific prompts |

### Topic Usage Examples

#### Text Generation
```bash
# Send a text prompt
rostopic pub /llm_prompt std_msgs/String "data: 'Explain how robots work'"

# Listen to responses
rostopic echo /llm_response
```

#### JSON Generation
```bash
# Send a JSON prompt
rostopic pub /llm_json_prompt std_msgs/String "data: 'Generate a JSON configuration for robot movement'"

# Listen to JSON responses
rostopic echo /llm_json_response
```

## ROS Services

The LLM node provides custom service interfaces for structured communication:

### Service Message Formats

#### LLMQuery Service
```
# Request
string prompt

# Response
bool success
string response
string error_message
```

#### LLMJsonQuery Service
```
# Request
string prompt
string schema  # Optional JSON schema for validation

# Response
bool success
string json_response
string error_message
```

#### GenerateBehaviorTree Service
```
# Request
string task_description
bool auto_assemble     # NEW: Automatically assemble the behavior tree after generation

# Response
bool success
string behavior_tree_json
string error_message
```

#### LLMStatus Service
```
# Request
# (empty)

# Response
bool success
string provider_name
bool is_available
string status_message
```

### Available Services

| Service | Service Type | Description |
|---------|-------------|-------------|
| `/llm_query` | `llm/LLMQuery` | Direct text query with prompt |
| `/llm_json_query` | `llm/LLMJsonQuery` | JSON query with optional schema |
| `/generate_behavior_tree` | `llm/GenerateBehaviorTree` | Generate behavior tree JSON |
| `/llm_status` | `llm/LLMStatus` | Check LLM provider status |

### Service Usage Examples

#### Text Query Service
```bash
# Direct text query with custom prompt
rosservice call /llm_query "prompt: 'What is artificial intelligence?'"
```

#### JSON Query Service
```bash
# Direct JSON query with prompt and optional schema
rosservice call /llm_json_query "prompt: 'Create a JSON object with robot commands for navigation' schema: ''"

# With JSON schema validation (for Gemini)
rosservice call /llm_json_query "prompt: 'Generate robot configuration' schema: '{\"type\": \"object\", \"properties\": {\"joints\": {\"type\": \"array\"}}}'"
```

#### Behavior Tree Generation
```bash
# Method 1: Using proper YAML multiline format (RECOMMENDED)
rosservice call /generate_behavior_tree "
task_description: 'Pick up a red cube and place it on the blue table'
auto_assemble: true
"

# Method 2: Using inline YAML with proper structure
rosservice call /generate_behavior_tree "{task_description: 'Sort objects by color', auto_assemble: false}"

# Method 3: Using traditional ROS service call format
rosservice call /generate_behavior_tree -- "task_description: 'Pick up object'" "auto_assemble: true"

# Method 4: Generate with default task (auto_assemble defaults to true if omitted)
rosservice call /generate_behavior_tree "task_description: ''"
```

**Important**: Avoid this syntax (causes YAML parsing error):
```bash
# ❌ WRONG - causes parsing error
rosservice call /generate_behavior_tree "task_description: 'Pick up a red cube' auto_assemble: true"
```

#### Status Check
```bash
# Check LLM provider status and availability
rosservice call /llm_status
```

## Usage Examples

### Example 1: Basic Text Generation Workflow

```bash
# Terminal 1: Start the LLM node
rosrun llm llm_node.py _provider:=gemini

# Terminal 2: Send prompts and monitor responses
rostopic pub /llm_prompt std_msgs/String "data: 'Describe the three laws of robotics'"
rostopic echo /llm_response
```

### Example 2: JSON Configuration Generation

```bash
# Terminal 1: Start the LLM node
rosrun llm llm_node.py

# Terminal 2: Generate JSON configuration
rostopic pub /llm_json_prompt std_msgs/String "data: 'Create a JSON configuration for a 6-DOF robot arm with joint limits'"

# Terminal 3: Monitor JSON responses
rostopic echo /llm_json_response
```

### Example 3: Behavior Tree Generation and Auto-Assembly

```bash
# Terminal 1: Start the behavior tree node
rosrun behavior_tree behavior_tree_node.py

# Terminal 2: Start the LLM node
rosrun llm llm_node.py _provider:=gemini

# Terminal 3: Generate comprehensive robotic task with auto-assembly
rosservice call /generate_behavior_tree "
task_description: 'Robot should scan for red and blue blocks, sort them into separate zones, and return to home position'
auto_assemble: true
"

# Expected response format:
# success: True
# behavior_tree_json: "{ ... JSON configuration with detect_objects, pick_up, place_down, move_to_home ... }"
# error_message: ""

# Terminal 4: Monitor real-time behavior tree execution
rostopic echo /behavior_tree_status

# Terminal 5: Generate gripper test sequence
rosservice call /generate_behavior_tree "
task_description: 'Test gripper functionality by opening and closing it several times'
auto_assemble: true
"
```

### Example 4: Enhanced Behavior Tree Workflow with Complete Robot Integration

```bash
# Terminal 1: Start behavior tree node first
rosrun behavior_tree behavior_tree_node.py

# Terminal 2: Start YOLO detection node for vision
rosrun yolo_detection yolo_detection_node.py

# Terminal 3: Start robot control node
rosrun robot_control robot_control_node.py

# Terminal 4: Start LLM node 
rosrun llm llm_node.py _provider:=gemini

# Terminal 5: Generate and execute complete robotic workflows
rosservice call /generate_behavior_tree "
task_description: 'Sort objects by color: pick up red blocks and place them in the red zone at coordinates (0.12, -0.18, 0.18), pick up blue blocks and place them in the blue zone at (0.18, -0.12, 0.18), then return home'
auto_assemble: true
"

# Terminal 6: Monitor live execution status
rostopic echo /behavior_tree_status

# Terminal 7: Monitor YOLO detections
rostopic echo /yolo_detected_targets

# Example task variations:
# Simple pick and place:
rosservice call /generate_behavior_tree "
task_description: 'Pick up any object and place it at coordinates (0.15, -0.15, 0.18)'
auto_assemble: true
"

# Gripper control sequence:
rosservice call /generate_behavior_tree "
task_description: 'Open gripper, close gripper, then move to home position'
auto_assemble: true
"

# Complex multi-step task:
rosservice call /generate_behavior_tree "
task_description: 'Detect all objects, pick them up one by one, stack them at location (0.20, -0.10, 0.18), and ensure gripper is open when finished'
auto_assemble: true
"
```

## Behavior Tree Integration

The LLM node can generate behavior trees compatible with the behavior tree node and automatically assemble them when `auto_assemble` is enabled (default behavior).

### Auto-Assembly Feature
- **Enabled by default**: When `auto_assemble: true` (or omitted), the LLM node automatically calls `/assemble_behavior_tree` service after JSON generation
- **Seamless workflow**: Single service call goes from task description to active behavior tree
- **Robust error handling**: If assembly fails, JSON generation still succeeds with warning message
- **Manual control**: Set `auto_assemble: false` to only generate JSON without assembly

### Integration Flow
1. **JSON Generation**: LLM generates behavior tree JSON based on task description
2. **Auto-Assembly** (if enabled): Generated JSON is automatically sent to `/assemble_behavior_tree`
3. **Real-time Execution**: Behavior tree begins executing immediately with live status updates
4. **JSON Status Publishing**: Tree status published via `/behavior_tree_status` topic during execution
5. **Automatic Termination**: Tree terminates when root reaches SUCCESS or FAILURE status
6. **Ready for Next Task**: System waits for next behavior tree generation request

### Robot Integration Features
- **YOLO Vision Integration**: `detect_objects` subscribes to `/yolo_detected_targets` for real-time object detection
- **Robot Control Integration**: All robot behaviors use `/arm_command` service for Niryo robot control
- **Blackboard Data Sharing**: Detected objects and manipulation data shared between behaviors
- **Coordinate System**: Robot coordinates in meters, with configurable placement positions
- **Error Handling**: Graceful failure recovery with detailed error reporting

The generated JSON follows this structure:

```json
{
  "type": "sequence",
  "name": "MainTask",
  "children": [
    {
      "type": "detect_objects",
      "name": "DetectObjects"
    },
    {
      "type": "open_gripper",
      "name": "PrepareGripper"
    },
    {
      "type": "pick_up",
      "name": "PickUpObject"
    },
    {
      "type": "place_down",
      "name": "PlaceObject",
      "place_x": 0.15,
      "place_y": -0.15,
      "place_z": 0.18
    },
    {
      "type": "move_to_home",
      "name": "ReturnHome"
    }
  ]
}
```

### Available Behavior Types
- `detect_objects`: Object detection behavior
- `pick_up`: Object grasping behavior  
- `place_down`: Object placement behavior
- `sequence`: Execute children in order (all must succeed)
- `selector`: Try children until one succeeds

## Troubleshooting

### Common Issues

#### 1. API Key Not Found
```bash
# Error: "Failed to initialize provider"
# Solution: Set the API key
export GEMINI_API_KEY="your_api_key"
```

#### 2. Node Not Starting
```bash
# Error: "No module named 'setup'"
# Solution: Ensure you're in the correct directory and workspace is built
cd /root/catkin_ws
source devel/setup.bash
```

#### 3. No Response from LLM
```bash
# Check if provider is available
rosservice call /llm_status

# Check ROS topics
rostopic list | grep llm
```

#### 4. Service Call Errors
```bash
# Error: expected <block end>, but found '<scalar>' - YAML parsing error
# Problem: Incorrect service call syntax
# Wrong: rosservice call /generate_behavior_tree "task_description: 'Pick up cube' auto_assemble: true"
# 
# Solutions (choose one):
# Method 1 - Multiline YAML (RECOMMENDED):
rosservice call /generate_behavior_tree "
task_description: 'Pick up a red cube'
auto_assemble: true
"

# Method 2 - JSON-style inline:
rosservice call /generate_behavior_tree "{task_description: 'Pick up the red cube', auto_assemble: true}"

# Method 3 - Traditional ROS format:
rosservice call /generate_behavior_tree -- "task_description: 'Pick up cube'" "auto_assemble: true"

# Error: "No field name [data]" when calling /generate_behavior_tree
# Solution: Use the correct field names 'task_description' and 'auto_assemble'
# Wrong: rosservice call /generate_behavior_tree "data: ''"
# Correct: rosservice call /generate_behavior_tree "task_description: ''"

# Error: "Incompatible arguments to call service"
# Solution: Check the service message format using:
rosservice info /generate_behavior_tree

# Error: "JSON generated but assembly failed: Behavior tree assembly service not available"
# Solution: Start the behavior tree node before the LLM node:
rosrun behavior_tree behavior_tree_node.py
# Then restart LLM node:
rosrun llm llm_node.py _provider:=gemini

# Note: The behavior tree assembly service now uses a custom service interface:
# /assemble_behavior_tree (behavior_tree/AssembleBehaviorTree)
# Request: string behavior_tree_json
# Response: bool success, string message
```

#### 5. JSON Schema Validation Errors
```bash
# Error: "GenerateContentRequest.generation_config.response_schema.properties..."
# This indicates schema validation issues with Gemini API
# The node automatically falls back to generation without schema validation

# Error: "Invalid JSON generated: Expecting value: line 1 column 1 (char 0)"
# This means the LLM returned empty or malformed JSON
# Solutions:
# 1. Check API key is valid
rosservice call /llm_status

# 2. Try with a simpler task description
rosservice call /generate_behavior_tree "task_description: 'pick up object'"

# 3. Try using OpenAI provider instead
rosrun llm llm_node.py _provider:=openai
```

#### 6. Empty Response Issues
```bash
# Error: "Invalid JSON generated"
# This can happen with OpenAI provider. Try using Gemini for better JSON compliance
rosrun llm llm_node.py _provider:=gemini
```

### Debug Commands

```bash
# Check if node is running
rosnode list | grep llm

# Monitor node output
rosrun llm llm_node.py _provider:=gemini

# Check topics
rostopic list | grep llm

# Check services and their message formats
rosservice list | grep llm
rosservice info /llm_query
rosservice info /generate_behavior_tree

# Test basic functionality
rosservice call /llm_status

# Test each service with correct syntax (use proper YAML formatting)
rosservice call /llm_query "prompt: 'Hello'"
rosservice call /generate_behavior_tree "
task_description: 'test task'
auto_assemble: true
"
rosservice call /generate_behavior_tree "{task_description: 'test task', auto_assemble: false}"
```

### Log Levels
To get more detailed logging:

```bash
# Set ROS log level to DEBUG
export ROSCONSOLE_CONFIG_FILE=/path/to/debug/config
# Or modify the node to include more logging
```

## Advanced Usage

### Enhanced Task Description Capabilities

The LLM node now supports generating complex robotic behaviors with the following enhanced capabilities:

#### Supported Task Types
- **Object Detection and Sorting**: "Sort red and blue objects into separate zones"
- **Precise Manipulation**: "Pick up object and place it at coordinates (0.15, -0.15, 0.18)"
- **Gripper Control**: "Open gripper, pick up object, close gripper, move to home"
- **Multi-Step Workflows**: "Detect objects, pick each one, stack them at location X"
- **Conditional Behaviors**: "If red object detected, place in red zone, otherwise place in blue zone"

#### Task Description Best Practices
1. **Be Specific**: Include exact coordinates when precision is needed
2. **Mention Colors**: "red blocks", "blue objects" for color-based sorting
3. **Include End State**: "return to home position", "ensure gripper is open"
4. **Specify Zones**: "red zone", "blue zone", "stacking area"
5. **Use Action Verbs**: "detect", "pick up", "place", "sort", "stack"

#### Generated Behavior Capabilities
- **Vision Integration**: Automatic YOLO detection integration for object recognition
- **Coordinate Control**: Precise positioning with configurable place_x, place_y, place_z
- **Gripper Management**: Automatic gripper control with open/close behaviors
- **Error Recovery**: Robust error handling with graceful failure modes
- **Real-time Status**: Live execution monitoring via JSON status publishing

### Custom Prompt Templates
You can modify the behavior tree prompt template by editing the global constants in `llm_node.py`:

```python
BEHAVIOR_TREE_PROMPT_TEMPLATE = """
Your custom prompt template here...
{task_description}
"""
```

### Integration with Other Nodes
The LLM node is designed to work with:
- **Behavior Tree Node** (for automated task planning and execution)
- **Robot Control Node** (for physical execution)
- **Speech Recognition Node** (for voice commands)

### Enhanced Workflow
1. **Task Input**: Natural language task description via service call
2. **LLM Processing**: Gemini/OpenAI generates behavior tree JSON
3. **Auto-Assembly**: JSON automatically assembled into executable behavior tree
4. **Visualization**: Tree structure saved as PNG in `/frames` directory  
5. **Execution**: Behavior tree runs immediately with real-time status updates

### Service Dependencies
- **Required**: `/assemble_behavior_tree` service (behavior_tree/AssembleBehaviorTree) from behavior_tree node for auto-assembly
- **Optional**: Visualization services for enhanced debugging

**Updated Service Interface**:
```
# /assemble_behavior_tree service
string behavior_tree_json    # JSON configuration to assemble
---
bool success                 # Assembly success status
string message              # Status message or error details
```

### Performance Considerations
- Gemini provider generally provides better JSON compliance
- Response times vary based on prompt complexity
- Consider caching for repeated queries
- Auto-assembly adds minimal overhead (~100ms) for immediate execution

## New Features

### Auto-Assembly Integration (Latest Update)

The LLM node now features seamless integration with the behavior tree assembly service:

#### Key Benefits
- **One-Step Workflow**: Single service call from task description to running behavior tree
- **Automatic Error Recovery**: Graceful fallback if assembly service unavailable
- **Flexible Control**: Optional `auto_assemble` parameter for manual control
- **Enhanced Feedback**: Detailed status messages for both JSON generation and assembly

#### Usage Patterns

**Full Automation (Recommended)**:
```bash
rosservice call /generate_behavior_tree "
task_description: 'your task here'
auto_assemble: true
"
```

**JSON Generation Only**:
```bash
rosservice call /generate_behavior_tree "{task_description: 'your task here', auto_assemble: false}"
```

**Backward Compatibility**:
```bash
# auto_assemble defaults to true if omitted
rosservice call /generate_behavior_tree "task_description: 'your task here'"
```

#### Error Handling
- **Assembly Unavailable**: Returns JSON with warning message, allows manual assembly later
- **JSON Invalid**: Clear error messages with both original and cleaned responses
- **Service Reconnection**: Automatic retry logic for assembly service connections

---

## Testing

### Automated Testing
Use the included comprehensive test script:

```bash
# In the Docker container
cd /root/catkin_ws
source devel/setup.bash

# Start behavior tree node first (required for auto-assembly tests)
rosrun behavior_tree behavior_tree_node.py &

# Start LLM node
rosrun llm llm_node.py _provider:=gemini &

# Run comprehensive tests
rosrun llm test_behavior_tree_generation.py
```

The test script includes:
- **Auto-assembly enabled tests**: Verifies end-to-end workflow
- **JSON-only tests**: Validates generation without assembly
- **Error handling tests**: Confirms graceful failure modes
- **Multiple task types**: Tests various complexity levels

### Manual Testing
```bash
# Test basic functionality
rosservice call /llm_status

# Test JSON generation only
rosservice call /generate_behavior_tree "{task_description: 'pick up object', auto_assemble: false}"

# Test full workflow  
rosservice call /generate_behavior_tree "
task_description: 'sort red and blue blocks'
auto_assemble: true
"

# Check behavior tree status
rostopic echo /behavior_tree_status
```

For more information or issues, please refer to the project documentation or create an issue in the repository.

## System Architecture

### Component Integration
The LLM node is part of a comprehensive robotic system with the following components:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   LLM Node      │────│ Behavior Tree    │────│  Robot Control  │
│ (Task Planning) │    │   (Execution)    │    │  (Hardware)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │              ┌────────────────┐              │
         └──────────────│  YOLO Detection │──────────────┘
                        │   (Vision)     │
                        └────────────────┘
```

### Data Flow
1. **Task Input**: Natural language description received via `/llm_service`
2. **Behavior Generation**: LLM creates behavior tree JSON structure
3. **Tree Assembly**: Behavior tree node assembles and executes behaviors
4. **Vision Integration**: YOLO detection provides object coordinates
5. **Robot Execution**: Physical robot performs manipulation tasks
6. **Status Monitoring**: Real-time feedback via JSON publishing

### Deployment Configuration

#### Full System Launch
```bash
# Launch all system components
roslaunch niryo_web_interface all_nodes.launch
```

#### Component Status Monitoring
```bash
# Monitor behavior tree execution
rostopic echo /behavior_tree_status

# Monitor detected objects
rostopic echo /detected_objects_json

# Check LLM service availability
rosservice info /llm_service
```

#### System Health Checks
```bash
# Verify all nodes are running
rosnode list

# Check service availability
rosservice list | grep -E "(llm|behavior|robot)"

# Monitor topic activity
rostopic hz /behavior_tree_status
```

## Dependencies

### Required ROS Packages
- `rospy`: ROS Python bindings
- `std_msgs`: Standard ROS message types
- `behavior_tree`: Custom behavior tree package (for auto-assembly)

### Python Dependencies
- `google-generativeai`: Google Gemini API
- `openai`: OpenAI API
- `json`: JSON parsing
- `re`: Regular expressions

### System Requirements
- ROS Noetic
- Python 3.8+
- Internet connection for LLM APIs
- Valid API keys for chosen provider

````
