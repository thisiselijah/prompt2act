# LLM Node Usage Guide

This guide explains how to use the LLM (Large Language Model) node in ROS for text generation and behavior tree configuration.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Starting the LLM Node](#starting-the-llm-node)
- [ROS Topics](#ros-topics)
- [ROS Services](#ros-services)
- [Usage Examples](#usage-examples)
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
# Generate behavior tree with custom task description
rosservice call /generate_behavior_tree "task_description: 'Pick up a red cube and place it on the blue table'"

# Generate with default task (empty task_description uses default)
rosservice call /generate_behavior_tree "task_description: ''"
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

### Example 3: Behavior Tree Generation for Robot Tasks

```bash
# Terminal 1: Start the LLM node
rosrun llm llm_node.py _provider:=gemini

# Terminal 2: Generate behavior tree with specific task
rosservice call /generate_behavior_tree "task_description: 'Robot should scan the environment, find a red object, pick it up, navigate to a designated drop zone, and place the object'"

# Expected response format:
# success: True
# behavior_tree_json: "{ ... JSON configuration ... }"
# error_message: ""
```

### Example 4: Integration with Behavior Tree Node

```bash
# Terminal 1: Start all nodes
roslaunch launcher all_nodes.launch

# Terminal 2: Generate and extract behavior tree JSON
rosservice call /generate_behavior_tree "task_description: 'Pick up a bottle and place it in recycling bin'"

# Terminal 3: Use the response (example with command line JSON extraction)
# The response will have structured fields: success, behavior_tree_json, error_message
```

## Behavior Tree Integration

The LLM node can generate behavior trees compatible with the behavior tree node. The generated JSON follows this structure:

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
      "type": "pick_up",
      "name": "PickUpObject"
    },
    {
      "type": "place_down",
      "name": "PlaceObject"
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
# Error: "No field name [data]" when calling /generate_behavior_tree
# Solution: Use the correct field name 'task_description'
# Wrong: rosservice call /generate_behavior_tree "data: ''"
# Correct: rosservice call /generate_behavior_tree "task_description: ''"

# Error: "Incompatible arguments to call service"
# Solution: Check the service message format using:
rosservice info /generate_behavior_tree
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

# Test each service with correct syntax
rosservice call /llm_query "prompt: 'Hello'"
rosservice call /generate_behavior_tree "task_description: 'test task'"
```

### Log Levels
To get more detailed logging:

```bash
# Set ROS log level to DEBUG
export ROSCONSOLE_CONFIG_FILE=/path/to/debug/config
# Or modify the node to include more logging
```

## Advanced Usage

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
- Behavior Tree Node (for automated task planning)
- Robot Control Node (for execution)
- Speech Recognition Node (for voice commands)

### Performance Considerations
- Gemini provider generally provides better JSON compliance
- Response times vary based on prompt complexity
- Consider caching for repeated queries

---

For more information or issues, please refer to the project documentation or create an issue in the repository.
