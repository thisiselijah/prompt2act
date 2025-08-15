# YOLO Detection Package

This ROS package provides YOLO-based object detection capabilities for robotic applications.

## Overview

The YOLO detection package is responsible for:
- Real-time object detection using YOLO models
- ArUco marker-based coordinate transformation
- Publishing detected object data in robot coordinate frame

## Features

- **YOLO Object Detection**: Uses YOLO model to detect objects (red_block, blue_block)
- **ArUco Coordinate System**: Uses ArUco markers for perspective transformation
- **Robot Coordinate Mapping**: Converts camera coordinates to robot coordinate frame
- **JSON Output**: Publishes detection results as JSON messages

## Architecture

### Core Components

1. **YOLO Detection Node** (`yolo_detection_node.py`)
   - Main detection and processing node
   - Handles camera input and coordinate transformations
   - Publishes detection results

### ROS Topics

#### Subscribed Topics
- **`/web_camera/image_raw`** (sensor_msgs/Image)
  - Camera input for object detection

#### Published Topics  
- **`/yolo_detected_targets`** (std_msgs/String)
  - JSON-formatted detection results with robot coordinates

### Files Structure

```
yolo_detection/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package dependencies  
├── launch/
│   └── yolo_detection_node.launch  # Launch file for detection node
├── model/
│   └── best.pt             # YOLO model weights
├── scripts/
│   └── yolo_detection_node.py      # Main detection node
└── README.md               # This documentation
```

## Usage

### Launching the Detection Node

```bash
# Launch individual node
roslaunch yolo_detection yolo_detection_node.launch

# Or include in larger system via niryo_web_interface main.launch
roslaunch niryo_web_interface main.launch
```

### Detection Output Format

The node publishes JSON-formatted detection results:

```json
[
  {
    "cx": 320,
    "cy": 240,
    "pts": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]],
    "roll": 0.785,
    "label": "red_block",
    "x": 0.25,
    "y": -0.05
  }
]
```

Where:
- `cx, cy`: Center pixel coordinates in camera frame
- `pts`: Corner points of detected object bounding box
- `roll`: Object rotation in radians
- `label`: Object type (red_block, blue_block)
- `x, y`: Robot coordinate position in meters

## Configuration

### ArUco Setup
The system expects 4 ArUco markers (IDs 0-3) positioned as:
- ID 3: Left-top corner
- ID 2: Right-top corner  
- ID 1: Right-bottom corner
- ID 0: Left-bottom corner

### Table Dimensions
Default table size (can be modified in code):
- Width: 23 cm
- Height: 30 cm

### Robot Coordinate Frame
- Origin: (0.2, -0.1) in robot base frame
- X-axis: Forward direction
- Y-axis: Left direction

## Dependencies

### ROS Dependencies
- rospy
- sensor_msgs
- std_msgs
- cv_bridge

### Python Dependencies
- OpenCV (cv2)
- NumPy
- YOLO (ultralytics)
- rospkg

### System Dependencies
- YOLO model file (`model/best.pt`)
- Camera hardware/simulation

## Integration

This package integrates with:
- **Camera System**: Receives images from web camera node
- **Behavior Trees**: Detection results used by behavior tree behaviors
- **Robot Control**: Coordinates used for robot movement planning

## Decoupling Benefits

By separating YOLO detection into its own package:
- ✅ **Modularity**: Independent development and testing
- ✅ **Reusability**: Can be used by multiple applications  
- ✅ **Maintenance**: Easier to update detection algorithms
- ✅ **Dependencies**: Clear separation of vision-specific dependencies
- ✅ **Deployment**: Can be deployed independently on different machines
