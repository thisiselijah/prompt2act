# Robot Control Primitive Actions

This document describes all the primitive actions available in the robot control service. These actions are designed to be atomic operations that can be composed in behavior trees.

## Service Interface
- **Service Name**: `/niryo_arm_command_service`
- **Service Type**: `robot_control/RobotCommand`
- **Request**: `string command`
- **Response**: `bool success`, `string message`

## Primitive Actions

### 1. Gripper Control Actions

#### `open_gripper`
- **Description**: Opens the robot gripper
- **Parameters**: None
- **Example**: `open_gripper`
- **Returns**: `success=True` if gripper opened successfully

#### `close_gripper`
- **Description**: Closes the robot gripper
- **Parameters**: None
- **Example**: `close_gripper`
- **Returns**: `success=True` if gripper closed successfully

#### `release_gripper`
- **Description**: Releases the gripper with tool
- **Parameters**: None
- **Example**: `release_gripper`
- **Returns**: `success=True` if gripper released successfully

### 2. Movement Actions

#### `get_current_pose`
- **Description**: Gets the current pose of the robot
- **Parameters**: None
- **Example**: `get_current_pose`
- **Returns**: `success=True` with current pose information in the message

#### `move_to_home_and_sleep`
- **Description**: Moves the robot to its home position and puts it to sleep
- **Parameters**: None
- **Example**: `move_to_home_and_sleep`
- **Returns**: `success=True` if moved to home and sleep successfully

#### `move_to_pose`
- **Description**: Moves the robot to a specific pose with full 6DOF control
- **Parameters**: `x,y,z,roll,pitch,yaw` (all in meters and radians)
- **Example**: `move_to_pose:0.2,0.1,0.3,0.0,1.438,-0.35`
- **Returns**: `success=True` if moved to pose successfully

### 3. Pick and Place Actions

#### `pick_at`
- **Description**: Picks an object at the specified position on the table surface
- **Parameters**: `x,y,roll` (x,y in meters, roll in radians, z is fixed at 0.163m)
- **Example**: `pick_at:0.2,0.1,0.5`
- **Returns**: `success=True` if pick operation completed successfully
- **Behavior**: Opens gripper → moves to position → closes gripper

#### `place_at`
- **Description**: Places an object at the specified pose
- **Parameters**: `x,y,z,roll,pitch,yaw` (position in meters, orientation in radians)
- **Example**: `place_at:0.3,0.2,0.2,0.0,1.438,-0.35`
- **Returns**: `success=True` if place operation completed successfully
- **Behavior**: Moves to position → opens gripper

### 4. Robot State Actions

#### `enable_learning_mode`
- **Description**: Enables learning mode (allows manual movement)
- **Parameters**: None
- **Example**: `enable_learning_mode`
- **Returns**: `success=True` if learning mode enabled successfully

#### `disable_learning_mode`
- **Description**: Disables learning mode (enables motor control)
- **Parameters**: None
- **Example**: `disable_learning_mode`
- **Returns**: `success=True` if learning mode disabled successfully

#### `calibrate`
- **Description**: Performs automatic calibration of the robot
- **Parameters**: None
- **Example**: `calibrate`
- **Returns**: `success=True` if calibration completed successfully

### 5. Safety Actions

#### `stop`
- **Description**: Stops all robot movement immediately
- **Parameters**: None
- **Example**: `stop`
- **Returns**: `success=True` if robot stopped successfully

### 6. Check Actions (for Behavior Tree Conditions)

#### `check_calibration`
- **Description**: Checks if the robot needs calibration
- **Parameters**: None
- **Example**: `check_calibration`
- **Returns**: `success=True` if robot needs calibration, `success=False` if already calibrated
- **Note**: Uses `niryo.need_calibration()` method

### 7. Legacy Commands (Backward Compatibility)

#### `open` (deprecated, use `open_gripper`)
- **Description**: Opens the gripper (legacy command)
- **Parameters**: None

#### `close` (deprecated, use `close_gripper`)
- **Description**: Closes the gripper (legacy command)
- **Parameters**: None

#### `learning_mode` (deprecated, use `enable_learning_mode`)
- **Description**: Enables learning mode (legacy command)
- **Parameters**: None

#### `move_to_home` (deprecated, use `move_to_home_and_sleep`)
- **Description**: Moves to home position (legacy command)
- **Parameters**: None

## Summary of Available Commands

Based on the current implementation, the following commands are available:

### Implemented Commands:
1. **Gripper Control**:
   - `open_gripper` - Opens the gripper
   - `close_gripper` - Closes the gripper
   - `release_gripper` - Releases gripper with tool

2. **Movement**:
   - `get_current_pose` - Returns current robot pose
   - `move_to_home_and_sleep` - Moves to home and sleeps
   - `move_to_pose:x,y,z,roll,pitch,yaw` - Moves to specific pose

3. **Pick and Place**:
   - `pick_at:x,y,roll` - Picks object at position (z=0.163m fixed)
   - `place_at:x,y,z,roll,pitch,yaw` - Places object at pose

4. **Robot State**:
   - `enable_learning_mode` - Enables learning mode
   - `disable_learning_mode` - Disables learning mode
   - `calibrate` - Performs auto-calibration

5. **Safety**:
   - `stop` - Stops robot movement

6. **Diagnostics**:
   - `check_calibration` - Checks if calibration is needed

7. **Legacy Support** (via subscriber):
   - `open`, `close`, `learning_mode`, `move_to_home`, `pick_at:x,y,roll`

## Usage in Behavior Trees

These primitive actions are designed to be used as leaf nodes in behavior trees. Each action returns:
- `success=True` when the action completes successfully
- `success=False` when the action fails

Example behavior tree node implementation:
```python
import rospy
from robot_control.srv import RobotCommand

def execute_primitive_action(action_command):
    try:
        service = rospy.ServiceProxy('/niryo_arm_command_service', RobotCommand)
        response = service(command=action_command)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False
```

## Error Handling

All actions include comprehensive error handling:
- Invalid parameters will return `success=False` with a descriptive error message
- Hardware errors (e.g., robot disconnection) will return `success=False`
- Exception details are logged to ROS logger and included in the response message
- Format errors for parameterized commands provide clear guidance on correct usage

## Implementation Notes

- The robot connection is established at startup with IP "192.168.232.26"
- Auto-calibration is performed if the robot is not calibrated at startup
- Both service and subscriber interfaces are available for backward compatibility
- All pose operations use `PoseObject` from the pyniryo library
- Fixed table surface height (z=0.163m) is used for pick operations

## Coordinate System

- **X-axis**: Forward/backward (positive forward)
- **Y-axis**: Left/right (positive left)
- **Z-axis**: Up/down (positive up)
- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis  
- **Yaw**: Rotation around Z-axis

All positions are in meters, all orientations are in radians.
