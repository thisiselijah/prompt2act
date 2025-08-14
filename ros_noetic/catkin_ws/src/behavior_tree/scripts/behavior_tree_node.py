#!/usr/bin/python3.8

import sys
print(sys.executable) 

import py_trees
import rospy
import json
import os
import time
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String
from behavior_tree.srv import helloworld, helloworldResponse, AssembleBehaviorTree, AssembleBehaviorTreeResponse
from robot_control.srv import RobotCommand, RobotCommandRequest

# ============================================================================
# GLOBAL CONFIGURATION
# ============================================================================
TICK_FREQUENCY_HZ = 2.0  # Behavior tree execution frequency (Hz)
ENABLE_DETAILED_LOGGING = False  # Enable detailed tick logging
# ============================================================================

# Import for behavior tree JSON serialization
try:
    # JSON serialization is built-in, no external dependencies needed
    JSON_SERIALIZATION_AVAILABLE = True
except ImportError:
    rospy.logwarn("JSON serialization not available")
    JSON_SERIALIZATION_AVAILABLE = False

# --- Define behaviors
class DetectObjects(py_trees.behaviour.Behaviour):
    """Behavior that subscribes to YOLO detection data and stores detected objects"""
    
    def __init__(self, name):
        super(DetectObjects, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.detected_objects = []
        self.subscriber = rospy.Subscriber('/yolo_detected_targets', String, self._detection_callback)
        
    def _detection_callback(self, msg):
        """Callback to receive YOLO detection data"""
        try:
            self.detected_objects = json.loads(msg.data)
            self.logger.info(f"Received {len(self.detected_objects)} detected objects")
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse detection data: {e}")
            self.detected_objects = []

    def update(self):
        """Check if objects are detected"""
        if self.detected_objects:
            self.logger.info(f"Objects detected: {len(self.detected_objects)} items")
            # Store detection data in blackboard for other behaviors to use
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.detected_objects = self.detected_objects
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("No objects detected, continuing to search...")
            return py_trees.common.Status.RUNNING

class PickUp(py_trees.behaviour.Behaviour):
    """Behavior to pick up objects using robot control service"""
    
    def __init__(self, name):
        super(PickUp, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.robot_service = None
        self.picked_object = None
        
    def setup(self, **kwargs):
        """Setup the robot control service client"""
        try:
            rospy.wait_for_service('/arm_command', timeout=5.0)
            self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
            self.logger.info("Robot control service connected")
            return True
        except rospy.ROSException as e:
            self.logger.error(f"Failed to connect to robot service: {e}")
            return False

    def update(self):
        """Execute pick up behavior"""
        if not self.robot_service:
            self.logger.error("Robot service not available")
            return py_trees.common.Status.FAILURE
            
        try:
            # Get detected objects from blackboard
            self.blackboard = self.attach_blackboard_client(name=self.name)
            detected_objects = getattr(self.blackboard, 'detected_objects', [])
            
            if not detected_objects:
                self.logger.warn("No objects available to pick up")
                return py_trees.common.Status.FAILURE
                
            # Pick the first detected object
            target_object = detected_objects[0]
            x = target_object['x']
            y = target_object['y'] 
            roll = target_object['roll']
            
            # Send pick command to robot
            command = f"pick_at:{x},{y},{roll}"
            req = RobotCommandRequest()
            req.command = command
            
            response = self.robot_service(req)
            
            if response.success:
                self.logger.info(f"Successfully picked up object at ({x:.2f}, {y:.2f})")
                # Store picked object info for place behavior
                self.blackboard.picked_object = target_object
                # Remove picked object from detected list
                detected_objects.remove(target_object)
                self.blackboard.detected_objects = detected_objects
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"Pick up failed: {response.message}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"Error during pick up: {e}")
            return py_trees.common.Status.FAILURE

class PlaceDown(py_trees.behaviour.Behaviour):
    """Behavior to place objects using robot control service"""
    
    def __init__(self, name, place_x=0.15, place_y=-0.15, place_z=0.18):
        super(PlaceDown, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.robot_service = None
        self.place_x = place_x
        self.place_y = place_y
        self.place_z = place_z
        
    def setup(self, **kwargs):
        """Setup the robot control service client"""
        try:
            rospy.wait_for_service('/arm_command', timeout=5.0)
            self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
            self.logger.info("Robot control service connected")
            return True
        except rospy.ROSException as e:
            self.logger.error(f"Failed to connect to robot service: {e}")
            return False

    def update(self):
        """Execute place down behavior"""
        if not self.robot_service:
            self.logger.error("Robot service not available")
            return py_trees.common.Status.FAILURE
            
        try:
            # Get picked object from blackboard
            self.blackboard = self.attach_blackboard_client(name=self.name)
            picked_object = getattr(self.blackboard, 'picked_object', None)
            
            if not picked_object:
                self.logger.warn("No object available to place down")
                return py_trees.common.Status.FAILURE
                
            # Use original object orientation for placing
            roll = picked_object.get('roll', 0.0)
            pitch = 1.438  # Default pitch for placing
            yaw = -0.35    # Default yaw for placing
            
            # Send place command to robot
            command = f"place_at:{self.place_x},{self.place_y},{self.place_z},{roll},{pitch},{yaw}"
            req = RobotCommandRequest()
            req.command = command
            
            response = self.robot_service(req)
            
            if response.success:
                self.logger.info(f"Successfully placed object at ({self.place_x:.2f}, {self.place_y:.2f}, {self.place_z:.2f})")
                # Clear picked object from blackboard
                self.blackboard.picked_object = None
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"Place down failed: {response.message}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"Error during place down: {e}")
            return py_trees.common.Status.FAILURE

class OpenGripper(py_trees.behaviour.Behaviour):
    """Behavior to open robot gripper"""
    
    def __init__(self, name):
        super(OpenGripper, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.robot_service = None
        
    def setup(self, **kwargs):
        """Setup the robot control service client"""
        try:
            rospy.wait_for_service('/arm_command', timeout=5.0)
            self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
            return True
        except rospy.ROSException as e:
            self.logger.error(f"Failed to connect to robot service: {e}")
            return False

    def update(self):
        """Execute open gripper behavior"""
        if not self.robot_service:
            return py_trees.common.Status.FAILURE
            
        try:
            req = RobotCommandRequest()
            req.command = "open_gripper"
            response = self.robot_service(req)
            
            if response.success:
                self.logger.info("Gripper opened successfully")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"Failed to open gripper: {response.message}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"Error opening gripper: {e}")
            return py_trees.common.Status.FAILURE

class CloseGripper(py_trees.behaviour.Behaviour):
    """Behavior to close robot gripper"""
    
    def __init__(self, name):
        super(CloseGripper, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.robot_service = None
        
    def setup(self, **kwargs):
        """Setup the robot control service client"""
        try:
            rospy.wait_for_service('/arm_command', timeout=5.0)
            self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
            return True
        except rospy.ROSException as e:
            self.logger.error(f"Failed to connect to robot service: {e}")
            return False

    def update(self):
        """Execute close gripper behavior"""
        if not self.robot_service:
            return py_trees.common.Status.FAILURE
            
        try:
            req = RobotCommandRequest()
            req.command = "close_gripper"
            response = self.robot_service(req)
            
            if response.success:
                self.logger.info("Gripper closed successfully")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"Failed to close gripper: {response.message}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"Error closing gripper: {e}")
            return py_trees.common.Status.FAILURE

class MoveToHome(py_trees.behaviour.Behaviour):
    """Behavior to move robot to home position"""
    
    def __init__(self, name):
        super(MoveToHome, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.robot_service = None
        
    def setup(self, **kwargs):
        """Setup the robot control service client"""
        try:
            rospy.wait_for_service('/arm_command', timeout=5.0)
            self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
            return True
        except rospy.ROSException as e:
            self.logger.error(f"Failed to connect to robot service: {e}")
            return False

    def update(self):
        """Execute move to home behavior"""
        if not self.robot_service:
            return py_trees.common.Status.FAILURE
            
        try:
            req = RobotCommandRequest()
            req.command = "move_to_home_and_sleep"
            response = self.robot_service(req)
            
            if response.success:
                self.logger.info("Moved to home position successfully")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"Failed to move to home: {response.message}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"Error moving to home: {e}")
            return py_trees.common.Status.FAILURE

# --- Visualization Functions ---
class BehaviorTreeJSONPublisher:
    """Class to handle behavior tree JSON serialization and ROS publishing"""
    
    def __init__(self, output_topic="behavior_tree_status"):
        self.output_topic = output_topic
        self.frame_counter = 0
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        
        # Create ROS publisher for JSON data
        self.publisher = rospy.Publisher(
            self.output_topic, 
            String, 
            queue_size=10
        )
        
        rospy.loginfo(f"BehaviorTreeJSONPublisher initialized on topic: {self.output_topic}")
    
    def get_snapshot_visitor(self):
        """Get the snapshot visitor for adding to behavior tree"""
        return self.snapshot_visitor
    
    def serialize_tree_structure(self, tree):
        """
        Serialize the behavior tree structure to JSON
        Args:
            tree: py_trees.trees.BehaviourTree object
        Returns:
            dict: JSON-serializable tree structure
        """
        if not tree or not tree.root:
            return None
            
        def serialize_node(node):
            """Recursively serialize a behavior tree node"""
            node_data = {
                "id": str(node.id),
                "name": node.name,
                "type": type(node).__name__,
                "children": []
            }
            
            # Add type-specific information
            if isinstance(node, py_trees.composites.Selector):
                node_data["composite_type"] = "selector"
            elif isinstance(node, py_trees.composites.Sequence):
                node_data["composite_type"] = "sequence"
            elif isinstance(node, py_trees.composites.Parallel):
                node_data["composite_type"] = "parallel"
            else:
                node_data["composite_type"] = "behaviour"
            
            # Serialize children
            for child in node.children:
                child_data = serialize_node(child)
                node_data["children"].append(child_data)
                
            return node_data
        
        return serialize_node(tree.root)
    
    def serialize_tree_status(self, tree):
        """
        Serialize the current behavior tree status to JSON
        Args:
            tree: py_trees.trees.BehaviourTree object
        Returns:
            dict: JSON-serializable tree status
        """
        if not tree or not tree.root:
            return None
            
        # Get current status from snapshot visitor
        snapshot_nodes = self.snapshot_visitor.nodes if self.snapshot_visitor else {}
        
        def extract_node_info(node):
            """Extract status information for a node"""
            status = "INVALID"
            if node.id in snapshot_nodes:
                status = str(snapshot_nodes[node.id])
            elif hasattr(node, 'status'):
                status = str(node.status)
                
            return {
                "id": str(node.id),
                "name": node.name,
                "status": status,
                "type": type(node).__name__
            }
        
        # Collect all node statuses
        node_statuses = []
        node_statuses.append(extract_node_info(tree.root))
        for node in tree.root.iterate():
            node_statuses.append(extract_node_info(node))
        
        return {
            "timestamp": time.time(),
            "frame_count": self.frame_counter,
            "tree_status": str(tree.root.status) if hasattr(tree.root, 'status') else "INVALID",
            "nodes": node_statuses
        }
    
    def publish_tree_data(self, tree, include_structure=False):
        """
        Publish behavior tree data as JSON via ROS topic
        Args:
            tree: py_trees.trees.BehaviourTree object
            include_structure: Whether to include tree structure data
        Returns:
            bool: Success status
        """
        try:
            if not tree or not tree.root:
                rospy.logwarn("No tree to publish")
                return False
            
            # Create JSON payload
            json_data = {
                "type": "behavior_tree_status",
                "timestamp": time.time(),
                "frame_count": self.frame_counter
            }
            
            # Add structure if requested
            if include_structure:
                structure = self.serialize_tree_structure(tree)
                if structure:
                    json_data["structure"] = structure
            
            # Add current status
            status = self.serialize_tree_status(tree)
            if status:
                json_data["status"] = status
            
            # Publish JSON message
            json_string = json.dumps(json_data, indent=2)
            msg = String()
            msg.data = json_string
            
            self.publisher.publish(msg)
            self.frame_counter += 1
            
            rospy.logdebug(f"Published behavior tree JSON data (frame {self.frame_counter})")
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to publish behavior tree JSON data: {str(e)}")
            return False
    
    def publish_tree_summary(self, tree, tree_config=None):
        """
        Publish a complete tree summary with structure and status
        Args:
            tree: py_trees.trees.BehaviourTree object  
            tree_config: Original JSON configuration (optional)
        """
        try:
            if not tree or not tree.root:
                return False
            
            success = self.publish_tree_data(tree, include_structure=True)
            if success:
                rospy.loginfo(f"Published complete tree summary (frame {self.frame_counter})")
            return success
                
        except Exception as e:
            rospy.logerr(f"Failed to publish tree summary: {str(e)}")
            return False
        
        return False

# --- Service section ---

# Define a server behavior factory service
def assemble_behavior_tree_service():
    """
    Create ROS service to assemble behavior tree from JSON
    """
    def handle_assemble_tree(req):
        """
        Service callback to assemble behavior tree from JSON
        Args:
            req: Service request containing JSON string in req.behavior_tree_json
        Returns:
            AssembleBehaviorTreeResponse: success status and message
        """
        try:
            # Parse the JSON from the service request
            tree_config = json.loads(req.behavior_tree_json)
            rospy.loginfo(f"Received behavior tree configuration: {tree_config}")
            
            # Assemble the behavior tree based on JSON configuration
            root = assemble_tree_from_json(tree_config)
            
            # Store the assembled tree globally
            global current_behavior_tree, visualizer, last_tree_config
            current_behavior_tree = py_trees.trees.BehaviourTree(root)
            current_behavior_tree.setup(timeout=15)
            last_tree_config = tree_config
            
            # Add snapshot visitor for JSON publishing if available
            if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                # Remove existing snapshot visitor if any
                current_behavior_tree.visitors = [v for v in current_behavior_tree.visitors 
                                                if not isinstance(v, py_trees.visitors.SnapshotVisitor)]
                # Add our snapshot visitor
                current_behavior_tree.visitors.append(json_publisher.get_snapshot_visitor())
                
                # Publish initial tree summary with structure
                json_publisher.publish_tree_summary(current_behavior_tree, tree_config)
            
            rospy.loginfo("Behavior tree assembled successfully")
            return AssembleBehaviorTreeResponse(success=True, message="Behavior tree assembled successfully")
            
        except json.JSONDecodeError as e:
            error_msg = f"Failed to parse JSON: {str(e)}"
            rospy.logerr(error_msg)
            return AssembleBehaviorTreeResponse(success=False, message=error_msg)
            
        except Exception as e:
            error_msg = f"Failed to assemble behavior tree: {str(e)}"
            rospy.logerr(error_msg)
            return AssembleBehaviorTreeResponse(success=False, message=error_msg)
    
    # Create the service
    service = rospy.Service('assemble_behavior_tree', AssembleBehaviorTree, handle_assemble_tree)
    rospy.loginfo("Behavior tree assembly service started")
    return service

def assemble_tree_from_json(config):
    """
    Assemble a behavior tree from JSON configuration
    Args:
        config (dict): JSON configuration for the behavior tree
    Returns:
        py_trees behavior: Root of the assembled tree
    """
    if config.get('type') == 'sequence':
        root = py_trees.composites.Sequence(config.get('name', 'Root'), memory=False)
        
        # Add children from the configuration
        for child_config in config.get('children', []):
            child = create_behavior_from_config(child_config)
            if child:
                root.add_child(child)
                
    elif config.get('type') == 'selector':
        root = py_trees.composites.Selector(config.get('name', 'Root'), memory=False)
        
        # Add children from the configuration
        for child_config in config.get('children', []):
            child = create_behavior_from_config(child_config)
            if child:
                root.add_child(child)
                
    else:
        # Default return null if no valid type is found
        rospy.logwarn(f"Unknown root behavior type: {config.get('type')}")
        return None
    
    return root

def create_behavior_from_config(config):
    """
    Create individual behaviors from configuration
    Args:
        config (dict): Configuration for a single behavior
    Returns:
        py_trees behavior: The created behavior
    """
    behavior_type = config.get('type')
    behavior_name = config.get('name', 'UnnamedBehavior')
    
    if behavior_type == 'detect_objects':
        return DetectObjects(behavior_name)
    elif behavior_type == 'pick_up':
        return PickUp(behavior_name)
    elif behavior_type == 'place_down':
        # Allow custom place coordinates from config
        place_x = config.get('place_x', 0.15)
        place_y = config.get('place_y', -0.15) 
        place_z = config.get('place_z', 0.18)
        return PlaceDown(behavior_name, place_x, place_y, place_z)
    elif behavior_type == 'open_gripper':
        return OpenGripper(behavior_name)
    elif behavior_type == 'close_gripper':
        return CloseGripper(behavior_name)
    elif behavior_type == 'move_to_home':
        return MoveToHome(behavior_name)
    elif behavior_type == 'sequence':
        sequence = py_trees.composites.Sequence(behavior_name, memory=False)
        for child_config in config.get('children', []):
            child = create_behavior_from_config(child_config)
            if child:
                sequence.add_child(child)
        return sequence
    elif behavior_type == 'selector':
        selector = py_trees.composites.Selector(behavior_name, memory=False)
        for child_config in config.get('children', []):
            child = create_behavior_from_config(child_config)
            if child:
                selector.add_child(child)
        return selector
    else:
        rospy.logwarn(f"Unknown behavior type: {behavior_type}")
        return None
    
# Hello World Service
def hello_world_service():
    def handle_hello(req):
        response = helloworldResponse()
        response.response = "Hello World"
        return response
    
    service = rospy.Service('hello_world', helloworld, handle_hello)
    rospy.loginfo("Hello World service started")
    return service

# Global variables to store current behavior tree and JSON publisher
current_behavior_tree = None
json_publisher = None
last_tree_config = None 

def main():
    """
    Main function to initialize ROS node and run behavior tree
    """
    # Initialize the ROS node
    rospy.init_node('behavior_tree_node', anonymous=True)
    rospy.loginfo("Behavior Tree Node started")
    rospy.loginfo(f"Configuration: Tick frequency = {TICK_FREQUENCY_HZ} Hz")
    
    # Initialize JSON publisher
    global json_publisher
    if JSON_SERIALIZATION_AVAILABLE:
        json_publisher = BehaviorTreeJSONPublisher("behavior_tree_status")
        rospy.loginfo("Behavior tree JSON publisher initialized")
    else:
        rospy.logwarn("JSON serialization disabled")
    
    # Start the behavior tree assembly service
    service0 = assemble_behavior_tree_service()
    service1 = hello_world_service()
    
    # Set the update rate using global frequency variable
    rate = rospy.Rate(TICK_FREQUENCY_HZ)
    rospy.loginfo(f"Behavior tree tick frequency set to {TICK_FREQUENCY_HZ} Hz")
    
    # Initialize tick counter
    tick_count = 0
    
    try:
        # Main execution loop
        while not rospy.is_shutdown():
            # Tick the current behavior tree if it exists
            global current_behavior_tree
            if current_behavior_tree:
                # Perform the tick
                current_behavior_tree.tick()
                tick_count += 1
                
                # Log the tree status with tick information
                status = current_behavior_tree.root.status
                if ENABLE_DETAILED_LOGGING:
                    rospy.loginfo(f"Tick {tick_count}: Tree status = {status}")
                else:
                    rospy.loginfo_throttle(2.0, f"Tick {tick_count}: Tree status = {status}")
                
                # Publish JSON data after each tick if publisher is available
                if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                    json_publisher.publish_tree_data(current_behavior_tree)
                    if ENABLE_DETAILED_LOGGING:
                        rospy.logdebug(f"Published JSON data for tick {tick_count}")
                
                # Check if behavior tree completed successfully
                if status == py_trees.common.Status.SUCCESS:
                    rospy.loginfo(f"🎉 Behavior tree completed successfully after {tick_count} ticks!")
                    rospy.loginfo("Tree terminated. Waiting for next behavior tree task...")
                    
                    # Publish final status
                    if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                        json_publisher.publish_tree_data(current_behavior_tree, include_structure=True)
                    
                    # Clean up current tree
                    current_behavior_tree.shutdown()
                    current_behavior_tree = None
                    tick_count = 0
                    rospy.loginfo("Behavior tree cleared. Ready for new task.")
                    
                elif status == py_trees.common.Status.FAILURE:
                    rospy.logwarn(f"❌ Behavior tree failed after {tick_count} ticks!")
                    rospy.loginfo("Tree terminated due to failure. Waiting for next behavior tree task...")
                    
                    # Publish final status
                    if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                        json_publisher.publish_tree_data(current_behavior_tree, include_structure=True)
                    
                    # Clean up current tree
                    current_behavior_tree.shutdown()
                    current_behavior_tree = None
                    tick_count = 0
                    rospy.loginfo("Failed behavior tree cleared. Ready for new task.")
            
            # Sleep to maintain the desired rate
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Behavior Tree Node shutting down")
    finally:
        # Clean shutdown
        if current_behavior_tree:
            current_behavior_tree.shutdown()
        rospy.loginfo(f"Behavior tree node completed {tick_count} ticks")

if __name__ == '__main__':
    main()
