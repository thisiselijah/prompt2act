#!/usr/bin/python3.8

import sys
print(sys.executable) 

import py_trees
import py_trees.blackboard
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
MAX_TICKS_BEFORE_TIMEOUT = 100  # Maximum ticks before considering task stuck (5 minutes at 1Hz)
STUCK_CHECK_INTERVAL = 25  # Check for stuck condition every N ticks
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
        self.subscriber = None
        # Initialize blackboard for py_trees 0.7.x compatibility
        self.blackboard = py_trees.blackboard.Blackboard()
        
    def setup(self, timeout=None):
        """Setup the YOLO detection subscriber"""
        try:
            # Create subscriber only when behavior tree is actively running
            if not self.subscriber:
                self.subscriber = rospy.Subscriber('/yolo_detected_targets', String, self._detection_callback)
                self.logger.info("YOLO detection subscriber created")
            return True
        except Exception as e:
            self.logger.error(f"Failed to setup YOLO detection: {e}")
            return False
    
    def terminate(self, new_status):
        """Clean up subscriber when behavior terminates"""
        try:
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
                self.logger.info("YOLO detection subscriber unregistered")
        except Exception as e:
            self.logger.error(f"Error during DetectObjects cleanup: {e}")
        super(DetectObjects, self).terminate(new_status)
        
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
            self.logger.info(f"✅ Objects detected: {len(self.detected_objects)} items")
            
            # Print detailed information about detected objects
            for i, obj in enumerate(self.detected_objects):
                obj_info = f"  Object {i+1}: "
                if 'class' in obj:
                    obj_info += f"class='{obj['class']}' "
                if 'x' in obj and 'y' in obj:
                    obj_info += f"position=({obj['x']:.3f}, {obj['y']:.3f}) "
                if 'confidence' in obj:
                    obj_info += f"confidence={obj['confidence']:.2f} "
                if 'roll' in obj:
                    obj_info += f"roll={obj['roll']:.3f}"
                self.logger.info(obj_info)
            
            # Store detection data in blackboard for other behaviors to use
            self.blackboard.set('detected_objects', self.detected_objects)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("🔍 No objects detected, continuing to search...")
            # Return RUNNING to keep the detection behavior active
            # This allows the behavior tree to continue and retry detection
            return py_trees.common.Status.RUNNING

class PickUp(py_trees.behaviour.Behaviour):
    """Behavior to pick up objects using robot control service"""
    
    def __init__(self, name):
        super(PickUp, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.robot_service = None
        self.picked_object = None
        # Initialize blackboard for py_trees 0.7.x compatibility
        self.blackboard = py_trees.blackboard.Blackboard()
        
    def setup(self, timeout=None):
        """Setup the robot control service client"""
        try:
            # Don't wait for service during setup - defer to update() method
            # This prevents setup failures when robot services are not yet available
            self.logger.info("Setup completed - will connect to robot service when needed")
            return True
        except Exception as e:
            self.logger.error(f"Setup failed: {e}")
            return False

    def update(self):
        """Execute pick up behavior"""
        # Connect to service if not already connected
        if not self.robot_service:
            try:
                rospy.wait_for_service('/arm_command', timeout=2.0)
                self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
                self.logger.info("Robot control service connected")
            except rospy.ROSException as e:
                self.logger.error(f"❌ Robot service not available: {e}")
                return py_trees.common.Status.FAILURE
            
        try:
            # Get detected objects from blackboard
            detected_objects = self.blackboard.get('detected_objects') or []
            
            if not detected_objects:
                self.logger.warn("⚠️ No objects available to pick up")
                # Return RUNNING instead of FAILURE to allow retrying when objects become available
                return py_trees.common.Status.RUNNING
                
            # Pick the first detected object
            target_object = detected_objects[0]
            x = target_object.get('x', 0.0)
            y = target_object.get('y', 0.0) 
            roll = target_object.get('roll', 0.0)
            obj_class = target_object.get('class', 'unknown')
            
            self.logger.info(f"🦾 Attempting to pick up {obj_class} at ({x:.3f}, {y:.3f})")
            
            # Send pick command to robot
            command = f"pick_at:{x},{y},{roll}"
            req = RobotCommandRequest()
            req.command = command
            
            response = self.robot_service(req)
            
            if response.success:
                self.logger.info(f"✅ Successfully picked up {obj_class} at ({x:.2f}, {y:.2f})")
                # Store picked object info for place behavior
                self.blackboard.set('picked_object', target_object)
                # Remove picked object from detected list
                detected_objects.remove(target_object)
                self.blackboard.set('detected_objects', detected_objects)
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"❌ Pick up failed: {response.message}")
                # Return FAILURE for robot execution errors
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"❌ Error during pick up: {e}")
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
        # Initialize blackboard for py_trees 0.7.x compatibility
        self.blackboard = py_trees.blackboard.Blackboard()
        
    def setup(self, timeout=None):
        """Setup the robot control service client"""
        try:
            # Don't wait for service during setup - defer to update() method
            # This prevents setup failures when robot services are not yet available
            self.logger.info("Setup completed - will connect to robot service when needed")
            return True
        except Exception as e:
            self.logger.error(f"Setup failed: {e}")
            return False

    def update(self):
        """Execute place down behavior"""
        # Connect to service if not already connected
        if not self.robot_service:
            try:
                rospy.wait_for_service('/arm_command', timeout=2.0)
                self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
                self.logger.info("Robot control service connected")
            except rospy.ROSException as e:
                self.logger.error(f"❌ Robot service not available: {e}")
                return py_trees.common.Status.FAILURE
            
        try:
            # Get picked object from blackboard
            picked_object = self.blackboard.get('picked_object')
            
            if not picked_object:
                self.logger.warn("⚠️ No object available to place down")
                # Return RUNNING to wait for an object to be picked
                return py_trees.common.Status.RUNNING
                
            # Use original object orientation for placing
            roll = picked_object.get('roll', 0.0)
            pitch = 1.438  # Default pitch for placing
            yaw = -0.35    # Default yaw for placing
            obj_class = picked_object.get('class', 'unknown')
            
            self.logger.info(f"📍 Placing {obj_class} at ({self.place_x:.3f}, {self.place_y:.3f}, {self.place_z:.3f})")
            
            # Send place command to robot
            command = f"place_at:{self.place_x},{self.place_y},{self.place_z},{roll},{pitch},{yaw}"
            req = RobotCommandRequest()
            req.command = command
            
            response = self.robot_service(req)
            
            if response.success:
                self.logger.info(f"✅ Successfully placed {obj_class} at ({self.place_x:.2f}, {self.place_y:.2f}, {self.place_z:.2f})")
                # Clear picked object from blackboard
                self.blackboard.set('picked_object', None)
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"❌ Place down failed: {response.message}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"❌ Error during place down: {e}")
            return py_trees.common.Status.FAILURE

class OpenGripper(py_trees.behaviour.Behaviour):
    """Behavior to open robot gripper"""
    
    def __init__(self, name):
        super(OpenGripper, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)
        self.robot_service = None
        
    def setup(self, timeout=None):
        """Setup the robot control service client"""
        try:
            # Don't wait for service during setup - defer to update() method
            # This prevents setup failures when robot services are not yet available
            self.logger.info("Setup completed - will connect to robot service when needed")
            return True
        except Exception as e:
            self.logger.error(f"Setup failed: {e}")
            return False

    def update(self):
        """Execute open gripper behavior"""
        # Connect to service if not already connected
        if not self.robot_service:
            try:
                rospy.wait_for_service('/arm_command', timeout=2.0)
                self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
                self.logger.info("Robot control service connected")
            except rospy.ROSException as e:
                self.logger.error(f"❌ Robot service not available: {e}")
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
        
    def setup(self, timeout=None):
        """Setup the robot control service client"""
        try:
            # Don't wait for service during setup - defer to update() method
            # This prevents setup failures when robot services are not yet available
            self.logger.info("Setup completed - will connect to robot service when needed")
            return True
        except Exception as e:
            self.logger.error(f"Setup failed: {e}")
            return False

    def update(self):
        """Execute close gripper behavior"""
        # Connect to service if not already connected
        if not self.robot_service:
            try:
                rospy.wait_for_service('/arm_command', timeout=2.0)
                self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
                self.logger.info("Robot control service connected")
            except rospy.ROSException as e:
                self.logger.error(f"❌ Robot service not available: {e}")
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
        
    def setup(self, timeout=None):
        """Setup the robot control service client"""
        try:
            # Don't wait for service during setup - defer to update() method
            # This prevents setup failures when robot services are not yet available
            self.logger.info("Setup completed - will connect to robot service when needed")
            return True
        except Exception as e:
            self.logger.error(f"Setup failed: {e}")
            return False

    def update(self):
        """Execute move to home behavior"""
        # Connect to service if not already connected
        if not self.robot_service:
            try:
                rospy.wait_for_service('/arm_command', timeout=2.0)
                self.robot_service = rospy.ServiceProxy('/arm_command', RobotCommand)
                self.logger.info("Robot control service connected")
            except rospy.ROSException as e:
                self.logger.error(f"❌ Robot service not available: {e}")
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
            
            # Check if root was successfully created
            if root is None:
                error_msg = "Failed to create behavior tree root from configuration"
                rospy.logerr(error_msg)
                return AssembleBehaviorTreeResponse(success=False, message=error_msg)
            
            # Store the assembled tree globally
            global current_behavior_tree, last_tree_config, current_tick_count
            current_behavior_tree = py_trees.trees.BehaviourTree(root)
            
            # Setup the behavior tree with more resilient error handling
            try:
                setup_success = current_behavior_tree.setup(timeout=15)
                if not setup_success:
                    rospy.logwarn("Some behaviors failed to setup - tree may have limited functionality")
                    rospy.logwarn("This is normal if robot services are not yet available")
                    # Don't fail completely - allow tree to run and behaviors will connect when services become available
                
                last_tree_config = tree_config
                
                # Reset tick counter for new task
                current_tick_count = 0
                rospy.loginfo("Global tick counter reset for new behavior tree task")
                
                # Add snapshot visitor for JSON publishing if available
                if json_publisher and JSON_SERIALIZATION_AVAILABLE and current_behavior_tree:
                    # Remove existing snapshot visitor if any
                    current_behavior_tree.visitors = [v for v in current_behavior_tree.visitors 
                                                    if not isinstance(v, py_trees.visitors.SnapshotVisitor)]
                    # Add our snapshot visitor
                    current_behavior_tree.visitors.append(json_publisher.get_snapshot_visitor())
                    
                    # Publish initial tree summary with structure
                    json_publisher.publish_tree_summary(current_behavior_tree, tree_config)
                
                rospy.loginfo("Behavior tree assembled successfully")
                return AssembleBehaviorTreeResponse(success=True, message="Behavior tree assembled successfully")
                
            except Exception as setup_error:
                rospy.logerr(f"Error during tree setup: {setup_error}")
                rospy.logwarn("Continuing with tree assembly - behaviors will connect to services when available")
                
                # Store config and continue - don't fail the entire assembly
                last_tree_config = tree_config
                current_tick_count = 0
                
                return AssembleBehaviorTreeResponse(success=True, 
                    message=f"Behavior tree assembled with setup warnings: {setup_error}")
            
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
        py_trees behavior: Root of the assembled tree, or None if failed
    """
    if not config:
        rospy.logerr("Empty configuration provided")
        return None
        
    root_type = config.get('type')
    root_name = config.get('name', 'Root')
    
    if root_type == 'sequence':
        root = py_trees.composites.Sequence(root_name, memory=False)
        
        # Add children from the configuration
        for child_config in config.get('children', []):
            child = create_behavior_from_config(child_config)
            if child:
                root.add_child(child)
            else:
                rospy.logwarn(f"Failed to create child behavior from config: {child_config}")
                
    elif root_type == 'selector':
        root = py_trees.composites.Selector(root_name, memory=False)
        
        # Add children from the configuration
        for child_config in config.get('children', []):
            child = create_behavior_from_config(child_config)
            if child:
                root.add_child(child)
            else:
                rospy.logwarn(f"Failed to create child behavior from config: {child_config}")
                
    else:
        # Unknown root behavior type
        rospy.logerr(f"Unknown or missing root behavior type: {root_type}")
        return None
    
    # Validate that root has at least one child
    if len(root.children) == 0:
        rospy.logerr(f"Root behavior '{root_name}' has no valid children")
        return None
    
    rospy.loginfo(f"Successfully assembled {root_type} root with {len(root.children)} children")
    return root

def create_behavior_from_config(config):
    """
    Create individual behaviors from configuration
    Args:
        config (dict): Configuration for a single behavior
    Returns:
        py_trees behavior: The created behavior, or None if failed
    """
    if not config:
        rospy.logwarn("Empty behavior configuration provided")
        return None
        
    behavior_type = config.get('type')
    behavior_name = config.get('name', 'UnnamedBehavior')
    
    try:
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
                else:
                    rospy.logwarn(f"Failed to create child for sequence '{behavior_name}'")
            return sequence
        elif behavior_type == 'selector':
            selector = py_trees.composites.Selector(behavior_name, memory=False)
            for child_config in config.get('children', []):
                child = create_behavior_from_config(child_config)
                if child:
                    selector.add_child(child)
                else:
                    rospy.logwarn(f"Failed to create child for selector '{behavior_name}'")
            return selector
        else:
            rospy.logerr(f"Unknown behavior type: {behavior_type}")
            return None
            
    except Exception as e:
        rospy.logerr(f"Error creating behavior '{behavior_name}' of type '{behavior_type}': {e}")
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

def cleanup_behavior_tree(tree):
    """
    Properly clean up all behaviors in the tree
    Args:
        tree: py_trees.trees.BehaviourTree object
    """
    if not tree or not tree.root:
        return
    
    try:
        # Iterate through all behaviors and call terminate method
        for behavior in tree.root.iterate():
            if hasattr(behavior, 'terminate'):
                behavior.terminate(py_trees.common.Status.INVALID)
        
        # Call terminate on root as well
        if hasattr(tree.root, 'terminate'):
            tree.root.terminate(py_trees.common.Status.INVALID)
            
        rospy.loginfo("Behavior tree cleanup completed")
        
    except Exception as e:
        rospy.logerr(f"Error during behavior tree cleanup: {e}")

# Global variables to store current behavior tree and JSON publisher
current_behavior_tree = None
json_publisher = None
last_tree_config = None 
current_tick_count = 0  # Global tick counter for monitoring stuck tasks 

def main():
    """
    Main function to initialize ROS node and run behavior tree
    """
    # Declare global variables at the top of function
    global json_publisher, current_behavior_tree, current_tick_count
    
    # Initialize the ROS node
    rospy.init_node('behavior_tree_node', anonymous=True)
    rospy.loginfo("Behavior Tree Node started")
    rospy.loginfo(f"Configuration: Tick frequency = {TICK_FREQUENCY_HZ} Hz")
    
    # Initialize JSON publisher
    if JSON_SERIALIZATION_AVAILABLE:
        json_publisher = BehaviorTreeJSONPublisher("behavior_tree_json")
        rospy.loginfo("Behavior tree JSON publisher initialized on topic: /behavior_tree_json")
    else:
        rospy.logwarn("JSON serialization disabled - tree visualization unavailable")
    
    # Start the behavior tree assembly service
    service0 = assemble_behavior_tree_service()
    service1 = hello_world_service()
    
    # Set the update rate using global frequency variable
    rate = rospy.Rate(TICK_FREQUENCY_HZ)
    rospy.loginfo(f"Behavior tree tick frequency set to {TICK_FREQUENCY_HZ} Hz")
    rospy.loginfo(f"Task stuck detection: max {MAX_TICKS_BEFORE_TIMEOUT} ticks ({MAX_TICKS_BEFORE_TIMEOUT/TICK_FREQUENCY_HZ:.1f} seconds)")
    
    # Initialize global tick counter
    current_tick_count = 0
    
    try:
        # Main execution loop
        while not rospy.is_shutdown():
            # Tick the current behavior tree if it exists
            if current_behavior_tree:
                # Perform the tick
                current_behavior_tree.tick()
                current_tick_count += 1
                
                # Get the root status after ticking
                status = current_behavior_tree.root.status
                
                # Log the tree status with tick information
                if ENABLE_DETAILED_LOGGING:
                    rospy.loginfo(f"Tick {current_tick_count}: Tree status = {status}")
                else:
                    rospy.loginfo_throttle(2.0, f"Tick {current_tick_count}: Tree status = {status}")
                
                # Publish JSON data after each tick if publisher is available
                if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                    json_publisher.publish_tree_data(current_behavior_tree)
                    if ENABLE_DETAILED_LOGGING:
                        rospy.logdebug(f"Published JSON data for tick {current_tick_count}")
                
                # Check if behavior tree completed successfully
                if status == py_trees.common.Status.SUCCESS:
                    rospy.loginfo(f"🎉 Behavior tree completed successfully after {current_tick_count} ticks!")
                    rospy.loginfo("Tree terminated. Waiting for next behavior tree task...")
                    
                    # Publish final status
                    if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                        json_publisher.publish_tree_data(current_behavior_tree, include_structure=True)
                    
                    # Properly clean up all behaviors before clearing tree
                    cleanup_behavior_tree(current_behavior_tree)
                    
                    # Clean up current tree and reset counter
                    current_behavior_tree = None
                    current_tick_count = 0
                    rospy.loginfo("Behavior tree cleared. Ready for new task.")
                    
                elif status == py_trees.common.Status.FAILURE:
                    rospy.logwarn(f"❌ Behavior tree failed after {current_tick_count} ticks!")
                    rospy.loginfo("Tree terminated due to failure. Waiting for next behavior tree task...")
                    
                    # Publish final status
                    if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                        json_publisher.publish_tree_data(current_behavior_tree, include_structure=True)
                    
                    # Properly clean up all behaviors before clearing tree
                    cleanup_behavior_tree(current_behavior_tree)
                    
                    # Clean up current tree and reset counter
                    current_behavior_tree = None
                    current_tick_count = 0
                    rospy.loginfo("Failed behavior tree cleared. Ready for new task.")
                    
                elif status == py_trees.common.Status.RUNNING:
                    # Tree is still executing - this is normal operation
                    if current_tick_count % 20 == 0:  # Log every 20 ticks to avoid spam
                        rospy.loginfo(f"🔄 Behavior tree running (tick {current_tick_count})")
                    
                    # Check for stuck task condition
                    if current_tick_count >= MAX_TICKS_BEFORE_TIMEOUT:
                        rospy.logerr(f"⏰ Task appears stuck after {current_tick_count} ticks ({current_tick_count/TICK_FREQUENCY_HZ:.1f} seconds)")
                        rospy.logerr("Terminating stuck behavior tree and waiting for new task...")
                        
                        # Publish final status before cleanup
                        if json_publisher and JSON_SERIALIZATION_AVAILABLE:
                            json_publisher.publish_tree_data(current_behavior_tree, include_structure=True)
                        
                        # Properly clean up all behaviors before clearing tree
                        cleanup_behavior_tree(current_behavior_tree)
                        
                        # Clean up stuck tree and reset counter
                        current_behavior_tree = None
                        current_tick_count = 0
                        rospy.logwarn("Stuck behavior tree cleared. Ready for new task.")
                    
                    elif current_tick_count % STUCK_CHECK_INTERVAL == 0:
                        # Periodic stuck check warning
                        remaining_ticks = MAX_TICKS_BEFORE_TIMEOUT - current_tick_count
                        remaining_time = remaining_ticks / TICK_FREQUENCY_HZ
                        rospy.logwarn(f"⚠️ Task running for {current_tick_count} ticks, will timeout in {remaining_ticks} ticks ({remaining_time:.1f}s)")
                
                elif status == py_trees.common.Status.INVALID:
                    rospy.logwarn(f"⚠️ Behavior tree has invalid status after {current_tick_count} ticks")
                    
                    # Check if invalid status persists for too long
                    if current_tick_count > 100:  # Reset after many invalid ticks
                        rospy.logerr("Resetting behavior tree due to persistent invalid status")
                        cleanup_behavior_tree(current_behavior_tree)
                        current_behavior_tree = None
                        current_tick_count = 0
            
            # Sleep to maintain the desired rate
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Behavior Tree Node shutting down")
    finally:
        # Clean shutdown with proper cleanup
        if current_behavior_tree:
            cleanup_behavior_tree(current_behavior_tree)
            current_behavior_tree = None
        rospy.loginfo(f"Behavior tree node completed {current_tick_count} ticks")

if __name__ == '__main__':
    main()
