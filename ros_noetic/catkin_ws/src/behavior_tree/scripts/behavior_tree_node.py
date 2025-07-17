#!/usr/bin/env python3
import py_trees
import rospy
import json
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String

# --- Define behaviors
class DetectObjects(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(DetectObjects, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)

    def update(self):
        self.logger.info("Foo1 is running")
        return py_trees.common.Status.SUCCESS

# Define root class
def create_root():
    """
    Create the root of the behavior tree
    """
    # Create the root behavior (a sequence that runs behaviors in order)
    root = py_trees.composites.Sequence("BehaviorTreeRoot", memory=False)
    
    # Add behaviors to the sequence
    detect_objects = DetectObjects("DetectObjects")
    root.add_child(detect_objects)
    
    return root

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
            req: Service request containing JSON string in req.data
        Returns:
            SetBoolResponse: success status and message
        """
        try:
            # Parse the JSON from the service request
            tree_config = json.loads(req.data)
            rospy.loginfo(f"Received behavior tree configuration: {tree_config}")
            
            # Assemble the behavior tree based on JSON configuration
            root = assemble_tree_from_json(tree_config)
            
            # Store the assembled tree globally (you might want to use a class instead)
            global current_behavior_tree
            current_behavior_tree = py_trees.trees.BehaviourTree(root)
            current_behavior_tree.setup(timeout=15)
            
            rospy.loginfo("Behavior tree assembled successfully")
            return SetBoolResponse(success=True, message="Behavior tree assembled successfully")
            
        except json.JSONDecodeError as e:
            error_msg = f"Failed to parse JSON: {str(e)}"
            rospy.logerr(error_msg)
            return SetBoolResponse(success=False, message=error_msg)
            
        except Exception as e:
            error_msg = f"Failed to assemble behavior tree: {str(e)}"
            rospy.logerr(error_msg)
            return SetBoolResponse(success=False, message=error_msg)
    
    # Create the service
    service = rospy.Service('assemble_behavior_tree', SetBool, handle_assemble_tree)
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
        # Default to a single behavior or fallback to create_root()
        root = create_behavior_from_config(config)
        if not root:
            rospy.logwarn("Unknown configuration, using default root")
            root = create_root()
    
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
    """
    Create a simple ROS service that responds with 'Hello World'
    """
    def handle_hello(req):
        """
        Service callback to respond with 'Hello World'
        Args:
            req: Service request (not used)
        Returns:
            String: 'Hello World' response
        """
        return String(data="Hello World")
    
    # Create the service
    service = rospy.Service('hello_world', String, handle_hello)
    rospy.loginfo("Hello World service started")
    return service

# Global variable to store current behavior tree
current_behavior_tree = None 

def main():
    """
    Main function to initialize ROS node and run behavior tree
    """
    # Initialize the ROS node
    rospy.init_node('behavior_tree_node', anonymous=True)
    rospy.loginfo("Behavior Tree Node started")
    
    # Start the behavior tree assembly service
    service0 = assemble_behavior_tree_service()
    service1 = hello_world_service()
    
    # Initialize with default behavior tree
    global current_behavior_tree
    root = create_root()
    current_behavior_tree = py_trees.trees.BehaviourTree(root)
    current_behavior_tree.setup(timeout=15)
    
    # Set the update rate (10 Hz)
    rate = rospy.Rate(10)
    
    try:
        # Main execution loop
        while not rospy.is_shutdown():
            # Tick the current behavior tree if it exists
            if current_behavior_tree:
                current_behavior_tree.tick()
                
                # Log the tree status
                rospy.loginfo_throttle(1.0, f"Tree status: {current_behavior_tree.root.status}")
            
            # Sleep to maintain the desired rate
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Behavior Tree Node shutting down")
    finally:
        # Clean shutdown
        if current_behavior_tree:
            current_behavior_tree.shutdown()

if __name__ == '__main__':
    main()
