#!/usr/bin/python3
import py_trees
import rospy
import json
import os
import time
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String
from behavior_tree.srv import helloworld, helloworldResponse

# Import for visualization
try:
    import pydot
    VISUALIZATION_AVAILABLE = True
except ImportError:
    rospy.logwarn("pydot not available. Visualization features will be disabled.")
    VISUALIZATION_AVAILABLE = False

# --- Define behaviors
class DetectObjects(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(DetectObjects, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)

    def update(self):
        self.logger.info("Foo1 is running")
        return py_trees.common.Status.SUCCESS

class PickUp(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PickUp, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)

    def update(self):
        self.logger.info("Foo2 is running")
        return py_trees.common.Status.SUCCESS

class PlaceDown(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PlaceDown, self).__init__(name)
        self.logger = py_trees.logging.Logger(name)

    def update(self):
        self.logger.info("Foo3 is running")
        return py_trees.common.Status.SUCCESS

# --- Visualization Functions ---
def generate_pydot_graph_with_status(root, snapshot_visitor=None):
    """
    Generate a pydot graph with behavior tree nodes colored by their status
    Args:
        root: Root node of the behavior tree
        snapshot_visitor: py_trees SnapshotVisitor to get node statuses
    Returns:
        pydot.Dot: Graph object
    """
    if not VISUALIZATION_AVAILABLE:
        rospy.logwarn("Visualization not available - pydot not installed")
        return None
    
    status_to_color = {
        py_trees.common.Status.SUCCESS: "limegreen",
        py_trees.common.Status.FAILURE: "red", 
        py_trees.common.Status.RUNNING: "yellow",
        py_trees.common.Status.INVALID: "lightgrey",
    }

    def get_node_attributes(node, snapshot_nodes=None):
        # Default color
        color = "lightgrey"
        # If node is in snapshot, override with status color
        if snapshot_nodes and node.id in snapshot_nodes:
            color = status_to_color.get(snapshot_nodes[node.id], "lightgrey")
        elif hasattr(node, 'status'):
            color = status_to_color.get(node.status, "lightgrey")
        
        # Node shape based on type
        if isinstance(node, py_trees.composites.Selector):
            shape = "octagon"
        elif isinstance(node, py_trees.composites.Sequence):
            shape = "box"
        elif isinstance(node, py_trees.composites.Parallel):
            shape = "parallelogram"
        else:  # Behaviour
            shape = "ellipse"
        
        return (shape, color)

    graph = pydot.Dot(graph_type='digraph')
    graph.set_node_defaults(fontname='Arial', fontsize='11')

    # Collect all nodes
    nodes = {root.id: root}
    for child in root.iterate():
        nodes[child.id] = child

    # Get snapshot data if available
    snapshot_nodes = snapshot_visitor.nodes if snapshot_visitor else None

    # Add nodes to graph
    for node_id, node in nodes.items():
        shape, color = get_node_attributes(node, snapshot_nodes)
        pydot_node = pydot.Node(
            name=str(node.id),  # Use unique ID as node name
            label=node.name.replace('\n', ' '),
            shape=shape,
            style="filled",
            fillcolor=color
        )
        graph.add_node(pydot_node)
        
        # Add edges to parent
        if node.parent:
            edge = pydot.Edge(str(node.parent.id), str(node.id))
            graph.add_edge(edge)
            
    return graph

def render_dot_tree_with_status(root, snapshot_visitor, filepath):
    """
    Render behavior tree to PNG file with status coloring
    Args:
        root: Root node of behavior tree
        snapshot_visitor: SnapshotVisitor for status information
        filepath: Output file path (without extension)
    """
    if not VISUALIZATION_AVAILABLE:
        rospy.logwarn("Cannot render tree - pydot not available")
        return False
    
    try:
        graph = generate_pydot_graph_with_status(root, snapshot_visitor)
        if graph:
            png_filepath = f"{filepath}.png"
            rospy.loginfo(f"Writing behavior tree visualization: {png_filepath}")
            graph.write_png(png_filepath)
            return True
    except Exception as e:
        rospy.logerr(f"Failed to render behavior tree: {str(e)}")
    
    return False

class BehaviorTreeVisualizer:
    """Class to handle behavior tree visualization and frame generation"""
    
    def __init__(self, output_dir="/frames"):
        self.output_dir = output_dir
        self.frame_counter = 0
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo(f"Created visualization output directory: {self.output_dir}")
    
    def get_snapshot_visitor(self):
        """Get the snapshot visitor for adding to behavior tree"""
        return self.snapshot_visitor
    
    def visualize_tree(self, tree, filename_prefix="behavior_tree"):
        """
        Visualize the current behavior tree and save as PNG
        Args:
            tree: py_trees.trees.BehaviourTree object
            filename_prefix: prefix for the output filename
        Returns:
            bool: Success status
        """
        if not VISUALIZATION_AVAILABLE:
            return False
        
        try:
            if tree and tree.root:
                filename = f"{filename_prefix}_frame_{self.frame_counter:04d}"
                filepath = os.path.join(self.output_dir, filename)
                
                success = render_dot_tree_with_status(tree.root, self.snapshot_visitor, filepath)
                if success:
                    self.frame_counter += 1
                    rospy.loginfo(f"Saved behavior tree frame {self.frame_counter}")
                return success
            else:
                rospy.logwarn("No tree to visualize")
                return False
                
        except Exception as e:
            rospy.logerr(f"Failed to visualize behavior tree: {str(e)}")
            return False
    
    def create_tree_summary(self, tree, tree_config=None):
        """
        Create a summary visualization of the tree structure
        Args:
            tree: py_trees.trees.BehaviourTree object  
            tree_config: Original JSON configuration (optional)
        """
        if not VISUALIZATION_AVAILABLE:
            return False
        
        try:
            if tree and tree.root:
                filename = f"tree_summary_{int(time.time())}"
                filepath = os.path.join(self.output_dir, filename)
                
                success = render_dot_tree_with_status(tree.root, self.snapshot_visitor, filepath)
                if success:
                    rospy.loginfo(f"Created tree summary: {filepath}.png")
                return success
                
        except Exception as e:
            rospy.logerr(f"Failed to create tree summary: {str(e)}")
        
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
            
            # Store the assembled tree globally
            global current_behavior_tree, visualizer, last_tree_config
            current_behavior_tree = py_trees.trees.BehaviourTree(root)
            current_behavior_tree.setup(timeout=15)
            last_tree_config = tree_config
            
            # Add snapshot visitor for visualization if available
            if visualizer and VISUALIZATION_AVAILABLE:
                # Remove existing snapshot visitor if any
                current_behavior_tree.visitors = [v for v in current_behavior_tree.visitors 
                                                if not isinstance(v, py_trees.visitors.SnapshotVisitor)]
                # Add our snapshot visitor
                current_behavior_tree.visitors.append(visualizer.get_snapshot_visitor())
                
                # Create initial tree summary
                visualizer.create_tree_summary(current_behavior_tree, tree_config)
            
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
        return PlaceDown(behavior_name)
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

# Global variables to store current behavior tree and visualizer
current_behavior_tree = None
visualizer = None
last_tree_config = None 

def main():
    """
    Main function to initialize ROS node and run behavior tree
    """
    # Initialize the ROS node
    rospy.init_node('behavior_tree_node', anonymous=True)
    rospy.loginfo("Behavior Tree Node started")
    
    # Initialize visualizer
    global visualizer
    if VISUALIZATION_AVAILABLE:
        visualizer = BehaviorTreeVisualizer()
        rospy.loginfo("Behavior tree visualizer initialized")
    else:
        rospy.logwarn("Visualization disabled - pydot not available")
    
    # Start the behavior tree assembly service
    service0 = assemble_behavior_tree_service()
    service1 = hello_world_service()
    
    # Set the update rate (10 Hz)
    rate = rospy.Rate(10)
    
    try:
        # Main execution loop
        while not rospy.is_shutdown():
            # Tick the current behavior tree if it exists
            global current_behavior_tree
            if current_behavior_tree:
                current_behavior_tree.tick()
                
                # Log the tree status
                rospy.loginfo_throttle(1.0, f"Tree status: {current_behavior_tree.root.status}")
                
                # Visualize tree every few ticks if visualizer is available
                if visualizer and VISUALIZATION_AVAILABLE:
                    # Visualize every 10 ticks (1 second at 10Hz)
                    if hasattr(current_behavior_tree, '_tick_count'):
                        current_behavior_tree._tick_count += 1
                    else:
                        current_behavior_tree._tick_count = 1
                    
                    if current_behavior_tree._tick_count % 10 == 0:
                        visualizer.visualize_tree(current_behavior_tree)
            
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
