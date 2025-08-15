import py_trees
import time
import os
import shutil
import imageio.v2 as imageio
import pydot  # <--- 我們需要 pydot 來建立自己的函式

# --- (行為和建立樹的函式保持不變) ---
class GetOutOfBed(py_trees.behaviour.Behaviour):
    def __init__(self, name="GetOutOfBed"):
        super(GetOutOfBed, self).__init__(name)
    def update(self):
        self.feedback_message = "I am up!"
        return py_trees.common.Status.SUCCESS

class MakeCoffee(py_trees.behaviour.Behaviour):
    def __init__(self, name="MakeCoffee"):
        super(MakeCoffee, self).__init__(name)
        self.start_time = None
    def initialise(self):
        self.start_time = time.time()
        self.feedback_message = "Grinding beans..."
    def update(self):
        if time.time() - self.start_time > 2.0:
            self.feedback_message = "Coffee is ready!"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Brewing..."
            return py_trees.common.Status.RUNNING

class ListenToMusic(py_trees.behaviour.Behaviour):
    def __init__(self, name="ListenToMusic"):
        super(ListenToMusic, self).__init__(name)
    def update(self):
        self.feedback_message = "La la la~"
        return py_trees.common.Status.RUNNING

class DressUp(py_trees.behaviour.Behaviour):
    def __init__(self, name, clothing_type):
        super(DressUp, self).__init__(name)
        self.clothing_type = clothing_type
    def update(self):
        self.feedback_message = f"Dressed up in {self.clothing_type}"
        return py_trees.common.Status.SUCCESS

class KeepSleeping(py_trees.behaviour.Behaviour):
    def __init__(self, name="KeepSleeping"):
        super(KeepSleeping, self).__init__(name)
    def update(self):
        self.feedback_message = "ZzzZzz..."
        return py_trees.common.Status.SUCCESS

def create_morning_routine_tree():
    is_weekday_check = py_trees.blackboard.CheckBlackboardVariable(
        name="Is it a weekday?", variable_name="is_weekday", expected_value=True)
    
    # --- 【THE FIX IS HERE】 ---
    # Change the policy to succeed as soon as one child succeeds.
    morning_prep_parallel = py_trees.composites.Parallel(
        name="Morning Prep",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE  # <-- 從 SUCCESS_ON_ALL 改為 SUCCESS_ON_ONE
    )
    # ---------------------------

    morning_prep_parallel.add_children([MakeCoffee(), ListenToMusic()])
    weekday_routine = py_trees.composites.Sequence(name="Weekday Routine")
    weekday_routine.add_children([is_weekday_check, GetOutOfBed(), morning_prep_parallel,
                                  DressUp(name="DressFormal", clothing_type="formal wear")])
    weekend_routine = py_trees.composites.Sequence(name="Weekend Routine")
    weekend_routine.add_children([KeepSleeping(), DressUp(name="DressCasual", clothing_type="casual wear")])
    root = py_trees.composites.Selector(name="Morning Decision")
    root.add_children([weekday_routine, weekend_routine])
    return root

def generate_pydot_graph_with_status(root, snapshot_visitor):
    """
    一個增強版的函式，它會根據 SnapshotVisitor 的資料來為節點上色。
    """
    status_to_color = {
        py_trees.common.Status.SUCCESS: "limegreen",
        py_trees.common.Status.FAILURE: "red",
        py_trees.common.Status.RUNNING: "yellow",
        py_trees.common.Status.INVALID: "grey",
    }

    def get_node_attributes(node, snapshot_nodes):
        # 預設顏色
        color = "lightgrey"
        # 如果節點在快照中，就用狀態顏色覆蓋
        if node.id in snapshot_nodes:
            color = status_to_color[snapshot_nodes[node.id]]
        
        # 節點形狀
        if isinstance(node, py_trees.composites.Selector):
            shape = "octagon"
        elif isinstance(node, py_trees.composites.Sequence):
            shape = "box"
        elif isinstance(node, py_trees.composites.Parallel):
            shape = "parallelogram"
        else: # Behaviour
            shape = "ellipse"
        
        return (shape, color)

    graph = pydot.Dot(graph_type='digraph')
    graph.set_node_defaults(fontname='Arial', fontsize='11')

    nodes = {root.id: root}
    for child in root.iterate():
        nodes[child.id] = child

    for node_id, node in nodes.items():
        shape, color = get_node_attributes(node, snapshot_visitor.nodes)
        pydot_node = pydot.Node(
            name=str(node.id), # 使用唯一的 ID 作為節點名稱
            label=node.name.replace('\n', ' '),
            shape=shape,
            style="filled",
            fillcolor=color
        )
        graph.add_node(pydot_node)
        if node.parent:
            edge = pydot.Edge(str(node.parent.id), str(node.id))
            graph.add_edge(edge)
            
    return graph

def render_dot_tree_with_status(root, snapshot_visitor, name):
    """使用我們的增強版函式來產生圖片"""
    graph = generate_pydot_graph_with_status(root, snapshot_visitor)
    filename_wo_extension = name
    print(f"Writing {filename_wo_extension}.png")
    graph.write_png(f"{filename_wo_extension}.png")

# --- 主程式 ---
def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("is_weekday", True)
    behaviour_tree = py_trees.trees.BehaviourTree(root=create_morning_routine_tree())
    
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.visitors.append(snapshot_visitor)

    print("--- Generating Animation Frames (with custom renderer) ---")
    animation_frames = []
    frames_dir = "frames"
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)

    for i in range(1, 6):
        print(f"\n--------- Tick {i} ---------")
        behaviour_tree.tick()
        
        frame_basename = f"{frames_dir}/frame_{i:02d}"
        
        # 【呼叫我們自己的函式】
        render_dot_tree_with_status(behaviour_tree.root, snapshot_visitor, frame_basename)
        animation_frames.append(f"{frame_basename}.png")
        
        if behaviour_tree.root.status != py_trees.common.Status.RUNNING:
            break
        
        time.sleep(1)

    print("\n--- Compiling GIF from frames ---")
    gif_filename = "morning_routine_animation.gif"
    with imageio.get_writer(gif_filename, mode='I', duration=1000.0, loop=0) as writer:
        for filename in animation_frames:
            image = imageio.imread(filename)
            writer.append_data(image)
    
    print(f"--- Animation saved to {gif_filename} ---")
    print("--- Cleaning up temporary frame files ---")
    # shutil.rmtree(frames_dir) 
    print("--- Done ---")

if __name__ == '__main__':
    main()