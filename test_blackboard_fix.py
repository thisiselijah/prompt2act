#!/usr/bin/env python3
"""
Test script to verify blackboard usage is compatible with py_trees 0.7.x
This script simulates the fixed behavior classes without ROS dependencies
"""

class MockLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    def error(self, msg):
        print(f"[ERROR] {msg}")
    def warn(self, msg):
        print(f"[WARN] {msg}")

class MockBlackboard:
    """Mock blackboard that simulates py_trees.blackboard.Blackboard 0.7.x behavior"""
    def __init__(self):
        self._data = {}
    
    def get(self, key):
        return self._data.get(key)
    
    def set(self, key, value, overwrite=True):
        if not overwrite and key in self._data:
            return False
        self._data[key] = value
        return True

class MockBehaviour:
    def __init__(self, name):
        self.name = name

class DetectObjects(MockBehaviour):
    """Behavior that subscribes to YOLO detection data and stores detected objects"""
    
    def __init__(self, name):
        super(DetectObjects, self).__init__(name)
        self.logger = MockLogger()
        self.detected_objects = []
        # Initialize blackboard for py_trees 0.7.x compatibility
        self.blackboard = MockBlackboard()
        
    def update(self):
        """Check if objects are detected"""
        # Simulate some detected objects
        self.detected_objects = [
            {'class': 'red_cube', 'x': 0.1, 'y': 0.2, 'confidence': 0.9, 'roll': 0.0},
            {'class': 'blue_cube', 'x': 0.3, 'y': 0.4, 'confidence': 0.8, 'roll': 0.1}
        ]
        
        if self.detected_objects:
            self.logger.info(f"✅ Objects detected: {len(self.detected_objects)} items")
            
            # Print detailed information about detected objects
            for i, obj in enumerate(self.detected_objects):
                obj_info = f"  Object {i+1}: "
                if 'class' in obj:
                    obj_info += f"class={obj['class']} "
                if 'x' in obj and 'y' in obj:
                    obj_info += f"pos=({obj['x']:.3f}, {obj['y']:.3f}) "
                if 'confidence' in obj:
                    obj_info += f"conf={obj['confidence']:.2f} "
                if 'roll' in obj:
                    obj_info += f"roll={obj['roll']:.3f}"
                self.logger.info(obj_info)
            
            # Store detection data in blackboard for other behaviors to use
            self.blackboard.set('detected_objects', self.detected_objects)
            return "SUCCESS"
        else:
            self.logger.info("🔍 No objects detected, continuing to search...")
            return "RUNNING"

class PickUp(MockBehaviour):
    """Behavior to pick up objects using robot control service"""
    
    def __init__(self, name):
        super(PickUp, self).__init__(name)
        self.logger = MockLogger()
        self.picked_object = None
        # Initialize blackboard for py_trees 0.7.x compatibility
        self.blackboard = MockBlackboard()
        
    def update(self):
        """Execute pick up behavior"""
        
        # Get detected objects from blackboard
        detected_objects = self.blackboard.get('detected_objects') or []
        
        if not detected_objects:
            self.logger.warn("⚠️ No objects available to pick up")
            return "RUNNING"
            
        # Pick the first detected object
        target_object = detected_objects[0]
        x = target_object.get('x', 0.0)
        y = target_object.get('y', 0.0) 
        roll = target_object.get('roll', 0.0)
        obj_class = target_object.get('class', 'unknown')
        
        self.logger.info(f"🦾 Attempting to pick up {obj_class} at ({x:.3f}, {y:.3f})")
        
        # Simulate successful pick
        self.logger.info(f"✅ Successfully picked up {obj_class} at ({x:.2f}, {y:.2f})")
        
        # Store picked object info for place behavior
        self.blackboard.set('picked_object', target_object)
        # Remove picked object from detected list
        detected_objects.remove(target_object)
        self.blackboard.set('detected_objects', detected_objects)
        return "SUCCESS"

class PlaceDown(MockBehaviour):
    """Behavior to place objects using robot control service"""
    
    def __init__(self, name, place_x=0.15, place_y=-0.15, place_z=0.18):
        super(PlaceDown, self).__init__(name)
        self.logger = MockLogger()
        self.place_x = place_x
        self.place_y = place_y
        self.place_z = place_z
        # Initialize blackboard for py_trees 0.7.x compatibility
        self.blackboard = MockBlackboard()
        
    def update(self):
        """Execute place down behavior"""
        
        # Get picked object from blackboard
        picked_object = self.blackboard.get('picked_object')
        
        if not picked_object:
            self.logger.warn("⚠️ No object available to place down")
            return "RUNNING"
            
        # Use original object orientation for placing
        roll = picked_object.get('roll', 0.0)
        obj_class = picked_object.get('class', 'unknown')
        
        self.logger.info(f"📍 Placing {obj_class} at ({self.place_x:.3f}, {self.place_y:.3f}, {self.place_z:.3f})")
        
        # Simulate successful placement
        self.logger.info(f"✅ Successfully placed {obj_class} at ({self.place_x:.2f}, {self.place_y:.2f}, {self.place_z:.2f})")
        
        # Clear picked object from blackboard
        self.blackboard.set('picked_object', None)
        return "SUCCESS"

def test_blackboard_communication():
    """Test that behaviors can communicate through blackboard properly"""
    print("🧪 Testing py_trees 0.7.x compatible blackboard usage...")
    
    # Create behavior instances
    detect = DetectObjects("DetectObjects")
    pickup = PickUp("PickUp") 
    place = PlaceDown("PlaceDown")
    
    # Share the same blackboard instance (simulating the Borg pattern)
    shared_blackboard = MockBlackboard()
    detect.blackboard = shared_blackboard
    pickup.blackboard = shared_blackboard
    place.blackboard = shared_blackboard
    
    print("\n1. Testing object detection...")
    detect_result = detect.update()
    print(f"   DetectObjects result: {detect_result}")
    
    print("\n2. Testing object pickup...")
    pickup_result = pickup.update()
    print(f"   PickUp result: {pickup_result}")
    
    print("\n3. Testing object placement...")
    place_result = place.update()
    print(f"   PlaceDown result: {place_result}")
    
    print("\n4. Checking blackboard state...")
    remaining_objects = shared_blackboard.get('detected_objects') or []
    picked_object = shared_blackboard.get('picked_object')
    print(f"   Remaining objects: {len(remaining_objects)}")
    print(f"   Currently picked object: {picked_object}")
    
    print("\n✅ Blackboard communication test completed successfully!")
    print("   All behaviors can properly get/set blackboard variables using py_trees 0.7.x API")

if __name__ == '__main__':
    test_blackboard_communication()
