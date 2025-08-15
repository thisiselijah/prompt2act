#!/usr/bin/env python3

"""
Test script to verify behavior tree cleanup functionality
This script simulates the behavior cleanup without requiring ROS dependencies
"""

class MockLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def error(self, msg):
        print(f"[ERROR] {msg}")

class MockSubscriber:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback
        self.is_registered = True
        print(f"[MOCK] Created subscriber for topic: {topic}")
    
    def unregister(self):
        if self.is_registered:
            self.is_registered = False
            print(f"[MOCK] Unregistered subscriber for topic: {self.topic}")
        else:
            print(f"[MOCK] Subscriber already unregistered: {self.topic}")

class MockBehavior:
    def __init__(self, name):
        self.name = name
        self.logger = MockLogger()
        self.subscriber = None
        
    def setup(self):
        if not self.subscriber:
            self.subscriber = MockSubscriber('/test_topic', self._callback)
            self.logger.info(f"Setup complete for {self.name}")
            
    def terminate(self, status):
        try:
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
                self.logger.info(f"Cleanup completed for {self.name}")
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
    
    def _callback(self, msg):
        pass

def test_behavior_cleanup():
    """Test that demonstrates the fixed behavior cleanup"""
    print("=== Testing Behavior Cleanup ===")
    
    # Create a mock behavior (similar to DetectObjects)
    behavior = MockBehavior("TestDetectObjects")
    
    # Setup the behavior (creates subscriber)
    behavior.setup()
    print(f"Subscriber registered: {behavior.subscriber.is_registered}")
    
    # Simulate behavior tree completion and cleanup
    print("\n--- Simulating behavior tree SUCCESS termination ---")
    behavior.terminate("SUCCESS")
    
    # Verify cleanup
    if behavior.subscriber is None:
        print("✅ SUCCESS: Subscriber properly cleaned up")
    else:
        print("❌ FAILURE: Subscriber still exists after cleanup")
    
    print("\n=== Test Complete ===")

if __name__ == "__main__":
    test_behavior_cleanup()
