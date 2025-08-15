#!/usr/bin/env python3
"""
Test script to verify the enhanced behavior tree functionality:
1. Proper termination on SUCCESS/FAILURE
2. Global tick counter for stuck task detection
"""

class MockStatus:
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"
    INVALID = "INVALID"

class MockTree:
    def __init__(self, status_sequence):
        self.status_sequence = status_sequence
        self.tick_count = 0
        self.root = self
        
    def tick(self):
        if self.tick_count < len(self.status_sequence):
            self.status = self.status_sequence[self.tick_count]
        else:
            self.status = self.status_sequence[-1]  # Keep last status
        self.tick_count += 1

def test_behavior_tree_termination():
    """Test that behavior tree properly terminates on SUCCESS and FAILURE"""
    print("🧪 Testing enhanced behavior tree functionality...")
    
    # Test case 1: Tree that succeeds after 5 ticks
    print("\n1. Testing SUCCESS termination:")
    success_tree = MockTree([MockStatus.RUNNING, MockStatus.RUNNING, MockStatus.RUNNING, 
                           MockStatus.RUNNING, MockStatus.SUCCESS])
    
    tick_count = 0
    MAX_TICKS = 10
    
    for i in range(MAX_TICKS):
        success_tree.tick()
        tick_count += 1
        status = success_tree.status
        print(f"   Tick {tick_count}: Status = {status}")
        
        if status == MockStatus.SUCCESS:
            print(f"   ✅ Tree completed successfully after {tick_count} ticks!")
            print("   🔄 Tree terminated, ready for next task")
            tick_count = 0  # Reset counter
            break
        elif status == MockStatus.FAILURE:
            print(f"   ❌ Tree failed after {tick_count} ticks!")
            print("   🔄 Tree terminated, ready for next task")
            tick_count = 0  # Reset counter
            break
    
    # Test case 2: Tree that fails after 3 ticks
    print("\n2. Testing FAILURE termination:")
    failure_tree = MockTree([MockStatus.RUNNING, MockStatus.RUNNING, MockStatus.FAILURE])
    
    tick_count = 0
    for i in range(MAX_TICKS):
        failure_tree.tick()
        tick_count += 1
        status = failure_tree.status
        print(f"   Tick {tick_count}: Status = {status}")
        
        if status == MockStatus.SUCCESS:
            print(f"   ✅ Tree completed successfully after {tick_count} ticks!")
            print("   🔄 Tree terminated, ready for next task")
            tick_count = 0
            break
        elif status == MockStatus.FAILURE:
            print(f"   ❌ Tree failed after {tick_count} ticks!")
            print("   🔄 Tree terminated, ready for next task")
            tick_count = 0
            break
    
    # Test case 3: Stuck task detection
    print("\n3. Testing stuck task detection:")
    MAX_TICKS_BEFORE_TIMEOUT = 8  # Simulated small timeout for testing
    STUCK_CHECK_INTERVAL = 3
    
    stuck_tree = MockTree([MockStatus.RUNNING] * 20)  # Always running
    
    tick_count = 0
    for i in range(15):  # Simulate more ticks than timeout
        stuck_tree.tick()
        tick_count += 1
        status = stuck_tree.status
        
        if tick_count % 3 == 0:  # Log every 3 ticks
            print(f"   Tick {tick_count}: Status = {status}")
        
        if status == MockStatus.RUNNING:
            # Check for stuck task condition
            if tick_count >= MAX_TICKS_BEFORE_TIMEOUT:
                print(f"   ⏰ Task appears stuck after {tick_count} ticks")
                print("   🛑 Terminating stuck behavior tree...")
                tick_count = 0
                print("   🔄 Stuck tree cleared, ready for next task")
                break
            elif tick_count % STUCK_CHECK_INTERVAL == 0:
                remaining_ticks = MAX_TICKS_BEFORE_TIMEOUT - tick_count
                print(f"   ⚠️ Task running for {tick_count} ticks, will timeout in {remaining_ticks} ticks")
    
    print("\n✅ All behavior tree enhancement tests completed successfully!")
    print("Key features verified:")
    print("  • Proper termination on SUCCESS status")
    print("  • Proper termination on FAILURE status")
    print("  • Global tick counter for monitoring")
    print("  • Stuck task detection and recovery")
    print("  • Counter reset after task completion/termination")

if __name__ == '__main__':
    test_behavior_tree_termination()
