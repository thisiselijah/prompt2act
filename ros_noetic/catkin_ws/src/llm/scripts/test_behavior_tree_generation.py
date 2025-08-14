#!/root/.pyenv/versions/3.9.19/bin/python3.9

"""
Test script for behavior tree generation with automatic assembly
"""

import rospy
from llm.srv import GenerateBehaviorTree, GenerateBehaviorTreeRequest
import sys

def test_behavior_tree_generation():
    """Test the behavior tree generation and assembly service"""
    rospy.init_node('test_behavior_tree_generation', anonymous=True)
    
    # Wait for the service to be available
    rospy.loginfo("Waiting for behavior tree generation service...")
    rospy.wait_for_service('/generate_behavior_tree')
    
    # Create service proxy
    generate_tree_service = rospy.ServiceProxy('/generate_behavior_tree', GenerateBehaviorTree)
    
    # Test cases
    test_cases = [
        {
            "description": "Pick up a red cube and place it on the table",
            "auto_assemble": True
        },
        {
            "description": "Find all objects, pick up the smallest one, and place it in the box",
            "auto_assemble": True
        },
        {
            "description": "Detect objects and sort them by color",
            "auto_assemble": False  # Just generate JSON, don't assemble
        }
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== Test Case {i} ===")
        rospy.loginfo(f"Task: {test_case['description']}")
        rospy.loginfo(f"Auto-assemble: {test_case['auto_assemble']}")
        
        try:
            # Create request
            request = GenerateBehaviorTreeRequest()
            request.task_description = test_case['description']
            request.auto_assemble = test_case['auto_assemble']
            
            # Call service
            response = generate_tree_service(request)
            
            if response.success:
                rospy.loginfo("✓ Success!")
                rospy.loginfo(f"Generated JSON: {response.behavior_tree_json}")
                if response.error_message:
                    rospy.logwarn(f"Warning: {response.error_message}")
            else:
                rospy.logerr(f"✗ Failed: {response.error_message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"✗ Service call failed: {e}")
        
        # Wait a bit between test cases
        rospy.sleep(2.0)
    
    rospy.loginfo("\n=== Test completed ===")

if __name__ == '__main__':
    try:
        test_behavior_tree_generation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Test stopped by user")
