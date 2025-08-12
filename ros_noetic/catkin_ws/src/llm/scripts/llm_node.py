#!/root/.pyenv/versions/3.9.19/bin/python3.9

import sys
print(sys.executable) 

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse
import json
from setup import OpenAIProvider, GeminiProvider

# Global constants for behavior tree generation
BEHAVIOR_TREE_PROMPT_TEMPLATE = """
Generate a behavior tree JSON configuration for a robot manipulation task. The task description is: {task_description}

Please create a JSON structure with the following requirements:
1. Root node should be either 'sequence' or 'selector'
2. Include appropriate behavior types: 'detect_objects', 'pick_up', 'place_down'
3. Use proper nesting for complex behaviors
4. Each node should have 'type' and 'name' fields
5. Composite nodes (sequence/selector) should have 'children' array

Available behavior types:
- detect_objects: For detecting objects in the environment
- pick_up: For picking up objects
- place_down: For placing objects down
- sequence: Execute children in order (all must succeed)
- selector: Try children until one succeeds

Example format:
{{
  "type": "sequence",
  "name": "MainTask",
  "children": [
    {{
      "type": "detect_objects",
      "name": "DetectObjects"
    }},
    {{
      "type": "pick_up", 
      "name": "PickUpObject"
    }},
    {{
      "type": "place_down",
      "name": "PlaceObject"
    }}
  ]
}}

Task description: {task_description}
"""

BEHAVIOR_TREE_SCHEMA = {
    "type": "object",
    "properties": {
        "type": {"type": "string", "enum": ["sequence", "selector"]},
        "name": {"type": "string"},
        "children": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "type": {"type": "string"},
                    "name": {"type": "string"},
                    "children": {"type": "array"}
                },
                "required": ["type", "name"]
            }
        }
    },
    "required": ["type", "name", "children"]
}

DEFAULT_TASK_DESCRIPTION = "Pick up an object and place it in a designated location"


class LLMNode:
    """Main LLM Node class"""
    
    def __init__(self):
        self.providers = {
            'openai': OpenAIProvider(),
            'gemini': GeminiProvider()
        }
        self.current_provider = None
        
        # ROS publishers and subscribers
        self.response_pub = rospy.Publisher('/llm_response', String, queue_size=10)
        self.json_response_pub = rospy.Publisher('/llm_json_response', String, queue_size=10)
        self.prompt_sub = rospy.Subscriber('/llm_prompt', String, self.prompt_callback)
        self.json_prompt_sub = rospy.Subscriber('/llm_json_prompt', String, self.json_prompt_callback)
        
        # ROS services
        self.query_service = rospy.Service('/llm_query', Trigger, self.query_service_callback)
        self.json_query_service = rospy.Service('/llm_json_query', SetBool, self.json_query_service_callback)
        self.behavior_tree_service = rospy.Service('/generate_behavior_tree', SetBool, self.generate_behavior_tree_callback)
        self.status_service = rospy.Service('/llm_status', Trigger, self.status_service_callback)
    
    def initialize_provider(self, provider_type: str) -> bool:
        """Initialize the specified LLM provider"""
        if provider_type not in self.providers:
            rospy.logerr(f"Unknown provider: {provider_type}")
            return False
        
        provider = self.providers[provider_type]
        if provider.initialize():
            self.current_provider = provider
            rospy.loginfo(f"Successfully initialized {provider_type} provider")
            return True
        else:
            rospy.logerr(f"Failed to initialize {provider_type} provider")
            return False
    
    def prompt_callback(self, msg):
        """Handle incoming prompts from topic"""
        if not self.current_provider:
            rospy.logwarn("No LLM provider initialized")
            return
        
        prompt = msg.data
        rospy.loginfo(f"Received prompt: {prompt[:100]}...")
        
        try:
            response = self.current_provider.generate_response(prompt)
            rospy.loginfo(f"Generated response: {response[:100]}...")
            
            # Publish response
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)
            
        except Exception as e:
            rospy.logerr(f"Error generating response: {e}")
    
    def json_prompt_callback(self, msg):
        """Handle incoming JSON prompts from topic"""
        if not self.current_provider:
            rospy.logwarn("No LLM provider initialized")
            return
        
        prompt = msg.data
        rospy.loginfo(f"Received JSON prompt: {prompt[:100]}...")
        
        try:
            response = self.current_provider.generate_json_response(prompt)
            rospy.loginfo(f"Generated JSON response: {response[:100]}...")
            
            # Publish JSON response
            response_msg = String()
            response_msg.data = response
            self.json_response_pub.publish(response_msg)
            
        except Exception as e:
            rospy.logerr(f"Error generating JSON response: {e}")
    
    def query_service_callback(self, req):
        """Service callback for direct LLM queries"""
        if not self.current_provider:
            return TriggerResponse(success=False, message="No LLM provider initialized")
        
        try:
            # For service calls, we could get the prompt from a parameter
            prompt = rospy.get_param('~query_prompt', 'Hello, how are you?')
            response = self.current_provider.generate_response(prompt)
            
            return TriggerResponse(success=True, message=response)
        except Exception as e:
            return TriggerResponse(success=False, message=f"Error: {str(e)}")
    
    def json_query_service_callback(self, req):
        """Service callback for JSON LLM queries"""
        if not self.current_provider:
            return SetBoolResponse(success=False, message="No LLM provider initialized")
        
        try:
            prompt = req.data if hasattr(req, 'data') and req.data else "Generate a simple JSON object"
            response = self.current_provider.generate_json_response(prompt)
            
            return SetBoolResponse(success=True, message=response)
        except Exception as e:
            return SetBoolResponse(success=False, message=f"Error: {str(e)}")
    
    def generate_behavior_tree_callback(self, req):
        """Service callback for generating behavior tree JSON"""
        if not self.current_provider:
            return SetBoolResponse(success=False, message="No LLM provider initialized")
        
        try:
            # Get task description from request or use default
            task_description = req.data if hasattr(req, 'data') and req.data else DEFAULT_TASK_DESCRIPTION
            
            # Format the prompt with the task description
            behavior_tree_prompt = BEHAVIOR_TREE_PROMPT_TEMPLATE.format(task_description=task_description)
            
            # Generate JSON response using schema (for Gemini)
            if hasattr(self.current_provider, 'generate_json_response'):
                response = self.current_provider.generate_json_response(behavior_tree_prompt, BEHAVIOR_TREE_SCHEMA)
            else:
                response = self.current_provider.generate_json_response(behavior_tree_prompt)
            
            # Validate that it's proper JSON
            try:
                json.loads(response)
                rospy.loginfo(f"Generated behavior tree JSON: {response}")
                return SetBoolResponse(success=True, message=response)
            except json.JSONDecodeError as e:
                rospy.logerr(f"Generated response is not valid JSON: {e}")
                return SetBoolResponse(success=False, message=f"Invalid JSON generated: {str(e)}")
                
        except Exception as e:
            rospy.logerr(f"Error generating behavior tree: {e}")
            return SetBoolResponse(success=False, message=f"Error: {str(e)}")
    
    def status_service_callback(self, req):
        """Service callback for checking LLM status"""
        if not self.current_provider:
            return TriggerResponse(success=False, message="No LLM provider initialized")
        
        try:
            is_available = self.current_provider.is_available()
            status = "Available" if is_available else "Unavailable"
            return TriggerResponse(success=is_available, message=f"LLM Status: {status}")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Error checking status: {str(e)}")


def main():
    """
    Main function to initialize the ROS node
    """
    # Initialize the ROS node
    rospy.init_node('llm_node', anonymous=True)
    
    # Create LLM node instance
    llm_node = LLMNode()
    
    # Initialize provider (default to gemini)
    provider_type = rospy.get_param('~provider', 'gemini')
    if llm_node.initialize_provider(provider_type):
        rospy.loginfo("LLM Node has started successfully.")
    else:
        rospy.logerr("Failed to initialize LLM provider")
        return
    
    # Keep the node running until it is shut down
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("LLM Node shutting down...")
        

if __name__ == '__main__':
    main()
