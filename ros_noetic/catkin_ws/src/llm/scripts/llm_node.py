#!/root/.pyenv/versions/3.9.19/bin/python3.9

import sys
print(sys.executable) 

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest
from behavior_tree.srv import AssembleBehaviorTree, AssembleBehaviorTreeRequest
from llm.srv import LLMQuery, LLMQueryResponse, LLMJsonQuery, LLMJsonQueryResponse, GenerateBehaviorTree, GenerateBehaviorTreeResponse, LLMStatus, LLMStatusResponse
import json
import os
import re

# 加入 scripts 資料夾路徑，確保 import 從原本腳本所在目錄載入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from setup import OpenAIProvider, GeminiProvider



# Global constants for behavior tree generation
BEHAVIOR_TREE_PROMPT_TEMPLATE = """
Generate a behavior tree JSON configuration for a robot manipulation task.

Task description: {task_description}

IMPORTANT: Return ONLY valid JSON object with proper formatting, no explanations or additional text.
Use double quotes for all strings and proper escaping.

Requirements:
1. Root node must be 'sequence' or 'selector'
2. Include appropriate behavior types: 'detect_objects', 'pick_up', 'place_down', 'open_gripper', 'close_gripper', 'move_to_home'
3. Use proper nesting for complex behaviors
4. Each node must have 'type' and 'name' fields
5. Composite nodes (sequence/selector) must have 'children' array
6. Leaf nodes (behavior actions) should NOT have 'children' field
7. For 'place_down' behavior, you can optionally include 'place_x', 'place_y', 'place_z' parameters
8. Use only double quotes (") for strings, no single quotes
9. No trailing commas
10. All string values must be properly escaped

Available behavior types:
- detect_objects: For detecting objects in the environment using YOLO vision
- pick_up: For picking up objects using robot arm and gripper
- place_down: For placing objects at specified coordinates (supports place_x, place_y, place_z parameters)
- open_gripper: For opening the robot gripper
- close_gripper: For closing the robot gripper
- move_to_home: For moving robot to home/rest position
- sequence: Execute children in order (all must succeed)
- selector: Try children until one succeeds

Return ONLY this JSON format (replace content as needed):
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
      "name": "PlaceObject",
      "place_x": 0.15,
      "place_y": -0.15,
      "place_z": 0.18
    }},
    {{
      "type": "move_to_home",
      "name": "ReturnHome"
    }}
  ]
}}

Task: {task_description}
"""

BEHAVIOR_TREE_SCHEMA = {
    "type": "object",
    "properties": {
        "type": {
            "type": "string",
            "enum": ["sequence", "selector"]
        },
        "name": {
            "type": "string"
        },
        "children": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "type": {
                        "type": "string",
                        "enum": ["detect_objects", "pick_up", "place_down", "open_gripper", "close_gripper", "move_to_home", "sequence", "selector"]
                    },
                    "name": {
                        "type": "string"
                    },
                    "place_x": {
                        "type": "number"
                    },
                    "place_y": {
                        "type": "number"
                    },
                    "place_z": {
                        "type": "number"
                    }
                },
                "required": ["type", "name"],
                "additionalProperties": False
            }
        }
    },
    "required": ["type", "name", "children"],
    "additionalProperties": False
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
        
        # ROS services with custom service messages
        self.query_service = rospy.Service('/llm_query', LLMQuery, self.query_service_callback)
        self.json_query_service = rospy.Service('/llm_json_query', LLMJsonQuery, self.json_query_service_callback)
        self.behavior_tree_service = rospy.Service('/generate_behavior_tree', GenerateBehaviorTree, self.generate_behavior_tree_callback)
        self.status_service = rospy.Service('/llm_status', LLMStatus, self.status_service_callback)
        
        # Service client for behavior tree assembly
        self.behavior_tree_assembly_client = None
        self._initialize_behavior_tree_client()
    
    def _initialize_behavior_tree_client(self):
        """Initialize the behavior tree assembly service client"""
        try:
            rospy.loginfo("Waiting for behavior tree assembly service...")
            rospy.wait_for_service('/assemble_behavior_tree', timeout=5.0)
            self.behavior_tree_assembly_client = rospy.ServiceProxy('/assemble_behavior_tree', AssembleBehaviorTree)
            rospy.loginfo("Behavior tree assembly service client initialized")
        except rospy.ROSException as e:
            rospy.logwarn(f"Behavior tree assembly service not available: {e}")
            self.behavior_tree_assembly_client = None
    
    def _call_behavior_tree_assembly(self, json_string):
        """
        Call the behavior tree assembly service with the generated JSON
        Args:
            json_string (str): JSON configuration for the behavior tree
        Returns:
            tuple: (success, message)
        """
        if not self.behavior_tree_assembly_client:
            # Try to reinitialize the client
            self._initialize_behavior_tree_client()
            if not self.behavior_tree_assembly_client:
                return False, "Behavior tree assembly service not available"
        
        try:
            request = AssembleBehaviorTreeRequest()
            request.behavior_tree_json = json_string
            response = self.behavior_tree_assembly_client(request)
            return response.success, response.message
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call behavior tree assembly service: {e}")
            return False, f"Service call failed: {str(e)}"
    
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
            return LLMQueryResponse(
                success=False, 
                response="", 
                error_message="No LLM provider initialized"
            )
        
        try:
            prompt = req.prompt if req.prompt else "Hello, how are you?"
            response = self.current_provider.generate_response(prompt)
            
            return LLMQueryResponse(
                success=True, 
                response=response, 
                error_message=""
            )
        except Exception as e:
            return LLMQueryResponse(
                success=False, 
                response="", 
                error_message=str(e)
            )
    
    def json_query_service_callback(self, req):
        """Service callback for JSON LLM queries"""
        if not self.current_provider:
            return LLMJsonQueryResponse(
                success=False, 
                json_response="", 
                error_message="No LLM provider initialized"
            )
        
        try:
            prompt = req.prompt if req.prompt else "Generate a simple JSON object"
            schema = json.loads(req.schema) if req.schema else None
            
            response = self.current_provider.generate_json_response(prompt, schema)
            
            return LLMJsonQueryResponse(
                success=True, 
                json_response=response, 
                error_message=""
            )
        except Exception as e:
            return LLMJsonQueryResponse(
                success=False, 
                json_response="", 
                error_message=str(e)
            )
    
    def generate_behavior_tree_callback(self, req):
        """Service callback for generating behavior tree JSON"""
        if not self.current_provider:
            return GenerateBehaviorTreeResponse(
                success=False, 
                behavior_tree_json="", 
                error_message="No LLM provider initialized"
            )
        
        try:
            # Get task description from request or use default
            task_description = req.task_description if req.task_description else DEFAULT_TASK_DESCRIPTION
            
            rospy.loginfo(f"Processing behavior tree generation request: task='{task_description}' (auto-assembly enabled by default)")
            
            # Format the prompt with the task description
            behavior_tree_prompt = BEHAVIOR_TREE_PROMPT_TEMPLATE.format(task_description=task_description)
            
            # Generate JSON response (schema validation not supported in current SDK version)
            try:
                rospy.loginfo("Generating behavior tree JSON...")
                response = self.current_provider.generate_json_response(behavior_tree_prompt)
                rospy.loginfo("JSON generation successful")
                
            except Exception as generation_error:
                rospy.logerr(f"JSON generation failed: {generation_error}")
                return GenerateBehaviorTreeResponse(
                    success=False, 
                    behavior_tree_json="", 
                    error_message=f"LLM generation failed: {str(generation_error)}"
                )
            
            # Check if response is empty or None
            if not response or response.strip() == "":
                rospy.logerr("Empty response received from LLM provider")
                return GenerateBehaviorTreeResponse(
                    success=False, 
                    behavior_tree_json="", 
                    error_message="Empty response from LLM provider"
                )
            
            # Clean the response to extract JSON if it has extra text
            cleaned_response = self._extract_json_from_response(response)
            
            # Validate that it's proper JSON with cross-platform error handling
            try:
                # Additional cleaning for cross-platform JSON parsing
                cleaned_response = self._sanitize_json_for_parsing(cleaned_response)
                parsed_json = json.loads(cleaned_response)
                rospy.loginfo(f"Generated behavior tree JSON: {cleaned_response}")
                
                # Basic validation of the structure
                if 'type' not in parsed_json or 'name' not in parsed_json:
                    rospy.logwarn("Generated JSON missing required fields (type/name)")
                
                # Always call the behavior tree assembly service automatically
                assembly_success, assembly_message = self._call_behavior_tree_assembly(cleaned_response)
                
                if assembly_success:
                    rospy.loginfo(f"Behavior tree assembled successfully: {assembly_message}")
                    return GenerateBehaviorTreeResponse(
                        success=True, 
                        behavior_tree_json=cleaned_response, 
                        error_message=""
                    )
                else:
                    rospy.logwarn(f"Behavior tree assembly failed: {assembly_message}")
                    # Still return success for JSON generation, but include assembly warning
                    return GenerateBehaviorTreeResponse(
                        success=True, 
                        behavior_tree_json=cleaned_response, 
                        error_message=f"JSON generated but assembly failed: {assembly_message}"
                    )
                
            except json.JSONDecodeError as e:
                rospy.logerr(f"Generated response is not valid JSON: {e}")
                rospy.logerr(f"Original response length: {len(response)} chars")
                rospy.logerr(f"Cleaned response length: {len(cleaned_response)} chars")

                # Show first 200 chars of each for debugging
                rospy.logerr(f"Original response (first 200 chars): {response[:200]}")
                rospy.logerr(f"Cleaned response (first 200 chars): {cleaned_response[:200]}")

                return GenerateBehaviorTreeResponse(
                    success=False,
                    behavior_tree_json="",
                    error_message=f"Invalid JSON generated: {str(e)}"
                )
                
        except Exception as e:
            rospy.logerr(f"Error generating behavior tree: {e}")
            return GenerateBehaviorTreeResponse(
                success=False, 
                behavior_tree_json="", 
                error_message=str(e)
            )
    
    def status_service_callback(self, req):
        """Service callback for checking LLM status"""
        if not self.current_provider:
            return LLMStatusResponse(
                success=False, 
                provider_name="None", 
                is_available=False, 
                status_message="No LLM provider initialized"
            )
        
        try:
            is_available = self.current_provider.is_available()
            provider_name = type(self.current_provider).__name__.replace('Provider', '')
            status_msg = "Available" if is_available else "Unavailable"
            
            return LLMStatusResponse(
                success=True, 
                provider_name=provider_name, 
                is_available=is_available, 
                status_message=status_msg
            )
        except Exception as e:
            return LLMStatusResponse(
                success=False, 
                provider_name="Unknown", 
                is_available=False, 
                status_message=f"Error checking status: {str(e)}"
            )
    
    def _extract_json_from_response(self, response: str) -> str:
        """Extract JSON from response that might contain additional text"""
        try:
            # Ensure we have a string
            if not isinstance(response, str):
                response = str(response)
            
            # First, try to find JSON within code blocks
            code_block_patterns = [
                r'```(?:json|JSON)?\s*(\{.*?\})\s*```',  # Standard markdown
                r'`{3,}\s*(?:json|JSON)?\s*(\{.*?\})\s*`{3,}',  # Flexible backtick count
                r'~~~(?:json|JSON)?\s*(\{.*?\})\s*~~~',  # Alternative code block style
                r'<code[^>]*>\s*(\{.*?\})\s*</code>',  # HTML code tags
                r'(?:json|JSON):\s*(\{.*?\})',  # Label prefix
            ]
            
            for pattern in code_block_patterns:
                try:
                    json_blocks = re.findall(pattern, response, re.DOTALL | re.IGNORECASE)
                    if json_blocks:
                        candidate = json_blocks[0].strip()
                        if self._is_valid_json_structure(candidate):
                            return candidate
                except Exception:
                    continue
            
            # Look for JSON objects in the text with brace matching
            json_candidates = []
            
            # Find all potential JSON start positions
            try:
                for match in re.finditer(r'\{', response):
                    start_idx = match.start()
                    json_str = self._extract_balanced_braces(response, start_idx)
                    if json_str and self._is_valid_json_structure(json_str):
                        json_candidates.append(json_str)
                
                # Return the longest valid JSON (likely the most complete)
                if json_candidates:
                    return max(json_candidates, key=len)
                    
            except Exception:
                pass
            
            # Last resort: try to clean and return the original response
            cleaned_response = self._clean_response_text(response)
            return cleaned_response
                
        except Exception:
            # If extraction fails, return cleaned original response
            return self._clean_response_text(response)
    
    def _extract_balanced_braces(self, text: str, start_idx: int) -> str:
        """Extract JSON with proper brace balancing, handling strings and escape sequences"""
        if start_idx >= len(text) or text[start_idx] != '{':
            return ""
        
        brace_count = 0
        in_string = False
        escape_next = False
        end_idx = start_idx
        
        for i in range(start_idx, len(text)):
            char = text[i]
            
            if escape_next:
                escape_next = False
                continue
            
            if char == '\\' and in_string:
                escape_next = True
                continue
            
            if char == '"' and not escape_next:
                in_string = not in_string
                continue
            
            if not in_string:
                if char == '{':
                    brace_count += 1
                elif char == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        end_idx = i
                        break
        
        if brace_count == 0:
            return text[start_idx:end_idx + 1].strip()
        else:
            return ""
    
    def _is_valid_json_structure(self, text: str) -> bool:
        """Check if text has basic JSON structure without full parsing"""
        if not text or len(text) < 2:
            return False
        
        text = text.strip()
        
        # Must start with { and end with }
        if not (text.startswith('{') and text.endswith('}')):
            return False
        
        # Should contain basic JSON patterns
        has_quotes = bool(re.search(r'"[^"]*"', text))
        has_colons = ':' in text
        
        return has_quotes and has_colons
    
    def _clean_response_text(self, response: str) -> str:
        """Clean response text while preserving JSON formatting"""
        try:
            # Remove common prefixes/suffixes that LLMs might add
            response = re.sub(r'^.*?(?=\{)', '', response, flags=re.DOTALL)  # Remove text before first {
            response = re.sub(r'\}.*?$', '}', response, flags=re.DOTALL)     # Remove text after last }
            
            # Only normalize excessive whitespace, but preserve reasonable formatting
            # Remove leading/trailing whitespace from each line, but keep line breaks
            lines = response.split('\n')
            cleaned_lines = [line.strip() for line in lines if line.strip()]
            response = '\n'.join(cleaned_lines)
            
            return response.strip()
            
        except Exception:
            return response.strip() if response else ""
    
    def _sanitize_json_for_parsing(self, json_str: str) -> str:
        """Sanitize JSON string for parsing while preserving formatting"""
        try:
            # Ensure we're working with a proper string
            if not isinstance(json_str, str):
                json_str = str(json_str)
            
            # Fix common JSON formatting issues without destroying formatting
            # Replace single quotes with double quotes (but not inside strings)
            json_str = re.sub(r"(?<!\\)'([^']*?)(?<!\\)'", r'"\1"', json_str)
            
            # Fix trailing commas before closing braces/brackets
            json_str = re.sub(r',(\s*[}\]])', r'\1', json_str)
            
            # Validate and pretty-format if possible
            try:
                import json
                parsed = json.loads(json_str)
                # Re-format with proper indentation
                json_str = json.dumps(parsed, indent=2, ensure_ascii=False)
            except (json.JSONDecodeError, UnicodeDecodeError):
                # If parsing fails, just do minimal cleaning
                # Ensure proper spacing around colons and commas only if not already formatted
                if not re.search(r'\n\s+', json_str):  # Check if already formatted
                    json_str = re.sub(r':\s*', ': ', json_str)
                    json_str = re.sub(r',\s*', ', ', json_str)
            
            return json_str.strip()
            
        except Exception:
            return json_str.strip() if json_str else "{}"
    
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
