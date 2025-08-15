#!/root/.pyenv/versions/3.9.19/bin/python3.9

import sys
print(sys.executable) 

# Ensure UTF-8 encoding for cross-platform compatibility
import locale
import os

# Set UTF-8 encoding for text processing
if sys.platform == "win32":
    # Windows-specific encoding setup
    os.environ['PYTHONIOENCODING'] = 'utf-8'
elif sys.platform == "darwin":
    # macOS-specific setup
    locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')
else:
    # Linux and other Unix-like systems
    try:
        locale.setlocale(locale.LC_ALL, 'C.UTF-8')
    except locale.Error:
        try:
            locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')
        except locale.Error:
            pass  # Use system default

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest
from behavior_tree.srv import AssembleBehaviorTree, AssembleBehaviorTreeRequest
from llm.srv import LLMQuery, LLMQueryResponse, LLMJsonQuery, LLMJsonQueryResponse, GenerateBehaviorTree, GenerateBehaviorTreeResponse, LLMStatus, LLMStatusResponse
import json
import os
import re
import unicodedata

# 加入 scripts 資料夾路徑，確保 import 從原本腳本所在目錄載入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from setup import OpenAIProvider, GeminiProvider



# Global constants for behavior tree generation
BEHAVIOR_TREE_PROMPT_TEMPLATE = """
Generate a behavior tree JSON configuration for a robot manipulation task.

Task description: {task_description}

IMPORTANT: Return ONLY valid JSON object with proper formatting, no explanations or additional text.
Use double quotes for all strings, proper escaping, and ensure cross-platform compatibility.
CRITICAL: Use only standard ASCII characters and double quotes (") - no smart quotes, em dashes, or Unicode characters.

Requirements:
1. Root node must be 'sequence' or 'selector'
2. Include appropriate behavior types: 'detect_objects', 'pick_up', 'place_down', 'open_gripper', 'close_gripper', 'move_to_home'
3. Use proper nesting for complex behaviors
4. Each node must have 'type' and 'name' fields
5. Composite nodes (sequence/selector) must have 'children' array
6. Leaf nodes (behavior actions) should NOT have 'children' field
7. For 'place_down' behavior, you can optionally include 'place_x', 'place_y', 'place_z' parameters
8. Use only standard double quotes (") for strings, no smart quotes like " or "
9. No trailing commas
10. All string values must be properly escaped
11. Use only ASCII characters - no Unicode characters that could cause encoding issues

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
            
            # Handle auto_assemble parameter with proper error handling
            try:
                auto_assemble = req.auto_assemble if hasattr(req, 'auto_assemble') else True  # Default to True for backward compatibility
            except AttributeError:
                rospy.logwarn("auto_assemble parameter not found in request, defaulting to True")
                auto_assemble = True
            
            rospy.loginfo(f"Processing behavior tree generation request: task='{task_description}', auto_assemble={auto_assemble}")
            
            # Format the prompt with the task description
            behavior_tree_prompt = BEHAVIOR_TREE_PROMPT_TEMPLATE.format(task_description=task_description)
            
            # Generate JSON response (schema validation not supported in current SDK version)
            try:
                rospy.loginfo("Generating behavior tree JSON...")
                response = self.current_provider.generate_json_response(behavior_tree_prompt)
                rospy.loginfo("JSON generation successful")
                
                # Early validation: Check if response is an error message about encoding
                if response and len(response) < 500:  # Error messages are typically short
                    encoding_error_patterns = [
                        r"ascii.*codec.*can't.*encode",
                        r"Error:.*codec.*can't.*encode", 
                        r"Google Gemini.*API.*error.*codec",
                        r"ordinal not in range.*128"
                    ]
                    
                    for pattern in encoding_error_patterns:
                        if re.search(pattern, response, re.IGNORECASE):
                            rospy.logerr(f"Provider returned encoding error instead of JSON: {response}")
                            return GenerateBehaviorTreeResponse(
                                success=False,
                                behavior_tree_json="",
                                error_message=f"Provider encoding error: {response}"
                            )
                
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
                
                # Call the behavior tree assembly service if requested
                if auto_assemble:
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
                else:
                    # Just return the generated JSON without assembling
                    rospy.loginfo("Behavior tree JSON generated (auto-assembly disabled)")
                    return GenerateBehaviorTreeResponse(
                        success=True, 
                        behavior_tree_json=cleaned_response, 
                        error_message=""
                    )
                
            except json.JSONDecodeError as e:
                rospy.logerr(f"Generated response is not valid JSON: {e}")
                rospy.logerr(f"Original response length: {len(response)} chars")
                rospy.logerr(f"Cleaned response length: {len(cleaned_response)} chars")

                # Log Unicode issues specifically
                try:
                    unicode_chars = [char for char in cleaned_response if ord(char) > 127]
                except Exception:
                    unicode_chars = []

                if unicode_chars:
                    rospy.logwarn(f"Unicode characters detected: {set(unicode_chars)}")

                # Show first 200 chars of each for debugging
                rospy.logerr(f"Original response (first 200 chars): {response[:200]}")
                rospy.logerr(f"Cleaned response (first 200 chars): {cleaned_response[:200]}")

                # Try one more time with aggressive cleaning
                try:
                    rospy.loginfo("Attempting aggressive JSON cleanup due to parsing failure")
                    fallback_cleaned = self._aggressive_json_cleanup(cleaned_response)
                    rospy.loginfo(f"Aggressive cleanup result length: {len(fallback_cleaned)} chars")
                    rospy.loginfo(f"Aggressive cleanup result (first 200 chars): {fallback_cleaned[:200]}")

                    # Enhanced error detection for encoding/exception messages
                    error_indicators = [
                        r'ascii.*codec.*can\'t.*encode',
                        r'codec can\'t encode character', 
                        r'ordinal not in range',
                        r'\\bu201[cd]\\b',  # Unicode escape sequences for smart quotes
                        r'UnicodeEncodeError',
                        r'UnicodeDecodeError', 
                        r'Traceback.*most recent call',
                        r'"Error":\s*".*codec',
                        r'position.*ordinal not in range'
                    ]
                    
                    is_error_message = False
                    for pattern in error_indicators:
                        if re.search(pattern, fallback_cleaned[:500], re.IGNORECASE):
                            is_error_message = True
                            rospy.logerr(f"Detected error message pattern: {pattern}")
                            break
                    
                    if is_error_message:
                        rospy.logerr(f"Aggressive cleanup produced an error-like string instead of JSON: {fallback_cleaned[:200]}")
                        return GenerateBehaviorTreeResponse(
                            success=False,
                            behavior_tree_json="",
                            error_message="Failed to recover valid JSON from LLM response (encoding/error string returned)."
                        )

                    # Try to extract balanced JSON from the cleaned result to avoid "Extra data" errors
                    candidate_json = None
                    first_brace = fallback_cleaned.find('{')
                    if first_brace != -1:
                        candidate_json = self._extract_balanced_braces(fallback_cleaned, first_brace)

                    if not candidate_json:
                        rospy.logerr("No balanced JSON object found inside aggressive-cleaned result")
                        rospy.logerr(f"Fallback cleaned preview (200 chars): {fallback_cleaned[:200]}")
                        return GenerateBehaviorTreeResponse(
                            success=False,
                            behavior_tree_json="",
                            error_message="Aggressive cleanup did not produce a parsable JSON object."
                        )

                    # Validate structure before attempting to parse
                    if not self._is_valid_json_structure(candidate_json):
                        rospy.logerr("Extracted candidate JSON does not look valid; aborting parse")
                        rospy.logerr(f"Candidate JSON preview: {candidate_json[:200]}")
                        return GenerateBehaviorTreeResponse(
                            success=False,
                            behavior_tree_json="",
                            error_message="Extracted JSON candidate did not pass structure validation."
                        )

                    # Parse the extracted candidate JSON
                    parsed_json = json.loads(candidate_json)
                    rospy.loginfo("Successfully parsed JSON after aggressive cleanup")

                    # Use the validated candidate_json for assembly
                    final_json_str = candidate_json

                    if auto_assemble:
                        assembly_success, assembly_message = self._call_behavior_tree_assembly(final_json_str)
                        return GenerateBehaviorTreeResponse(
                            success=True if assembly_success else False,
                            behavior_tree_json=final_json_str if assembly_success else "",
                            error_message="" if assembly_success else assembly_message
                        )
                    else:
                        return GenerateBehaviorTreeResponse(
                            success=True,
                            behavior_tree_json=final_json_str,
                            error_message="JSON parsed after aggressive cleanup"
                        )

                except json.JSONDecodeError as fallback_error:
                    rospy.logerr(f"Fallback JSON parsing also failed: {fallback_error}")
                    rospy.logerr(f"Fallback result (first 200 chars): {fallback_cleaned[:200] if 'fallback_cleaned' in locals() else 'N/A'}")
                    return GenerateBehaviorTreeResponse(
                        success=False,
                        behavior_tree_json="",
                        error_message=f"Invalid JSON generated: {str(e)}. Fallback also failed: {str(fallback_error)}"
                    )
                except Exception as cleanup_error:
                    rospy.logerr(f"Aggressive cleanup threw exception: {cleanup_error}")
                    return GenerateBehaviorTreeResponse(
                        success=False,
                        behavior_tree_json="",
                        error_message=f"Invalid JSON generated: {str(e)}. Cleanup failed: {str(cleanup_error)}"
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
        """Extract JSON from response that might contain additional text - cross-platform compatible"""
        try:
            import re
            import unicodedata
            
            # Ensure we have a string and handle encoding issues early
            if not isinstance(response, str):
                response = str(response)
            
            # Handle potential encoding issues from Windows environments
            try:
                # Try to encode/decode to catch encoding issues early
                response = response.encode('utf-8', errors='replace').decode('utf-8')
            except Exception as e:
                rospy.logwarn(f"Encoding normalization failed: {e}")
                # Fallback to ASCII-safe conversion
                response = response.encode('ascii', errors='ignore').decode('ascii')
            
            # Normalize unicode characters for cross-platform compatibility
            try:
                response = unicodedata.normalize('NFKC', response)
            except Exception as e:
                rospy.logwarn(f"Unicode normalization failed: {e}")
            
            # Handle different line endings (Windows CRLF, Unix LF, Mac CR)
            response = response.replace('\r\n', '\n').replace('\r', '\n')
            
            # Remove zero-width characters that can cause parsing issues
            response = re.sub(r'[\u200b-\u200f\ufeff]', '', response)
            
            # Replace common problematic Unicode characters early
            unicode_fixes = {
                '\u201c': '"', '\u201d': '"',  # Smart quotes
                '\u2018': "'", '\u2019': "'",  # Smart single quotes
                '\u2013': '-', '\u2014': '-',  # Dashes
            }
            for bad_char, good_char in unicode_fixes.items():
                response = response.replace(bad_char, good_char)
            
            # First, try to find JSON within code blocks with more flexible patterns
            # Handle various markdown code block formats
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
                except Exception as e:
                    rospy.logwarn(f"Code block pattern matching failed: {e}")
                    continue
            
            # Look for JSON objects in the text with improved brace matching
            # Handle nested objects and arrays properly
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
                    
            except Exception as e:
                rospy.logwarn(f"Brace matching failed: {e}")
            
            # Last resort: try to clean and return the original response
            cleaned_response = self._clean_response_text(response)
            return cleaned_response
                
        except Exception as e:
            rospy.logwarn(f"JSON extraction failed: {e}")
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
        import re
        has_quotes = bool(re.search(r'"[^"]*"', text))
        has_colons = ':' in text
        
        return has_quotes and has_colons
    
    def _clean_response_text(self, response: str) -> str:
        """Clean response text for cross-platform compatibility"""
        try:
            import re
            
            # Remove common prefixes/suffixes that LLMs might add
            response = re.sub(r'^.*?(?=\{)', '', response, flags=re.DOTALL)  # Remove text before first {
            response = re.sub(r'\}.*?$', '}', response, flags=re.DOTALL)     # Remove text after last }
            
            # Normalize whitespace
            response = re.sub(r'\s+', ' ', response)
            
            # Remove control characters except newlines and tabs
            response = re.sub(r'[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]', '', response)
            
            return response.strip()
            
        except Exception:
            return response.strip() if response else ""
    
    def _sanitize_json_for_parsing(self, json_str: str) -> str:
        """Sanitize JSON string for reliable parsing across platforms"""
        try:
            import re
            import unicodedata
            
            # Ensure we're working with a proper string
            if not isinstance(json_str, str):
                json_str = str(json_str)
            
            # Normalize Unicode characters to prevent encoding issues
            json_str = unicodedata.normalize('NFKC', json_str)
            
            # Remove byte order marks (BOM) that can appear on Windows
            json_str = json_str.lstrip('\ufeff\ufffe\ufbff')
            
            # Replace smart quotes and other problematic Unicode characters
            unicode_replacements = {
                '\u201c': '"',  # Left double quotation mark
                '\u201d': '"',  # Right double quotation mark  
                '\u2018': "'",  # Left single quotation mark
                '\u2019': "'",  # Right single quotation mark
                '\u2013': '-',  # En dash
                '\u2014': '-',  # Em dash
                '\u2026': '...',  # Horizontal ellipsis
                '\u00a0': ' ',  # Non-breaking space
                '\u2003': ' ',  # Em space
                '\u2002': ' ',  # En space
                '\u2009': ' ',  # Thin space
                '\u200b': '',   # Zero-width space
                '\u200c': '',   # Zero-width non-joiner
                '\u200d': '',   # Zero-width joiner
                '\ufeff': '',   # Zero-width no-break space (BOM)
            }
            
            for unicode_char, replacement in unicode_replacements.items():
                json_str = json_str.replace(unicode_char, replacement)
            
            # Remove any remaining non-ASCII characters that could cause issues
            # but preserve basic JSON structure characters
            json_str = ''.join(char if ord(char) < 128 or char in '{}[],:' else ' ' for char in json_str)
            
            # Fix common JSON formatting issues
            # Replace single quotes with double quotes (but not inside strings)
            json_str = re.sub(r"(?<!\\)'([^']*?)(?<!\\)'", r'"\1"', json_str)
            
            # Fix trailing commas before closing braces/brackets
            json_str = re.sub(r',(\s*[}\]])', r'\1', json_str)
            
            # Ensure proper spacing around colons and commas
            json_str = re.sub(r':\s*', ': ', json_str)
            json_str = re.sub(r',\s*', ', ', json_str)
            
            # Remove any null bytes that might cause issues
            json_str = json_str.replace('\x00', '')
            
            # Clean up excessive whitespace
            json_str = re.sub(r'\s+', ' ', json_str)
            
            return json_str.strip()
            
        except Exception as e:
            rospy.logwarn(f"JSON sanitization failed: {e}")
            # Fallback: aggressive ASCII-only cleanup
            try:
                # Convert to ASCII, ignoring errors
                ascii_str = json_str.encode('ascii', 'ignore').decode('ascii')
                return ascii_str.strip() if ascii_str else "{}"
            except Exception:
                return json_str.strip() if json_str else "{}"
    
    def _aggressive_json_cleanup(self, json_str: str) -> str:
        """Aggressive JSON cleanup as last resort for cross-platform compatibility"""
        try:
            import re
            import unicodedata
            
            rospy.loginfo("Applying aggressive JSON cleanup for Unicode issues")
            
            # Ensure we're working with a proper string
            if not isinstance(json_str, str):
                try:
                    json_str = str(json_str)
                except Exception as convert_error:
                    rospy.logerr(f"Failed to convert input to string: {convert_error}")
                    return '{"type": "sequence", "name": "ConversionError", "children": []}'
            
            # Early detection of error messages in the input
            error_patterns = [
                r"ascii.*codec.*can't.*encode",
                r"codec can't encode character",
                r"ordinal not in range",
                r"\\u201[cd]",  # Smart quotes unicode codes
                r"UnicodeEncodeError",
                r"UnicodeDecodeError",
                r"Traceback.*most recent call",
                r"Google Gemini.*API.*error",  # Specific provider errors
                r"OpenAI.*API.*error",  # OpenAI provider errors
                r"Error:.*codec.*can't.*encode",  # Windows VM specific pattern
                r"position \d+:.*ordinal not in range",  # Encoding position errors
                r"API.*error.*codec.*can't.*encode"  # API-specific encoding errors
            ]
            
            for pattern in error_patterns:
                if re.search(pattern, json_str[:500], re.IGNORECASE):
                    rospy.logerr(f"Input appears to be an error message, not JSON: {json_str[:200]}")
                    return '{"type": "sequence", "name": "ErrorMessageDetected", "children": []}'
            
            # Aggressive ASCII conversion first to prevent encoding errors
            try:
                # Step 1: Convert problematic Unicode characters before any other processing
                unicode_map = {
                    # Smart quotes (most problematic)
                    '\u201c': '"', '\u201d': '"',  # Left/right double quotes
                    '\u2018': "'", '\u2019': "'",  # Left/right single quotes
                    '\u00ab': '"', '\u00bb': '"',  # Guillemets
                    '\u2039': "'", '\u203a': "'",  # Single guillemets
                    
                    # Dashes and hyphens
                    '\u2013': '-', '\u2014': '-',  # En/Em dash
                    '\u2212': '-',                 # Minus sign
                    '\u00ad': '-',                 # Soft hyphen
                    
                    # Spaces and control characters
                    '\u00a0': ' ',  # Non-breaking space
                    '\u2000': ' ', '\u2001': ' ', '\u2002': ' ', '\u2003': ' ',
                    '\u2004': ' ', '\u2005': ' ', '\u2006': ' ', '\u2007': ' ',
                    '\u2008': ' ', '\u2009': ' ', '\u200a': ' ', '\u200b': '',
                    '\u202f': ' ', '\u205f': ' ', '\u3000': ' ',
                    '\ufeff': '',     # BOM
                    '\u200c': '',     # Zero-width non-joiner
                    '\u200d': '',     # Zero-width joiner
                    
                    # Other problematic characters
                    '\u2026': '...',  # Ellipsis
                    '\u2022': '*',    # Bullet
                    '\u00b7': '*',    # Middle dot
                }
                
                # Apply character replacements
                for unicode_char, replacement in unicode_map.items():
                    json_str = json_str.replace(unicode_char, replacement)
                
                # Now try Unicode normalization safely
                try:
                    json_str = unicodedata.normalize('NFKC', json_str)
                except Exception as norm_error:
                    rospy.logwarn(f"Unicode normalization failed, continuing with character replacement: {norm_error}")
                
            except Exception as unicode_error:
                rospy.logwarn(f"Unicode processing failed: {unicode_error}")
                # Fallback: aggressive ASCII-only filtering
                try:
                    json_str = json_str.encode('ascii', 'ignore').decode('ascii')
                except Exception as ascii_error:
                    rospy.logerr(f"ASCII conversion also failed: {ascii_error}")
                    return '{"type": "sequence", "name": "EncodingFailure", "children": []}'
            
            # Remove all non-printable and problematic characters
            # Keep: letters, digits, basic punctuation, and JSON structure characters
            safe_chars = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789{}[]":,.-_ \t\n\r')
            json_str = ''.join(char if char in safe_chars else ' ' for char in json_str)
            
            # Fix common JSON formatting issues
            try:
                # Replace any remaining problematic quotes
                json_str = re.sub(r'[""''`]', '"', json_str)
                
                # Ensure all property names are quoted
                json_str = re.sub(r'(\w+)(\s*:\s*)', r'"\1"\2', json_str)
                
                # Fix boolean values to lowercase
                json_str = re.sub(r'\bTrue\b', 'true', json_str, flags=re.IGNORECASE)
                json_str = re.sub(r'\bFalse\b', 'false', json_str, flags=re.IGNORECASE)
                json_str = re.sub(r'\bNull\b', 'null', json_str, flags=re.IGNORECASE)
                json_str = re.sub(r'\bNone\b', 'null', json_str, flags=re.IGNORECASE)
                
                # Remove trailing commas
                json_str = re.sub(r',(\s*[}\]])', r'\1', json_str)
                
                # Remove duplicate quotes
                json_str = re.sub(r'"{2,}', '"', json_str)
                
                # Fix malformed JSON patterns
                json_str = re.sub(r'"\s*:\s*"([^"]*?)"\s*([,}])', r'": "\1"\2', json_str)
                json_str = re.sub(r':\s*([^",}\]]+)([,}])', r': "\1"\2', json_str)
                
                # Normalize spacing
                json_str = re.sub(r'\s+', ' ', json_str)
                
            except Exception as regex_error:
                rospy.logwarn(f"Regex processing failed: {regex_error}")
            
            # Final structure validation and repair
            json_str = json_str.strip()
            
            # Ensure proper JSON structure
            if not json_str.startswith('{'):
                start_idx = json_str.find('{')
                if start_idx != -1:
                    json_str = json_str[start_idx:]
                else:
                    # No valid JSON structure found
                    rospy.logwarn("No JSON structure found after cleanup")
                    return '{"type": "sequence", "name": "NoJSONFound", "children": []}'
            
            if not json_str.endswith('}'):
                end_idx = json_str.rfind('}')
                if end_idx != -1:
                    json_str = json_str[:end_idx + 1]
                else:
                    # Add closing brace if missing
                    json_str += '}'
            
            # Final validation: check if result looks like an error message
            # Be more specific about what constitutes an invalid result
            if len(json_str) < 10:
                rospy.logwarn(f"Cleanup result too short: {json_str}")
                return '{"type": "sequence", "name": "CleanupInvalid", "children": []}'
            
            # Check for specific error indicators, not just the word "error" or "codec"
            error_indicators_in_result = [
                r'codec.*can\'t.*encode',
                r'UnicodeEncodeError',
                r'UnicodeDecodeError', 
                r'Traceback.*most recent call',
                r'ordinal not in range',
                r'ascii.*codec.*error'
            ]
            
            has_error_indicators = any(re.search(pattern, json_str, re.IGNORECASE) for pattern in error_indicators_in_result)
            
            if has_error_indicators:
                rospy.logwarn(f"Cleanup result contains error indicators: {json_str}")
                return '{"type": "sequence", "name": "CleanupInvalid", "children": []}'
            
            rospy.loginfo(f"Aggressive cleanup completed. Result length: {len(json_str)}")
            return json_str
            
        except Exception as e:
            rospy.logerr(f"Aggressive cleanup failed with exception: {e}")
            # Ultimate fallback: return a valid minimal JSON structure
            return '{"type": "sequence", "name": "FallbackTask", "children": []}'

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
