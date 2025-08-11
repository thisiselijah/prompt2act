#!/root/.pyenv/versions/3.9.19/bin/python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import json
from setup import OpenAIProvider, GeminiProvider


class LLMNode:
    """Main LLM Node class"""
    
    def __init__(self):
        self.providers = {
            'openai': OpenAIProvider(),
            'gemini': GeminiProvider()
        }
        self.current_provider = None
        self.config = {}
        
        # ROS publishers and subscribers
        self.response_pub = rospy.Publisher('/llm_response', String, queue_size=10)
        self.prompt_sub = rospy.Subscriber('/llm_prompt', String, self.prompt_callback)
        
        # ROS services
        self.query_service = rospy.Service('/llm_query', Trigger, self.query_service_callback)
        self.status_service = rospy.Service('/llm_status', Trigger, self.status_service_callback)
    
    def load_config(self):
        """Load configuration from ROS parameters"""
        try:
            # Get provider type
            provider_type = rospy.get_param('~provider', 'openai')
            
            # Get provider-specific config
            if provider_type == 'openai':
                self.config = {
                    'api_key': rospy.get_param('~openai_api_key', ''),
                    'model': rospy.get_param('~openai_model', 'gpt-3.5-turbo'),
                    'max_tokens': rospy.get_param('~max_tokens', 1000),
                    'temperature': rospy.get_param('~temperature', 0.7)
                }
            elif provider_type == 'gemini':
                self.config = {
                    'api_key': rospy.get_param('~gemini_api_key', ''), 
                    'model': rospy.get_param('~gemini_model', 'gemini-2.0-flash'),
                    'temperature': rospy.get_param('~temperature', 0.7),
                    'max_output_tokens': rospy.get_param('~max_output_tokens', 1000),
                    'top_p': rospy.get_param('~top_p', 0.95),
                    'top_k': rospy.get_param('~top_k', 40),
                    'stop_sequences': rospy.get_param('~stop_sequences', [])
                }
            
            return provider_type
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            return None
    
    def initialize_provider(self, provider_type: str) -> bool:
        """Initialize the specified LLM provider"""
        if provider_type not in self.providers:
            rospy.logerr(f"Unknown provider: {provider_type}")
            return False
        
        provider = self.providers[provider_type]
        if provider.initialize(self.config):
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
    
    # Load configuration and initialize provider
    provider_type = llm_node.load_config()
    if provider_type and llm_node.initialize_provider(provider_type):
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
