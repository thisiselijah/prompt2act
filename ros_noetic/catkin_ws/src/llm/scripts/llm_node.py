#!/root/.pyenv/versions/3.9.19/bin/python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import json
import os
from abc import ABC, abstractmethod
import requests
import openai
from google import genai
from google.genai import types
from typing import Dict, Any, Optional


class LLMProvider(ABC):
    """Abstract base class for LLM providers"""
    
    @abstractmethod
    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize the LLM provider with configuration"""
        pass
    
    @abstractmethod
    def generate_response(self, prompt: str, **kwargs) -> str:
        """Generate response from the LLM"""
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """Check if the LLM service is available"""
        pass


class OpenAIProvider(LLMProvider):
    """OpenAI GPT provider"""
    
    def __init__(self):
        self.client = None
        self.model = "gpt-3.5-turbo"
        self.max_tokens = 1000
        self.temperature = 0.7
        
    def initialize(self, config: Dict[str, Any]) -> bool:
        try:
            api_key = config.get('api_key') or os.getenv('OPENAI_API_KEY')
            if not api_key:
                rospy.logerr("OpenAI API key not found")
                return False
                
            openai.api_key = api_key
            self.client = openai.OpenAI(api_key=api_key)
            self.model = config.get('model', self.model)
            self.max_tokens = config.get('max_tokens', self.max_tokens)
            self.temperature = config.get('temperature', self.temperature)
            
            rospy.loginfo(f"OpenAI provider initialized with model: {self.model}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize OpenAI provider: {e}")
            return False
    
    def generate_response(self, prompt: str, **kwargs) -> str:
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                max_tokens=kwargs.get('max_tokens', self.max_tokens),
                temperature=kwargs.get('temperature', self.temperature)
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            rospy.logerr(f"OpenAI API error: {e}")
            return f"Error: {str(e)}"
    
    def is_available(self) -> bool:
        try:
            # Simple test request
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": "test"}],
                max_tokens=1
            )
            return True
        except:
            return False

class GeminiProvider(LLMProvider):
    """Google Gemini AI provider using new GenAI SDK"""
    
    def __init__(self):
        self.client = None
        self.model_name = "gemini-2.0-flash"  # Latest recommended model
        self.config = None

    def initialize(self, config: Dict[str, Any]) -> bool:
        try:
            api_key = config.get('api_key') or os.getenv('GEMINI_API_KEY') or os.getenv('GOOGLE_API_KEY')
            if not api_key:
                rospy.logerr("Google Gemini API key not found. Set GEMINI_API_KEY or GOOGLE_API_KEY environment variable")
                return False

            # Initialize the client with API key
            self.client = genai.Client(api_key=api_key)

            # Set model configuration
            self.model_name = config.get('model', self.model_name)

            # Create generation config using new SDK
            self.config = types.GenerateContentConfig(
                temperature=config.get('temperature', 0.7),
                max_output_tokens=config.get('max_output_tokens', 1000),
                top_p=config.get('top_p', 0.95),
                top_k=config.get('top_k', 40),
                stop_sequences=config.get('stop_sequences', []),
                safety_settings=[
                    types.SafetySetting(
                        category='HARM_CATEGORY_HARASSMENT',
                        threshold='BLOCK_MEDIUM_AND_ABOVE'
                    ),
                    types.SafetySetting(
                        category='HARM_CATEGORY_HATE_SPEECH',
                        threshold='BLOCK_MEDIUM_AND_ABOVE'
                    ),
                    types.SafetySetting(
                        category='HARM_CATEGORY_SEXUALLY_EXPLICIT',
                        threshold='BLOCK_MEDIUM_AND_ABOVE'
                    ),
                    types.SafetySetting(
                        category='HARM_CATEGORY_DANGEROUS_CONTENT',
                        threshold='BLOCK_MEDIUM_AND_ABOVE'
                    )
                ]
            )

            rospy.loginfo(f"Google Gemini provider initialized with model: {self.model_name}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize Google Gemini provider: {e}")
            return False

    def generate_response(self, prompt: str, **kwargs) -> str:
        try:
            # Create custom config if kwargs are provided
            if kwargs:
                custom_config = types.GenerateContentConfig(
                    temperature=kwargs.get('temperature', self.config.temperature),
                    max_output_tokens=kwargs.get('max_output_tokens', self.config.max_output_tokens),
                    top_p=kwargs.get('top_p', self.config.top_p),
                    top_k=kwargs.get('top_k', self.config.top_k),
                    stop_sequences=kwargs.get('stop_sequences', []),
                    safety_settings=self.config.safety_settings
                )
                response = self.client.models.generate_content(
                    model=self.model_name,
                    contents=prompt,
                    config=custom_config
                )
            else:
                response = self.client.models.generate_content(
                    model=self.model_name,
                    contents=prompt,
                    config=self.config
                )

            # Handle response according to new SDK
            if hasattr(response, 'text') and response.text:
                return response.text.strip()
            elif hasattr(response, 'candidates') and response.candidates:
                candidate = response.candidates[0]
                if hasattr(candidate, 'finish_reason'):
                    reason = candidate.finish_reason
                    if reason == types.FinishReason.SAFETY:
                        return "Response blocked due to safety concerns"
                    elif reason == types.FinishReason.RECITATION:
                        return "Response blocked due to recitation concerns"
                    elif reason == types.FinishReason.OTHER:
                        return "Response blocked for other reasons"
                # Try to get content from candidate
                if hasattr(candidate, 'content') and hasattr(candidate.content, 'parts'):
                    if candidate.content.parts:
                        return candidate.content.parts[0].text.strip()
                return "No text content in response"
            elif hasattr(response, 'prompt_feedback') and response.prompt_feedback:
                if getattr(response.prompt_feedback, 'block_reason', None):
                    return f"Prompt blocked: {response.prompt_feedback.block_reason}"
                return "No response generated"
            else:
                return "No response generated"
        except Exception as e:
            rospy.logerr(f"Google Gemini API error: {e}")
            return f"Error: {str(e)}"

    def is_available(self) -> bool:
        try:
            if not self.client:
                return False
            response = self.client.models.generate_content(
                model=self.model_name,
                contents="Hello",
                config=types.GenerateContentConfig(max_output_tokens=1)
            )
            return hasattr(response, 'text') and response.text is not None
        except Exception as e:
            rospy.logwarn(f"Gemini availability check failed: {e}")
            return False


# class OllamaProvider(LLMProvider):
    """Local Ollama provider"""
    
    def __init__(self):
        self.base_url = "http://localhost:11434"
        self.model = "llama2"
        
    def initialize(self, config: Dict[str, Any]) -> bool:
        try:
            self.base_url = config.get('base_url', self.base_url)
            self.model = config.get('model', self.model)
            
            # Test connection
            response = requests.get(f"{self.base_url}/api/tags", timeout=5)
            if response.status_code == 200:
                rospy.loginfo(f"Ollama provider initialized with model: {self.model}")
                return True
            else:
                rospy.logerr("Failed to connect to Ollama server")
                return False
        except Exception as e:
            rospy.logerr(f"Failed to initialize Ollama provider: {e}")
            return False
    
    def generate_response(self, prompt: str, **kwargs) -> str:
        try:
            data = {
                "model": self.model,
                "prompt": prompt,
                "stream": False
            }
            
            response = requests.post(
                f"{self.base_url}/api/generate",
                json=data,
                timeout=30
            )
            
            if response.status_code == 200:
                return response.json().get('response', '')
            else:
                return f"Error: HTTP {response.status_code}"
        except Exception as e:
            rospy.logerr(f"Ollama API error: {e}")
            return f"Error: {str(e)}"
    
    def is_available(self) -> bool:
        try:
            response = requests.get(f"{self.base_url}/api/tags", timeout=5)
            return response.status_code == 200
        except:
            return False

# class HuggingFaceProvider(LLMProvider):
    """Hugging Face API provider"""
    
    def __init__(self):
        self.api_url = "https://api-inference.huggingface.co/models/"
        self.model = "microsoft/DialoGPT-medium"
        self.headers = {}
        
    def initialize(self, config: Dict[str, Any]) -> bool:
        try:
            api_key = config.get('api_key') or os.getenv('HUGGINGFACE_API_KEY')
            if not api_key:
                rospy.logerr("Hugging Face API key not found")
                return False
                
            self.headers = {"Authorization": f"Bearer {api_key}"}
            self.model = config.get('model', self.model)
            
            rospy.loginfo(f"Hugging Face provider initialized with model: {self.model}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize Hugging Face provider: {e}")
            return False
    
    def generate_response(self, prompt: str, **kwargs) -> str:
        try:
            data = {"inputs": prompt}
            response = requests.post(
                f"{self.api_url}{self.model}",
                headers=self.headers,
                json=data,
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                if isinstance(result, list) and len(result) > 0:
                    return result[0].get('generated_text', '')
                return str(result)
            else:
                return f"Error: HTTP {response.status_code}"
        except Exception as e:
            rospy.logerr(f"Hugging Face API error: {e}")
            return f"Error: {str(e)}"
    
    def is_available(self) -> bool:
        try:
            response = requests.post(
                f"{self.api_url}{self.model}",
                headers=self.headers,
                json={"inputs": "test"},
                timeout=10
            )
            return response.status_code == 200
        except:
            return False


class LLMNode:
    """Main LLM Node class"""
    
    def __init__(self):
        self.providers = {
            'openai': OpenAIProvider(),
            # 'ollama': OllamaProvider(),
            # 'huggingface': HuggingFaceProvider(),
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
            elif provider_type == 'ollama':
                self.config = {
                    'base_url': rospy.get_param('~ollama_url', 'http://localhost:11434'),
                    'model': rospy.get_param('~ollama_model', 'llama2')
                }
            elif provider_type == 'huggingface':
                self.config = {
                    'api_key': rospy.get_param('~huggingface_api_key', ''),
                    'model': rospy.get_param('~huggingface_model', 'microsoft/DialoGPT-medium')
                }
            elif provider_type == 'gemini':
                self.config = {
                    'api_key': rospy.get_param('~gemini_api_key', '') or rospy.get_param('~google_api_key', ''),
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
