#!/root/.pyenv/versions/3.9.19/bin/python3

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
                print("OpenAI API key not found")
                return False
                
            openai.api_key = api_key
            self.client = openai.OpenAI(api_key=api_key)
            self.model = config.get('model', self.model)
            self.max_tokens = config.get('max_tokens', self.max_tokens)
            self.temperature = config.get('temperature', self.temperature)
            
            print(f"OpenAI provider initialized with model: {self.model}")
            return True
        except Exception as e:
            print(f"Failed to initialize OpenAI provider: {e}")
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
            print(f"OpenAI API error: {e}")
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
        self.model_name = "gemini-2.5-flash"  # Latest recommended model
        self.config = None

    def initialize(self, config: Dict[str, Any]) -> bool:
        try:
            api_key = config.get('api_key') or os.getenv('GEMINI_API_KEY') or os.getenv('GOOGLE_API_KEY')
            if not api_key:
                print("Google Gemini API key not found. Set GEMINI_API_KEY or GOOGLE_API_KEY environment variable")
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

            print(f"Google Gemini provider initialized with model: {self.model_name}")
            return True
        except Exception as e:
            print(f"Failed to initialize Google Gemini provider: {e}")
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
            print(f"Google Gemini API error: {e}")
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
            print(f"Gemini availability check failed: {e}")
            return False