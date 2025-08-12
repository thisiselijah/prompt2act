#!/root/.pyenv/versions/3.9.19/bin/python3.9


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
    def initialize(self) -> bool:
        """Initialize the LLM provider"""
        pass
    
    @abstractmethod
    def generate_response(self, prompt: str) -> str:
        """Generate response from the LLM"""
        pass
    
    @abstractmethod
    def generate_json_response(self, prompt: str, schema: Optional[Any] = None) -> str:
        """Generate JSON response from the LLM"""
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """Check if the LLM service is available"""
        pass


class OpenAIProvider(LLMProvider):
    """OpenAI GPT provider"""
    
    def __init__(self):
        self.client = None
        
    def initialize(self) -> bool:
        try:
            self.client = openai.OpenAI()
            print("OpenAI provider initialized")
            return True
        except Exception as e:
            print(f"Failed to initialize OpenAI provider: {e}")
            return False
    
    def generate_response(self, prompt: str) -> str:
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}]
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            print(f"OpenAI API error: {e}")
            return f"Error: {str(e)}"
    
    def generate_json_response(self, prompt: str, schema: Optional[Any] = None) -> str:
        try:
            # For OpenAI, we'll request JSON format in the prompt
            json_prompt = f"{prompt}\n\nPlease respond with valid JSON format only."
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": json_prompt}],
                response_format={"type": "json_object"}
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            print(f"OpenAI API error: {e}")
            return f"Error: {str(e)}"
    
    def is_available(self) -> bool:
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
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

    def initialize(self) -> bool:
        try:
            self.client = genai.Client()
            print("Google Gemini provider initialized")
            return True
        except Exception as e:
            print(f"Failed to initialize Google Gemini provider: {e}")
            return False

    def generate_response(self, prompt: str) -> str:
        try:
            response = self.client.models.generate_content(
                model="gemini-2.5-flash",
                contents=prompt
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

    def generate_json_response(self, prompt: str, schema: Optional[Any] = None) -> str:
        try:
            config = {
                "response_mime_type": "application/json"
            }
            
            # Add schema if provided
            if schema:
                config["response_schema"] = schema
                
            response = self.client.models.generate_content(
                model="gemini-2.5-flash",
                contents=prompt,
                config=config
            )

            # Handle response - for JSON, we want the text content
            if hasattr(response, 'text') and response.text:
                return response.text.strip()
            elif hasattr(response, 'candidates') and response.candidates:
                candidate = response.candidates[0]
                if hasattr(candidate, 'content') and hasattr(candidate.content, 'parts'):
                    if candidate.content.parts:
                        return candidate.content.parts[0].text.strip()
                return "No JSON content in response"
            else:
                return "No JSON response generated"
        except Exception as e:
            print(f"Google Gemini JSON API error: {e}")
            return f"Error: {str(e)}"

    def is_available(self) -> bool:
        try:
            if not self.client:
                return False
            response = self.client.models.generate_content(
                model="gemini-2.5-flash",
                contents="Hello"
            )
            return hasattr(response, 'text') and response.text is not None
        except Exception as e:
            print(f"Gemini availability check failed: {e}")
            return False