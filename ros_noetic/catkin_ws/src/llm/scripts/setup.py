#!/root/.pyenv/versions/3.9.19/bin/python3.9
import os
import sys
import locale
import unicodedata
import re
from abc import ABC, abstractmethod
import requests
import openai
from google import genai
from google.genai import types
from typing import Dict, Any, Optional

# Configure encoding for cross-platform compatibility
if sys.platform == "win32":
    # Windows-specific encoding setup
    os.environ['PYTHONIOENCODING'] = 'utf-8'
elif sys.platform == "darwin":
    # macOS-specific setup
    try:
        locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')
    except locale.Error:
        pass
else:
    # Linux and other Unix-like systems
    try:
        locale.setlocale(locale.LC_ALL, 'C.UTF-8')
    except locale.Error:
        try:
            locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')
        except locale.Error:
            pass  # Use system default


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
    
    def _sanitize_for_encoding(self, text: str) -> str:
        """Sanitize text for cross-platform encoding compatibility"""
        try:
            if not isinstance(text, str):
                text = str(text)
            
            # Normalize Unicode characters
            text = unicodedata.normalize('NFKC', text)
            
            # Replace problematic Unicode characters with safe equivalents
            unicode_replacements = {
                '\u201c': '"',  # Left double quotation mark
                '\u201d': '"',  # Right double quotation mark
                '\u2018': "'",  # Left single quotation mark
                '\u2019': "'",  # Right single quotation mark
                '\u2013': '-',  # En dash
                '\u2014': '-',  # Em dash
                '\u2026': '...',  # Horizontal ellipsis
                '\u00a0': ' ',  # Non-breaking space
                '\ufeff': '',   # Byte order mark
            }
            
            for unicode_char, replacement in unicode_replacements.items():
                text = text.replace(unicode_char, replacement)
            
            # For Windows VM environments, be extra cautious with encoding
            if sys.platform == "win32":
                try:
                    # Test if the text can be encoded as ASCII
                    text.encode('ascii')
                except UnicodeEncodeError:
                    # Fall back to removing non-ASCII characters
                    text = ''.join(char if ord(char) < 128 else '?' for char in text)
            
            return text
            
        except Exception as e:
            print(f"Text sanitization failed: {e}")
            # Ultimate fallback: ASCII-only text
            try:
                return ''.join(char if ord(char) < 128 else '?' for char in str(text))
            except Exception:
                return "Text encoding error"


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
            result = response.choices[0].message.content.strip()
            return self._sanitize_for_encoding(result)
        except Exception as e:
            error_msg = f"Error: {str(e)}"
            return self._sanitize_for_encoding(error_msg)
    
    def generate_json_response(self, prompt: str, schema: Optional[Any] = None) -> str:
        try:
            # For OpenAI, we'll request JSON format in the prompt
            json_prompt = f"{prompt}\n\nPlease respond with valid JSON format only."
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": json_prompt}],
                response_format={"type": "json_object"}
            )
            result = response.choices[0].message.content.strip()
            return self._sanitize_for_encoding(result)
        except Exception as e:
            error_msg = f"Error: {str(e)}"
            return self._sanitize_for_encoding(error_msg)
    
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
            result_text = ""
            if hasattr(response, 'text') and response.text:
                result_text = response.text.strip()
            elif hasattr(response, 'candidates') and response.candidates:
                candidate = response.candidates[0]
                if hasattr(candidate, 'finish_reason'):
                    reason = candidate.finish_reason
                    if reason == types.FinishReason.SAFETY:
                        result_text = "Response blocked due to safety concerns"
                    elif reason == types.FinishReason.RECITATION:
                        result_text = "Response blocked due to recitation concerns"
                    elif reason == types.FinishReason.OTHER:
                        result_text = "Response blocked for other reasons"
                # Try to get content from candidate
                if hasattr(candidate, 'content') and hasattr(candidate.content, 'parts'):
                    if candidate.content.parts:
                        result_text = candidate.content.parts[0].text.strip()
                if not result_text:
                    result_text = "No text content in response"
            elif hasattr(response, 'prompt_feedback') and response.prompt_feedback:
                if getattr(response.prompt_feedback, 'block_reason', None):
                    result_text = f"Prompt blocked: {response.prompt_feedback.block_reason}"
                else:
                    result_text = "No response generated"
            else:
                result_text = "No response generated"
            
            return self._sanitize_for_encoding(result_text)
            
        except Exception as e:
            print(f"Google Gemini API error: {e}")
            error_msg = f"Error: {str(e)}"
            return self._sanitize_for_encoding(error_msg)

    def generate_json_response(self, prompt: str, schema: Optional[Any] = None) -> str:
        try:
            # For the current Google GenAI SDK, we need to use a simpler approach
            # Add JSON instruction to the prompt instead of using generation_config
            json_prompt = f"""{prompt}

Please respond with valid JSON format only. Do not include any explanations or additional text.
Use only ASCII characters and standard double quotes (") for JSON strings."""
            
            response = self.client.models.generate_content(
                model="gemini-2.5-flash",
                contents=json_prompt
            )

            # Handle response - for JSON, we want the text content
            result_text = ""
            if hasattr(response, 'text') and response.text:
                result_text = response.text.strip()
            elif hasattr(response, 'candidates') and response.candidates:
                candidate = response.candidates[0]
                if hasattr(candidate, 'content') and hasattr(candidate.content, 'parts'):
                    if candidate.content.parts:
                        result_text = candidate.content.parts[0].text.strip()
                if not result_text:
                    result_text = "No JSON content in response"
            else:
                result_text = "No JSON response generated"
            
            return self._sanitize_for_encoding(result_text)
            
        except Exception as e:
            print(f"Google Gemini JSON API error: {e}")
            error_msg = f"Error: {str(e)}"
            return self._sanitize_for_encoding(error_msg)

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