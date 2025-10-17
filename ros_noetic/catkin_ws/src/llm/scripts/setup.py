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

# Ensure stdout/stderr can print UTF-8 safely (helps when host defaults to ASCII)
try:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    # Non-fatal if not supported in this environment
    pass

# Cache for environment info to avoid repeated checks
_ENV_INFO_PRINTED = False

def print_environment_info(force: bool = False):
    """
    Print current running environment information for debugging
    Args:
        force: Force print even if already printed before
    """
    global _ENV_INFO_PRINTED
    
    # Skip if already printed and not forced
    if _ENV_INFO_PRINTED and not force:
        return
    
    _ENV_INFO_PRINTED = True
    
    print("=" * 60)
    print("ENVIRONMENT INFORMATION")
    print("=" * 60)
    
    # Platform information
    print(f"Platform: {sys.platform}")
    print(f"Python version: {sys.version}")
    print(f"Python executable: {sys.executable}")
    
    # Encoding information
    print(f"Default encoding: {sys.getdefaultencoding()}")
    print(f"File system encoding: {sys.getfilesystemencoding()}")
    print(f"stdout encoding: {getattr(sys.stdout, 'encoding', 'Unknown')}")
    print(f"stderr encoding: {getattr(sys.stderr, 'encoding', 'Unknown')}")
    
    # Locale information
    try:
        import locale
        print(f"Locale (LC_ALL): {locale.getlocale()}")
        print(f"Default locale: {locale.getdefaultlocale()}")
    except Exception as e:
        print(f"Locale info error: {e}")
    
    # Environment variables
    env_vars = ['PYTHONIOENCODING', 'LANG', 'LC_ALL', 'LC_CTYPE', 'TERM', 'SHELL']
    print("\nEnvironment Variables:")
    for var in env_vars:
        value = os.environ.get(var, 'Not set')
        print(f"  {var}: {value}")
    
    # VM/Container detection
    print(f"\nContainer/VM Detection:")
    try:
        # Check for virtualization
        import subprocess
        try:
            result = subprocess.run(['hostnamectl'], capture_output=True, text=True, timeout=2)
            if 'Virtualization:' in result.stdout:
                virt_line = [line for line in result.stdout.split('\n') if 'Virtualization:' in line]
                if virt_line:
                    print(f"  Virtualization: {virt_line[0].split(':', 1)[1].strip()}")
            else:
                print("  Virtualization: Not detected")
        except:
            print("  Virtualization: Detection failed")
        
        # Check for Docker
        if os.path.exists('/.dockerenv'):
            print("  Docker: Running in Docker container")
        else:
            print("  Docker: Not detected")
            
        # Check for WSL
        try:
            with open('/proc/version', 'r') as f:
                if 'Microsoft' in f.read():
                    print("  WSL: Running in Windows Subsystem for Linux")
                else:
                    print("  WSL: Not detected")
        except:
            print("  WSL: Detection failed")
            
    except Exception as e:
        print(f"  Detection error: {e}")
    
    # Test encoding capabilities
    print(f"\nEncoding Test:")
    test_chars = ['\u201c', '\u201d', '\u2018', '\u2019', '中文', '🚀']
    for char in test_chars:
        try:
            char.encode('utf-8')
            utf8_ok = "✓"
        except:
            utf8_ok = "✗"
        try:
            char.encode('ascii')
            ascii_ok = "✓"
        except:
            ascii_ok = "✗"
        print(f"  '{char}' UTF-8:{utf8_ok} ASCII:{ascii_ok}")
    
    print("=" * 60)

# Only print environment information if explicitly enabled via environment variable
if os.environ.get('LLM_DEBUG_ENV', '').lower() in ('1', 'true', 'yes'):
    print_environment_info()

# ============================================================================
# UNICODE REPLACEMENTS (Pre-defined for better performance)
# ============================================================================
UNICODE_REPLACEMENTS = {
    '\u201c': '"',   # Left double quotation mark
    '\u201d': '"',   # Right double quotation mark  
    '\u2018': "'",   # Left single quotation mark
    '\u2019': "'",   # Right single quotation mark
    '\u2013': '-',   # En dash
    '\u2014': '-',   # Em dash
    '\u2026': '...',  # Horizontal ellipsis
    '\u00a0': ' ',   # Non-breaking space
    '\ufeff': '',    # Byte order mark
    '\u00ab': '"',   # Left guillemet
    '\u00bb': '"',   # Right guillemet
    '\u2039': "'",   # Single left guillemet
    '\u203a': "'",   # Single right guillemet
}


def create_error_json(error_type: str, message: str = "") -> str:
    """
    Create a standardized error JSON response
    Args:
        error_type: Type of error (e.g., 'unicode_encoding_error', 'api_error')
        message: Optional error message
    Returns:
        JSON string with error information
    """
    error_data = {
        "error": error_type,
        "type": "sequence",
        "name": "Error",
        "children": []
    }
    
    if message:
        error_data["message"] = message
    
    import json
    return json.dumps(error_data)


def extract_text_from_candidates(response) -> str:
    """
    Extract text from API response candidates (for Gemini API)
    Args:
        response: API response object with candidates
    Returns:
        Extracted text string or empty string
    """
    result_chunks = []
    
    if hasattr(response, 'candidates') and response.candidates:
        for candidate in response.candidates:
            # Map finish reasons to messages if applicable
            try:
                reason = getattr(candidate, 'finish_reason', None)
                if reason == types.FinishReason.SAFETY:
                    result_chunks.append("Response blocked due to safety concerns")
                elif reason == types.FinishReason.RECITATION:
                    result_chunks.append("Response blocked due to recitation concerns")
                elif reason == types.FinishReason.OTHER:
                    if not getattr(candidate, 'content', None):
                        result_chunks.append("Response blocked for other reasons")
            except Exception:
                pass

            content = getattr(candidate, 'content', None)
            parts = getattr(content, 'parts', None) if content is not None else None
            if parts:
                for p in parts:
                    text_part = getattr(p, 'text', None)
                    if text_part:
                        result_chunks.append(text_part)
    
    # Fallbacks if no candidates/parts or empty content
    if not result_chunks and hasattr(response, 'prompt_feedback') and response.prompt_feedback:
        block_reason = getattr(response.prompt_feedback, 'block_reason', None)
        if block_reason:
            result_chunks.append(f"Prompt blocked: {block_reason}")
    
    return " ".join([chunk.strip() for chunk in result_chunks if chunk]).strip()


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
        """Sanitize text for cross-platform encoding compatibility while preserving formatting"""
        try:
            if not isinstance(text, str):
                text = str(text)
            
            # First, handle the text as bytes if needed to catch encoding issues early
            try:
                # If we can't encode as UTF-8, we have encoding issues
                text.encode('utf-8')
            except UnicodeEncodeError:
                # Convert problematic characters to safe equivalents immediately
                text = text.encode('ascii', errors='ignore').decode('ascii')
            
            # Normalize Unicode characters (only if no encoding errors)
            try:
                text = unicodedata.normalize('NFKC', text)
            except Exception:
                # If normalization fails, skip it
                pass
            
            # Replace problematic Unicode characters with safe equivalents using pre-defined dict
            for unicode_char, replacement in UNICODE_REPLACEMENTS.items():
                text = text.replace(unicode_char, replacement)
            
            # For Windows VM environments, be extra cautious with encoding
            if sys.platform == "win32":
                try:
                    # Test if the text can be encoded as ASCII
                    text.encode('ascii')
                except UnicodeEncodeError:
                    # Fall back to removing non-ASCII characters
                    text = ''.join(char if ord(char) < 128 else '?' for char in text)
            
            # Final safety check - ensure the result can be safely handled
            try:
                # Test encoding to catch any remaining issues
                text.encode('utf-8')
                return text
            except UnicodeEncodeError:
                # Ultimate fallback: ASCII-only
                return ''.join(char if ord(char) < 128 else '?' for char in text)
            
        except Exception as e:
            print(f"Text sanitization failed: {e}")
            # Ultimate fallback: ASCII-only text
            try:
                if isinstance(text, str):
                    return ''.join(char if ord(char) < 128 else '?' for char in text)
                else:
                    return "Text encoding error - invalid input"
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
        except UnicodeEncodeError as unicode_error:
            print(f"OpenAI API Unicode encoding error: {unicode_error}")
            error_msg = "Unicode encoding error in API response"
            return self._sanitize_for_encoding(error_msg)
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
        except UnicodeEncodeError as unicode_error:
            print(f"OpenAI JSON API Unicode encoding error: {unicode_error}")
            return create_error_json("unicode_encoding_error")
        except Exception as e:
            error_msg = self._sanitize_for_encoding(str(e))
            if "codec" in error_msg.lower() or "encode" in error_msg.lower():
                return create_error_json("api_encoding_error")
            else:
                return create_error_json("api_error", error_msg)
    
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

            # 使用共用函數提取文字，然後進行編碼清理
            result_text = extract_text_from_candidates(response)
            
            if result_text:
                # 對提取的文字進行編碼清理
                result_text = self._sanitize_for_encoding(result_text)
            
            if not result_text:
                result_text = "No response generated"

            return result_text
            
        except UnicodeEncodeError as unicode_error:
            print(f"Google Gemini API Unicode encoding error: {unicode_error}")
            # Handle Unicode errors specifically
            error_msg = "Unicode encoding error in API response"
            return self._sanitize_for_encoding(error_msg)
        except Exception as e:
            print(f"Google Gemini API error: {e}")
            # Sanitize the error message itself to prevent propagation of encoding issues
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

            # 使用共用函數提取文字，然後進行編碼清理
            result_text = extract_text_from_candidates(response)
            
            if result_text:
                result_text = self._sanitize_for_encoding(result_text)
            
            if not result_text:
                # Provide clearer fallback for JSON flows
                result_text = '{"error": "no_json_content"}'

            # Prefer UTF-8: Python str 已是 Unicode，僅檢查可否以 UTF-8 編碼
            try:
                _ = result_text.encode('utf-8')
            except UnicodeEncodeError:
                # 保留 Unicode，僅做正規化以避免特殊符號型態差異
                try:
                    result_text = unicodedata.normalize('NFKC', result_text)
                except Exception:
                    pass

            return result_text or '{"error": "empty_response"}'
            
        except UnicodeEncodeError as unicode_error:
            print(f"Google Gemini JSON API Unicode encoding error: {unicode_error}")
            return create_error_json("unicode_encoding_error")
        except Exception as e:
            print(f"Google Gemini JSON API error: {e}")
            error_msg = self._sanitize_for_encoding(str(e))
            if "codec" in error_msg.lower() or "encode" in error_msg.lower():
                return create_error_json("api_encoding_error")
            else:
                return create_error_json("api_error", error_msg)

    def is_available(self) -> bool:
        try:
            if not self.client:
                return False
            response = self.client.models.generate_content(
                model="gemini-2.5-flash",
                contents="Hello"
            )
            # Consider available if we get at least one candidate or any parts
            if hasattr(response, 'candidates') and response.candidates:
                for candidate in response.candidates:
                    content = getattr(candidate, 'content', None)
                    parts = getattr(content, 'parts', None) if content is not None else None
                    if parts:
                        return True
            # Fallback to True if no error was thrown (some SDK versions may omit candidates for trivial prompts)
            return True
        except Exception as e:
            print(f"Gemini availability check failed: {e}")
            return False