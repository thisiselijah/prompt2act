#!/usr/bin/env python3
"""
Test script for cross-platform JSON parsing enhancements
Tests various problematic JSON formats that might be encountered across different systems
"""

import json
import sys
import os
import unicodedata

# Add the scripts directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

class MockLLMNode:
    """Mock LLM Node for testing JSON parsing methods"""
    
    def _extract_json_from_response(self, response: str) -> str:
        """Extract JSON from response that might contain additional text - cross-platform compatible"""
        try:
            import re
            import unicodedata
            
            # Normalize unicode characters for cross-platform compatibility
            response = unicodedata.normalize('NFKC', response)
            
            # Handle different line endings (Windows CRLF, Unix LF, Mac CR)
            response = response.replace('\r\n', '\n').replace('\r', '\n')
            
            # Remove zero-width characters that can cause parsing issues
            response = re.sub(r'[\u200b-\u200f\ufeff]', '', response)
            
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
                json_blocks = re.findall(pattern, response, re.DOTALL | re.IGNORECASE)
                if json_blocks:
                    candidate = json_blocks[0].strip()
                    if self._is_valid_json_structure(candidate):
                        return candidate
            
            # Look for JSON objects in the text with improved brace matching
            # Handle nested objects and arrays properly
            json_candidates = []
            
            # Find all potential JSON start positions
            for match in re.finditer(r'\{', response):
                start_idx = match.start()
                json_str = self._extract_balanced_braces(response, start_idx)
                if json_str and self._is_valid_json_structure(json_str):
                    json_candidates.append(json_str)
            
            # Return the longest valid JSON (likely the most complete)
            if json_candidates:
                return max(json_candidates, key=len)
            
            # Last resort: try to clean and return the original response
            cleaned_response = self._clean_response_text(response)
            return cleaned_response
                
        except Exception as e:
            print(f"JSON extraction failed: {e}")
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
            
            # Remove byte order marks (BOM) that can appear on Windows
            json_str = json_str.lstrip('\ufeff\ufffe')
            
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
            
            return json_str.strip()
            
        except Exception:
            return json_str.strip() if json_str else ""
    
    def _aggressive_json_cleanup(self, json_str: str) -> str:
        """Aggressive JSON cleanup as last resort for cross-platform compatibility"""
        try:
            import re
            
            # Remove all non-printable characters except necessary whitespace
            json_str = re.sub(r'[^\x20-\x7E\n\r\t]', '', json_str)
            
            # Fix common issues with quotes
            # Replace smart quotes with regular quotes
            json_str = json_str.replace('"', '"').replace('"', '"')
            json_str = json_str.replace(''', "'").replace(''', "'")
            
            # Ensure all property names are quoted
            json_str = re.sub(r'(\w+)(\s*:\s*)', r'"\1"\2', json_str)
            
            # Fix boolean values to lowercase
            json_str = re.sub(r'\bTrue\b', 'true', json_str)
            json_str = re.sub(r'\bFalse\b', 'false', json_str)
            json_str = re.sub(r'\bNull\b', 'null', json_str)
            json_str = re.sub(r'\bNone\b', 'null', json_str)
            
            # Remove trailing commas more aggressively
            json_str = re.sub(r',(\s*[}\]])', r'\1', json_str)
            
            # Remove duplicate quotes
            json_str = re.sub(r'"{2,}', '"', json_str)
            
            # Ensure consistent spacing
            json_str = re.sub(r'\s+', ' ', json_str)
            
            return json_str.strip()
            
        except Exception:
            return json_str.strip() if json_str else ""


def test_json_parsing():
    """Test various problematic JSON formats"""
    
    mock_node = MockLLMNode()
    
    test_cases = [
        # Standard case
        {
            "name": "Standard JSON",
            "input": '{"type": "sequence", "name": "TestTask", "children": []}',
            "should_parse": True
        },
        
        # With markdown code block
        {
            "name": "Markdown code block",
            "input": '''Here's the JSON:
```json
{"type": "sequence", "name": "TestTask", "children": []}
```
That's it!''',
            "should_parse": True
        },
        
        # With Windows line endings
        {
            "name": "Windows line endings",
            "input": '{\r\n  "type": "sequence",\r\n  "name": "TestTask",\r\n  "children": []\r\n}',
            "should_parse": True
        },
        
        # With BOM (Byte Order Mark)
        {
            "name": "With BOM",
            "input": '\ufeff{"type": "sequence", "name": "TestTask", "children": []}',
            "should_parse": True
        },
        
        # With smart quotes
        {
            "name": "Smart quotes",
            "input": '{"type": "sequence", "name": "TestTask", "children": []}',
            "should_parse": True
        },
        
        # With trailing comma
        {
            "name": "Trailing comma",
            "input": '{"type": "sequence", "name": "TestTask", "children": [],}',
            "should_parse": True
        },
        
        # With single quotes
        {
            "name": "Single quotes",
            "input": "{'type': 'sequence', 'name': 'TestTask', 'children': []}",
            "should_parse": True
        },
        
        # Nested JSON with escaped quotes
        {
            "name": "Nested with escapes",
            "input": '{"type": "sequence", "name": "Test\\"Task", "children": [{"type": "pick_up", "name": "Pick"}]}',
            "should_parse": True
        },
        
        # With Python-style booleans
        {
            "name": "Python booleans",
            "input": '{"type": "sequence", "name": "TestTask", "active": True, "enabled": False, "value": None}',
            "should_parse": True
        },
        
        # With extra whitespace
        {
            "name": "Extra whitespace",
            "input": '''  {
                "type"  :  "sequence"  ,
                "name"  :  "TestTask"  ,
                "children"  :  [  ]
            }  ''',
            "should_parse": True
        }
    ]
    
    print("Testing Cross-Platform JSON Parsing")
    print("=" * 50)
    
    results = {"passed": 0, "failed": 0}
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\nTest {i}: {test_case['name']}")
        print("-" * 30)
        
        try:
            # Extract JSON
            extracted = mock_node._extract_json_from_response(test_case["input"])
            print(f"Extracted: {extracted[:100]}{'...' if len(extracted) > 100 else ''}")
            
            # Sanitize
            sanitized = mock_node._sanitize_json_for_parsing(extracted)
            print(f"Sanitized: {sanitized[:100]}{'...' if len(sanitized) > 100 else ''}")
            
            # Try to parse
            try:
                parsed = json.loads(sanitized)
                print(f"✅ Successfully parsed: {type(parsed)}")
                if test_case["should_parse"]:
                    results["passed"] += 1
                    print("✅ Test PASSED")
                else:
                    results["failed"] += 1
                    print("❌ Test FAILED (expected to fail but passed)")
                    
            except json.JSONDecodeError as e:
                print(f"❌ JSON parsing failed: {e}")
                
                # Try aggressive cleanup
                try:
                    aggressive = mock_node._aggressive_json_cleanup(sanitized)
                    parsed = json.loads(aggressive)
                    print(f"✅ Aggressive cleanup succeeded: {aggressive[:100]}{'...' if len(aggressive) > 100 else ''}")
                    if test_case["should_parse"]:
                        results["passed"] += 1
                        print("✅ Test PASSED (after aggressive cleanup)")
                    else:
                        results["failed"] += 1
                        print("❌ Test FAILED (expected to fail but passed after cleanup)")
                        
                except json.JSONDecodeError as e2:
                    print(f"❌ Aggressive cleanup also failed: {e2}")
                    if test_case["should_parse"]:
                        results["failed"] += 1
                        print("❌ Test FAILED")
                    else:
                        results["passed"] += 1
                        print("✅ Test PASSED (expected to fail)")
                        
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
            results["failed"] += 1
            print("❌ Test FAILED (unexpected error)")
    
    # Summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    print(f"Total tests: {len(test_cases)}")
    print(f"Passed: {results['passed']}")
    print(f"Failed: {results['failed']}")
    print(f"Success rate: {results['passed']/len(test_cases)*100:.1f}%")
    
    if results["failed"] == 0:
        print("🎉 All tests passed!")
        return True
    else:
        print("⚠️  Some tests failed")
        return False


if __name__ == "__main__":
    success = test_json_parsing()
    sys.exit(0 if success else 1)
