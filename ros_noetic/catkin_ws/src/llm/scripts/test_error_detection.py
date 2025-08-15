#!/usr/bin/env python3

"""
Test script to verify error message detection in aggressive JSON cleanup
"""

import sys
import os
import re
import unicodedata

def aggressive_json_cleanup_test(json_str: str) -> str:
    """Test version of aggressive JSON cleanup for error detection testing"""
    try:
        print(f"[INFO] Applying aggressive JSON cleanup for Unicode issues")
        
        # Ensure we're working with a proper string
        if not isinstance(json_str, str):
            try:
                json_str = str(json_str)
            except Exception as convert_error:
                print(f"[ERROR] Failed to convert input to string: {convert_error}")
                return '{"type": "sequence", "name": "ConversionError", "children": []}'
        
        # Early detection of error messages in the input
        error_patterns = [
            r"ascii.*codec.*can't.*encode",
            r"codec can't encode character",
            r"ordinal not in range",
            r"\\u201[cd]",  # Smart quotes unicode codes
            r"UnicodeEncodeError",
            r"UnicodeDecodeError",
            r"Traceback.*most recent call"
        ]
        
        for pattern in error_patterns:
            if re.search(pattern, json_str[:500], re.IGNORECASE):
                print(f"[ERROR] Input appears to be an error message, not JSON: {json_str[:200]}")
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
                print(f"[WARN] Unicode normalization failed, continuing with character replacement: {norm_error}")
            
        except Exception as unicode_error:
            print(f"[WARN] Unicode processing failed: {unicode_error}")
            # Fallback: aggressive ASCII-only filtering
            try:
                json_str = json_str.encode('ascii', 'ignore').decode('ascii')
            except Exception as ascii_error:
                print(f"[ERROR] ASCII conversion also failed: {ascii_error}")
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
            print(f"[WARN] Regex processing failed: {regex_error}")
        
        # Final structure validation and repair
        json_str = json_str.strip()
        
        # Ensure proper JSON structure
        if not json_str.startswith('{'):
            start_idx = json_str.find('{')
            if start_idx != -1:
                json_str = json_str[start_idx:]
            else:
                # No valid JSON structure found
                print(f"[WARN] No JSON structure found after cleanup")
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
            print(f"[WARN] Cleanup result too short: {json_str}")
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
            print(f"[WARN] Cleanup result contains error indicators: {json_str}")
            return '{"type": "sequence", "name": "CleanupInvalid", "children": []}'
        
        print(f"[INFO] Aggressive cleanup completed. Result length: {len(json_str)}")
        return json_str
        
    except Exception as e:
        print(f"[ERROR] Aggressive cleanup failed with exception: {e}")
        # Ultimate fallback: return a valid minimal JSON structure
        return '{"type": "sequence", "name": "FallbackTask", "children": []}'

def test_error_detection():
    """Test error message detection in aggressive cleanup"""
    
    print("Testing Error Message Detection in Aggressive JSON Cleanup")
    print("=" * 60)
    
    # Test cases with error messages that should be detected and rejected
    error_test_cases = [
        {
            "name": "ASCII codec error (exact from logs)",
            "input": '"Error": "ascii" codec can\'t encode character "\\u201d in position "0": ordinal not in range(128)',
            "should_detect": True
        },
        {
            "name": "Unicode encode error",
            "input": "UnicodeEncodeError: 'ascii' codec can't encode character '\\u201d' in position 0: ordinal not in range(128)",
            "should_detect": True
        },
        {
            "name": "Codec error with position",
            "input": 'Error: ascii codec can\'t encode character at position 0: ordinal not in range(128)',
            "should_detect": True
        },
        {
            "name": "Valid JSON (should NOT be detected as error)",
            "input": '{"type": "sequence", "name": "TestTask", "children": []}',
            "should_detect": False
        },
        {
            "name": "JSON with word 'error' in value (should NOT be detected)",
            "input": '{"type": "sequence", "name": "ErrorHandling", "children": []}',
            "should_detect": False
        },
        {
            "name": "Traceback error message",
            "input": "Traceback (most recent call last): File error occurred",
            "should_detect": True
        }
    ]
    
    passed = 0
    failed = 0
    
    for i, test_case in enumerate(error_test_cases, 1):
        print(f"\nTest {i}: {test_case['name']}")
        print("-" * 40)
        print(f"Input: {test_case['input'][:100]}...")
        
        try:
            # Test the aggressive cleanup function
            result = aggressive_json_cleanup_test(test_case['input'])
            
            # Check if the result indicates error detection
            is_error_detected = any([
                "ErrorMessageDetected" in result,
                "ConversionError" in result, 
                "EncodingFailure" in result,
                "CleanupInvalid" in result,
                "NoJSONFound" in result
            ])
            
            expected = test_case['should_detect']
            
            if is_error_detected == expected:
                print(f"✅ PASSED - Error detection: {is_error_detected} (expected: {expected})")
                print(f"   Cleanup result: {result}")
                passed += 1
            else:
                print(f"❌ FAILED - Error detection: {is_error_detected} (expected: {expected})")
                print(f"   Cleanup result: {result}")
                failed += 1
                
        except Exception as e:
            print(f"❌ FAILED - Exception during test: {e}")
            failed += 1
    
    print("\n" + "=" * 60)
    print("ERROR DETECTION TEST SUMMARY")
    print("=" * 60)
    print(f"Total tests: {len(error_test_cases)}")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Success rate: {passed/len(error_test_cases)*100:.1f}%")
    
    if failed == 0:
        print("🎉 All error detection tests passed!")
    else:
        print(f"⚠️  {failed} tests failed")
    
    return failed == 0

if __name__ == '__main__':
    success = test_error_detection()
    sys.exit(0 if success else 1)
