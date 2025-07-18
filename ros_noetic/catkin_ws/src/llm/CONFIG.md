# LLM Node Configuration Examples

## Environment Variables
You can set these environment variables for API keys:
```bash
export OPENAI_API_KEY="your-openai-api-key"
export HUGGINGFACE_API_KEY="your-huggingface-api-key"
export GOOGLE_API_KEY="your-google-gemini-api-key"
```

## Launch with Different Providers

### OpenAI GPT
```bash
roslaunch llm llm_node.launch provider:=openai openai_model:=gpt-4
```

### Local Ollama
```bash
# First start Ollama server
ollama serve

# Then launch the node
roslaunch llm llm_node.launch provider:=ollama ollama_model:=llama2
```

### Hugging Face
```bash
roslaunch llm llm_node.launch provider:=huggingface huggingface_model:=microsoft/DialoGPT-large
```

### Google Gemini
```bash
roslaunch llm llm_node.launch provider:=gemini gemini_model:=gemini-pro top_p:=0.8 top_k:=50
```

## ROS Topics and Services

### Topics
- `/llm_prompt` (std_msgs/String) - Send prompts to LLM
- `/llm_response` (std_msgs/String) - Receive LLM responses

### Services
- `/llm_query` (std_srvs/Trigger) - Direct query service
- `/llm_status` (std_srvs/Trigger) - Check LLM availability

## Usage Examples

### Using Topics
```bash
# Send a prompt
rostopic pub /llm_prompt std_msgs/String "data: 'What is the capital of France?'"

# Listen for response
rostopic echo /llm_response
```

### Using Services
```bash
# Check status
rosservice call /llm_status

# Query (uses default prompt from parameter)
rosservice call /llm_query
```

## Adding New Providers

To add a new LLM provider:

1. Create a new class inheriting from `LLMProvider`
2. Implement the required methods:
   - `initialize(config)`
   - `generate_response(prompt, **kwargs)`
   - `is_available()`
3. Add it to the `providers` dictionary in `LLMNode.__init__()`
4. Update the configuration loading in `load_config()`

Example:
```python
class CustomProvider(LLMProvider):
    def initialize(self, config):
        # Initialize your provider
        return True
    
    def generate_response(self, prompt, **kwargs):
        # Generate response
        return "response"
    
    def is_available(self):
        # Check availability
        return True
```
