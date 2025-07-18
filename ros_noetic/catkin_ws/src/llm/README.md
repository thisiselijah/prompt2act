# LLM Node

A scalable ROS node for integrating Large Language Models (LLMs) into robotics applications. Supports multiple LLM providers with a unified interface.

## Features

- **Multi-Provider Support**: OpenAI GPT, Ollama (local), Hugging Face
- **Unified Interface**: Same ROS topics/services regardless of provider
- **Easy Configuration**: ROS parameters and environment variables
- **Scalable Architecture**: Easy to add new LLM providers
- **Error Handling**: Comprehensive error handling and logging
- **Status Monitoring**: Check LLM availability and health

## Supported Providers

### OpenAI GPT
- Models: GPT-3.5-turbo, GPT-4, etc.
- Requires: OpenAI API key
- Configuration: Model, temperature, max_tokens

### Ollama (Local)
- Models: Llama2, Mistral, CodeLlama, etc.
- Requires: Local Ollama server
- Configuration: Server URL, model name

### Google Gemini
- Models: gemini-2.0-flash, gemini-1.5-flash, etc.
- Requires: Google AI Studio API key (GEMINI_API_KEY or GOOGLE_API_KEY)
- Configuration: Model, temperature, max_output_tokens, top_p, top_k
- Features: Advanced safety settings, content generation controls
- **New**: Uses the updated Google GenAI SDK for improved performance

### Hugging Face
- Models: Any Hugging Face model with inference API
- Requires: Hugging Face API key
- Configuration: Model name

## Installation

1. **Install Python dependencies**:
   ```bash
   cd /path/to/catkin_ws/src/llm
   pip install -r requirements.txt
   ```

2. **Build the ROS package**:
   ```bash
   cd /path/to/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Set up API keys** (if using cloud providers):
   ```bash
   export OPENAI_API_KEY="your-openai-api-key"
   export HUGGINGFACE_API_KEY="your-huggingface-api-key"
   export GEMINI_API_KEY="your-google-gemini-api-key"
   # Alternative for Gemini:
   # export GOOGLE_API_KEY="your-google-gemini-api-key"
   ```

## Usage

### Quick Start

1. **Launch with OpenAI**:
   ```bash
   roslaunch llm llm_node.launch provider:=openai
   ```

2. **Launch with local Ollama**:
   ```bash
   # First start Ollama server
   ollama serve
   
   # Then launch the node
   roslaunch llm llm_node.launch provider:=ollama
   ```

3. **Launch with Google Gemini**:
   ```bash
   roslaunch llm llm_node.launch provider:=gemini gemini_model:=gemini-2.0-flash
   ```

4. **Test the node**:
   ```bash
   # Test using topics
   python scripts/llm_test.py topic
   
   # Test using services
   python scripts/llm_test.py service
   ```

### ROS Interface

#### Topics
- **`/llm_prompt`** (std_msgs/String): Send prompts to LLM
- **`/llm_response`** (std_msgs/String): Receive LLM responses

#### Services
- **`/llm_query`** (std_srvs/Trigger): Direct query service
- **`/llm_status`** (std_srvs/Trigger): Check LLM availability

#### Examples

**Using Topics**:
```bash
# Send a prompt
rostopic pub /llm_prompt std_msgs/String "data: 'Explain what a robot is'"

# Listen for response
rostopic echo /llm_response
```

**Using Services**:
```bash
# Check status
rosservice call /llm_status

# Query (uses default prompt)
rosservice call /llm_query
```

## Configuration

### Launch Parameters

```xml
<node name="llm_node" pkg="llm" type="llm_node.py">
    <!-- Provider Selection -->
    <param name="provider" value="openai" />
    
    <!-- OpenAI Configuration -->
    <param name="openai_model" value="gpt-4" />
    <param name="max_tokens" value="1500" />
    <param name="temperature" value="0.8" />
    
    <!-- Ollama Configuration -->
    <param name="ollama_url" value="http://localhost:11434" />
    <param name="ollama_model" value="llama2" />
    
    <!-- Hugging Face Configuration -->
    <param name="huggingface_model" value="microsoft/DialoGPT-large" />
    
    <!-- Google Gemini Configuration -->
    <param name="gemini_model" value="gemini-2.0-flash" />
    <param name="max_output_tokens" value="2000" />
    <param name="temperature" value="0.9" />
    <param name="top_p" value="0.8" />
    <param name="top_k" value="50" />
</node>
```

### Environment Variables

```bash
# API Keys
export OPENAI_API_KEY="sk-..."
export HUGGINGFACE_API_KEY="hf_..."

# Alternative configuration
export LLM_PROVIDER="ollama"
export OLLAMA_MODEL="mistral"
```

## Integration with Behavior Trees

The LLM node can be easily integrated with behavior trees for robot decision making:

```python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

class LLMActionNode:
    def __init__(self):
        self.prompt_pub = rospy.Publisher('/llm_prompt', String, queue_size=10)
        self.response = None
        self.response_sub = rospy.Subscriber('/llm_response', String, self.response_callback)
    
    def response_callback(self, msg):
        self.response = msg.data
    
    def execute_query(self, prompt):
        # Send prompt
        msg = String()
        msg.data = prompt
        self.prompt_pub.publish(msg)
        
        # Wait for response
        timeout = 10
        rate = rospy.Rate(10)
        elapsed = 0
        
        while not self.response and elapsed < timeout:
            rate.sleep()
            elapsed += 0.1
        
        return self.response
```

## Adding New Providers

To add a new LLM provider:

1. **Create a new provider class**:
   ```python
   class MyCustomProvider(LLMProvider):
       def initialize(self, config):
           # Initialize your provider
           return True
       
       def generate_response(self, prompt, **kwargs):
           # Generate response from your LLM
           return "response"
       
       def is_available(self):
           # Check if provider is available
           return True
   ```

2. **Register the provider**:
   ```python
   # In LLMNode.__init__()
   self.providers['mycustom'] = MyCustomProvider()
   ```

3. **Add configuration support**:
   ```python
   # In load_config()
   elif provider_type == 'mycustom':
       self.config = {
           'api_key': rospy.get_param('~mycustom_api_key', ''),
           'model': rospy.get_param('~mycustom_model', 'default-model')
       }
   ```

## Troubleshooting

### Common Issues

1. **"No LLM provider initialized"**:
   - Check that the provider is correctly specified
   - Verify API keys are set
   - Check network connectivity

2. **OpenAI API errors**:
   - Verify API key is valid
   - Check rate limits and quotas
   - Ensure model is accessible

3. **Ollama connection failed**:
   - Start Ollama server: `ollama serve`
   - Check if the model is downloaded: `ollama list`
   - Verify server URL and port

4. **Timeout errors**:
   - Increase timeout values
   - Check network latency
   - Consider using a faster model

### Debugging

Enable debug logging:
```bash
roslaunch llm llm_node.launch --screen
```

Check ROS logs:
```bash
roscd llm
tail -f ~/.ros/log/latest/llm-llm_node-*.log
```

## Performance Considerations

- **Latency**: Cloud providers typically have higher latency than local models
- **Cost**: OpenAI and Hugging Face charge per token
- **Privacy**: Local models (Ollama) keep data private
- **Reliability**: Local models are not dependent on internet connectivity

## License

[Your License Here]

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add your provider following the existing patterns
4. Test thoroughly
5. Submit a pull request

## Migration Guide

### Google Gemini Provider Migration (v2.0)

The Google Gemini provider has been updated to use the new Google GenAI SDK. If you're upgrading from an earlier version:

#### Required Changes:

1. **Update dependencies**:
   ```bash
   pip uninstall google-generativeai
   pip install google-genai>=0.2.0
   ```

2. **Environment Variables**: 
   - Preferred: `GEMINI_API_KEY` (new)
   - Still supported: `GOOGLE_API_KEY` (legacy)

3. **Model Names**:
   - Old: `gemini-pro`, `gemini-pro-vision`
   - New: `gemini-2.0-flash`, `gemini-1.5-flash` (recommended)

4. **Launch File Updates**:
   ```xml
   <!-- Old Configuration -->
   <param name="gemini_model" value="gemini-pro" />
   
   <!-- New Configuration -->
   <param name="gemini_model" value="gemini-2.0-flash" />
   ```

#### Benefits of Migration:
- Better performance and reliability
- Access to latest Gemini models (2.0 series)
- Improved error handling and safety features
- Future-proof compatibility

#### Breaking Changes:
- Requires new `google-genai` package instead of `google-generativeai`
- Some legacy model names are deprecated
- Response structure may be slightly different (handled internally)
