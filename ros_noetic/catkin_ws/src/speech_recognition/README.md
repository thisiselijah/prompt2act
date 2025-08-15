# Speech Recognition to Behavior Tree Integration

This package provides a ROS node that integrates speech recognition with the LLM service to create an audio-to-behavior-tree workflow.

## Overview

The speech recognition node offers a service that:
1. **Converts audio files to text** using Google's Web Speech API
2. **Sends the transcribed text** as a task description to the LLM service
3. **Automatically generates and assembles** a behavior tree based on the spoken command

## Architecture

```
Audio File → Speech Recognition → LLM Service → Behavior Tree Assembly → Robot Execution
    ↓              ↓                  ↓               ↓                    ↓
  .wav/.mp3    Google Speech API   Gemini/OpenAI   py_trees           Niryo Robot
```

## Dependencies

### ROS Packages
- `rospy`
- `std_msgs` 
- `message_generation`
- `message_runtime`
- `llm` (LLM service package)

### Python Packages
- `speech_recognition`
- `pydub` (for audio format conversion)
- `google-cloud-speech` (optional, for enhanced recognition)

### System Dependencies
- `ffmpeg` (for audio format conversion via pydub)

## Service Definition

### ProcessAudioCommand Service

```
# Request
string audio_file_path    # Path to the audio file
string language          # Language code (e.g., 'en-US', 'zh-CN')
---
# Response
bool success             # Whether the entire process was successful
string transcribed_text  # The transcribed text from audio
string behavior_tree_json # The generated behavior tree JSON (auto-assembled)
string error_message     # Error message if process failed
```

**Note**: Behavior tree assembly is now automatic - no need to specify auto_assemble parameter!

## Usage

### 1. Start Required Services

```bash
# Terminal 1: Start the behavior tree node (required for auto-assembly)
rosrun behavior_tree behavior_tree_node.py

# Terminal 2: Start the LLM node
rosrun llm llm_node.py _provider:=gemini

# Terminal 3: Start the speech recognition node
rosrun speech_recognition speech_recognition_node.py
```

### 2. Call the Audio Processing Service

#### Basic Usage
```bash
rosservice call /process_audio_command '{
  audio_file_path: "/path/to/your/audio.wav",
  language: "en-US"
}'
```

#### Advanced Usage Examples

**English Commands:**
```bash
# Simple pick and place
rosservice call /process_audio_command '{
  audio_file_path: "/recordings/pickup_command.wav",
  language: "en-US"
}'

# Complex sorting task  
rosservice call /process_audio_command '{
  audio_file_path: "/recordings/sort_objects.wav",
  language: "en-US"
}'
```

**Chinese Commands:**
```bash
rosservice call /process_audio_command '{
  audio_file_path: "/recordings/chinese_command.wav", 
  language: "zh-CN"
}'
```

**Note**: Behavior tree assembly is now automatic for all requests!

### 3. Example Workflow

1. **Record an audio command** (e.g., "Pick up the red block and place it on the blue table")
2. **Save as audio file** (.wav, .mp3, .m4a, etc.)
3. **Call the service** with the file path
4. **Monitor the results:**
   - Check transcribed text accuracy
   - Review generated behavior tree JSON
   - Watch robot execution (automatic)

## Supported Audio Formats

- **Primary:** WAV, FLAC, AIFF, AIFF-C
- **Converted:** MP3, M4A, OGG (automatically converted to WAV via pydub)

## Example Voice Commands

### Basic Commands
- "Pick up the object"
- "Move to home position" 
- "Open the gripper"
- "Close the gripper"

### Complex Commands
- "Pick up the red cube and place it at coordinates 0.15, -0.15, 0.18"
- "Sort the objects by color: red blocks go to the red zone, blue blocks go to the blue zone"
- "Detect all objects, pick them up one by one, and stack them at location 0.20, -0.10, 0.18"
- "Test the gripper by opening and closing it, then return to home"

### Multi-language Support
- **English:** "Pick up the red block"
- **Chinese:** "拿起红色方块" (zhōngwén: "nā qǐ hóngsè fāngkuài")
- **Spanish:** "Recoge el bloque rojo"
- **French:** "Ramasse le bloc rouge"

## Response Examples

### Successful Response
```json
{
  "success": true,
  "transcribed_text": "Pick up the red cube and place it on the table",
  "behavior_tree_json": "{\"type\": \"sequence\", \"name\": \"MainTask\", \"children\": [...]}",
  "error_message": ""
}
```

### Error Response  
```json
{
  "success": false,
  "transcribed_text": "Could not understand audio",
  "behavior_tree_json": "",
  "error_message": "Speech recognition failed: Could not understand audio"
}
```

## Integration with Robot Workflow

### Complete System Integration
```bash
# Start all required nodes
rosrun behavior_tree behavior_tree_node.py &
rosrun yolo_detection yolo_detection_node.py &
rosrun robot_control robot_control_node.py &
rosrun llm llm_node.py _provider:=gemini &
rosrun speech_recognition speech_recognition_node.py &

# Now you can give voice commands that will be executed by the robot!
rosservice call /process_audio_command '{
  audio_file_path: "/recordings/robot_task.wav",
  language: "en-US"
}'
```

### Monitoring Execution
```bash
# Monitor behavior tree status
rostopic echo /behavior_tree_status

# Monitor YOLO detections
rostopic echo /yolo_detected_targets

# Monitor robot commands
rostopic echo /arm_command
```

## Troubleshooting

### Common Issues

#### 1. Audio File Not Found
```
Error: Audio file not found at /path/to/audio.wav
```
**Solution:** Ensure the audio file exists and the path is correct.

#### 2. Speech Recognition Service Unavailable
```
Error: Speech recognition service not available
```
**Solution:** Start the speech recognition node:
```bash
rosrun speech_recognition speech_recognition_node.py
```

#### 3. LLM Service Not Available
```
Error: LLM service not available
```
**Solution:** Start the LLM node:
```bash
rosrun llm llm_node.py _provider:=gemini
```

#### 4. Google Speech API Errors
```
Error: Could not request results from Google Web Speech API service
```
**Solution:** Check internet connection and ensure Google Speech API is accessible.

#### 5. Audio Format Issues
```
Error: Could not understand audio
```
**Solutions:**
- Ensure clear audio recording
- Try different audio formats
- Check language code is correct
- Reduce background noise

#### 6. Behavior Tree Assembly Failed
```
Error: JSON generated but assembly failed
```
**Solution:** Start the behavior tree node before calling the service:
```bash
rosrun behavior_tree behavior_tree_node.py
```

### Debug Commands

```bash
# Check if services are running
rosservice list | grep -E "(audio|llm|behavior)"

# Test speech recognition node
rosservice call /process_audio_command '{audio_file_path: "/test.wav", language: "en-US"}'

# Check LLM service directly  
rosservice call /generate_behavior_tree 'task_description: "test task"'

# Monitor node output
rosrun speech_recognition speech_recognition_node.py
```

## Performance Tips

1. **Audio Quality:** Use clear, noise-free recordings for best transcription results
2. **File Format:** WAV files generally provide better recognition accuracy
3. **Language Codes:** Use precise language codes (e.g., 'en-US' not 'en')
4. **Command Clarity:** Speak clearly and use robot-relevant vocabulary
5. **File Paths:** Use absolute paths to avoid file not found errors

## Advanced Features

### Custom Language Models
The system supports various language codes for international use:
- `en-US`: English (US)
- `en-GB`: English (UK) 
- `zh-CN`: Chinese (Simplified)
- `zh-TW`: Chinese (Traditional)
- `es-ES`: Spanish
- `fr-FR`: French
- `de-DE`: German
- `ja-JP`: Japanese
- `ko-KR`: Korean

### Integration with Other Nodes
The speech recognition node can be easily integrated with:
- **Robot Control Systems:** Direct robot command execution
- **Vision Systems:** Combined audio-visual task understanding
- **Planning Systems:** High-level task planning from voice commands
- **Human-Robot Interaction:** Natural language robot control interfaces

## Future Enhancements

- **Real-time audio streaming** instead of file-based processing
- **Custom wake words** for hands-free operation  
- **Multi-speaker recognition** for team-based robot control
- **Voice feedback** with text-to-speech responses
- **Continuous listening mode** for ongoing voice control
