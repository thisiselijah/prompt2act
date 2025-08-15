#!/root/.pyenv/versions/3.9.19/bin/python3.9

import sys
print(sys.executable) 

import rospy
import speech_recognition as sr
from pydub import AudioSegment
import os
from speech_recognition.srv import ProcessAudioCommand, ProcessAudioCommandResponse
from llm.srv import GenerateBehaviorTree, GenerateBehaviorTreeRequest

class SpeechRecognitionNode:
    """ROS node for speech recognition that integrates with LLM for behavior tree generation"""
    
    def __init__(self):
        """Initialize the speech recognition node"""
        self.recognizer = sr.Recognizer()
        
        # Service client for LLM behavior tree generation
        self.llm_client = None
        self._initialize_llm_client()
        
        # Create the speech recognition service
        self.audio_command_service = rospy.Service(
            '/process_audio_command', 
            ProcessAudioCommand, 
            self.process_audio_command_callback
        )
        
        rospy.loginfo("Speech Recognition Node initialized")
        rospy.loginfo("Audio command processing service available at /process_audio_command")
    
    def _initialize_llm_client(self):
        """Initialize the LLM service client"""
        try:
            rospy.loginfo("Waiting for LLM behavior tree generation service...")
            rospy.wait_for_service('/generate_behavior_tree', timeout=5.0)
            self.llm_client = rospy.ServiceProxy('/generate_behavior_tree', GenerateBehaviorTree)
            rospy.loginfo("LLM service client initialized")
        except rospy.ROSException as e:
            rospy.logwarn(f"LLM service not available: {e}")
            self.llm_client = None
    
    def _call_llm_service(self, task_description):
        """
        Call the LLM service to generate behavior tree from task description
        Args:
            task_description (str): The transcribed text to use as task description
        Returns:
            tuple: (success, behavior_tree_json, error_message)
        """
        if not self.llm_client:
            # Try to reinitialize the client
            self._initialize_llm_client()
            if not self.llm_client:
                return False, "", "LLM service not available"
        
        try:
            request = GenerateBehaviorTreeRequest()
            request.task_description = task_description
            response = self.llm_client(request)
            return response.success, response.behavior_tree_json, response.error_message
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call LLM service: {e}")
            return False, "", f"LLM service call failed: {str(e)}"
    
    def process_audio_command_callback(self, req):
        """
        Service callback for processing audio commands and generating behavior trees
        Args:
            req: ProcessAudioCommand service request containing audio_file_path and language
        Returns:
            ProcessAudioCommandResponse: Response with success status, transcribed text, behavior tree JSON, and error message
        """
        try:
            audio_file_path = req.audio_file_path
            language = req.language if req.language else 'en-US'
            
            rospy.loginfo(f"Processing audio command from: {audio_file_path} (language: {language})")
            
            # Step 1: Convert audio to text
            transcribed_text = self.audio_to_text(audio_file_path, language)
            
            # Check if transcription was successful
            if transcribed_text.startswith("Error") or transcribed_text.startswith("Could not"):
                rospy.logwarn(f"Transcription failed: {transcribed_text}")
                return ProcessAudioCommandResponse(
                    success=False,
                    transcribed_text=transcribed_text,
                    behavior_tree_json="",
                    error_message=f"Speech recognition failed: {transcribed_text}"
                )
            
            rospy.loginfo(f"Successfully transcribed audio: {transcribed_text}")
            
            # Step 2: Generate behavior tree from transcribed text using LLM service (auto-assembly is now automatic)
            llm_success, behavior_tree_json, llm_error = self._call_llm_service(transcribed_text)
            
            if not llm_success:
                rospy.logerr(f"LLM behavior tree generation failed: {llm_error}")
                return ProcessAudioCommandResponse(
                    success=False,
                    transcribed_text=transcribed_text,
                    behavior_tree_json="",
                    error_message=f"Behavior tree generation failed: {llm_error}"
                )
            
            rospy.loginfo(f"Successfully generated and assembled behavior tree from audio command")
            
            return ProcessAudioCommandResponse(
                success=True,
                transcribed_text=transcribed_text,
                behavior_tree_json=behavior_tree_json,
                error_message=""
            )
                
        except Exception as e:
            error_msg = f"Unexpected error during audio command processing: {str(e)}"
            rospy.logerr(error_msg)
            return ProcessAudioCommandResponse(
                success=False,
                transcribed_text="",
                behavior_tree_json="",
                error_message=error_msg
            )

    def audio_to_text(self, audio_file_path, language='en-US'):
        """
        Transforms audio from a given file into text using Google's Web Speech API,
        converting to a compatible format (WAV) if necessary.

        Args:
            audio_file_path (str): The path to the audio file (e.g., .wav, .mp3, .m4a).
            language (str): The language tag for the transcription, e.g., 'en-US' for English, 'zh-CN' for Chinese.

        Returns:
            str: The transcribed text, or an error message if transcription fails.
        """
        # Check if the file needs conversion
        base, ext = os.path.splitext(audio_file_path)
        if ext.lower() not in ['.wav', '.flac', '.aiff', '.aifc']:
            try:
                rospy.loginfo(f"Converting {ext} file to WAV format...")
                audio = AudioSegment.from_file(audio_file_path)
                wav_file_path = base + ".wav"
                audio.export(wav_file_path, format="wav")
                audio_file_path = wav_file_path
                rospy.loginfo("Audio conversion successful.")
            except Exception as e:
                return f"Error converting audio file: {e}"

        try:
            with sr.AudioFile(audio_file_path) as source:
                rospy.loginfo(f"Processing audio file: {audio_file_path}")
                audio_data = self.recognizer.record(source)  # read the entire audio file

            rospy.loginfo(f"Transcribing audio in {language}...")
            text = self.recognizer.recognize_google(audio_data, language=language)
            return text

        except sr.UnknownValueError:
            return "Could not understand audio"
        except sr.RequestError as e:
            return f"Could not request results from Google Web Speech API service; {e}"
        except FileNotFoundError:
            return f"Error: Audio file not found at {audio_file_path}"
        except Exception as e:
            return f"An unexpected error occurred: {e}"

def main():
    """
    Main function to initialize the ROS node and start the speech recognition service
    """
    # Initialize the ROS node
    rospy.init_node('speech_recognition_node', anonymous=True)
    
    # Create speech recognition node instance
    speech_node = SpeechRecognitionNode()
    
    rospy.loginfo("Speech Recognition Node is ready to process audio commands")
    rospy.loginfo("Call service with: rosservice call /process_audio_command '{audio_file_path: \"/path/to/audio.wav\", language: \"en-US\"}'")
    rospy.loginfo("This will:")
    rospy.loginfo("  1. Convert audio to text using Google Speech Recognition")
    rospy.loginfo("  2. Use transcribed text as task_description for LLM")
    rospy.loginfo("  3. Generate and automatically assemble behavior tree")
    
    # Keep the node running
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Speech Recognition Node shutting down...")

if __name__ == "__main__":
    main()

