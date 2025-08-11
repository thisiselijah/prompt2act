#!/root/.pyenv/versions/3.9.19/bin/python3

import rospy
import speech_recognition as sr
from pydub import AudioSegment
import os

def audio_to_text(audio_file_path, language='en-US'):
    """
    Transforms audio from a given file into text using Google's Web Speech API,
    converting to a compatible format (WAV) if necessary.

    Args:
        audio_file_path (str): The path to the audio file (e.g., .wav, .mp3, .m4a).
        language (str): The language tag for the transcription, e.g., 'en-US' for English, 'zh-CN' for Chinese.

    Returns:
        str: The transcribed text, or an error message if transcription fails.
    """
    recognizer = sr.Recognizer()
    
    # Check if the file needs conversion
    base, ext = os.path.splitext(audio_file_path)
    if ext.lower() not in ['.wav', '.flac', '.aiff', '.aifc']:
        try:
            print(f"Converting {ext} file to WAV format...")
            audio = AudioSegment.from_file(audio_file_path)
            wav_file_path = base + ".wav"
            audio.export(wav_file_path, format="wav")
            audio_file_path = wav_file_path
            print("Conversion successful.")
        except Exception as e:
            return f"Error converting audio file: {e}"

    try:
        with sr.AudioFile(audio_file_path) as source:
            print(f"Processing audio file: {audio_file_path}")
            audio_data = recognizer.record(source)  # read the entire audio file

        print(f"Transcribing audio in {language}...")
        text = recognizer.recognize_google(audio_data, language=language)
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
    # Example usage:
    # Replace 'your_audio_file.wav' with the actual path to your audio file.
    # Supported formats include WAV, AIFF, AIFF-C, FLAC, and others like MP3, M4A (with pydub).
    # For pydub to handle non-WAV files (like MP3, M4A), you need to have ffmpeg or libav installed
    # and accessible in your system's PATH.

    audio_file = input("Please enter the path to your audio file (e.g., /path/to/your_audio.mp3): ")
    language_code = input("Please enter the language code (e.g., en-US for English, zh-CN for Chinese): ") or 'en-US'

    if audio_file:
        transcribed_text = audio_to_text(audio_file, language=language_code)
        print("\n--- Transcribed Text ---")
        print(transcribed_text)
    else:
        print("No audio file path provided. Exiting.")

if __name__ == "__main__":
    main()

