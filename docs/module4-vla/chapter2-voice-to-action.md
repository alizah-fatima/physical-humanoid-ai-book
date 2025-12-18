---
sidebar_position: 2
---

# Chapter 2: Voice-to-Action - Speech Recognition with OpenAI Whisper

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement speech recognition systems using OpenAI Whisper
- Configure and optimize Whisper models for robotics applications
- Process voice commands in real-time for robot control
- Integrate speech recognition with ROS 2 audio systems
- Implement alternative speech recognition systems (Vosk, Coqui STT)
- Design robust voice command processing pipelines
- Apply audio preprocessing techniques for improved recognition

## Introduction to Speech Recognition for Robotics

### The Role of Speech Recognition in VLA Systems

Speech recognition serves as the primary interface between human operators and VLA-enabled robots. It enables natural, hands-free interaction that feels intuitive to users. In robotics applications, speech recognition faces unique challenges:

- **Environmental Noise**: Robots operate in various acoustic environments
- **Real-time Processing**: Commands need to be processed quickly for responsive interaction
- **Domain Specificity**: Robot commands follow specific patterns and vocabularies
- **Safety Criticality**: Misunderstood commands could lead to unsafe robot behavior

### Why OpenAI Whisper for Robotics?

OpenAI Whisper has emerged as a leading choice for robotics applications due to:

- **Multilingual Support**: Works with 99+ languages out of the box
- **Robustness**: Performs well in noisy environments
- **Open Source**: Freely available and modifiable
- **High Accuracy**: State-of-the-art performance across domains
- **Flexibility**: Available in multiple model sizes for different hardware

## OpenAI Whisper Setup and Configuration

### Installation and Dependencies

```bash
# Install Whisper with pip
pip install openai-whisper

# On Ubuntu, install additional dependencies
sudo apt update
sudo apt install ffmpeg python3-pyaudio

# Install additional audio processing libraries
pip install soundfile librosa pyaudio
```

### Basic Whisper Usage

```python
import whisper
import torch

# Check if CUDA is available for GPU acceleration
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load different model sizes based on your needs:
# - "tiny": Fastest, least accurate (75MB)
# - "base": Good balance (145MB)
# - "small": Better accuracy (484MB)
# - "medium": High quality (1.5GB)
# - "large": Highest quality (3.0GB)
model = whisper.load_model("base").to(device)

# Load audio file
audio_path = "path/to/your/audio.wav"
audio = whisper.load_audio(audio_path)
audio = whisper.pad_or_trim(audio)

# Convert to log-Mel spectrogram
mel = whisper.log_mel_spectrogram(audio).to(model.device)

# Detect language (optional)
_, probs = model.detect_language(mel)
detected_language = max(probs, key=probs.get)
print(f"Detected language: {detected_language}")

# Decode audio
options = whisper.DecodingOptions(fp16=torch.cuda.is_available())
result = whisper.decode(model, mel, options)

print(f"Transcribed text: {result.text}")
```

### Whisper Configuration for Robotics

```python
# Advanced Whisper configuration for robotics applications
import whisper
import numpy as np

class WhisperRobotConfig:
    def __init__(self):
        # Model configuration
        self.model_size = "base"  # Balance between speed and accuracy
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Audio processing parameters
        self.sample_rate = 16000  # Standard for speech recognition
        self.audio_chunk_size = 16000  # 1 second of audio at 16kHz

        # Recognition parameters
        self.language = "en"  # Set to specific language for better accuracy
        self.temperature = 0.0  # Deterministic decoding
        self.best_of = 5  # Number of options to generate
        self.beam_size = 5  # Beam search size
        self.patience = 1.0  # Patience factor for beam search

        # Thresholds and filters
        self.confidence_threshold = 0.7  # Minimum confidence for acceptance
        self.min_silence_duration = 0.5  # Minimum silence before processing
        self.max_audio_duration = 10.0  # Maximum audio duration to process

class WhisperRobotInterface:
    def __init__(self, config: WhisperRobotConfig):
        self.config = config
        self.model = whisper.load_model(config.model_size).to(config.device)
        self.is_initialized = False

        # Initialize audio processing components
        self.audio_buffer = np.array([], dtype=np.float32)
        self.last_transcription = ""
        self.confidence_score = 0.0

        self.is_initialized = True
        print(f"Whisper Robot Interface initialized on {config.device}")

    def preprocess_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """Preprocess audio data for Whisper"""
        # Normalize audio to [-1, 1] range
        audio_data = audio_data.astype(np.float32)
        if audio_data.max() != 0:
            audio_data = audio_data / np.abs(audio_data).max()

        # Pad or trim to appropriate length
        target_length = self.config.audio_chunk_size
        if len(audio_data) < target_length:
            # Pad with zeros
            audio_data = np.pad(audio_data, (0, target_length - len(audio_data)), 'constant')
        elif len(audio_data) > target_length:
            # Trim to target length
            audio_data = audio_data[:target_length]

        return audio_data

    def transcribe_audio(self, audio_data: np.ndarray) -> tuple[str, float]:
        """Transcribe audio using Whisper with confidence scoring"""
        try:
            # Preprocess audio
            processed_audio = self.preprocess_audio(audio_data)

            # Convert to tensor and move to device
            audio_tensor = torch.from_numpy(processed_audio).to(self.config.device)

            # Pad or trim audio to required length
            audio_tensor = whisper.pad_or_trim(audio_tensor)

            # Compute log-Mel spectrogram
            mel = whisper.log_mel_spectrogram(audio_tensor).to(self.config.device)

            # Decode with specific options for robotics
            options = whisper.DecodingOptions(
                language=self.config.language,
                temperature=self.config.temperature,
                best_of=self.config.best_of,
                beam_size=self.config.beam_size,
                patience=self.config.patience,
                fp16=torch.cuda.is_available() if self.config.device == "cuda" else False
            )

            result = whisper.decode(self.model, mel, options)

            # Calculate confidence (simplified approach)
            # In practice, you might use more sophisticated confidence measures
            confidence = self.calculate_confidence(result)

            return result.text.strip(), confidence

        except Exception as e:
            print(f"Error in transcription: {e}")
            return "", 0.0

    def calculate_confidence(self, result) -> float:
        """Calculate confidence score for transcription"""
        # This is a simplified confidence calculation
        # In practice, you might use log probabilities or other metrics
        if hasattr(result, 'avg_logprob') and result.avg_logprob is not None:
            # Convert log probability to confidence (simple normalization)
            # Higher logprob = more confidence
            logprob = result.avg_logprob
            # Normalize logprob to 0-1 range (empirical scaling)
            confidence = max(0.0, min(1.0, (logprob + 2.0) / 2.0))
            return confidence
        else:
            # Fallback confidence calculation
            return 0.5  # Neutral confidence

    def is_command_valid(self, text: str, confidence: float) -> bool:
        """Validate if transcription is a valid command"""
        if not text.strip():
            return False

        if confidence < self.config.confidence_threshold:
            return False

        # Additional validation could include:
        # - Checking if text contains robot command keywords
        # - Verifying text length is reasonable
        # - Checking for profanity or inappropriate content

        return True
```

## Real-Time Speech Recognition for Robotics

### Continuous Audio Processing

```python
import pyaudio
import threading
import queue
import time
from dataclasses import dataclass
from typing import Callable, Optional

@dataclass
class SpeechRecognitionResult:
    """Result from speech recognition"""
    text: str
    confidence: float
    is_valid: bool
    timestamp: float
    processing_time: float

class RealTimeWhisperRecognizer:
    def __init__(self, whisper_interface: WhisperRobotInterface):
        self.whisper_interface = whisper_interface
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # Audio stream parameters
        self.chunk_size = 1024  # Frames per buffer
        self.sample_rate = 16000
        self.channels = 1
        self.format = pyaudio.paInt16

        # Processing state
        self.is_listening = False
        self.audio_thread = None
        self.processing_thread = None

        # Callback for processing results
        self.result_callback: Optional[Callable[[SpeechRecognitionResult], None]] = None

    def start_listening(self, result_callback: Callable[[SpeechRecognitionResult], None] = None):
        """Start real-time listening"""
        self.result_callback = result_callback
        self.is_listening = True

        # Start audio recording thread
        self.audio_thread = threading.Thread(target=self._record_audio, daemon=True)
        self.audio_thread.start()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self._process_audio, daemon=True)
        self.processing_thread.start()

        print("Started real-time listening")

    def stop_listening(self):
        """Stop real-time listening"""
        self.is_listening = False
        if self.audio_thread:
            self.audio_thread.join(timeout=1.0)
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
        print("Stopped real-time listening")

    def _record_audio(self):
        """Record audio in a separate thread"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        print("Recording started...")

        while self.is_listening:
            try:
                data = stream.read(self.chunk_size, exception_on_overflow=False)

                # Convert to numpy array
                audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                audio_data = audio_data / 32768.0  # Normalize to [-1, 1]

                # Add to processing queue
                self.audio_queue.put(audio_data)

            except Exception as e:
                print(f"Error recording audio: {e}")
                break

        stream.stop_stream()
        stream.close()
        p.terminate()

    def _process_audio(self):
        """Process audio chunks in a separate thread"""
        audio_buffer = np.array([], dtype=np.float32)
        buffer_duration = 0.0
        min_speech_duration = 1.0  # Minimum speech duration to process
        silence_threshold = 0.01  # Threshold for detecting silence

        while self.is_listening:
            try:
                # Get audio data from queue
                audio_chunk = self.audio_queue.get(timeout=0.1)

                # Add to buffer
                audio_buffer = np.concatenate([audio_buffer, audio_chunk])
                buffer_duration = len(audio_buffer) / self.sample_rate

                # Check if buffer has enough data and contains speech
                if buffer_duration >= min_speech_duration:
                    # Check if this is likely speech (not just silence)
                    if np.max(np.abs(audio_buffer)) > silence_threshold:
                        # Process this chunk
                        start_time = time.time()

                        # Transcribe audio
                        text, confidence = self.whisper_interface.transcribe_audio(audio_buffer)

                        # Check if it's a valid command
                        is_valid = self.whisper_interface.is_command_valid(text, confidence)

                        # Create result
                        result = SpeechRecognitionResult(
                            text=text,
                            confidence=confidence,
                            is_valid=is_valid,
                            timestamp=time.time(),
                            processing_time=time.time() - start_time
                        )

                        # Call result callback if provided
                        if self.result_callback:
                            self.result_callback(result)

                    # Clear buffer after processing
                    audio_buffer = np.array([], dtype=np.float32)
                    buffer_duration = 0.0
                else:
                    # Continue accumulating audio
                    continue

            except queue.Empty:
                # No audio data available, continue
                continue
            except Exception as e:
                print(f"Error processing audio: {e}")
                break
```

### Wake Word Detection

```python
import speech_recognition as sr
import threading

class WakeWordDetector:
    def __init__(self, wake_word="robot"):
        self.wake_word = wake_word.lower()
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.is_awake = False
        self.wake_callback = None

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def set_wake_callback(self, callback: Callable[[], None]):
        """Set callback for when wake word is detected"""
        self.wake_callback = callback

    def start_wake_word_detection(self):
        """Start wake word detection in background"""
        def detect_wake_word():
            while True:
                try:
                    with self.microphone as source:
                        # Listen for audio with timeout
                        audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=2.0)

                    # Recognize speech
                    text = self.recognizer.recognize_google(audio).lower()

                    # Check for wake word
                    if self.wake_word in text:
                        print(f"Wake word '{self.wake_word}' detected!")
                        self.is_awake = True

                        # Call wake callback if set
                        if self.wake_callback:
                            self.wake_callback()

                        # Reset awake state after a delay
                        threading.Timer(5.0, self._reset_awake_state).start()

                except sr.WaitTimeoutError:
                    # No speech detected, continue listening
                    continue
                except sr.UnknownValueError:
                    # Could not understand audio, continue
                    continue
                except sr.RequestError:
                    # Error with speech recognition service
                    print("Speech recognition service error")
                    time.sleep(1)
                    continue
                except Exception as e:
                    print(f"Error in wake word detection: {e}")
                    time.sleep(1)
                    continue

        # Start wake word detection in background thread
        thread = threading.Thread(target=detect_wake_word, daemon=True)
        thread.start()

    def _reset_awake_state(self):
        """Reset awake state after timeout"""
        self.is_awake = False
```

## Alternative Speech Recognition Systems

### Vosk Integration

Vosk is an excellent alternative for robotics applications due to its lightweight nature and offline capabilities:

```python
# Install Vosk: pip install vosk
from vosk import Model, KaldiRecognizer
import json
import pyaudio

class VoskRecognizer:
    def __init__(self, model_path: str = "model"):
        """
        Initialize Vosk recognizer
        Download model from: https://alphacephei.com/vosk/models
        """
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.is_initialized = True

        # Audio stream parameters
        self.chunk_size = 8192
        self.sample_rate = 16000
        self.channels = 1
        self.format = pyaudio.paInt16

    def recognize_continuous(self, audio_callback: Callable[[str], None]):
        """Continuous speech recognition"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        print("Vosk listening... Press Ctrl+C to stop")

        try:
            while True:
                data = stream.read(self.chunk_size)
                if len(data) == 0:
                    break

                if self.recognizer.AcceptWaveform(data):
                    result = self.recognizer.Result()
                    result_dict = json.loads(result)

                    if 'text' in result_dict and result_dict['text']:
                        recognized_text = result_dict['text']
                        audio_callback(recognized_text)

        except KeyboardInterrupt:
            print("\nStopping Vosk recognition...")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    def recognize_single_phrase(self, timeout: float = 5.0) -> str:
        """Recognize a single phrase with timeout"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        start_time = time.time()
        result_text = ""

        try:
            while time.time() - start_time < timeout:
                data = stream.read(self.chunk_size)

                if self.recognizer.AcceptWaveform(data):
                    result = self.recognizer.Result()
                    result_dict = json.loads(result)

                    if 'text' in result_dict and result_dict['text']:
                        result_text = result_dict['text']
                        break

        except Exception as e:
            print(f"Error in Vosk recognition: {e}")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

        return result_text
```

### Coqui STT Integration

Coqui STT (formerly Mozilla DeepSpeech) offers another open-source option:

```python
# Install Coqui STT: pip install coqui-stt
from stt import Model
import numpy as np
import wave
import subprocess

class CoquiRecognizer:
    def __init__(self, model_path: str, scorer_path: str = None):
        """
        Initialize Coqui STT recognizer
        Download model from: https://coqui.ai/models
        """
        self.model = Model(model_path)

        if scorer_path:
            self.model.enableExternalScorer(scorer_path)

        # Set beam width for better accuracy
        self.model.setBeamWidth(1024)

        self.is_initialized = True

    def transcribe_audio_file(self, audio_file_path: str) -> str:
        """Transcribe audio file using Coqui STT"""
        # Load audio file
        fin = wave.open(audio_file_path, 'rb')
        audio = np.frombuffer(fin.readframes(fin.getnframes()), np.int16)
        fin.close()

        # Transcribe
        text = self.model.stt(audio)
        return text

    def transcribe_audio_data(self, audio_data: np.ndarray) -> str:
        """Transcribe audio data (numpy array)"""
        # Ensure audio data is in the right format (16-bit integers)
        if audio_data.dtype != np.int16:
            # Convert to 16-bit if needed
            audio_data = (audio_data * 32767).astype(np.int16)

        # Transcribe
        text = self.model.stt(audio_data)
        return text

    def transcribe_stream(self, audio_generator):
        """Transcribe audio in streaming mode"""
        # Initialize streaming
        stream = self.model.createStream()

        for audio_chunk in audio_generator:
            # Feed audio chunk to stream
            stream.feedAudioContent(audio_chunk)

        # Finish and get result
        text = stream.finishStream()
        return text
```

## Audio Preprocessing for Robotics

### Noise Reduction and Enhancement

```python
import librosa
import numpy as np
from scipy import signal

class AudioPreprocessor:
    def __init__(self):
        self.sample_rate = 16000
        self.frame_length = 2048
        self.hop_length = 512

    def denoise_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """Reduce noise in audio signal"""
        # Apply spectral gating for noise reduction
        # This is a simplified approach - more sophisticated methods exist

        # Compute STFT
        stft = librosa.stft(audio_data, n_fft=self.frame_length, hop_length=self.hop_length)

        # Compute magnitude and phase
        magnitude = np.abs(stft)
        phase = np.angle(stft)

        # Estimate noise profile (using first portion of audio as noise reference)
        noise_profile = np.mean(magnitude[:, :50], axis=1, keepdims=True)  # First 50 frames

        # Apply spectral subtraction
        enhanced_magnitude = np.maximum(magnitude - noise_profile * 0.3, 0)  # 30% noise reduction

        # Reconstruct signal
        enhanced_stft = enhanced_magnitude * np.exp(1j * phase)
        enhanced_audio = librosa.istft(enhanced_stft, hop_length=self.hop_length)

        return enhanced_audio.astype(np.float32)

    def enhance_speech(self, audio_data: np.ndarray) -> np.ndarray:
        """Enhance speech signal using various techniques"""
        # Apply pre-emphasis filter
        enhanced = self.pre_emphasis_filter(audio_data)

        # Normalize amplitude
        enhanced = self.normalize_audio(enhanced)

        # Apply noise reduction
        enhanced = self.denoise_audio(enhanced)

        return enhanced

    def pre_emphasis_filter(self, audio_data: np.ndarray, coeff: float = 0.97) -> np.ndarray:
        """Apply pre-emphasis filter to enhance high frequencies"""
        return np.append(audio_data[0], audio_data[1:] - coeff * audio_data[:-1])

    def normalize_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """Normalize audio to unit amplitude"""
        max_val = np.max(np.abs(audio_data))
        if max_val > 0:
            return audio_data / max_val
        return audio_data

    def vad_simple(self, audio_data: np.ndarray, threshold_db: float = -30) -> np.ndarray:
        """Simple voice activity detection"""
        # Convert to dB
        rms = np.sqrt(np.mean(audio_data**2, axis=0))
        db = 20 * np.log10(rms + 1e-10)  # Add small value to avoid log(0)

        # Create mask for speech regions
        speech_mask = db > threshold_db

        # Apply mask to audio
        return audio_data * speech_mask

    def bandpass_filter(self, audio_data: np.ndarray, low_freq: float = 300, high_freq: float = 3400) -> np.ndarray:
        """Apply bandpass filter to focus on speech frequencies"""
        nyquist = self.sample_rate / 2
        low = low_freq / nyquist
        high = high_freq / nyquist

        # Design Butterworth bandpass filter
        b, a = signal.butter(4, [low, high], btype='band', analog=False)

        # Apply filter
        filtered_audio = signal.filtfilt(b, a, audio_data)

        return filtered_audio.astype(np.float32)
```

### Real-Time Audio Processing Pipeline

```python
class RealTimeAudioProcessor:
    def __init__(self):
        self.preprocessor = AudioPreprocessor()
        self.buffer_size = 4096  # Samples
        self.overlap_ratio = 0.5  # 50% overlap
        self.min_speech_energy = 0.001  # Minimum energy threshold

        # Circular buffer for real-time processing
        self.audio_buffer = np.zeros(self.buffer_size * 2, dtype=np.float32)
        self.buffer_fill_level = 0

    def process_audio_chunk(self, audio_chunk: np.ndarray) -> np.ndarray:
        """Process a chunk of audio in real-time"""
        # Add new audio to buffer
        chunk_size = len(audio_chunk)

        if self.buffer_fill_level + chunk_size > len(self.audio_buffer):
            # Shift buffer to make room
            shift_amount = self.buffer_fill_level + chunk_size - len(self.audio_buffer)
            self.audio_buffer[:-shift_amount] = self.audio_buffer[shift_amount:]
            self.buffer_fill_level -= shift_amount

        # Add new chunk to buffer
        self.audio_buffer[self.buffer_fill_level:self.buffer_fill_level + chunk_size] = audio_chunk
        self.buffer_fill_level += chunk_size

        # Process if we have enough audio
        if self.buffer_fill_level >= self.buffer_size:
            # Process the oldest complete buffer
            processing_buffer = self.audio_buffer[:self.buffer_size].copy()

            # Apply preprocessing
            processed_audio = self.preprocessor.enhance_speech(processing_buffer)

            # Remove processed portion from buffer
            self.audio_buffer[:-self.buffer_size] = self.audio_buffer[self.buffer_size:]
            self.buffer_fill_level -= self.buffer_size

            return processed_audio

        return np.array([])  # Not enough audio to process yet

    def detect_speech_activity(self, audio_data: np.ndarray) -> bool:
        """Detect if speech is present in audio"""
        # Calculate energy
        energy = np.mean(audio_data**2)

        # Check if energy exceeds threshold
        return energy > self.min_speech_energy
```

## ROS 2 Integration for Audio Processing

### Audio Message Processing

```python
# ROS 2 node for speech recognition integration
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from audio_common_msgs.msg import AudioData as AudioDataMsg
import numpy as np
from typing import Optional

class SpeechRecognitionROSNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')

        # Initialize Whisper interface
        config = WhisperRobotConfig()
        self.whisper_interface = WhisperRobotInterface(config)

        # Initialize audio preprocessor
        self.audio_preprocessor = RealTimeAudioProcessor()

        # Initialize wake word detector
        self.wake_word_detector = WakeWordDetector(wake_word="hey robot")

        # ROS 2 publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData, '/audio_input', self.audio_callback, 10)

        self.command_pub = self.create_publisher(
            String, '/robot_voice_commands', 10)

        self.transcription_pub = self.create_publisher(
            String, '/transcriptions', 10)

        # State variables
        self.is_listening_for_command = False
        self.command_timeout = self.create_timer(5.0, self.command_timeout_callback)

        # Wake word detection
        self.wake_word_detector.set_wake_callback(self.wake_detected_callback)
        self.wake_word_detector.start_wake_word_detection()

        self.get_logger().info('Speech Recognition ROS Node initialized')

    def audio_callback(self, msg: AudioData):
        """Process incoming audio messages"""
        try:
            # Convert ROS AudioData to numpy array
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32)
            audio_data = audio_data / 32768.0  # Normalize to [-1, 1]

            # Preprocess audio
            processed_audio = self.audio_preprocessor.process_audio_chunk(audio_data)

            if len(processed_audio) > 0:
                # Check for speech activity
                has_speech = self.audio_preprocessor.detect_speech_activity(processed_audio)

                if has_speech:
                    if self.is_listening_for_command:
                        # Transcribe the audio
                        text, confidence = self.whisper_interface.transcribe_audio(processed_audio)

                        if text and confidence > 0.7:
                            # Publish transcription
                            transcription_msg = String()
                            transcription_msg.data = text
                            self.transcription_pub.publish(transcription_msg)

                            # Publish as robot command
                            command_msg = String()
                            command_msg.data = text
                            self.command_pub.publish(command_msg)

                            self.get_logger().info(f'Transcribed: "{text}" (conf: {confidence:.2f})')

                            # Reset listening state
                            self.is_listening_for_command = False
                        else:
                            self.get_logger().info(f'Low confidence transcription: "{text}" (conf: {confidence:.2f})')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def wake_detected_callback(self):
        """Callback when wake word is detected"""
        self.get_logger().info('Wake word detected - ready for command')
        self.is_listening_for_command = True

        # Reset command timeout
        self.reset_command_timeout()

    def command_timeout_callback(self):
        """Timer callback for command timeout"""
        if self.is_listening_for_command:
            self.get_logger().info('Command timeout - stopped listening')
            self.is_listening_for_command = False

    def reset_command_timeout(self):
        """Reset the command timeout timer"""
        # Cancel current timer and start new one
        self.command_timeout.reset()

def main(args=None):
    rclpy.init(args=args)
    speech_node = SpeechRecognitionROSNode()

    try:
        rclpy.spin(speech_node)
    except KeyboardInterrupt:
        pass
    finally:
        speech_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Processing Pipeline

### Command Validation and Filtering

```python
import re
from typing import List, Dict, Tuple

class VoiceCommandValidator:
    def __init__(self):
        # Define valid command patterns
        self.valid_patterns = [
            # Navigation commands
            r'go to (the )?\w+',
            r'move to (the )?\w+',
            r'go (straight|left|right)',
            r'turn (left|right)',

            # Manipulation commands
            r'(pick|grasp|take) (the )?\w+',
            r'(place|put) (the )?\w+',
            r'pick up (the )?\w+',

            # Interaction commands
            r'greet (the )?\w+',
            r'say hello to (the )?\w+',
            r'tell (the )?\w+',

            # Query commands
            r'where is (the )?\w+',
            r'find (the )?\w+',
            r'look for (the )?\w+'
        ]

        # Define prohibited content patterns
        self.prohibited_patterns = [
            r'\b(hate|kill|destroy|break|attack)\b',
            r'\b(inappropriate|offensive|profanity)\b'
        ]

        # Confidence thresholds
        self.min_confidence = 0.6
        self.command_length_min = 2
        self.command_length_max = 50

    def validate_command(self, text: str, confidence: float) -> Tuple[bool, str]:
        """Validate voice command"""
        if not text or not text.strip():
            return False, "Empty command"

        if confidence < self.min_confidence:
            return False, f"Low confidence: {confidence:.2f}"

        text_clean = text.strip().lower()

        if len(text_clean) < self.command_length_min:
            return False, "Command too short"

        if len(text_clean) > self.command_length_max:
            return False, "Command too long"

        # Check for prohibited content
        for pattern in self.prohibited_patterns:
            if re.search(pattern, text_clean):
                return False, "Command contains prohibited content"

        # Check if command matches valid patterns
        is_valid = any(re.match(pattern, text_clean) for pattern in self.valid_patterns)

        if not is_valid:
            return False, "Command doesn't match expected patterns"

        return True, "Valid command"

    def preprocess_command(self, text: str) -> str:
        """Preprocess command text"""
        # Remove punctuation and normalize
        text = re.sub(r'[^\w\s]', ' ', text)
        text = ' '.join(text.split())  # Remove extra whitespace
        text = text.strip()

        return text
```

### Command Queue and Processing Manager

```python
import asyncio
import queue
from dataclasses import dataclass
from typing import Optional, Callable
import time

@dataclass
class VoiceCommand:
    """Represents a voice command with metadata"""
    text: str
    confidence: float
    timestamp: float
    source: str  # 'voice', 'text', etc.
    priority: int = 0  # Higher number = higher priority

class VoiceCommandManager:
    def __init__(self, max_queue_size: int = 10):
        self.command_queue = asyncio.Queue(maxsize=max_queue_size)
        self.processed_commands = queue.Queue(maxsize=100)  # For history
        self.is_processing = False
        self.validator = VoiceCommandValidator()

        # Callbacks
        self.command_processed_callback: Optional[Callable[[VoiceCommand], None]] = None
        self.command_rejected_callback: Optional[Callable[[str, str], None]] = None

    async def add_command(self, text: str, confidence: float, source: str = 'voice') -> bool:
        """Add a command to the processing queue"""
        try:
            # Validate command
            is_valid, reason = self.validator.validate_command(text, confidence)

            if not is_valid:
                if self.command_rejected_callback:
                    self.command_rejected_callback(text, reason)
                return False

            # Preprocess command
            processed_text = self.validator.preprocess_command(text)

            # Create command object
            command = VoiceCommand(
                text=processed_text,
                confidence=confidence,
                timestamp=time.time(),
                source=source
            )

            # Add to queue
            await self.command_queue.put(command)
            return True

        except asyncio.QueueFull:
            print("Command queue is full")
            return False

    async def start_processing(self):
        """Start processing commands from the queue"""
        self.is_processing = True

        while self.is_processing:
            try:
                # Get command from queue (with timeout)
                command = await asyncio.wait_for(self.command_queue.get(), timeout=1.0)

                # Process command
                await self.process_command(command)

                # Add to history
                if not self.processed_commands.full():
                    self.processed_commands.put(command)

                # Mark as processed
                self.command_queue.task_done()

            except asyncio.TimeoutError:
                # No command available, continue
                continue
            except Exception as e:
                print(f"Error processing command: {e}")

    async def process_command(self, command: VoiceCommand):
        """Process a single command"""
        print(f"Processing command: '{command.text}' (confidence: {command.confidence:.2f})")

        # Call processing callback if available
        if self.command_processed_callback:
            self.command_processed_callback(command)

    def stop_processing(self):
        """Stop processing commands"""
        self.is_processing = False

    def get_command_history(self) -> List[VoiceCommand]:
        """Get command processing history"""
        history = []
        temp_queue = queue.Queue()

        # Copy queue contents
        while not self.processed_commands.empty():
            cmd = self.processed_commands.get()
            history.append(cmd)
            temp_queue.put(cmd)

        # Restore queue
        while not temp_queue.empty():
            self.processed_commands.put(temp_queue.get())

        return history
```

## Performance Optimization

### Model Optimization Techniques

```python
import torch
from transformers import WhisperForConditionalGeneration, WhisperProcessor
import onnxruntime as ort

class OptimizedWhisperInference:
    def __init__(self, model_path: str, optimize_for_device: str = "gpu"):
        """
        Initialize optimized Whisper inference
        """
        self.optimize_for_device = optimize_for_device

        if optimize_for_device == "cpu":
            # For CPU optimization, consider using ONNX runtime
            self.use_onnx = True
            self.onnx_session = self._load_onnx_model(model_path)
        else:
            # For GPU, use PyTorch with optimizations
            self.use_onnx = False
            self.model = WhisperForConditionalGeneration.from_pretrained(model_path)
            self.processor = WhisperProcessor.from_pretrained(model_path)

            # Move to device and optimize
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.model = self.model.to(device)

            # Apply optimizations
            if torch.cuda.is_available():
                self.model = torch.compile(self.model)  # PyTorch 2.0+ optimization

    def _load_onnx_model(self, model_path: str):
        """Load ONNX model for CPU optimization"""
        # This would typically involve converting the model to ONNX format first
        # For demonstration, assuming ONNX model exists
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 4  # Limit CPU threads
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        if self.optimize_for_device == "gpu" and "CUDAExecutionProvider" in ort.get_available_providers():
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        else:
            providers = ["CPUExecutionProvider"]

        return ort.InferenceSession(f"{model_path}/model.onnx", sess_options=session_options, providers=providers)

    def transcribe(self, audio_input) -> str:
        """Transcribe audio with optimized inference"""
        if self.use_onnx:
            return self._transcribe_onnx(audio_input)
        else:
            return self._transcribe_pytorch(audio_input)

    def _transcribe_pytorch(self, audio_input):
        """PyTorch-based transcription"""
        # Process audio
        inputs = self.processor(audio_input, sampling_rate=16000, return_tensors="pt")
        inputs = {k: v.to(self.model.device) for k, v in inputs.items()}

        # Generate
        with torch.no_grad():
            predicted_ids = self.model.generate(**inputs)

        # Decode
        transcription = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0]
        return transcription

    def _transcribe_onnx(self, audio_input):
        """ONNX-based transcription"""
        # Process audio for ONNX model
        # This is a simplified example - actual implementation would depend on model structure
        inputs = self.processor(audio_input, sampling_rate=16000, return_tensors="np")

        # Run inference
        outputs = self.onnx_session.run(None, {"input_features": inputs["input_features"]})

        # Decode outputs
        transcription = self.processor.batch_decode(outputs[0], skip_special_tokens=True)[0]
        return transcription

    def warm_up(self):
        """Warm up the model with dummy input"""
        dummy_audio = np.random.randn(16000)  # 1 second of dummy audio
        try:
            self.transcribe(dummy_audio)
            print("Model warmed up successfully")
        except Exception as e:
            print(f"Error warming up model: {e}")
```

## Troubleshooting Common Issues

### Audio Quality Issues

1. **Poor Recognition Accuracy**:
   - Check microphone quality and positioning
   - Ensure proper audio preprocessing
   - Verify sample rate matches model expectations (16kHz)
   - Consider using noise reduction techniques

2. **High Latency**:
   - Use smaller Whisper models for faster inference
   - Optimize audio chunk sizes
   - Consider using GPU acceleration
   - Implement streaming recognition

3. **Wake Word False Positives**:
   - Adjust sensitivity thresholds
   - Use more distinctive wake words
   - Implement acoustic modeling for wake word detection

### Common Error Solutions

```python
# Error handling and recovery strategies
class SpeechRecognitionErrorHandler:
    def __init__(self):
        self.error_counts = {}
        self.recovery_strategies = {
            'MicrophoneError': self.handle_microphone_error,
            'ModelLoadError': self.handle_model_load_error,
            'RecognitionError': self.handle_recognition_error,
            'MemoryError': self.handle_memory_error
        }

    def handle_error(self, error_type: str, error_details: str):
        """Handle specific error types with appropriate recovery"""
        self.error_counts[error_type] = self.error_counts.get(error_type, 0) + 1

        if error_type in self.recovery_strategies:
            return self.recovery_strategies[error_type](error_details)
        else:
            print(f"Unknown error type: {error_type}")
            return False

    def handle_microphone_error(self, details: str):
        """Handle microphone-related errors"""
        print(f"Microphone error: {details}")
        # Try to reinitialize audio input
        # Switch to alternative audio source if available
        return True

    def handle_model_load_error(self, details: str):
        """Handle model loading errors"""
        print(f"Model load error: {details}")
        # Try to load alternative model
        # Use fallback recognition system
        return True

    def handle_recognition_error(self, details: str):
        """Handle recognition errors"""
        print(f"Recognition error: {details}")
        # Continue with next audio chunk
        return True

    def handle_memory_error(self, details: str):
        """Handle memory errors"""
        print(f"Memory error: {details}")
        # Clear audio buffers
        # Use smaller model temporarily
        return True
```

## Summary

This chapter covered speech recognition for VLA systems:

- **OpenAI Whisper setup** and configuration for robotics applications
- **Real-time processing** techniques for continuous speech recognition
- **Alternative systems** like Vosk and Coqui STT for different use cases
- **Audio preprocessing** for improved recognition quality
- **ROS 2 integration** for robotics applications
- **Voice command processing** pipelines with validation and queuing
- **Performance optimization** techniques for efficient operation
- **Troubleshooting strategies** for common issues

The ability to process voice commands in real-time is crucial for natural human-robot interaction. With proper implementation of speech recognition systems, robots can respond to natural language commands and integrate seamlessly into human environments. The next chapter will explore cognitive planning, which translates these voice commands into sequences of robot actions.