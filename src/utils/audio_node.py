# #!/home/viju/ros_ws/venv/bin/python3

# import rclpy
# # from rclpy.node import Node
# from std_msgs.msg import String, Int16MultiArray
# from rclpy.logging import get_logger
# import numpy as np
# import pyaudio
# import vosk
# import json
# import threading
# import os

# class AudioProcessingNode():
#     def __init__(self):
#         # super().__init__('audio_processing_node')
#         self.logger = get_logger('audio_processing_node')
#         # Parameters for audio processing
#         # Parameters for audio processing (direct assignment instead of using declare_parameter)
#         self.model_path = os.path.expanduser('~/vosk-model/vosk-model-small-en-us-0.15')
#         self.SAMPLE_RATE = 16000
#         self.CHUNK = 1024
#         self.SAMPLE_WIDTH = 2  # 16-bit audio = 2 bytes
        
#         # Publisher for audio data and recognized text
#         # self.audio_publisher = self.create_publisher(Int16MultiArray, 'audio_data', 10)
#         # self.text_publisher = self.create_publisher(String, 'recognized_text', 10)
        
#         # Audio parameters
#         self.format = pyaudio.paInt16  # Audio format (16-bit PCM)
#         self.channels = 1  # Mono audio
        
#         # Initialize PyAudio
#         self.p = pyaudio.PyAudio()
#         self.stream = self.p.open(
#             format=self.format,
#             channels=self.channels,
#             rate=self.SAMPLE_RATE,
#             input=True,
#             frames_per_buffer=self.CHUNK
#         )
        
#         # Initialize Vosk model
#         try:
#             # Set log level to be less verbose
#             vosk.SetLogLevel(0)
#             self.logger.info(f'Loading Vosk model from: {self.model_path}')
#             self.model = vosk.Model(self.model_path)
#             self.vosk_rec = vosk.KaldiRecognizer(self.model, self.SAMPLE_RATE)
#             self.logger.info('Vosk model loaded successfully')
#         except Exception as e:
#             self.logger.error(f'Failed to load Vosk model: {e}')
#             self.logger.error(f'Please ensure the model exists at {self.model_path}')
#             return
        
#         # Start recording and processing in a separate thread
#         self.running = True
#         self.thread = threading.Thread(target=self.process_audio)
#         self.thread.daemon = True
#         self.thread.start()

#         self.logger.info('Audio processing node started. Recording and recognizing speech...')

#     def process_audio(self) -> str | None:
#         """Continuously process audio data in a separate thread"""
#         while self.running:
#             try:
#                 # Read audio data
#                 data = self.stream.read(self.CHUNK, exception_on_overflow=False)
#                 audio_data = np.frombuffer(data, dtype=np.int16)
                
#                 # Publish raw audio data
#                 audio_msg = Int16MultiArray()
#                 audio_msg.data = audio_data.tolist()
#                 # self.audio_publisher.publish(audio_msg)
                
#                 # Process with Vosk
#                 if self.vosk_rec.AcceptWaveform(data):
#                     result = json.loads(self.vosk_rec.Result())
#                     if 'text' in result and result['text']:
#                         text = result['text']
#                         self.logger.info(f'Recognized: "{text}"')
                        
#                         # Publish the recognized text
#                         # text_msg = String()
#                         # text_msg.data = text
#                         # self.text_publisher.publish(text_msg)
#                         return text
#                 else:
#                     # Optional: Process partial results
#                     partial = json.loads(self.vosk_rec.PartialResult())
#                     if 'partial' in partial and partial['partial'] and len(partial['partial']) > 3:
#                         self.logger.debug(f"Partial: {partial['partial']}")
#                         return partial['partial']
#             except Exception as e:
#                 self.logger.error(f"Error in audio processing: {e}")

#     def stop(self) -> bool:
#         """Stop audio processing and release resources"""
#         self.running = False
#         # if self.thread.is_alive():
#         #     self.thread.join(timeout=2.0)
        
#         self.stream.stop_stream()
#         self.stream.close()
#         self.p.terminate()
#         self.logger.info('Audio processing stopped')
#         return True


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = AudioProcessingNode()
    
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         print("Keyboard interrupt received. Shutting down...")
# #     finally:
# #         node.stop()
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()


#!/home/viju/ros_ws/venv/bin/python3

import rclpy
from std_msgs.msg import String, Int16MultiArray
from rclpy.logging import get_logger
import numpy as np
import pyaudio
import vosk
import json
import threading
import os
import time

class AudioProcessingNode():
    def __init__(self):
        # Initialize logger
        self.logger = get_logger('audio_processing_node')
        
        # Parameters for audio processing
        self.model_path = os.path.expanduser('~/vosk-model/vosk-model-small-en-us-0.15')
        self.SAMPLE_RATE = 16000
        self.CHUNK = 1024
        self.SAMPLE_WIDTH = 2  # 16-bit audio = 2 bytes
        
        # Speech recognition results
        self.latest_text = None
        self.latest_partial = None
        self.processing_active = False
        
        # Audio parameters
        self.format = pyaudio.paInt16  # Audio format (16-bit PCM)
        self.channels = 1  # Mono audio
        
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.open_audio_stream()
        
        # Initialize Vosk model
        try:
            # Set log level to be less verbose
            vosk.SetLogLevel(0)
            self.logger.info(f'Loading Vosk model from: {self.model_path}')
            self.model = vosk.Model(self.model_path)
            self.vosk_rec = vosk.KaldiRecognizer(self.model, self.SAMPLE_RATE)
            self.logger.info('Vosk model loaded successfully')
        except Exception as e:
            self.logger.error(f'Failed to load Vosk model: {e}')
            self.logger.error(f'Please ensure the model exists at {self.model_path}')
            return
        
        # Thread control
        self.running = False
        self.thread = None

    def open_audio_stream(self):
        """Open the audio stream if not already open"""
        if self.stream is None or not self.stream.is_active():
            self.stream = self.p.open(
                format=self.format,
                channels=self.channels,
                rate=self.SAMPLE_RATE,
                input=True,
                frames_per_buffer=self.CHUNK
            )

    def start_voice_processing(self):
        """Start voice processing in a separate thread"""
        if self.running:
            return True  # Already running
            
        # Make sure stream is open
        self.open_audio_stream()
        
        # Reset state
        self.latest_text = None
        self.latest_partial = None
        self.running = True
        self.processing_active = True
        
        # Start processing thread
        self.thread = threading.Thread(target=self._process_audio_thread)
        self.thread.daemon = True
        self.thread.start()
        
        self.logger.info('Audio processing started')
        return True

    def _process_audio_thread(self):
        """Internal method that runs in a separate thread to process audio continuously"""
        while self.running:
            try:
                # Read audio data
                data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                
                # Safety check for empty data
                if not data or len(data) == 0:
                    time.sleep(0.01)
                    continue
                
                try:
                    # Process with Vosk
                    if self.vosk_rec.AcceptWaveform(data):
                        result = json.loads(self.vosk_rec.Result())
                        if 'text' in result and result['text']:
                            self.latest_text = result['text']
                            self.logger.info(f'Recognized: "{self.latest_text}"')
                    else:
                        # Process partial results
                        partial = json.loads(self.vosk_rec.PartialResult())
                        if 'partial' in partial and partial['partial'] and len(partial['partial']) > 3:
                            self.latest_partial = partial['partial']
                            self.logger.debug(f"Partial: {self.latest_partial}")
                except Exception as vosk_error:
                    self.logger.error(f"Vosk processing error: {vosk_error}")
                    # Create a fresh recognizer to recover from errors
                    self.vosk_rec = vosk.KaldiRecognizer(self.model, self.SAMPLE_RATE)
                    
            except Exception as e:
                self.logger.error(f"Error in audio processing: {e}")
                time.sleep(0.1)  # Prevent tight error loop

    # def process_audio(self) -> str | None:
    #     """Start processing and return the latest recognized text (non-blocking)"""
    #     # Start processing if not already running
    #     if not self.running:
    #         self.start_voice_processing()
            
    #     # Return the latest text (or None if nothing recognized yet)
    #     if self.latest_text:
    #         return self.latest_text
    #     elif self.latest_partial:
    #         return self.latest_partial
    #     return None

    def get_latest_text(self) -> str | None:
        """Get the latest recognized text without blocking"""
        if self.latest_text:
            return self.latest_text
        elif self.latest_partial:
            return self.latest_partial
        return None

    def stop(self) -> bool:
        """Stop audio processing and release resources"""
        self.running = False
        self.processing_active = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
            
        self.logger.info('Audio processing stopped')
        return True