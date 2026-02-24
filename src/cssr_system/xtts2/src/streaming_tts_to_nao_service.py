#!/usr/bin/env python3.10
import rospy
import threading
import os
import rospkg
import time
import tempfile
import torch
import subprocess
import wave
import numpy as np
from queue import Queue
from cssr_system.srv import RealtimeTTS, RealtimeTTSResponse

try:
    from RealtimeTTS import TextToAudioStream, CoquiEngine
    REALTIME_TTS_AVAILABLE = True
except ImportError:
    REALTIME_TTS_AVAILABLE = False

class TTSServiceServer:
    def __init__(self):
        rospy.init_node("tts_service_server")
        device = "cuda" if torch.cuda.is_available() else "cpu"
        
        if not REALTIME_TTS_AVAILABLE:
            rospy.logerr("RealtimeTTS not available. Install it with: pip install RealtimeTTS")
            return
        
        # NAO/Pepper robot configuration
        self.nao_ip = rospy.get_param("~nao_ip", "172.29.111.230")
        self.nao_port = rospy.get_param("~nao_port", 9559)
        self.nao_password = rospy.get_param("~nao_password", "nao")
        self.chunk_duration = rospy.get_param("~chunk_duration", 15.0)  # seconds per chunk
        
        # Python 2 script paths for NAO communication
        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path("cssr_system"), "xtts2-")
        self.python2_path = '/usr/bin/python2'
        self.python2_script = os.path.join(path, "scripts", 'send_and_play_audio.py')
        self.package_path = path
        
        # Locate model files
        voice_clone_path = os.path.join(path, "voice_clones")
        voice_path = os.path.join(voice_clone_path, "pepper.wav")

        # Initialize Coqui XTTS engine
        try:
            engine = CoquiEngine(
                model_name="tts_models/multilingual/multi-dataset/xtts_v2",
                voice=voice_path,
                language="en",
                device=device
            )
            
            # Audio chunk queue for streaming
            self.chunk_queue = Queue()
            self.is_generating = False
            self.generation_complete = threading.Event()
            
            # Create stream with chunk callback
            self.stream = TextToAudioStream(
                engine,
                muted=True
            )
            
            rospy.loginfo(f"TTS Service Server initialized for NAO at {self.nao_ip}")
            rospy.loginfo(f"Streaming chunks of {self.chunk_duration}s duration")
        except Exception as e:
            rospy.logerr(f"Failed to initialize TTS engine: {e}")
            return
        
        # Create service server
        self.service = rospy.Service(
            "tts_service", RealtimeTTS, self.handle_tts_request
        )
        rospy.loginfo("TTS Service Server ready")
    
    def _on_audio_chunk(self, chunk):
        """Callback when audio chunk is generated"""
        if self.is_generating:
            self.chunk_queue.put(chunk)
    
    def handle_tts_request(self, req):
        response = RealtimeTTSResponse()
        
        if not req.text.strip():
            response.success = False
            response.message = "Empty text provided"
            return response
        
        rospy.loginfo(f"Received TTS request: {req.text}")
        
        # Clear queue and reset events
        while not self.chunk_queue.empty():
            self.chunk_queue.get()
        self.generation_complete.clear()
        
        # Use event to signal completion
        completion_event = threading.Event()
        
        # Container for result from thread
        result_container = {'success': False, 'message': ''}
        
        # Start generation and streaming in separate threads
        generation_thread = threading.Thread(
            target=self._generate_audio, 
            args=(req.text,)
        )
        
        streaming_thread = threading.Thread(
            target=self._stream_to_nao, 
            args=(result_container, completion_event)
        )
        
        generation_thread.start()
        streaming_thread.start()
        
        # Wait until done
        while not completion_event.is_set():
            if rospy.is_shutdown():
                self.stream.stop()
                self.is_generating = False
                generation_thread.join(timeout=1.0)
                streaming_thread.join(timeout=1.0)
                return RealtimeTTSResponse(False, "Service shutdown")
            time.sleep(0.1)
        
        generation_thread.join(timeout=5.0)
        streaming_thread.join(timeout=5.0)
        
        response.success = result_container['success']
        response.message = result_container['message']
        return response
    
    def _generate_audio(self, text):
        """Generate audio chunks in background"""
        try:
            self.is_generating = True
            rospy.loginfo("Starting audio generation...")
            
            # Feed text and start generation
            self.stream.feed(text)
            self.stream.play_async(on_audio_chunk=self._on_audio_chunk, muted=True)
            
            # Wait for generation to complete
            while self.stream.is_playing():
                time.sleep(0.05)
            
            rospy.loginfo("Audio generation completed")
            
        except Exception as e:
            rospy.logerr(f"Error during generation: {e}")
        finally:
            self.is_generating = False
            self.generation_complete.set()
    
    def _stream_to_nao(self, result_container, completion_event):
        """Stream audio chunks to NAO as they arrive"""
        temp_files = []
        chunk_count = 0
        
        try:
            # Sample rate from RealtimeTTS (typically 24000 or 22050)
            sample_rate = 24000
            chunk_size = int(sample_rate * self.chunk_duration * 2)  # 2 bytes per sample (16-bit)
            
            audio_buffer = b''
            first_chunk_sent = False
            
            while True:
                # Check if generation is complete and queue is empty
                if self.generation_complete.is_set() and self.chunk_queue.empty():
                    # Send any remaining audio in buffer
                    if len(audio_buffer) > 0:
                        chunk_count += 1
                        rospy.loginfo(f"Sending final chunk {chunk_count} to NAO...")
                        temp_file = self._send_chunk_to_nao(audio_buffer, sample_rate, chunk_count)
                        if temp_file:
                            temp_files.append(temp_file)
                    break
                
                # Get chunk from queue (with timeout)
                try:
                    chunk = self.chunk_queue.get(timeout=0.1)
                    
                    # Convert chunk to bytes if needed
                    if isinstance(chunk, np.ndarray):
                        chunk_bytes = (chunk * 32767).astype(np.int16).tobytes()
                    else:
                        chunk_bytes = chunk
                    
                    audio_buffer += chunk_bytes
                    
                    # When buffer reaches chunk size, send to NAO
                    if len(audio_buffer) >= chunk_size:
                        chunk_count += 1
                        
                        if not first_chunk_sent:
                            rospy.loginfo(f"Sending first chunk to NAO (streaming started)...")
                            first_chunk_sent = True
                        else:
                            rospy.loginfo(f"Sending chunk {chunk_count} to NAO...")
                        
                        # Send this chunk
                        temp_file = self._send_chunk_to_nao(audio_buffer[:chunk_size], sample_rate, chunk_count)
                        if temp_file:
                            temp_files.append(temp_file)
                        
                        # Keep remaining data in buffer
                        audio_buffer = audio_buffer[chunk_size:]
                
                except:
                    # Queue empty, continue waiting
                    pass
            
            rospy.loginfo(f"Streaming completed. Sent {chunk_count} chunks to NAO")
            
            result_container['success'] = True
            result_container['message'] = f"Streamed {chunk_count} chunks successfully"
            
        except Exception as e:
            rospy.logerr(f"Error during streaming: {e}")
            result_container['success'] = False
            result_container['message'] = str(e)
        finally:
            # Cleanup temp files
            for temp_file in temp_files:
                try:
                    os.remove(temp_file)
                except:
                    pass
            completion_event.set()
    
    def _send_chunk_to_nao(self, audio_data, sample_rate, chunk_num):
        """Send a single audio chunk to NAO"""
        temp_path = None
        try:
            # Create temporary WAV fil 
            with tempfile.NamedTemporaryFile(suffix=f"_chunk{chunk_num}.wav", delete=False) as temp_file:
                temp_path = temp_file.name
            
            # Write WAV fil
            with wave.open(temp_path, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(audio_data)
            
            # Send to NAO and play
            result = subprocess.run(
                [self.python2_path, self.python2_script, temp_path],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                rospy.logwarn(f"Chunk {chunk_num} playback warning: {result.stderr}")
            
            return temp_path
            
        except Exception as e:
            rospy.logwarn(f"Error sending chunk {chunk_num}: {e}")
            if temp_path:
                try:
                    os.remove(temp_path)
                except:
                    pass
            return None

if __name__ == "__main__":
    try:
        server = TTSServiceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
