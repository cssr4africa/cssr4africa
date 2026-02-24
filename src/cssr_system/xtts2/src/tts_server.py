#!/usr/bin/env python3.10


import rospy
from xtts2.srv import RealtimeTTS, RealtimeTTSResponse
import threading
import torch
import time
import os
import rospkg

rospack = rospkg.RosPack()
path = rospack.get_path('kinyarwanda_tts')
model_path = os.path.join(path,"model_files")


try:
    from RealtimeTTS import TextToAudioStream, CoquiEngine
    REALTIME_TTS_AVAILABLE = True
except ImportError:   
    REALTIME_TTS_AVAILABLE = False
    rospy.logwarn("RealtimeTTS not available. Install with: pip install RealtimeTTS")

class TTSServer:
    def __init__(self):
        rospy.init_node('tts_server')
        device = "cuda" if torch.cuda.is_available() else "cpu"
        
        if not REALTIME_TTS_AVAILABLE:
            rospy.logerr("RealtimeTTS library not found. Please install it first.")
            return
        voice_path = os.path.join(model_path,"conditioning_audio2.wav")
        
        try:
            # Initialize the Coqui Engine with XTTS v2
            engine = CoquiEngine(
                model_name="tts_models/multilingual/multi-dataset/xtts_v2",
                voice=voice_path,  # Optional: path to reference voice file for cloning
                language="en",  # Supported: en, es, fr, de, it, pt, pl, tr, ru, nl, cs, ar, zh-cn, ja, hu, ko
                device=device
            )
            self.stream = TextToAudioStream(engine)
            rospy.loginfo("TTS Server initialized with CoquiEngine (XTTS v2)")
            rospy.loginfo("Available voice clone:" f" {voice_path}")

        except Exception as e:
            rospy.logerr(f"Failed to initialize TTS engine: {e}")
            return
        
        # Create the service
        self.service = rospy.Service('realtime_tts', RealtimeTTS, self.handle_tts_request)
        rospy.loginfo("RealtimeTTS service ready")
    
    def handle_tts_request(self, req):
        """Handle incoming TTS requests"""
        try:
            if not req.text.strip():
                return RealtimeTTSResponse(False, "Empty text provided")
            
            rospy.loginfo(f"Processing TTS request: '{req.text}'")
            
            # Stream the text in a separate thread to avoid blocking the service
            thread = threading.Thread(target=self._stream_text, args=(req.text,))
            thread.daemon = True
            thread.start()
            
            return RealtimeTTSResponse(True, "TTS streaming started")
            
        except Exception as e:
            rospy.logerr(f"Error in TTS processing: {e}")
            return RealtimeTTSResponse(False, str(e))
    
    def _stream_text(self, text):
        """Stream the text using RealtimeTTS"""
        try:
            # Feed text to the stream and play immediately
            self.stream.feed(text)
            self.stream.play_async()  # Non-blocking play
            rospy.loginfo(f"Started streaming: '{text}'")
        except Exception as e:
            rospy.logerr(f"Error during streaming: {e}")

if __name__ == '__main__':
    try:
        server = TTSServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
