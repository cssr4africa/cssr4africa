#!/usr/bin/env python3.10
import rospy
import torch
import threading
import os
import rospkg
import actionlib
import time
from cssr_system.msg import english_stream_ttsAction, english_stream_ttsFeedback, english_stream_ttsResult

try:
    from RealtimeTTS import TextToAudioStream, CoquiEngine
    REALTIME_TTS_AVAILABLE = True
except ImportError:
    REALTIME_TTS_AVAILABLE = False

class TTSActionServer:
    def __init__(self):
        rospy.init_node("tts_action_server")
        device = "cuda" if torch.cuda.is_available() else "cpu"
        
        if not REALTIME_TTS_AVAILABLE:
            rospy.logerr("RealtimeTTS not available. Install it with: pip install RealtimeTTS")
            return
        
        # Locate model files
        rospack = rospkg.RosPack()
        path = rospack.get_path("kinyarwanda_tts")
        model_path = os.path.join(path, "model_files")
        voice_path = os.path.join(model_path, "conditioning_audio2.wav")
        
        # Initialize Coqui XTTS engine
        try:
            engine = CoquiEngine(
                model_name="tts_models/multilingual/multi-dataset/xtts_v2",
                voice=voice_path,
                language="en",
                device=device
            )
            self.stream = TextToAudioStream(engine)
            rospy.loginfo("english_stream_tts Action Server initialized")
        except Exception as e:
            rospy.logerr(f"Failed to initialize TTS engine: {e}")
            return
        
        # Create action server
        self.server = actionlib.SimpleActionServer(
            "tts_action", english_stream_ttsAction, self.execute_cb, False
        )
        self.server.start()
        rospy.loginfo("TTS Action Server ready")
    
    def execute_cb(self, goal):
        feedback = english_stream_ttsFeedback()
        result = english_stream_ttsResult()
        
        if not goal.text.strip():
            result.success = False
            result.message = "Empty text provided"
            self.server.set_aborted(result, result.message)
            return
        
        rospy.loginfo(f"Received TTS goal: {goal.text}")
        
        # Use an event to signal completion
        completion_event = threading.Event()
        
        # Run streaming in thread
        thread = threading.Thread(
            target=self._stream_text, 
            args=(goal.text, feedback, result, completion_event)
        )
        thread.start()
        
        # Wait until done or preempted
        rate = rospy.Rate(10)  # Check more frequently (10 Hz)
        while not completion_event.is_set():
            if self.server.is_preempt_requested():
                rospy.logwarn("english_stream_tts goal preempted")
                self.stream.stop()  # Stop the stream
                result.success = False
                result.message = "english_stream_tts preempted"
                self.server.set_preempted(result, result.message)
                thread.join(timeout=2.0)  # Wait for thread to finish
                return
            
            feedback.status = "Streaming..."
            self.server.publish_feedback(feedback)
            rate.sleep()
        
        # Wait for thread to fully complete
        thread.join(timeout=5.0)
        
        # When finished
        if result.success:
            self.server.set_succeeded(result, result.message)
        else:
            self.server.set_aborted(result, result.message)
    
    def _stream_text(self, text, feedback, result, completion_event):
        try:
            feedback.status = "Starting stream..."
            self.server.publish_feedback(feedback)
            
            # Feed text to the stream
            self.stream.feed(text)
            
            # Start async playback
            self.stream.play_async()
            rospy.loginfo(f"Started streaming: {text}")
            
            feedback.status = "Playing audio..."
            self.server.publish_feedback(feedback)
            
            # Wait until the stream is done playing
            # Method 1: Using is_playing() - recommended
            while self.stream.is_playing():
                time.sleep(0.1)  # Check every 100ms
            
            # Alternative Method 2: If is_playing() doesn't work, use wait()
            # Note: Uncomment this if Method 1 doesn't work for your version
            # self.stream.wait()
            
            rospy.loginfo("Streaming completed")
            feedback.status = "Done"
            self.server.publish_feedback(feedback)
            
            result.success = True
            result.message = "Streaming completed successfully"
            
        except Exception as e:
            rospy.logerr(f"Error during streaming: {e}")
            result.success = False
            result.message = str(e)
        finally:
            # Signal that we're done
            completion_event.set()

if __name__ == "__main__":
    try:
        server = TTSActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        