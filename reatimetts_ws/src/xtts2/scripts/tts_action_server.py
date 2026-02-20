#!/usr/bin/env python3
import rospy
import actionlib
import tempfile
import subprocess
import os
import rospkg
import torch
import soundfile as sf
from pathlib import Path
from TTS.utils.synthesizer import Synthesizer
from TTS.tts.configs.xtts_config import XttsConfig
from TTS.tts.models.xtts import Xtts

# Import action messages (you'll need to create these - see below)
from kinyarwanda_tts.msg import TTSAction, TTSFeedback, TTSResult

python2_path = '/usr/bin/python2'
rospack = rospkg.RosPack()
path = rospack.get_path('kinyarwanda_tts')
model_path_kin = os.path.join(path, "model_files")
python2_script = os.path.join(path, "scripts", "send_and_play_audio.py")


class UnifiedTTSActionServer:
    def __init__(self):
        rospy.init_node("tts_action_server")
        
        # --- Check CUDA ---
        self.use_cuda = torch.cuda.is_available()
        rospy.loginfo(f"CUDA available: {self.use_cuda}")
        
        # --- Init Kinyarwanda model ---
        rospy.loginfo("Loading Kinyarwanda model...")
        self.kin_synth = Synthesizer(
            os.path.join(model_path_kin, "model.pth"),
            os.path.join(model_path_kin, "config.json"),
            tts_speakers_file=os.path.join(model_path_kin, "speakers.pth"),
            use_cuda=self.use_cuda
        )
        rospy.loginfo("Kinyarwanda model loaded")
        
        # --- Init English XTTS ---
        rospy.loginfo("Loading English XTTS model...")
        base_dir = Path.home() / "models" / "v2.0.2"
        config = XttsConfig()
        config.load_json(base_dir / "config.json")
        self.eng_model = Xtts.init_from_config(config)
        self.eng_model.load_checkpoint(config, checkpoint_dir=base_dir, eval=True)
        if self.use_cuda:
            self.eng_model.cuda()
        self.eng_config = config
        rospy.loginfo("English XTTS model loaded")
        
        # Playback mode ("naoqi" or "local")
        self.playback_mode = rospy.get_param("~playback_mode", "local")
        
        # Create action server
        self.action_server = actionlib.SimpleActionServer(
            'tts_action',
            TTSAction,
            execute_cb=self.execute_tts,
            auto_start=False
        )
        self.action_server.start()
        
        rospy.loginfo("TTS Action Server ready: choose language = 'kinyarwanda' or 'english'.")
    
    def execute_tts(self, goal):
        """Execute the TTS action with feedback"""
        text = goal.text
        lang = goal.language.lower()
        
        rospy.loginfo(f"Received TTS action goal: '{text}' in language '{lang}'")
        
        # Create feedback and result objects
        feedback = TTSFeedback()
        result = TTSResult()
        
        try:
            # Send initial feedback
            feedback.status = "Initializing TTS"
            feedback.progress = 0.0
            self.action_server.publish_feedback(feedback)
            
            # Check for preemption
            if self.action_server.is_preempt_requested():
                rospy.loginfo("TTS action preempted")
                self.action_server.set_preempted()
                return
            
            # Generate speech based on language
            if lang == "kinyarwanda":
                feedback.status = "Synthesizing Kinyarwanda speech"
                feedback.progress = 0.2
                self.action_server.publish_feedback(feedback)
                
                wav = self.kin_synth.tts(
                    text,
                    speaker_wav=os.path.join(model_path_kin, "conditioning_audio.wav")
                )
                
                feedback.status = "Saving audio file"
                feedback.progress = 0.5
                self.action_server.publish_feedback(feedback)
                
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                    self.kin_synth.save_wav(wav, fp)
                    audio_file = fp.name
                    
            elif lang == "english":
                feedback.status = "Synthesizing English speech"
                feedback.progress = 0.2
                self.action_server.publish_feedback(feedback)
                
                outputs = self.eng_model.synthesize(
                    text,
                    self.eng_config,
                    speaker_wav=os.path.join(model_path_kin, "conditioning_audio.wav"),
                    gpt_cond_len=3,
                    language="en",
                    speed=1.0
                )
                
                feedback.status = "Saving audio file"
                feedback.progress = 0.5
                self.action_server.publish_feedback(feedback)
                
                wav = outputs["wav"]
                sr = self.eng_config.audio.sample_rate
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                    sf.write(fp.name, wav, sr)
                    audio_file = fp.name
            else:
                result.success = False
                result.message = f"Unsupported language: {lang}"
                result.audio_file_path = ""
                self.action_server.set_aborted(result)
                return
            
            # Check for preemption before playback
            if self.action_server.is_preempt_requested():
                rospy.loginfo("TTS action preempted before playback")
                os.remove(audio_file)
                self.action_server.set_preempted()
                return
            
            # Playback
            feedback.status = "Playing audio"
            feedback.progress = 0.7
            self.action_server.publish_feedback(feedback)
            
            if self.playback_mode == "naoqi":
                subprocess.run([python2_path, python2_script, audio_file])
            else:
                subprocess.run(["aplay", audio_file])
            
            # Final feedback
            feedback.status = "Completed"
            feedback.progress = 1.0
            self.action_server.publish_feedback(feedback)
            
            # Set successful result
            result.success = True
            result.message = f"Successfully spoken '{text}' in {lang}"
            result.audio_file_path = audio_file
            
            rospy.loginfo(f"TTS action succeeded: {result.message}")
            self.action_server.set_succeeded(result)
            
        except Exception as e:
            rospy.logerr(f"TTS action error: {e}")
            result.success = False
            result.message = str(e)
            result.audio_file_path = ""
            self.action_server.set_aborted(result)
    
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = UnifiedTTSActionServer()
        node.run()
    except rospy.ROSInterruptException:
        pass

