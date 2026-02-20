#!/usr/bin/env python3
"""
Enhanced TTS Node with Code-Switching 
Uses transformers (VITS/MMS) for both Kinyarwanda and English.
"""

import rospy
from std_msgs.msg import String
import tempfile
import subprocess
import os
import rospkg
import torch
import scipy.io.wavfile
import scipy.signal
import numpy as np
import shutil
import datetime
from transformers import VitsModel, AutoTokenizer
from TTS.utils.synthesizer import Synthesizer

# Import code-switching modules
try:
    from code_switch_preprocessor import CodeSwitchPhonemizer, Language
    CODE_SWITCH_AVAILABLE = True
except ImportError:
    CODE_SWITCH_AVAILABLE = False
    rospy.logwarn("Code-switching preprocessor not available. Install dependencies.")

rospack = rospkg.RosPack()
package_path = rospack.get_path('kinyarwanda_tts')
model_path = os.path.join(package_path, "model_files")

# Path to the Python 2 script for NAOqi playback
python2_path = '/usr/bin/python2'
python2_script = os.path.join(package_path, "scripts", 'send_and_play_audio.py')


class MixedLanguageTTS:
    def __init__(self, model_path):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"TTS Engine using device: {self.device}")
        
        # 1. Setup English Model (Transformers - VITS LJS)
        self.eng_model_id = "kakao-enterprise/vits-ljs"
        self.eng_model = None
        self.eng_tokenizer = None
        
        # 2. Setup Kinyarwanda Model (Coqui TTS - YourTTS/Digital Umuganda)
        self.kin_model_path = os.path.join(model_path, "model.pth")
        self.kin_config_path = os.path.join(model_path, "config.json")
        self.kin_speakers_path = os.path.join(model_path, "speakers.pth")
        self.kin_synthesizer = None
        
        # Cloning reference for YourTTS (optional, to match English voice)
        self.cloning_ref_path = os.path.join(model_path, "english_generated1.wav") # Default
        
        # Global sampling rate for output (will sync to English model usually 22050)
        self.target_sampling_rate = 22050 

    def load_models(self):
        # Load English
        rospy.loginfo(f"Loading English Model: {self.eng_model_id}...")
        try:
            self.eng_model = VitsModel.from_pretrained(self.eng_model_id).to(self.device)
            self.eng_tokenizer = AutoTokenizer.from_pretrained(self.eng_model_id)
            self.target_sampling_rate = self.eng_model.config.sampling_rate
            rospy.loginfo(f"English model loaded. Target SR: {self.target_sampling_rate}")
        except Exception as e:
            rospy.logerr(f"Failed to load English model: {e}")

        # Load Kinyarwanda
        rospy.loginfo("Loading Kinyarwanda Coqui Model...")
        try:
             self.kin_synthesizer = Synthesizer(
                self.kin_model_path,
                self.kin_config_path,
                tts_speakers_file=self.kin_speakers_path,
                use_cuda=torch.cuda.is_available()
            )
             rospy.loginfo("Kinyarwanda Coqui model loaded.")
        except Exception as e:
            rospy.logerr(f"Failed to load Kinyarwanda model: {e}")

    def generate_segment(self, text, lang_code):
        waveform = None
        sr = 22050 # Default fallback
        
        # ENGLISH GENERATION
        if lang_code == "eng":
            if not self.eng_model:
                self.load_models()
                
            inputs = self.eng_tokenizer(text, return_tensors="pt")
            with torch.no_grad():
                output = self.eng_model(**inputs.to(self.device)).waveform
            waveform = output.squeeze().float().cpu().numpy()
            sr = self.eng_model.config.sampling_rate
            
        # KINYARWANDA GENERATION
        else:
            if not self.kin_synthesizer:
                self.load_models()
            
            # Use Coqui TTS
            # speaker_wav is used for zero-shot cloning if the model supports it.
            # We can use a reference file.
            wav = self.kin_synthesizer.tts(
                text,
                speaker_wav=self.cloning_ref_path if os.path.exists(self.cloning_ref_path) else None
            )
            waveform = np.array(wav)
            sr = self.kin_synthesizer.output_sample_rate
            
        # Resample to match target (usually English model's rate)
        if self.target_sampling_rate and sr != self.target_sampling_rate:
            num_samples = int(len(waveform) * self.target_sampling_rate / sr)
            waveform = scipy.signal.resample(waveform, num_samples)
            
        return waveform

    def stitch_audio(self, segments, output_file="mixed_speech.wav"):
        """
        segments: list of tuples (text, lang_code)
        """
        if not self.eng_model and not self.kin_synthesizer:
             self.load_models()

        full_audio = []
        
        # Silence padding (0.05s)
        silence = np.zeros(int(self.target_sampling_rate * 0.05))
            
        for text, lang in segments:
            if not text.strip(): continue

            rospy.loginfo(f"Generating ({lang}): '{text}'")
            try:
                audio_segment = self.generate_segment(text, lang)
                if audio_segment is not None:
                    full_audio.append(audio_segment)
                    full_audio.append(silence)
            except Exception as e:
                rospy.logerr(f"Failed to generate segment for '{text}' ({lang}): {e}")
            
        if full_audio:
            final_waveform = np.concatenate(full_audio)
            
            # Debugging audio stats
            max_val = np.max(np.abs(final_waveform))
            rospy.loginfo(f"Generated waveform shape: {final_waveform.shape}, Max output amplitude: {max_val:.4f}")
            
            if max_val == 0:
                rospy.logwarn("Generated audio is pure silence!")
            
            # Normalize volume
            if max_val > 0:
                final_waveform = final_waveform / max_val
            
            # Convert to 16-bit PCM
            final_waveform_int16 = (final_waveform * 32767).astype(np.int16)
                
            scipy.io.wavfile.write(output_file, self.target_sampling_rate, final_waveform_int16)
            return True
        else:
            rospy.logwarn("No audio segments were generated.")
            return False

class EnhancedKinyarwandaTTSNode:
    """
    ROS Node wrapper for MixedLanguageTTS
    """
    
    def __init__(self):
        rospy.init_node('kinyarwandaTTS_mixed')
        
        # Initialize Engine
        self.engine = MixedLanguageTTS(model_path)
        self.engine.load_models()


        # Initialize Preprocessor
        self.preprocessor = CodeSwitchPhonemizer()
        
        # Load English Terms from param or default list
        self.english_terms = rospy.get_param('~english_terms', [
            "Carnegie Mellon University", "Lab", "Robot", "Professor",
            "Artificial Intelligence", "Computer", "Science", "Engineering",
            "Technology", "Internet", "Software", "Hardware",
            "University", "Research", "Data", "Network",
            "System", "Application", "Device", "Digital",
            "Python", "Java", "C++",
            "Machine Learning", "Deep Learning", "Neural Network",
            "Algorithm", "Function", "Variable", "Loop", "Condition",
            "Africa", "lynnx", "motion","Navigation",
            "save", "open", "close", "print", "share", "send", "Pepper","roomba",
            "receive", "connect", "disconnect", "sync", "backup", "restore",
            "pick and place","manipulation","robotics"
        ])

        # Playback settings
        self.playback_mode = rospy.get_param("~playback_mode", "naoqi") # Options: "local", "naoqi", "save"
        self.robot_ip = rospy.get_param("~robot_ip", "172.29.111.230")
        self.save_dir = rospy.get_param("~save_dir", os.path.join(os.path.expanduser("~"), "tts_saved_audio"))
        
        rospy.Subscriber('text_to_say', String, self.text_to_say_callback)
        rospy.loginfo(f"Mixed Language TTS Node Ready. Mode: {self.playback_mode}")

    def text_to_say_callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Processing: {text}")
        
        try:
            # Split text using CodeSwitchPhonemizer with specific terms
            _, segments_obj = self.preprocessor.process_text(text, known_english_terms=self.english_terms)
            
            # Convert segment objects to (text, code) tuples for the engine
            # Mapping: Language.ENGLISH -> "eng", Language.KINYARWANDA -> "kin"
            segments_tuples = []
            for seg in segments_obj:
                code = "eng" if seg.language == Language.ENGLISH else "kin"
                segments_tuples.append((seg.text, code))
            
            # Generate Audio
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                output_path = fp.name
            
            success = self.engine.stitch_audio(segments_tuples, output_file=output_path)
            
            if success:
                self.play_audio(output_path, text)
            
            # DEBUG: Keep file for inspection if needed.
            # Convert os.remove to a log message.
            rospy.loginfo(f"Audio file preserved at: {output_path}")
            # if os.path.exists(output_path):
            #     os.remove(output_path)
            
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

    def play_audio(self, audio_file, text=""):
        if self.playback_mode == "naoqi":
            res = subprocess.run([python2_path, python2_script, audio_file, self.robot_ip])
            if res.returncode != 0:
                rospy.logwarn("NAO playback failed. Falling back to local playback.")
                self.play_locally(audio_file)
        elif self.playback_mode == "save":
            self.save_audio_locally(audio_file, text)
        else:
            self.play_locally(audio_file)

    def save_audio_locally(self, audio_file, text):
        if not os.path.exists(self.save_dir):
            try:
                os.makedirs(self.save_dir)
                rospy.loginfo(f"Created save directory: {self.save_dir}")
            except OSError as e:
                rospy.logerr(f"Failed to create directory {self.save_dir}: {e}")
                return

        # Create filename from timestamp and text
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_text = "".join([c for c in text if c.isalnum() or c in (' ', '-', '_')]).strip().replace(' ', '_')[:30]
        filename = f"{timestamp}_{safe_text}.wav"
        dest_path = os.path.join(self.save_dir, filename)

        try:
            shutil.copy(audio_file, dest_path)
            rospy.loginfo(f"Audio saved to: {dest_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save audio: {e}")

    def play_locally(self, audio_file):
        rospy.loginfo(f"Attempting local playback of {audio_file}")
        
        if not os.path.exists(audio_file):
            rospy.logerr(f"Audio file does not exist: {audio_file}")
            return
            
        size = os.path.getsize(audio_file)
        rospy.loginfo(f"File size: {size} bytes")
        
        if size < 100:
             rospy.logwarn("File seems too small to be valid audio.")

        # Try aplay
        if shutil.which("aplay"):
            try:
                # Run aplay and capture output for debugging
                result = subprocess.run(["aplay", audio_file], capture_output=True, text=True)
                if result.returncode != 0:
                    rospy.logerr(f"aplay failed: {result.stderr}")
                else:
                    rospy.loginfo("aplay playback successful")
            except Exception as e:
                rospy.logerr(f"Exception while running aplay: {e}")
        elif shutil.which("xdg-open"):
            rospy.loginfo("Using xdg-open...")
            subprocess.run(["xdg-open", audio_file])
        else:
            rospy.logerr("No compatible audio player found (aplay, xdg-open).")

if __name__ == "__main__":
    try:
        node = EnhancedKinyarwandaTTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
