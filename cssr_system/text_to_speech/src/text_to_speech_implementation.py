""" 
text-to-speech_Implementation.py - implementation of the text-to-speech functionality

Author:     Muhirwa Richard
Date:       2025-04-18
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import rospy
from std_msgs.msg import String
import tempfile
import subprocess
import os
import sys
import threading
import time
from TTS.utils.synthesizer import Synthesizer
from configparser import ConfigParser
from io import StringIO
import contextlib

HEARTBEAT_MSG_PERIOD = 10

class TTSImplementation:
    
    # Class variables for paths
    _config_file_path = None
    _model_dir_path = None
    _python2_script_path = None
    
    @classmethod
    def set_paths(cls, config_file_path, model_dir_path, python2_script_path):
        """Set paths for the implementation class"""
        cls._config_file_path = config_file_path
        cls._model_dir_path = model_dir_path
        cls._python2_script_path = python2_script_path
    
    def __init__(self):
        """Initialize the TTS implementation with default configuration"""
        self.config = {
            'language': 'english',
            'verboseMode': True,
            'ip': "172.29.111.240",
            'port': '9559',
            'useCuda': False,
        }
        
        # Set up paths from class variables
        self.config_file_path = self._config_file_path
        self.model_files_dir = self._model_dir_path
        self.python2_path = '/usr/bin/python2'
        self.python2_script = self._python2_script_path
        self.supported_languages = ['english', 'kinyarwanda']
        
        # Initialize necessary components
        self.read_config()
        self.initialize_components()
    
    def read_config(self):
        """Read configuration from the configuration file"""
        try:
            if not os.path.exists(self.config_file_path):
                rospy.logwarn(f"Config file not found: {self.config_file_path}. Using default values.")
                return
                
            config_parser = ConfigParser()
            config_parser.read(self.config_file_path)
            
            # Read existing configuration parameters
            if 'DEFAULT' in config_parser:
                if 'language' in config_parser['DEFAULT']:
                    self.config['language'] = config_parser['DEFAULT']['language'].lower()
                if 'verboseMode' in config_parser['DEFAULT']:
                    self.config['verboseMode'] = config_parser['DEFAULT']['verboseMode'] == "True"
                if 'ip' in config_parser['DEFAULT']:
                    self.config['ip'] = config_parser['DEFAULT']['ip']
                if 'port' in config_parser['DEFAULT']:
                    self.config['port'] = config_parser['DEFAULT']['port']
                
                # Read useCuda configuration with fallback to default
                if config_parser.has_option('DEFAULT', 'useCuda'):
                    self.config['useCuda'] = config_parser['DEFAULT']['useCuda'].lower() in ['true', '1', 'yes', 'on']
                else:
                    rospy.logwarn("useCuda not found in configuration file. Using default value: False")
            
            rospy.loginfo(f"Configuration loaded from {self.config_file_path}")
                    
        except Exception as e:
            rospy.logwarn(f"Unable to read configuration file: {e}. Going with default values")
    
    @contextlib.contextmanager
    def suppress_output(self):
        """Context manager to suppress stdout and stderr based on verboseMode"""
        # If verboseMode is True, don't suppress anything
        if self.config.get('verboseMode', True):
            yield
        else:
            # If verboseMode is False, suppress all output
            old_stdout = sys.stdout
            old_stderr = sys.stderr
            try:
                with open(os.devnull, 'w') as devnull:
                    sys.stdout = devnull
                    sys.stderr = devnull
                    yield
            finally:
                sys.stdout = old_stdout
                sys.stderr = old_stderr
    
    def initialize_components(self):
        """Initialize the TTS components based on configuration"""
        try:
            # Initialize the publisher for English TTS
            self.pub = rospy.Publisher('/speech', String, queue_size=100)
            
            # Check CUDA availability if useCuda is enabled
            cuda_available = False
            if self.config['useCuda']:
                try:
                    import torch
                    cuda_available = torch.cuda.is_available()
                    if not cuda_available:
                        rospy.logwarn("CUDA requested but not available. Falling back to CPU.")
                except ImportError:
                    rospy.logwarn("PyTorch not available. Cannot use CUDA. Falling back to CPU.")
            
            # Use CUDA only if requested and available
            use_cuda_final = self.config['useCuda'] and cuda_available
            
            # Initialize the synthesizer for Kinyarwanda TTS with suppressed output
            with self.suppress_output():
                self.synthesizer = Synthesizer(
                    os.path.join(self.model_files_dir, "model.pth"),
                    os.path.join(self.model_files_dir, "config.json"),
                    tts_speakers_file=os.path.join(self.model_files_dir, "speakers.pth"),
                    encoder_checkpoint=os.path.join(self.model_files_dir, "SE_checkpoint.pth.tar"),
                    encoder_config=os.path.join(self.model_files_dir, "config_se.json"),
                    use_cuda=use_cuda_final
                )
            
            if self.config['verboseMode']:
                rospy.loginfo(f"Current working directory: {os.getcwd()}")
                rospy.loginfo(f"Model files directory: {self.model_files_dir}")
                rospy.loginfo(f"Using CUDA: {use_cuda_final}")
                rospy.loginfo("TTS components initialized successfully")
            
            # Start the heartbeat after successful initialization
            rospy.Timer(
                rospy.Duration(HEARTBEAT_MSG_PERIOD),
                lambda _: rospy.loginfo("textToSpeech: running"))
                    
        except Exception as e:
            rospy.logerr(f"Unable to load synthesizer: {e}")
            raise
    
    def say_text(self, message, language):
        """Generate speech from text in the specified language
        
        Args:
            message (str): The text to be spoken
            language (str): The language to use (english or kinyarwanda)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if language not in self.supported_languages:
            rospy.logerr(f"Unsupported language: {language}")
            return False
        
        try:
            if language == 'english':
                rospy.loginfo(f"Saying '{message}' in {language}")
                self.pub.publish(message)
            elif language == 'kinyarwanda':
                rospy.loginfo(f"Saying '{message}' in {language}")
                
                # Set environment variable for Python 2 script based on verboseMode
                if self.config.get('verboseMode', True):
                    os.environ['VERBOSE_AUDIO'] = '1'
                else:
                    os.environ['VERBOSE_AUDIO'] = '0'
                
                # Generate speech from text with suppressed output
                with self.suppress_output():
                    wav = self.synthesizer.tts(
                        message, 
                        speaker_wav=os.path.join(self.model_files_dir, "conditioning_audio.wav")
                    )
                
                # Save the audio to a temporary file
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                    temp_file_path = fp.name
                    with self.suppress_output():
                        self.synthesizer.save_wav(wav, fp)
                
                # Play the audio using the Python 2 script
                # Suppress subprocess output based on verboseMode
                if self.config.get('verboseMode', True):
                    # Allow all output in verbose mode
                    subprocess.run([
                        self.python2_path, 
                        self.python2_script,
                        temp_file_path,
                        self.config['ip'],
                        self.config['port']
                    ], check=True)
                else:
                    # Suppress subprocess output in non-verbose mode
                    subprocess.run([
                        self.python2_path, 
                        self.python2_script,
                        temp_file_path,
                        self.config['ip'],
                        self.config['port']
                    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                # Clean up temporary file
                os.unlink(temp_file_path)
            
            return True
        except Exception as e:
            rospy.logerr(f"Error generating speech: {e}")
            return False
