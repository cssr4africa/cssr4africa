"""
sound_detection_test_implementation.py Implementation code for running the Sound Detection and Processing unit test.

Author: Yohannes Tadesse Haile
Date: April 13, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import rospkg
import rospy
import os
import time
import json
import numpy as np
import soundfile as sf
from std_msgs.msg import Float32MultiArray, Float32
from unit_tests.msg import sound_detection_test_microphone_msg_file
from datetime import datetime
from threading import Lock

class SoundDetectionTest:
    """
    SoundDetectionTest records and analyzes audio data for testing the sound detection system.
    It can record both filtered and unfiltered audio and save direction data directly to a text file.
    """
    def __init__(self):
        """
        Initialize the SoundDetectionTest class.
        Sets up configuration, ROS subscribers, and data structures for audio analysis.
        """
        # Set node name for consistent logging
        self.node_name = rospy.get_name().lstrip('/')
        
        self.rospack = rospkg.RosPack()
        try:
            self.unit_test_package_path = self.rospack.get_path('unit_tests')
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"{self.node_name}: ROS package not found: {e}")
            raise RuntimeError(f"Required ROS package not found: {e}")

        # Read configuration
        self.config = self.read_json_file()
        if not self.config:
            rospy.logerr(f"{self.node_name}: Failed to load configuration. Exiting.")
            raise RuntimeError("Configuration file could not be loaded.")
        
        # Set up configuration parameters with defaults
        self.sample_rate = 48000
        self.save_dir = self.unit_test_package_path + '/sound_detection_test/data/'
        self.record_filtered = self.config.get("recordFiltered", True)
        self.record_unfiltered = self.config.get("recordUnfiltered", True)
        self.record_duration = self.config.get("recordDuration", 10)
        self.verbose_mode = self.config.get("verboseMode", False)
        
        # Direction data settings
        self.save_direction_data = self.config.get("saveDirectionData", True)
        
        # RMS normalization parameters
        self.target_rms = self.config.get("targetRMS", 0.2)
        self.apply_normalization = self.config.get("applyNormalization", True)
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Created directory for audio recordings: {self.save_dir}")

        # Timer for periodic status message
        self.status_timer = rospy.get_time()
        
        # Locks for thread safety
        self.filtered_lock = Lock()
        self.unfiltered_lock = Lock()
        self.direction_lock = Lock()
        
        # State variables for filtered audio
        self.filtered_audio_buffer = []
        self.is_recording_filtered = False
        self.filtered_recording_start_time = None
        
        # State variables for unfiltered audio
        self.unfiltered_audio_buffer = []
        self.is_recording_unfiltered = False
        self.unfiltered_recording_start_time = None
        
        # Direction data file handling
        self.direction_file = None
        self.direction_file_path = None
        self.direction_start_time = None
        
        # Get the original microphone topic from the config
        self.microphone_topic = self.extract_topics('Microphone')
        if not self.microphone_topic:
            rospy.logwarn(f"{self.node_name}: Microphone topic not found. Will only record filtered audio.")
            self.record_unfiltered = False
        
        # Initialize direction file if enabled
        if self.save_direction_data:
            self.init_direction_file()
        
        # Subscribe to the filtered audio signal topic
        if self.record_filtered:
            self.filtered_sub = rospy.Subscriber("soundDetection/signal", Float32MultiArray, self.filtered_audio_callback)
            rospy.loginfo(f"{self.node_name}: Subscribed to filtered audio: soundDetection/signal")
        
        # Subscribe to the original microphone topic for unfiltered audio
        if self.record_unfiltered and self.microphone_topic:
            self.unfiltered_sub = rospy.Subscriber(self.microphone_topic, sound_detection_test_microphone_msg_file, self.unfiltered_audio_callback)
            rospy.loginfo(f"{self.node_name}: Subscribed to unfiltered audio: {self.microphone_topic}")
        
        # Subscribe to the direction topic
        if self.save_direction_data:
            self.direction_sub = rospy.Subscriber("soundDetection/direction", Float32, self.direction_callback)
            rospy.loginfo(f"{self.node_name}: Subscribed to direction data: soundDetection/direction")
            rospy.loginfo(f"{self.node_name}: Direction data will be written directly to text file")
        
        rospy.loginfo(f"{self.node_name}: Initialized successfully")
        rospy.loginfo(f"{self.node_name}: Recording will automatically stop after {self.record_duration} seconds")
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def read_json_file(self):
        """
        Read the configuration file and return the parsed JSON object.
        
        Returns:
            dict: Configuration dictionary or None if file couldn't be read.
        """
        try:
            config_path = os.path.join(self.unit_test_package_path, 'sound_detection_test/config', 'sound_detection_test_configuration.json')
            if not os.path.exists(config_path):
                rospy.logerr(f"{self.node_name}: read_json_file: Configuration file not found at {config_path}")
                return None
                
            with open(config_path, 'r') as file:
                config = json.load(file)
                return config
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"{self.node_name}: read_json_file: Error decoding JSON file: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"{self.node_name}: read_json_file: Unexpected error: {e}")
            return None

    def extract_topics(self, topic_key):
        """
        Extract the topic name for a given key from the topics data file.
        
        Args:
            topic_key (str): Key to search for in the topics file.
            
        Returns:
            str: The topic name or None if not found.
        """
        try:
            config_path = os.path.join(self.unit_test_package_path, 'sound_detection_test/data', 'pepper_topics.dat')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        key, value = line.split(maxsplit=1)
                        if key.lower() == topic_key.lower():
                            return value
            else:
                rospy.logerr(f"{self.node_name}: Topics data file not found at {config_path}")
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"{self.node_name}: ROS package not found: {e}")
        return None
    
    def normalize_rms(self, audio_data, target_rms=None, min_rms=1e-10):
        """
        Apply RMS normalization to audio data.
        
        Args:
            audio_data (np.ndarray): Audio data to normalize
            target_rms (float): Target RMS value (typically 0.1-0.3)
            min_rms (float): Minimum RMS value to avoid division by zero
            
        Returns:
            np.ndarray: Normalized audio data
        """
        if target_rms is None:
            target_rms = self.target_rms
            
        # Calculate current RMS value
        rms_current = np.sqrt(np.mean(audio_data**2))
        
        # Skip normalization if RMS is too low (silent)
        if rms_current < min_rms:
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Audio too quiet for normalization (RMS: {rms_current:.6f})")
            return audio_data
        
        # Calculate scaling factor
        scaling_factor = target_rms / rms_current
        
        # Apply normalization
        normalized_data = audio_data * scaling_factor
        
        # Clip to prevent overflow
        normalized_data = np.clip(normalized_data, -1.0, 1.0)
        
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Applied RMS normalization - Before RMS: {rms_current:.4f}, After RMS: {target_rms:.4f}, Factor: {scaling_factor:.4f}")
        
        return normalized_data
        
    def init_direction_file(self):
        """
        Initialize the direction data text file with header information.
        """
        try:
            with self.direction_lock:
                # Close any existing file
                if self.direction_file is not None:
                    self.direction_file.close()
                    
                # Create new file with timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                self.direction_file_path = os.path.join(self.save_dir, f"sound_detection_test_direction_data_{timestamp}.txt")
                
                # Open file for writing
                self.direction_file = open(self.direction_file_path, 'w')
                
                # Write header
                self.direction_file.write("# Sound Direction Data\n")
                self.direction_file.write("# Time(s), Direction(degrees)\n")
                
                # Record start time for relative timestamps
                self.direction_start_time = time.time()
                
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: Initialized direction data file: {self.direction_file_path}")
                    
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error initializing direction file: {e}")
            self.direction_file = None
    
    def filtered_audio_callback(self, msg):
        """
        Process incoming filtered audio data from soundDetection/signal.
        
        The data is in Float32MultiArray format, with values in the range [-1.0, 1.0].
        
        Args:
            msg (Float32MultiArray): The audio data message
        """
        if not self.record_filtered:
            return
            
        # Convert message data to numpy array
        audio_data = np.array(msg.data, dtype=np.float32)
        
        with self.filtered_lock:
            # If we're currently recording, add to buffer
            if self.is_recording_filtered:
                self.filtered_audio_buffer.extend(audio_data.tolist())
            else:
                # Start a new recording
                self.is_recording_filtered = True
                self.filtered_recording_start_time = rospy.get_time()
                self.filtered_audio_buffer = audio_data.tolist()
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: Started new FILTERED audio recording")
            
            # Check if we've reached the recording duration
            buffer_time = rospy.get_time() - self.filtered_recording_start_time
            
            if buffer_time >= self.record_duration:
                self.save_filtered_audio()
    
    def unfiltered_audio_callback(self, msg):
        """
        Process incoming unfiltered audio data from the original microphone topic.
        
        The data is in microphone_test_msg_file message type with frontLeft and frontRight arrays.
        
        Args:
            msg (microphone_test_msg_file): The audio data message
        """
        if not self.record_unfiltered:
            return
            
        try:
            # Print a status message every 10 seconds
            if rospy.get_time() - self.status_timer > 10:
                rospy.loginfo(f"{self.node_name}: running")
                self.status_timer = rospy.get_time()

            # Extract only the left channel data from int16 array and normalize to float
            audio_data_int16 = np.array(msg.frontLeft, dtype=np.int16)
            audio_data = audio_data_int16.astype(np.float32) / 32767.0
            
            with self.unfiltered_lock:
                # If we're currently recording, add to buffer
                if self.is_recording_unfiltered:
                    self.unfiltered_audio_buffer.extend(audio_data.tolist())
                else:
                    # Start a new recording
                    self.is_recording_unfiltered = True
                    self.unfiltered_recording_start_time = rospy.get_time()
                    self.unfiltered_audio_buffer = audio_data.tolist()
                    rospy.loginfo(f"{self.node_name}: Started new UNFILTERED audio recording")
                
                # Check if we've reached the recording duration
                buffer_time = rospy.get_time() - self.unfiltered_recording_start_time
                
                if buffer_time >= self.record_duration:
                    self.save_unfiltered_audio()
                    
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in unfiltered audio callback: {e}")
    
    def direction_callback(self, msg):
        """
        Process incoming direction data from soundDetection/direction and write directly to file.
        
        Args:
            msg (Float32): The direction angle in degrees
        """
        if not self.save_direction_data or self.direction_file is None:
            return
            
        try:
            with self.direction_lock:
                # Calculate time since start
                current_time = time.time() - self.direction_start_time
                
                # Write data point to file
                self.direction_file.write(f"{current_time:.4f}, {msg.data:.4f}\n")
                
                # Flush to ensure data is written even if buffer isn't full
                self.direction_file.flush()
                
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error writing direction data: {e}")
    
    def save_filtered_audio(self):
        """Save the current filtered audio buffer as a WAV file with RMS normalization."""
        if not self.filtered_audio_buffer:
            rospy.logwarn(f"{self.node_name}: Cannot save empty filtered audio buffer")
            return
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            
            # Save as WAV file
            wav_filepath = os.path.join(self.save_dir, f"sound_detection_test_speech_filtered_{timestamp}.wav")
            
            # Convert audio buffer to numpy array
            audio_np = np.array(self.filtered_audio_buffer, dtype=np.float32)
            
            # Apply RMS normalization if enabled
            if self.apply_normalization:
                audio_np = self.normalize_rms(audio_np)
            
            # Save as WAV file
            sf.write(wav_filepath, audio_np, self.sample_rate, format='WAV')
            
            # Log success
            duration = len(audio_np) / self.sample_rate  # Duration in seconds
            if self.verbose_mode:
                normalization_status = "normalized " if self.apply_normalization else ""
                rospy.loginfo(f"{self.node_name}: Saved {duration:.2f}s {normalization_status}FILTERED audio to {wav_filepath}")
            
            # Reset state for next recording
            self.filtered_audio_buffer = []
            self.is_recording_filtered = False
            
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error saving filtered audio: {e}")
    
    def save_unfiltered_audio(self):
        """Save the current unfiltered audio buffer as a mono WAV file with RMS normalization."""
        if not self.unfiltered_audio_buffer:
            rospy.logwarn(f"{self.node_name}: Cannot save empty unfiltered audio buffer")
            return
            
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            
            # Save as WAV file
            wav_filepath = os.path.join(self.save_dir, f"sound_detection_test_unfiltered_{timestamp}.wav")
            
            # Convert audio buffer to numpy array
            audio_np = np.array(self.unfiltered_audio_buffer, dtype=np.float32)
            
            # Apply RMS normalization if enabled
            if self.apply_normalization:
                audio_np = self.normalize_rms(audio_np)
            
            # Save as WAV file (mono)
            sf.write(wav_filepath, audio_np, self.sample_rate, format='WAV')
            
            # Log success
            duration = len(audio_np) / self.sample_rate  # Duration in seconds
            if self.verbose_mode:
                normalization_status = "normalized " if self.apply_normalization else ""
                rospy.loginfo(f"{self.node_name}: Saved {duration:.2f}s {normalization_status}UNFILTERED mono audio to {wav_filepath}")
            
            # Reset state for next recording
            self.unfiltered_audio_buffer = []
            self.is_recording_unfiltered = False
            
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error saving unfiltered audio: {e}")
    
    def shutdown_hook(self):
        """
        Clean up resources when the node is shutting down.
        Close the direction data file if it's open.
        """
        try:
            if self.direction_file is not None:
                self.direction_file.close()
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: Closed direction data file: {self.direction_file_path}")
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error closing direction file: {e}")
            
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Shutting down")