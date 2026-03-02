"""
sound_detection_implementation.py Implementation code for running the sound detection and localization algorithm

Author: Yohannes Tadesse Haile
Date: April 13, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import math
import os
import json
import rospy
import std_msgs.msg
import webrtcvad
import rospkg
import numpy as np
import threading
import noisereduce as nr 
import soundfile as sf 
from datetime import datetime
from cssr_system.msg import sound_detection_microphone_msg_file
from threading import Lock
from std_msgs.msg import Float32MultiArray
from datetime import datetime

class SoundDetectionNode:
    """
    SoundDetectionNode processes audio data from a microphone topic, applies VAD to determine if speech is present,
    applies bandpass filtering and spectral subtraction on the left channel, and localizes the sound source by computing
    the interaural time difference (ITD) via GCC-PHAT.
    """
    def __init__(self):
        """
        Initialize the SoundDetectionNode.
        Sets up ROS subscribers, publishers, and loads configuration parameters.
        """
        # Set node name for consistent logging
        self.node_name = rospy.get_name().lstrip('/')
        
        # Get configuration parameters from the ROS parameter server
        self.config = rospy.get_param('/soundDetection', {})

        # Get param for unit_tests
        self.unit_tests = rospy.get_param('/soundDetection/unit_tests', False)
        
        # Set parameters from config
        self.frequency_sample = 48000
        self.speed_of_sound = 343.0
        self.distance_between_ears = self.config.get('distanceBetweenEars', 0.07)
        self.intensity_threshold = self.config.get('intensityThreshold', 3.9e-3)
        self.verbose_mode = self.config.get('verboseMode', False)

        # Initialize parameter for noise reduction filter 
        self.noise_type = self.config.get('stationary', True)
        self.prop_decrease = self.config.get('propDecrease', 0.9)
        
        # Buffer for localization (2 channels)
        self.localization_buffer_size = self.config.get('localizationBufferSize', 8192)
        self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
        self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
        self.accumulated_samples = 0

        # Initialize VAD with configurable aggressiveness mode
        self.vad_aggressiveness = self.config.get('vadAggressiveness', 1)
        self.vad = webrtcvad.Vad(self.vad_aggressiveness)
        self.vad_frame_duration = 0.02  # 20 ms (WebRTC VAD requires specific frame durations)
        self.vad_frame_size = int(self.frequency_sample * self.vad_frame_duration)

        # Initialize RMS parameters
        self.target_rms = self.config.get('targetRMS', 0.2)

         # Initialize timeout parameters
        self.last_audio_time = rospy.get_time()
        self.audio_timeout = self.config.get('audioTimeout', 2)  # Default 10 seconds timeout
        self.received_first_audio = False  # Flag to track if we've received any audio yet

        # Initialize noise reduction parameters
        self.context_duration = self.config.get('contextDuration', 2.0)  # Context window duration in seconds
        self.context_size = int(self.frequency_sample * self.context_duration)
        self.left_context_window = np.zeros(self.context_size, dtype=np.float32) 
        self.use_noise_reduction = self.config.get('useNoiseReduction', True)
        
        if self.use_noise_reduction and self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Noise reduction enabled for left channel with {self.context_duration}s context window")

        # Configurable time duration for saving the filtered audio
        self.save_audio_duration = self.config.get('recordDuration', 10)  # seconds

        # Add audio saving parameters for unit tests
        if self.unit_tests:
            self.save_audio = True
            self.sample_count = 0
            self.max_samples_to_save = self.save_audio_duration * self.frequency_sample 
            self.saved_samples = 0
            self.filtered_buffer = []
            
            # Create the output directory for the test data
            try:
                rospack = rospkg.RosPack()
                self.unit_test_path = os.path.join(rospack.get_path('unit_tests'), 'sound_detection_test/data')
                os.makedirs(self.unit_test_path, exist_ok=True)
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: Will save filtered audio to {self.unit_test_path}")
            except Exception as e:
                rospy.logerr(f"{self.node_name}: Error setting up test directory: {e}")
                self.save_audio = False
        else:
            self.save_audio = False

        # Retrieve the microphone topic from the configuration file
        microphone_topic = self.extract_topics('Microphone')
        if not microphone_topic:
            rospy.logerr(f"{self.node_name}: Microphone topic not found in topic file.")
            raise ValueError("Missing microphone topic configuration.")

        # Initialize thread lock for shared resources
        self.lock = Lock()
        
        # Timer for periodic status message
        self.last_status_time = rospy.get_time()

        # Set up ROS subscribers and publishers
        self.audio_sub = rospy.Subscriber(microphone_topic, sound_detection_microphone_msg_file, self.audio_callback)
        rospy.loginfo(f"{self.node_name}: Subscribed to {microphone_topic}")
        self.signal_pub = rospy.Publisher('/soundDetection/signal', std_msgs.msg.Float32MultiArray, queue_size=10)
        self.direction_pub = rospy.Publisher('/soundDetection/direction', std_msgs.msg.Float32, queue_size=10)

        if self.unit_tests:
            self.start_timeout_monitor()

    def start_timeout_monitor(self):
        """
        Start a background thread to monitor for audio timeouts.
        Shuts down the node if no audio is received within the timeout period,
        but only after at least one audio message has been received.
        """
        def monitor():
            rate = rospy.Rate(1)  # Check once per second
            while not rospy.is_shutdown():
                # Only check for timeouts if we've received at least one audio message
                if self.received_first_audio:
                    time_since_last = rospy.get_time() - self.last_audio_time
                    if time_since_last > self.audio_timeout:
                        rospy.logwarn(f"{self.node_name}: No audio received for {self.audio_timeout} seconds. Shutting down.")
                        rospy.signal_shutdown("No audio data.")
                rate.sleep()

        threading.Thread(target=monitor, daemon=True).start()
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Audio timeout monitor started (timeout: {self.audio_timeout}s)")

    @staticmethod
    def read_json_file(package_name):
        """
        Read and parse a JSON configuration file from the specified ROS package.
        
        Args:
            package_name (str): Name of the ROS package containing the config file
            
        Returns:
            dict: Configuration data from JSON file, or empty dict if file not found
        """
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path(package_name)
            
            # Determine the directory and file name based on the package name
            if package_name == 'unit_tests':
                directory = 'sound_detection_test/config'
                config_file = 'sound_detection_test_configuration.json'
            else:
                directory = 'sound_detection/config'
                config_file = 'sound_detection_configuration.json'
            
            config_path = os.path.join(package_path, directory, config_file)
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    data = json.load(file)
                    return data
            else:
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
                return {}
                
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package '{package_name}' not found: {e}")
            return {}
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing JSON configuration file: {e}")
            return {}
        except Exception as e:
            rospy.logerr(f"Unexpected error reading configuration file: {e}")
            return {}

    @staticmethod
    def extract_topics(topic_key):
        """
        Extract the topic name for a given key from the topics data file.
        
        Args:
            topic_key (str): Key to search for in the topics file
            
        Returns:
            str or None: The topic name if found, None otherwise
        """
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('cssr_system')
            config_path = os.path.join(package_path, 'sound_detection/data', 'pepper_topics.dat')
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
                rospy.logerr(f"Topics data file not found at {config_path}")
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package 'cssr_system' not found: {e}")
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
    
    def save_test_audio(self):
        """
        Save the collected filtered audio samples to a file for unit testing.
        Applies RMS normalization before saving to ensure consistent volume levels.
        Only called when unit_tests mode is enabled.
        """
        try:
            if not self.save_audio or len(self.filtered_buffer) == 0:
                return
                
            # Convert buffer to numpy array
            audio_data = np.array(self.filtered_buffer, dtype=np.float32)
            
            # Apply RMS normalization
            audio_data = self.normalize_rms(audio_data)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = os.path.join(self.unit_test_path, f"sound_detection_test_noise_filtered_audio_{timestamp}.wav")
            
            # Save to WAV file
            sf.write(filename, audio_data, self.frequency_sample)
            
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Saved normalized filtered audio test file to {filename}")
            
            # Reset buffer
            self.filtered_buffer = []
            self.saved_samples = 0
            
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error saving test audio: {e}")
    
    def apply_noise_reduction(self, current_block):
        """
        Apply noise reduction to an audio block using the rolling context window approach.
        Only processes the left channel.
        
        Args:
            current_block (np.ndarray): New audio block to process
            
        Returns:
            np.ndarray: Noise-reduced audio block
        """
        try:
            # Skip if noise reduction is disabled
            if not self.use_noise_reduction:
                return current_block
                
            # Update context window: shift old data left and add new block at the end
            block_size_actual = len(current_block)
            self.left_context_window = np.roll(self.left_context_window, -block_size_actual)
            self.left_context_window[-block_size_actual:] = current_block
            
            # Apply stationary noise reduction to the context window
            reduced_context = nr.reduce_noise(
                y=self.left_context_window,
                sr=self.frequency_sample,
                stationary=self.noise_type,
                prop_decrease=self.prop_decrease,
            )
            
            # Extract only the most recent block from the processed context
            processed_block = reduced_context[-block_size_actual:]
            
            return processed_block
            
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in noise reduction: {e}")
            return current_block  # Return original block on error
    
    def voice_detected(self, audio_frame):
        """
        Use Voice Activity Detection (VAD) to determine if voice is present.
        
        Args:
            audio_frame (np.ndarray): Audio frame to analyze
            
        Returns:
            bool: True if voice is detected, False otherwise
        """
        try:
            # Process the audio in VAD frame-sized chunks
            for start in range(0, len(audio_frame) - self.vad_frame_size + 1, self.vad_frame_size):
                frame = audio_frame[start:start + self.vad_frame_size]
                
                # Convert to int16 bytes for WebRTC VAD
                frame_bytes = (frame * 32767).astype(np.int16).tobytes()
                
                # Check if this frame contains speech
                if self.vad.is_speech(frame_bytes, self.frequency_sample):
                    return True
            return False
        except Exception as e:
            rospy.logwarn(f"{self.node_name}: Error in VAD processing: {e}")
            return False

    def audio_callback(self, msg):
        """
        Process incoming audio data from the microphone.
        If in unit test mode, collects filtered audio for testing.
        
        Args:
            msg (microphone_msg_file): The audio data message
        """
        try:
            self.last_audio_time = rospy.get_time()
    
            # If this is the first audio message, log it and set the flag
            if not self.received_first_audio:
                self.received_first_audio = True
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: First audio data received, timeout monitoring active")
            
            # Print a status message every 10 seconds
            current_time = rospy.get_time()
            if current_time - self.last_status_time >= 10:
                rospy.loginfo(f"{self.node_name}: running.")
                self.last_status_time = current_time
                
            # Process audio data
            sigIn_frontLeft, sigIn_frontRight = self.process_audio_data(msg)

            # Check intensity threshold
            if not self.is_intense_enough(sigIn_frontLeft):
                return
                       
            # Apply noise reduction only to the left channel
            sigIn_frontLeft_clean = self.apply_noise_reduction(sigIn_frontLeft)
            
            # Check for voice activity in the left channel (using noise-reduced signal for better detection)
            self.speech_detected = self.voice_detected(sigIn_frontLeft_clean)
            # Save filtered audio for unit tests if enabled and speech is detected
            if self.unit_tests and self.save_audio:
                # Add the filtered audio to the test buffer
                if self.saved_samples < self.max_samples_to_save:
                    self.filtered_buffer.extend(sigIn_frontLeft_clean)
                    self.saved_samples += len(sigIn_frontLeft_clean)
                    
                    # Log progress periodically
                    if self.saved_samples % (self.frequency_sample) == 0:  # Log every second
                        seconds = self.saved_samples / self.frequency_sample
                        total_seconds = self.max_samples_to_save / self.frequency_sample
                        if self.verbose_mode:
                            rospy.loginfo(f"{self.node_name}: Collected {seconds:.1f}/{total_seconds:.1f}s of audio for testing")
  
                # If we've collected enough samples, save the file
                if self.saved_samples >= self.max_samples_to_save:
                    self.save_test_audio()

            # If no speech detected, we can skip further processing
            if not self.speech_detected:
                return
            
            # Publish the noise-reduced left channel signal
            self.publish_signal(sigIn_frontLeft_clean)

            # Update localization buffers with RAW signals (not noise-reduced)
            with self.lock:
                self.update_buffers(sigIn_frontLeft, sigIn_frontRight)

            # Localization processing
            if self.accumulated_samples >= self.localization_buffer_size:
                
                # Perform localization
                self.localize(self.frontleft_buffer, self.frontright_buffer)
                
                # Reset buffers for next batch
                with self.lock:
                    self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
                    self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
                    self.accumulated_samples = 0

        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in audio_callback: {e}")

    def process_audio_data(self, msg):
        """
        Extract and normalize audio data from the message.
        
        Args:
            msg (microphone_msg_file): The audio data message
            
        Returns:
            tuple: (left_channel, right_channel) as normalized float32 arrays
        """
        try:
            # Convert int16 data to float32 and normalize to [-1.0, 1.0]
            sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
            sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0
            return sigIn_frontLeft, sigIn_frontRight
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error processing audio data: {e}")
            return (np.zeros(self.localization_buffer_size, dtype=np.float32),
                    np.zeros(self.localization_buffer_size, dtype=np.float32))

    def is_intense_enough(self, signal_data):
        """
        Check if the signal intensity exceeds the threshold.
        
        Args:
            signal_data (np.ndarray): The audio signal data
            
        Returns:
            bool: True if signal is intense enough, False otherwise
        """
        # Calculate root mean square (RMS) intensity
        intensity = np.sqrt(np.mean(signal_data ** 2))
        return intensity >= self.intensity_threshold

    def update_buffers(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Update the internal buffers with new audio data.
        
        Args:
            sigIn_frontLeft (np.ndarray): Left channel audio data
            sigIn_frontRight (np.ndarray): Right channel audio data
        """
        data_length = len(sigIn_frontLeft)
        if self.accumulated_samples + data_length <= self.localization_buffer_size:
            # There's room for all the new data
            start_index = self.accumulated_samples
            end_index = start_index + data_length
            self.frontleft_buffer[start_index:end_index] = sigIn_frontLeft
            self.frontright_buffer[start_index:end_index] = sigIn_frontRight
            self.accumulated_samples += data_length
        else:
            # Only part of the new data will fit
            remaining = self.localization_buffer_size - self.accumulated_samples
            if remaining > 0:
                self.frontleft_buffer[self.accumulated_samples:] = sigIn_frontLeft[:remaining]
                self.frontright_buffer[self.accumulated_samples:] = sigIn_frontRight[:remaining]
                self.accumulated_samples = self.localization_buffer_size

    def localize(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Localize the sound source using the GCC-PHAT algorithm.
        
        Args:
            sigIn_frontLeft (np.ndarray): Left channel audio data
            sigIn_frontRight (np.ndarray): Right channel audio data
        """
        try:
            # Calculate Interaural Time Difference (ITD)
            itd = self.gcc_phat(sigIn_frontLeft, sigIn_frontRight, self.frequency_sample)
            
            # Convert ITD to angle
            angle = self.calculate_angle(itd)
            
            # Publish the calculated angle
            self.publish_angle(angle)
        except Exception as e:
            rospy.logwarn(f"{self.node_name}: Error in localization: {e}")

    def gcc_phat(self, sig, ref_sig, fs, max_tau=None, interp=16):
        """
        Implement the GCC-PHAT algorithm for time delay estimation.
        
        Args:
            sig (np.ndarray): Signal from first channel
            ref_sig (np.ndarray): Signal from reference channel
            fs (int): Sampling frequency
            max_tau (float, optional): Maximum delay to consider
            interp (int, optional): Interpolation factor
            
        Returns:
            float: Estimated time delay in seconds
        """
        try:
            # Compute FFT length
            n = sig.shape[0] + ref_sig.shape[0]
            
            # Compute FFTs
            SIG = np.fft.rfft(sig, n=n)
            REFSIG = np.fft.rfft(ref_sig, n=n)
            
            # Compute cross-correlation in frequency domain
            R = SIG * np.conj(REFSIG)
            
            # Apply phase transform (PHAT)
            R /= (np.abs(R) + 1e-10)
            
            # Compute inverse FFT to get time-domain cross-correlation
            cc = np.fft.irfft(R, n=n)
            
            # Find maximum correlation
            max_shift = int(n / 2)
            if max_tau:
                max_shift = min(int(fs * max_tau), max_shift)
            
            # Concatenate the end and beginning of cc to align the shifts properly
            cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
            
            # Find the shift that gives maximum correlation
            shift = np.argmax(np.abs(cc)) - max_shift
            
            # Convert shift to time
            return shift / float(fs)
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in GCC-PHAT: {e}")
            return 

    def calculate_angle(self, itd):
        """
        Calculate the sound source angle from the ITD.
        
        Args:
            itd (float): Interaural Time Difference in seconds
            
        Returns:
            float: Sound source angle in degrees
        """
        try:
            # Calculate sine of the angle
            z = itd * (self.speed_of_sound / self.distance_between_ears)
            
            # Clamp value to valid range for arcsin
            z = max(-1.0, min(1.0, z))
            
            # Calculate angle in degrees
            angle = math.asin(z) * (180.0 / math.pi)

            # If the angel is not in [-67 , 67], skip it 
            if angle < -67 or angle > 67:
                return

            return angle
        except ValueError as e:
            rospy.logwarn(f"{self.node_name}: Invalid ITD for angle calculation: {e}")
            return

    def publish_angle(self, angle):
        """
        Publish the calculated angle to the direction topic.
        
        Args:
            angle (float): Sound source angle in degrees
        """
        if angle is None:
            return
        angle_msg = std_msgs.msg.Float32()
        angle_msg.data = angle
        self.direction_pub.publish(angle_msg)

    def publish_signal(self, signal_data):
        """
        Publish the processed signal to the signal topic.
        
        Args:
            signal_data (np.ndarray): Processed audio signal
        """
        signal_msg = Float32MultiArray()
        signal_msg.data = signal_data.tolist()
        self.signal_pub.publish(signal_msg)
        
    def on_shutdown(self):
        """
        Handle cleanup when the node is shutting down.
        Saves any remaining test audio.
        """
        if self.unit_tests and self.save_audio and len(self.filtered_buffer) > 0:
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Saving remaining test audio before shutdown")
            self.save_test_audio()

    def spin(self):
        """
        Main processing loop for the node.
        """
        # Register shutdown handler
        rospy.on_shutdown(self.on_shutdown)
        rospy.spin()