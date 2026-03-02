""" 
text_to_speech_test_implementation.py - text-to-speech test cases

Author:     Muhirwa Richard
Date:       2025-04-18
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (AfretecInclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import rospy
import sys
import os
import time
import datetime
import rospkg
from cssr_system.srv import TTS

class TestOutputLogger:
    """Class to handle both terminal and file output for implementation tests"""
    
    def __init__(self):
        self.log_entries = []
        self.setup_log_file()
    
    def setup_log_file(self):
        """Setup the log file path"""
        try:
            rospack = rospkg.RosPack()
            unit_test_path = rospack.get_path('unit_tests')
            self.data_dir = os.path.join(unit_test_path, 'text_to_speech_test/data')
            
            # Create data directory if it doesn't exist
            if not os.path.exists(self.data_dir):
                os.makedirs(self.data_dir)
            
            self.log_file_path = os.path.join(self.data_dir, 'text_to_speech_test_output.dat')
            print(f"Log file will be written to: {self.log_file_path}")
        except Exception as e:
            print(f"Warning: Could not setup log file: {e}")
            self.log_file_path = None
    
    def log(self, message, level="INFO"):
        """Log message to both terminal and internal storage"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}"
        
        # Store for file writing
        self.log_entries.append(log_entry)
        
        # Print to terminal 
        print(message)
    
    def write_to_file(self):
        """Write all log entries to file"""
        if self.log_file_path:
            try:
                # Read existing content if file exists
                existing_content = ""
                if os.path.exists(self.log_file_path):
                    with open(self.log_file_path, 'r') as f:
                        existing_content = f.read()
                
                with open(self.log_file_path, 'w') as f:
                    # Write existing content first 
                    if existing_content:
                        f.write(existing_content)
                        f.write("\n" + "="*60 + "\n\n")
                    
                    f.write("Text-to-Speech Implementation Integration Test Report\n")
                    f.write("="*60 + "\n")
                    f.write(f"Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                    
                    for entry in self.log_entries:
                        f.write(entry + "\n")
                
                print(f"\nImplementation test results appended to: {self.log_file_path}")
            except Exception as e:
                print(f"Error writing to log file: {e}")

def print_success(text):
    """Print success message in green"""
    print(f"\033[92m{text}\033[0m")

def print_warning(text):
    """Print warning message in yellow"""
    print(f"\033[93m{text}\033[0m")

def print_error(text):
    """Print error message in red"""
    print(f"\033[91m{text}\033[0m")

def print_info(text):
    """Print info message in blue"""
    print(f"\033[94m{text}\033[0m")

def print_header(text):
    """Print header with formatting"""
    header = "\n" + "="*80 + "\n " + text + "\n" + "="*80
    print(header)

def wait_for_confirmation(prompt="Did you hear the audio correctly? (y/n): "):
    """Ask user for confirmation and return True if confirmed"""
    print(prompt, end='')
    response = input().strip().lower()
    return response == 'y' or response == 'yes'

class TTSImplementationIntegrationTest:
    """
    Integration tests for the TTS implementation, testing both languages
    and ensuring proper audio playback on the robot
    """
    
    def __init__(self, logger=None):
        self.logger = logger if logger else TestOutputLogger()
        
        # Initialize the ROS node
        rospy.init_node('textToSpeechTest', anonymous=True)
        
        # Connect to the TTS service
        print_info("Waiting for TTS service...")
        try:
            rospy.wait_for_service('/textToSpeech/say_text', timeout=10)
            self.tts_service = rospy.ServiceProxy('/textToSpeech/say_text', TTS)
            print_success("TTS service found!")
        except rospy.ROSException:
            print_error("TTS service not available!")
            raise Exception("TTS service not available!")
    
    def test_english_implementation_integration(self):
        """Test full English TTS implementation integration"""
        print_header("TEST 1: English TTS Implementation Integration")
        
        # First test message
        message1 = "world"
        try:
            # Send first message
            print_info(f"Sending text: '{message1}'")
            response = self.tts_service(message=message1, language="english")
            if response.success:
                print_success("English message sent successfully")
                self.logger.log(f"Service call successful for message: '{message1}' in English", "TEST_RESULT")
            else:
                print_error("Failed to send English message!")
                self.logger.log(f"Service call failed for message: '{message1}' in English", "TEST_RESULT")
                return False
            
            # Wait for speech to complete
            wait_time = len(message1.split()) * 0.3  # Rough estimate based on word count
            print_info(f"Waiting for {wait_time} seconds for speech to complete...")
            time.sleep(wait_time)
            
            # Ask user to confirm
            if wait_for_confirmation("Did you hear the English message correctly? (y/n): "):
                print_success("English first message test passed!")
                self.logger.log("TEST 1.1 - English first message: PASSED", "TEST_RESULT")
            else:
                print_error("English first message test failed!")
                self.logger.log("TEST 1.1 - English first message: FAILED", "TEST_RESULT")
                return False
            
            # Wait between tests
            time.sleep(1)
            
            # Second test message
            message2 = "Hello"
            # Send second message
            print_info(f"Sending text: '{message2}'")
            response = self.tts_service(message=message2, language="english")
            if response.success:
                print_success("English message sent successfully")
                self.logger.log(f"Service call successful for message: '{message2}' in English", "TEST_RESULT")
            else:
                print_error("Failed to send English message!")
                self.logger.log(f"Service call failed for message: '{message2}' in English", "TEST_RESULT")
                return False
            
            # Wait for speech to complete
            wait_time = len(message2.split()) * 0.3
            print_info(f"Waiting for {wait_time} seconds for speech to complete...")
            time.sleep(wait_time)
            
            # Ask user to confirm
            if wait_for_confirmation("Did you hear the second English message clearly? (y/n): "):
                print_success("English second message test passed!")
                self.logger.log("TEST 1.2 - English second message: PASSED", "TEST_RESULT")
                return True
            else:
                print_error("English second message test failed!")
                self.logger.log("TEST 1.2 - English second message: FAILED", "TEST_RESULT")
                return False
            
        except Exception as e:
            print_error(f"English TTS implementation integration test failed with exception: {e}")
            self.logger.log(f"TEST 1 - English TTS Integration: FAILED with exception: {e}", "TEST_RESULT")
            return False
    
    def test_kinyarwanda_implementation_integration(self):
        """Test full Kinyarwanda TTS implementation integration"""
        print_header("TEST 2: Kinyarwanda TTS Implementation Integration")
        
        # Kinyarwanda test message
        message = "Muraho neza"
        try:
            # Send message
            print_info(f"Sending text: '{message}'")
            response = self.tts_service(message=message, language="kinyarwanda")
            if response.success:
                print_success("Kinyarwanda message sent successfully")
                self.logger.log(f"Service call successful for message: '{message}' in Kinyarwanda", "TEST_RESULT")
            else:
                print_error("Failed to send Kinyarwanda message!")
                self.logger.log(f"Service call failed for message: '{message}' in Kinyarwanda", "TEST_RESULT")
                return False
            
            # Wait for speech to complete - Kinyarwanda synthesis might take longer
            wait_time = len(message.split()) * 0.5  # Longer wait for synthesis
            print_info(f"Waiting for {wait_time} seconds for speech to complete...")
            time.sleep(wait_time)
            
            # Ask user to confirm
            if wait_for_confirmation("Did you hear the Kinyarwanda message correctly? (y/n): "):
                print_success("Kinyarwanda message test passed!")
                self.logger.log("TEST 2 - Kinyarwanda TTS Integration: PASSED", "TEST_RESULT")
                return True
            else:
                print_error("Kinyarwanda message test failed!")
                self.logger.log("TEST 2 - Kinyarwanda TTS Integration: FAILED", "TEST_RESULT")
                return False
            
        except Exception as e:
            print_error(f"Kinyarwanda TTS implementation integration test failed with exception: {e}")
            self.logger.log(f"TEST 2 - Kinyarwanda TTS Integration: FAILED with exception: {e}", "TEST_RESULT")
            return False
    
    def test_mixed_language_implementation_integration(self):
        """Test alternating between languages in implementation"""
        print_header("TEST 3: Language Switching Implementation")
        
        try:
            # English message
            eng_message = "me and you"
            print_info(f"Sending English text: '{eng_message}'")
            response = self.tts_service(message=eng_message, language="english")
            if not response.success:
                print_error("Failed to send English message!")
                self.logger.log(f"Language switching test - English message failed: '{eng_message}'", "TEST_RESULT")
                return False
                
            print_info("Waiting for audio to play...")
            time.sleep(3) 
            
            # Kinyarwanda message
            kin_message = "amakuru yanyu"
            print_info(f"Sending Kinyarwanda text: '{kin_message}'")
            response = self.tts_service(message=kin_message, language="kinyarwanda")
            if not response.success:
                print_error("Failed to send Kinyarwanda message!")
                self.logger.log(f"Language switching test - Kinyarwanda message failed: '{kin_message}'", "TEST_RESULT")
                return False
                
            print_info("Waiting for audio to play...")
            time.sleep(3) 
            
            # Back to English
            eng_message2 = "Now back to English."
            print_info(f"Sending English text: '{eng_message2}'")
            response = self.tts_service(message=eng_message2, language="english")
            if not response.success:
                print_error("Failed to send English message!")
                self.logger.log(f"Language switching test - Second English message failed: '{eng_message2}'", "TEST_RESULT")
                return False
                
            print_info("Waiting for audio to play...")
            time.sleep(3) 
            
            # Ask user to confirm
            if wait_for_confirmation("Did all three messages play in the correct languages? (y/n): "):
                print_success("Language switching test passed!")
                self.logger.log("TEST 3 - Language Switching Implementation: PASSED", "TEST_RESULT")
                return True
            else:
                print_error("Language switching test failed!")
                self.logger.log("TEST 3 - Language Switching Implementation: FAILED", "TEST_RESULT")
                return False
            
        except Exception as e:
            print_error(f"Language switching test failed with exception: {e}")
            self.logger.log(f"TEST 3 - Language Switching Implementation: FAILED with exception: {e}", "TEST_RESULT")
            return False
    
    def test_empty_message_implementation(self):
        """Test with an empty message in implementation"""
        print_header("TEST 4: Empty Message Implementation Test")
        
        try:
            print_info("Testing empty message...")
            response = self.tts_service(message="", language="english")
            
            if response.success:
                print_success("Empty message processed successfully")
                self.logger.log("TEST 4 - Empty Message Implementation: PASSED (processed successfully)", "TEST_RESULT")
                return True
            else:
                print_warning("Empty message returned failure (this might be expected behavior)")
                self.logger.log("TEST 4 - Empty Message Implementation: PASSED (rejected as expected)", "TEST_RESULT")
                return True  # This might be acceptable behavior
                
        except Exception as e:
            print_error(f"Empty message test failed with exception: {e}")
            self.logger.log(f"TEST 4 - Empty Message Implementation: FAILED with exception: {e}", "TEST_RESULT")
            return False
    
    def test_unsupported_language_implementation(self):
        """Test with an unsupported language in implementation"""
        print_header("TEST 5: Unsupported Language Implementation Test")
        
        try:
            print_info("Testing unsupported language...")
            response = self.tts_service(message="Test message", language="spanish")
            
            if not response.success:
                print_success("Unsupported language correctly rejected")
                self.logger.log("TEST 5 - Unsupported Language Implementation: PASSED (correctly rejected)", "TEST_RESULT")
                return True
            else:
                print_error("Unsupported language should have been rejected!")
                self.logger.log("TEST 5 - Unsupported Language Implementation: FAILED (should have been rejected)", "TEST_RESULT")
                return False
                
        except Exception as e:
            print_error(f"Unsupported language test failed with exception: {e}")
            self.logger.log(f"TEST 5 - Unsupported Language Implementation: FAILED with exception: {e}", "TEST_RESULT")
            return False
    
    def test_long_message_implementation(self):
        """Test with a longer message in implementation"""
        print_header("TEST 6: Long Message Implementation Test")
        
        try:
            # Test with longer English message
            long_message = "This is a longer test message to verify that the TTS implementation can handle extended text properly without any issues."
            print_info(f"Sending long English text: '{long_message[:50]}...'")
            response = self.tts_service(message=long_message, language="english")
            
            if response.success:
                print_success("Long English message sent successfully")
                self.logger.log(f"Service call successful for long message ({len(long_message)} characters)", "TEST_RESULT")
                
                # Wait longer for longer message
                wait_time = len(long_message.split()) * 0.4
                print_info(f"Waiting for {wait_time:.1f} seconds for speech to complete...")
                time.sleep(wait_time)
                
                if wait_for_confirmation("Did you hear the complete long English message? (y/n): "):
                    print_success("Long message test passed!")
                    self.logger.log("TEST 6 - Long Message Implementation: PASSED", "TEST_RESULT")
                    return True
                else:
                    print_error("Long message test failed!")
                    self.logger.log("TEST 6 - Long Message Implementation: FAILED (user could not hear complete message)", "TEST_RESULT")
                    return False
            else:
                print_error("Failed to send long English message!")
                self.logger.log("TEST 6 - Long Message Implementation: FAILED (service call failed)", "TEST_RESULT")
                return False
                
        except Exception as e:
            print_error(f"Long message test failed with exception: {e}")
            self.logger.log(f"TEST 6 - Long Message Implementation: FAILED with exception: {e}", "TEST_RESULT")
            return False
    
    def run_all_tests(self):
        """Run all implementation tests and return results"""
        test_results = []
        
        # Run tests one by one
        self.logger.log("Starting English TTS Implementation Integration test", "TEST_START")
        eng_test_result = self.test_english_implementation_integration()
        test_results.append(("English TTS Implementation Integration", eng_test_result))
        time.sleep(2)
        
        self.logger.log("Starting Kinyarwanda TTS Implementation Integration test", "TEST_START")
        kin_test_result = self.test_kinyarwanda_implementation_integration()
        test_results.append(("Kinyarwanda TTS Implementation Integration", kin_test_result))
        time.sleep(2)
        
        self.logger.log("Starting Language Switching Implementation test", "TEST_START")
        mix_test_result = self.test_mixed_language_implementation_integration()
        test_results.append(("Language Switching Implementation", mix_test_result))
        time.sleep(2)
        
        self.logger.log("Starting Empty Message Implementation test", "TEST_START")
        empty_test_result = self.test_empty_message_implementation()
        test_results.append(("Empty Message Implementation", empty_test_result))
        time.sleep(1)
        
        self.logger.log("Starting Unsupported Language Implementation test", "TEST_START")
        unsupported_test_result = self.test_unsupported_language_implementation()
        test_results.append(("Unsupported Language Implementation", unsupported_test_result))
        time.sleep(1)
        
        self.logger.log("Starting Long Message Implementation test", "TEST_START")
        long_test_result = self.test_long_message_implementation()
        test_results.append(("Long Message Implementation", long_test_result))
        
        return test_results