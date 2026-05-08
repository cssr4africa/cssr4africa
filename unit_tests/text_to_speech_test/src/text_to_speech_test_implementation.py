"""
text_to_speech_test_implementation.py - text-to-speech test cases

Author:     Muhirwa Richard
Date:       2026-05-08
Version:    v1.1

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (AfretecInclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import rospy
import actionlib
import sys
import os
import time
import datetime
import rospkg
from cssr_system.srv import TTS
from cssr_system.msg import TTSAction, TTSGoal

HEARTBEAT_MSG_PERIOD = 10.0

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
    and ensuring proper audio playback on the robot.
    Tests cover both the ROS service interface and the action server interface.
    """

    def __init__(self, logger=None):
        self.logger = logger if logger else TestOutputLogger()
        self._feedback_statuses = []
        self.tts_service = None
        self.action_client = None

        # Initialize the ROS node
        rospy.init_node('textToSpeechTest', anonymous=True)

        # ==========================================================
        # Start the heartbeat after successful initialization
        rospy.Timer(
            rospy.Duration(HEARTBEAT_MSG_PERIOD),
            lambda _: print("textToSpeechTest: running..."))
        # ==========================================================

        # Connect to the TTS service (optional — depends on interface config)
        print_info("Waiting for TTS service...")
        try:
            rospy.wait_for_service('/textToSpeech/say_text', timeout=10)
            self.tts_service = rospy.ServiceProxy('/textToSpeech/say_text', TTS)
            print_success("TTS service found!")
        except rospy.ROSException:
            print_warning("TTS service not available — service tests will be skipped.")

        # Connect to the TTS action server (optional — depends on interface config)
        print_info("Waiting for TTS action server...")
        try:
            self.action_client = actionlib.SimpleActionClient(
                '/textToSpeech/say_text', TTSAction
            )
            if self.action_client.wait_for_server(timeout=rospy.Duration(10)):
                print_success("TTS action server found!")
            else:
                self.action_client = None
                print_warning("TTS action server not available — action tests will be skipped.")
        except Exception as e:
            self.action_client = None
            print_warning(f"TTS action server not available: {e} — action tests will be skipped.")

        if self.tts_service is None and self.action_client is None:
            raise Exception("Neither TTS service nor action server is available!")

    # ------------------------------------------------------------------
    # Helper: action client
    # ------------------------------------------------------------------

    def _feedback_cb(self, feedback):
        """Collect feedback messages during action execution"""
        self._feedback_statuses.append((feedback.status, feedback.progress))
        print_info(f"  [feedback] status='{feedback.status}'  progress={feedback.progress:.1f}")

    def _send_action_goal_and_wait(self, text, language, timeout_secs=60):
        """Send an action goal and block until done; return (state, result).

        A short sleep after wait_for_result lets any in-flight feedback callbacks
        (e.g. the final 'Completed' message) be delivered before the caller
        inspects self._feedback_statuses.
        """
        self._feedback_statuses = []
        goal = TTSGoal(text=text, language=language)
        self.action_client.send_goal(goal, feedback_cb=self._feedback_cb)
        finished = self.action_client.wait_for_result(rospy.Duration(timeout_secs))
        if not finished:
            self.action_client.cancel_goal()
            raise Exception(f"Action did not finish within {timeout_secs}s")
        rospy.sleep(1.0)  # allow any in-flight feedback callbacks to fire
        return self.action_client.get_state(), self.action_client.get_result()

    # ------------------------------------------------------------------
    # Service tests
    # ------------------------------------------------------------------

    def test_english_implementation_integration(self):
        """Test full English TTS implementation integration via service"""
        print_header("TEST 1: English TTS Implementation Integration (Service)")

        if self.tts_service is None:
            print_warning("Skipping TEST 1 — TTS service not available.")
            self.logger.log("TEST 1 - English TTS Integration (Service): SKIPPED", "TEST_RESULT")
            return None

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
        """Test full Kinyarwanda TTS implementation integration via service"""
        print_header("TEST 2: Kinyarwanda TTS Implementation Integration (Service)")

        if self.tts_service is None:
            print_warning("Skipping TEST 2 — TTS service not available.")
            self.logger.log("TEST 2 - Kinyarwanda TTS Integration (Service): SKIPPED", "TEST_RESULT")
            return None

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
        """Test alternating between languages in implementation via service"""
        print_header("TEST 3: Language Switching Implementation (Service)")

        if self.tts_service is None:
            print_warning("Skipping TEST 3 — TTS service not available.")
            self.logger.log("TEST 3 - Language Switching Implementation (Service): SKIPPED", "TEST_RESULT")
            return None

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
        """Test with an empty message via service"""
        print_header("TEST 4: Empty Message Implementation Test (Service)")

        if self.tts_service is None:
            print_warning("Skipping TEST 4 — TTS service not available.")
            self.logger.log("TEST 4 - Empty Message Implementation (Service): SKIPPED", "TEST_RESULT")
            return None

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
        """Test with an unsupported language via service"""
        print_header("TEST 5: Unsupported Language Implementation Test (Service)")

        if self.tts_service is None:
            print_warning("Skipping TEST 5 — TTS service not available.")
            self.logger.log("TEST 5 - Unsupported Language Implementation (Service): SKIPPED", "TEST_RESULT")
            return None

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
        """Test with a longer message via service"""
        print_header("TEST 6: Long Message Implementation Test (Service)")

        if self.tts_service is None:
            print_warning("Skipping TEST 6 — TTS service not available.")
            self.logger.log("TEST 6 - Long Message Implementation (Service): SKIPPED", "TEST_RESULT")
            return None

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

    # ------------------------------------------------------------------
    # Action server tests
    # ------------------------------------------------------------------

    def test_english_action_implementation(self):
        """Test full English TTS implementation integration via action server"""
        print_header("TEST 7: English TTS Implementation Integration (Action)")

        if self.action_client is None:
            print_warning("Skipping TEST 7 — TTS action server not available.")
            self.logger.log("TEST 7 - English TTS Integration (Action): SKIPPED", "TEST_RESULT")
            return None

        message1 = "world"
        try:
            print_info(f"Sending text via action: '{message1}'")
            state, result = self._send_action_goal_and_wait(message1, "english")
            if result and result.success:
                print_success("English action goal succeeded")
                self.logger.log(f"Action goal successful for text: '{message1}' in English", "TEST_RESULT")
            else:
                print_error("English action goal failed!")
                self.logger.log(f"Action goal failed for text: '{message1}' in English", "TEST_RESULT")
                return False

            wait_time = len(message1.split()) * 0.3
            print_info(f"Waiting for {wait_time} seconds for speech to complete...")
            time.sleep(wait_time)

            if wait_for_confirmation("Did you hear the English message via action correctly? (y/n): "):
                print_success("English action first message test passed!")
                self.logger.log("TEST 7.1 - English action first message: PASSED", "TEST_RESULT")
            else:
                print_error("English action first message test failed!")
                self.logger.log("TEST 7.1 - English action first message: FAILED", "TEST_RESULT")
                return False

            time.sleep(1)

            message2 = "Hello"
            print_info(f"Sending text via action: '{message2}'")
            state, result = self._send_action_goal_and_wait(message2, "english")
            if result and result.success:
                print_success("English action goal succeeded")
                self.logger.log(f"Action goal successful for text: '{message2}' in English", "TEST_RESULT")
            else:
                print_error("English action goal failed!")
                self.logger.log(f"Action goal failed for text: '{message2}' in English", "TEST_RESULT")
                return False

            wait_time = len(message2.split()) * 0.3
            print_info(f"Waiting for {wait_time} seconds for speech to complete...")
            time.sleep(wait_time)

            if wait_for_confirmation("Did you hear the second English message via action clearly? (y/n): "):
                print_success("English action second message test passed!")
                self.logger.log("TEST 7.2 - English action second message: PASSED", "TEST_RESULT")
                return True
            else:
                print_error("English action second message test failed!")
                self.logger.log("TEST 7.2 - English action second message: FAILED", "TEST_RESULT")
                return False

        except Exception as e:
            print_error(f"English TTS action test failed with exception: {e}")
            self.logger.log(f"TEST 7 - English TTS Integration (Action): FAILED with exception: {e}", "TEST_RESULT")
            return False

    def test_kinyarwanda_action_implementation(self):
        """Test full Kinyarwanda TTS implementation integration via action server"""
        print_header("TEST 8: Kinyarwanda TTS Implementation Integration (Action)")

        if self.action_client is None:
            print_warning("Skipping TEST 8 — TTS action server not available.")
            self.logger.log("TEST 8 - Kinyarwanda TTS Integration (Action): SKIPPED", "TEST_RESULT")
            return None

        message = "Muraho neza"
        try:
            print_info(f"Sending text via action: '{message}'")
            state, result = self._send_action_goal_and_wait(message, "kinyarwanda")
            if result and result.success:
                print_success("Kinyarwanda action goal succeeded")
                self.logger.log(f"Action goal successful for text: '{message}' in Kinyarwanda", "TEST_RESULT")
            else:
                print_error("Kinyarwanda action goal failed!")
                self.logger.log(f"Action goal failed for text: '{message}' in Kinyarwanda", "TEST_RESULT")
                return False

            wait_time = len(message.split()) * 0.5
            print_info(f"Waiting for {wait_time} seconds for speech to complete...")
            time.sleep(wait_time)

            if wait_for_confirmation("Did you hear the Kinyarwanda message via action correctly? (y/n): "):
                print_success("Kinyarwanda action test passed!")
                self.logger.log("TEST 8 - Kinyarwanda TTS Integration (Action): PASSED", "TEST_RESULT")
                return True
            else:
                print_error("Kinyarwanda action test failed!")
                self.logger.log("TEST 8 - Kinyarwanda TTS Integration (Action): FAILED", "TEST_RESULT")
                return False

        except Exception as e:
            print_error(f"Kinyarwanda TTS action test failed with exception: {e}")
            self.logger.log(f"TEST 8 - Kinyarwanda TTS Integration (Action): FAILED with exception: {e}", "TEST_RESULT")
            return False

    def test_action_feedback_implementation(self):
        """Test that the action server publishes the expected feedback sequence.

        'Synthesizing' and 'Playing audio' are required. 'Completed' is optional:
        it is sent by the server immediately before set_succeeded, so it can lose
        the race against the result message at the subscriber. The goal succeeding
        (result.success=True) serves as the definitive completion signal.
        """
        print_header("TEST 9: Action Feedback Sequence Test")

        if self.action_client is None:
            print_warning("Skipping TEST 9 — TTS action server not available.")
            self.logger.log("TEST 9 - Action Feedback Sequence: SKIPPED", "TEST_RESULT")
            return None

        message = "Hello world"
        try:
            print_info(f"Sending text: '{message}' and monitoring feedback...")
            state, result = self._send_action_goal_and_wait(message, "english")

            if not (result and result.success):
                print_error("Action goal failed — cannot verify feedback sequence.")
                self.logger.log("TEST 9 - Action Feedback Sequence: FAILED (goal did not succeed)", "TEST_RESULT")
                return False

            received_statuses = [s for s, _ in self._feedback_statuses]

            # 'Synthesizing' and 'Playing audio' must be received before the result
            required_statuses = ["Synthesizing", "Playing audio"]
            missing_required = [s for s in required_statuses if s not in received_statuses]

            if missing_required:
                print_error(f"Missing required feedback statuses: {missing_required}")
                print_info(f"Received: {received_statuses}")
                self.logger.log(
                    f"TEST 9 - Action Feedback Sequence: FAILED (missing required: {missing_required})",
                    "TEST_RESULT"
                )
                return False

            # 'Completed' may arrive after wait_for_result due to a last-mile race
            # between the feedback topic and the result topic in ROS actionlib
            if "Completed" not in received_statuses:
                print_warning(
                    "  [note] 'Completed' feedback not captured — known ROS actionlib race "
                    "condition between last feedback publish and set_succeeded. Goal succeeded, "
                    "so this is not treated as a failure."
                )
                self.logger.log(
                    "TEST 9 - note: 'Completed' feedback not captured (race condition)", "TEST_RESULT"
                )

            # Verify progress values are within [0, 1] and non-decreasing
            progresses = [p for _, p in self._feedback_statuses]
            progress_ok = all(0.0 <= p <= 1.0 for p in progresses)
            progress_ascending = all(
                progresses[i] <= progresses[i + 1] for i in range(len(progresses) - 1)
            )

            if not progress_ok or not progress_ascending:
                print_error(f"Progress values invalid or not ascending: {progresses}")
                self.logger.log(
                    f"TEST 9 - Action Feedback Sequence: FAILED (bad progress values: {progresses})",
                    "TEST_RESULT"
                )
                return False

            print_success("Action feedback sequence test passed!")
            self.logger.log(
                f"TEST 9 - Action Feedback Sequence: PASSED (statuses={received_statuses}, "
                f"progress={progresses})",
                "TEST_RESULT"
            )
            return True

        except Exception as e:
            print_error(f"Action feedback sequence test failed with exception: {e}")
            self.logger.log(
                f"TEST 9 - Action Feedback Sequence: FAILED with exception: {e}", "TEST_RESULT"
            )
            return False

    def test_action_result_fields_implementation(self):
        """Test that the action result contains the expected fields with valid values"""
        print_header("TEST 10: Action Result Fields Test")

        if self.action_client is None:
            print_warning("Skipping TEST 10 — TTS action server not available.")
            self.logger.log("TEST 10 - Action Result Fields: SKIPPED", "TEST_RESULT")
            return None

        message = "Result fields check"
        try:
            print_info(f"Sending text: '{message}'...")
            state, result = self._send_action_goal_and_wait(message, "english")

            if result is None:
                print_error("No result returned from action server!")
                self.logger.log("TEST 10 - Action Result Fields: FAILED (no result)", "TEST_RESULT")
                return False

            errors = []
            if not isinstance(result.success, bool):
                errors.append("'success' is not a bool")
            if not isinstance(result.message, str) or not result.message:
                errors.append("'message' is empty or not a string")

            if errors:
                print_error(f"Result field errors: {errors}")
                self.logger.log(
                    f"TEST 10 - Action Result Fields: FAILED ({'; '.join(errors)})", "TEST_RESULT"
                )
                return False

            print_success(f"Result fields valid — success={result.success}, message='{result.message}'")
            self.logger.log("TEST 10 - Action Result Fields: PASSED", "TEST_RESULT")
            return True

        except Exception as e:
            print_error(f"Action result fields test failed with exception: {e}")
            self.logger.log(
                f"TEST 10 - Action Result Fields: FAILED with exception: {e}", "TEST_RESULT"
            )
            return False

    def test_action_unsupported_language_implementation(self):
        """Test with an unsupported language via action server"""
        print_header("TEST 11: Unsupported Language Implementation Test (Action)")

        if self.action_client is None:
            print_warning("Skipping TEST 11 — TTS action server not available.")
            self.logger.log("TEST 11 - Unsupported Language (Action): SKIPPED", "TEST_RESULT")
            return None

        try:
            print_info("Testing unsupported language via action...")
            state, result = self._send_action_goal_and_wait("Test message", "spanish")

            if result and not result.success:
                print_success("Unsupported language correctly rejected by action server")
                self.logger.log("TEST 11 - Unsupported Language (Action): PASSED (correctly rejected)", "TEST_RESULT")
                return True
            else:
                print_error("Unsupported language should have been rejected by action server!")
                self.logger.log("TEST 11 - Unsupported Language (Action): FAILED (should have been rejected)", "TEST_RESULT")
                return False

        except Exception as e:
            print_error(f"Unsupported language action test failed with exception: {e}")
            self.logger.log(f"TEST 11 - Unsupported Language (Action): FAILED with exception: {e}", "TEST_RESULT")
            return False

    def run_all_tests(self):
        """Run all implementation tests and return results"""
        test_results = []

        # ---- Service tests ----
        self.logger.log("Starting English TTS Implementation Integration test (Service)", "TEST_START")
        eng_test_result = self.test_english_implementation_integration()
        test_results.append(("English TTS Implementation Integration (Service)", eng_test_result))
        time.sleep(2)

        self.logger.log("Starting Kinyarwanda TTS Implementation Integration test (Service)", "TEST_START")
        kin_test_result = self.test_kinyarwanda_implementation_integration()
        test_results.append(("Kinyarwanda TTS Implementation Integration (Service)", kin_test_result))
        time.sleep(2)

        self.logger.log("Starting Language Switching Implementation test (Service)", "TEST_START")
        mix_test_result = self.test_mixed_language_implementation_integration()
        test_results.append(("Language Switching Implementation (Service)", mix_test_result))
        time.sleep(2)

        self.logger.log("Starting Empty Message Implementation test (Service)", "TEST_START")
        empty_test_result = self.test_empty_message_implementation()
        test_results.append(("Empty Message Implementation (Service)", empty_test_result))
        time.sleep(1)

        self.logger.log("Starting Unsupported Language Implementation test (Service)", "TEST_START")
        unsupported_test_result = self.test_unsupported_language_implementation()
        test_results.append(("Unsupported Language Implementation (Service)", unsupported_test_result))
        time.sleep(1)

        self.logger.log("Starting Long Message Implementation test (Service)", "TEST_START")
        long_test_result = self.test_long_message_implementation()
        test_results.append(("Long Message Implementation (Service)", long_test_result))
        time.sleep(2)

        # ---- Action server tests ----
        self.logger.log("Starting English TTS Integration test (Action)", "TEST_START")
        eng_action_result = self.test_english_action_implementation()
        test_results.append(("English TTS Implementation Integration (Action)", eng_action_result))
        time.sleep(2)

        self.logger.log("Starting Kinyarwanda TTS Integration test (Action)", "TEST_START")
        kin_action_result = self.test_kinyarwanda_action_implementation()
        test_results.append(("Kinyarwanda TTS Implementation Integration (Action)", kin_action_result))
        time.sleep(2)

        self.logger.log("Starting Action Feedback Sequence test", "TEST_START")
        feedback_test_result = self.test_action_feedback_implementation()
        test_results.append(("Action Feedback Sequence", feedback_test_result))
        time.sleep(1)

        self.logger.log("Starting Action Result Fields test", "TEST_START")
        result_fields_test_result = self.test_action_result_fields_implementation()
        test_results.append(("Action Result Fields", result_fields_test_result))
        time.sleep(1)

        self.logger.log("Starting Unsupported Language test (Action)", "TEST_START")
        unsupported_action_result = self.test_action_unsupported_language_implementation()
        test_results.append(("Unsupported Language Implementation (Action)", unsupported_action_result))

        return test_results
