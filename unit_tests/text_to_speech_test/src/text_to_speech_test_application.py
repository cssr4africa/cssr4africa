#!/usr/bin/env python3

""" 
text_to_speech_test_application.py - ROS-based integration test application for validating Text-to-Speech (TTS) service implementation

Author:     Muhirwa Richard
Date:       2025-04-18
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (AfretecInclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
> text_to_speech_test_application.py - ROS-based integration test application for validating Text-to-Speech (TTS) service implementation.

This application serves as a comprehensive testing framework for validating the functionality and integration of a Text-to-Speech (TTS) service
within a ROS environment.
The primary purpose is to ensure that the TTS implementation correctly processes text input in both English and Kinyarwanda languages and produces 
appropriate audio output on pepper robot.

> Libraries
    - rospy: ROS Python client library for node initialization and service handling
    - sys: System-specific parameters and functions for exit codes
    - text_to_speech_test_implementation: Custom module containing:
        - TTSImplementationIntegrationTest
        - TestOutputLogger
        - print_success, print_warning, print_error, print_info, print_header: Colored output functions

> Parameters
    > Command-line Parameters
        - None (no command-line arguments required)

    > Configuration File Parameters
        - None (no configuration file used)

> Subscribed Topics and Message Types
    - None (no topics subscribed)

> Published Topics and Message Types
    - None (no topics published)

> Services Invoked
    - /textToSpeech/say_text (cssr_system.srv.TTS): Text-to-speech service for converting text to audio output

> Services Advertised and Request Message
    - None (no services advertised)

> Input Data Files
    - None (no input data files required)

> Output Data Files
    - text_to_speech_test_output.dat: Test results and logging output file stored in unit_tests/text_to_speech_test/data/

> Configuration Files
    - None (no configuration files used)

> Example Instantiation of the Module
    rosrun unit_tests text_to_speech_test_application.py

- Author:  Muhirwa Richard, Carnegie Mellon University Africa
- Email:   muhirwarichard1@gmail.com
- Date:    2025-04-18
- Version: v1.0
"""

import rospy
import sys
from text_to_speech_test_implementation import (
    TTSImplementationIntegrationTest,
    TestOutputLogger,
    print_success,
    print_warning,
    print_error,
    print_info,
    print_header
)

software_version = "version v1.0"

class TTSTestApplication:
    """Main application class for running TTS tests"""
    
    def __init__(self):
        self.logger = TestOutputLogger()
    
    def display_welcome_message(self):
        """Display welcome and setup information"""
        copyright_message = (
            f"textToSpeechTest  {software_version}\n"
            "\t\t\t    This project is funded by the African Engineering and Technology Network (Afretec)\n"
            "\t\t\t    Inclusive Digital Transformation Research Grant Programme.\n"
            "\t\t\t    Website: www.cssr4africa.org\n"
            "\t\t\t    This program comes with ABSOLUTELY NO WARRANTY."
         )
        
        # Construct the copyright message
        print_info(copyright_message)
        
        print_info("textToSpeechTest: startup.")
        print_info("textToSpeechTest: publishing to /speech.")
        print_info("textToSpeechTest: /textToSpeech/say_text service advertised")
        print("\n")
        print_header("TTS IMPLEMENTATION INTEGRATION TESTS")
        print_info("This test will verify the TTS implementation with actual service calls.")
        print_info("The test will check if the implementation correctly processes all test scenarios.")
        print_warning("Make sure the TTS service is running before proceeding.")

    def get_user_confirmation_to_start(self):
        """Get user confirmation to start testing"""
        input("Press Enter to begin implementation testing...")
        self.logger.log("User started implementation testing", "USER_INPUT")
    
    def display_test_summary(self, test_results):
        """Display comprehensive test summary"""
        print_header("IMPLEMENTATION TEST SUMMARY")
        
        passed_tests = 0
        total_tests = len(test_results)
        
        self.logger.log("FINAL TEST RESULTS:", "SUMMARY")
        for test_name, result in test_results:
            if result:
                print_success(f" ✓ {test_name} - PASSED")
                self.logger.log(f"{test_name} - PASSED", "SUMMARY")
                passed_tests += 1
            else:
                print_error(f" ✗ {test_name} - FAILED")
                self.logger.log(f"SUMMARY: {test_name} - FAILED", "SUMMARY")
        
        # Log test statistics
        self.logger.log(f"Total tests run: {total_tests}", "SUMMARY")
        self.logger.log(f"Passed: {passed_tests}", "SUMMARY")
        self.logger.log(f"Failed: {total_tests - passed_tests}", "SUMMARY")
        
        # Display test statistics
        print(f"\nTotal tests run: {total_tests}")
        print(f"Passed: {passed_tests}")
        print(f"Failed: {total_tests - passed_tests}")
        
        return passed_tests, total_tests
    
    def display_final_result(self, passed_tests, total_tests):
        """Display final test result"""
        if passed_tests == total_tests:
            print_success("\n ALL IMPLEMENTATION TESTS PASSED! The TTS implementation is working correctly.")
            self.logger.log("FINAL RESULT: ALL IMPLEMENTATION TESTS PASSED", "FINAL_RESULT")
            return True
        else:
            failed_tests = total_tests - passed_tests
            print_error(f"\n {failed_tests} IMPLEMENTATION TESTS FAILED! Please check the TTS implementation.")
            self.logger.log(f"FINAL RESULT: {failed_tests} IMPLEMENTATION TESTS FAILED", "FINAL_RESULT")
            return False
    
    def run_tests(self):
        """Main method to run all TTS implementation tests"""
        try:
            # Display welcome message
            self.display_welcome_message()
            
            # Get user confirmation to start
            self.get_user_confirmation_to_start()
            
            # Create test instance and run tests
            tester = TTSImplementationIntegrationTest(self.logger)
            test_results = tester.run_all_tests()
            
            # Display test summary
            passed_tests, total_tests = self.display_test_summary(test_results)
            
            # Display final result
            success = self.display_final_result(passed_tests, total_tests)
            
            # Write results to file
            self.logger.write_to_file()
            
            return success
            
        except Exception as e:
            print_error(f"Error during implementation testing: {e}")
            self.logger.log(f"CRITICAL ERROR during implementation testing: {e}", "CRITICAL_ERROR")
            self.logger.write_to_file()
            return False

def main():
    """Main entry point for the application"""
    try:
        # Create and run the test application
        app = TTSTestApplication()
        success = app.run_tests()
        
        # Clean shutdown
        rospy.signal_shutdown("Implementation test completed")
        
        # Exit with appropriate code
        if success:
            sys.exit(0)
        else:
            sys.exit(1)
            
    except rospy.ROSInterruptException:
        print_warning("Implementation test interrupted!")
        # Create logger instance for cleanup
        logger = TestOutputLogger()
        logger.log("Implementation test interrupted by ROS", "WARNING")
        logger.write_to_file()
        sys.exit(1)
    except Exception as e:
        print_error(f"Error during implementation testing: {e}")
        # Create logger instance for cleanup
        logger = TestOutputLogger()
        logger.log(f"Unexpected error during implementation testing: {e}", "CRITICAL_ERROR")
        logger.write_to_file()
        sys.exit(1)

if __name__ == '__main__':
    main()