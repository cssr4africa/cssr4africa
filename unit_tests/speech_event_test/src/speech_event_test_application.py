#!/usr/bin/env python3

"""
speech_event_test_application.py - speechEvent test ROS node definition

Author:     Clifford Onyonka
Date:       2025-02-23
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
speech_event_test_application.py - speechEvent test ROS node definition

This program defines a ROS node that is used to test speechEvent. It tests
both the end-to-end speech transcription process, and the set_language
ROS service that sets the transcription language to be used by speechEvent.

Libraries:
- Ubuntu libraries: None
- Python libraries: rospy

Parameters:
- None

Command-line Parameters:
- None

Configuration File Parameters:
- waitTimeout                       60

Subscribed Topics and Message Types:
- /speechEvent/text                 std_msgs/String

Published Topics and Message Types:
- None

Services Invoked:
- /speechEvent/set_language
- /speechEventDriver/set_next_test_file

Services Advertised and Request Message:
- None

Input Data Files:
- pepper_topics.dat

Output Data Files:
- speech_event_test_output.dat

Configuration Files:
- speech_event_test_configuration.ini

Example Instantiation of the Module:
- rosrun unit_tests speech_event_test_application.py

Author:     Clifford Onyonka, Carnegie Mellon University Africa
Email:      cliffor2@andrew.cmu.edu
Date:       2025-02-23
Version:    v1.0
"""

import os
import time
import sys
from datetime import datetime

import rospy

sys.path.insert(0, os.path.dirname(__file__))
import speech_event_test_implementation as se_imp


COLOR_BLUE = "\033[94m"
COLOR_RESET = "\033[0m"


def is_speech_event_running(speech_transcription_topic):
    is_running = False
    for (topic, _) in rospy.get_published_topics():
        if topic.strip() == speech_transcription_topic.strip():
            is_running = True
            break
    return is_running


if __name__ == "__main__":
    current_file_path = os.path.abspath(__file__)
    current_file_dir = os.path.dirname(current_file_path)
    config_file_path = os.path.join(
        os.path.dirname(current_file_dir), "config", "speech_event_test_configuration.ini"
    )
    topics_file_path = os.path.join(
        os.path.dirname(current_file_dir), "data", "pepper_topics.dat"
    )
    output_file_path = os.path.join(
        os.path.dirname(current_file_dir), "data", "speech_event_test_output.dat"
    )
    test_reports = []
    config = se_imp.parse_config_file(config_file_path)
    topics = se_imp.parse_config_file(topics_file_path)
    mode = config["mode"].strip().lower()
    count = 0

    rospy.init_node("speechEventTest", anonymous=True)

    mode = rospy.get_param("/speechEvent/param_mode", default=mode).strip().lower()

    rospy.Timer(
        rospy.Duration(int(config["heartbeatMsgPeriod"])),
        lambda _: rospy.loginfo("speechEventTest: running")
    )

    while not is_speech_event_running(topics["speechEvent"]):
        rospy.logwarn("speechEventTest: waiting for Speech Event to start running ...") if count % 5 == 0 else "pass"
        time.sleep(1)
        count += 1
        if count >= int(config["waitTimeout"]):
            break

    se_imp.initialise(topics["speechEvent"].strip(), mode)

    rospy.loginfo(
        f"""speechEventTest v1.0

        \r{' ' * 28}This project is funded by the African Engineering and Technology Network (Afretec)
        \r{' ' * 28}Inclusive Digital Transformation Research Grant Programme.

        \r{' ' * 28}Website: www.cssr4africa.org

        \r{' ' * 28}This program comes with ABSOLUTELY NO WARRANTY.
        """
    )
    rospy.loginfo("speechEventTest: start-up")
    rospy.loginfo(f"speechEventTest: subscribed to /speechEvent/text")

    report = se_imp.test_speech_event__set_enabled()
    test_reports.append(report)
    rospy.loginfo(report.replace("\n", f"\n{' ' * 28}"))

    report = se_imp.test_speech_event__set_language()
    test_reports.append(report)
    rospy.loginfo(report.replace("\n", f"\n{' ' * 28}"))

    report = se_imp.test_speech_event__transcription()
    test_reports.append(report)
    rospy.loginfo(report.replace("\n", f"\n{' ' * 28}"))

    combined_test_reports = f"\r{'=' * 80}\n\n".join(test_reports)
    report_text = f"""
    \rSpeech Event Test Report
    \rDate: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
    \r
    \r{'=' * 80}
    \r{combined_test_reports}
    """

    with open(output_file_path, "w") as f:
        f.write(report_text)

    rospy.loginfo(
        f"""{COLOR_BLUE}{'=' * 80}
        \r{' ' * 28}speechEventTest: SpeechEvent tests completed
        \r{' ' * 28}Check the 'speech_event_test_output.dat' file for the test report
        \r{' ' * 28}{'=' * 80}{COLOR_RESET}
        """
    )
