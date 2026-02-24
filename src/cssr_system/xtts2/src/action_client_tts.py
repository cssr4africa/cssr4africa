#!/usr/bin/env python3.10

import rospy
import actionlib
from xtts2.msg import english_stream_ttsAction, english_stream_ttsGoal


def feedback_cb(feedback):
    rospy.loginfo(f"Feedback: {feedback.status}")


def tts_client(text: str):
    client = actionlib.SimpleActionClient("tts_action", english_stream_ttsAction)
    client.wait_for_server()

    goal = english_stream_ttsGoal(text=text)
    client.send_goal(goal, feedback_cb=feedback_cb)

    rospy.loginfo("Goal sent, waiting for result...")
    client.wait_for_result()
    result = client.get_result()   

    if result.success:
        rospy.loginfo(f"TTS success: {result.message}")
    else:
        rospy.logerr(f"TTS failed: {result.message}")


if __name__ == "__main__":
    rospy.init_node("tts_action_client")

    print("TTS Action Client ready! Type text to speak.")
    while not rospy.is_shutdown():
        try:
            text = input("> ").strip()
            if text.lower() in ["quit", "exit"]:
                break
            if text:
                tts_client(text)
        except (EOFError, KeyboardInterrupt):
            break
