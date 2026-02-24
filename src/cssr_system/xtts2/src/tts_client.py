#!/usr/bin/env python3.10



import rospy
from xtts2.srv import RealtimeTTS

def tts_client(text: str) -> bool:
    """Call the realtime_tts service with the given text."""
    rospy.wait_for_service("realtime_tts")
    try:
        proxy = rospy.ServiceProxy("realtime_tts", RealtimeTTS)
        resp = proxy(text)
        if resp.success:
            rospy.loginfo(f"TTS OK: {resp.message}")
        else:
            rospy.logerr(f"TTS failed: {resp.message}")
        return resp.success

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call error: {e}")
        return False

if __name__ == "__main__":
    rospy.init_node("tts_client")

    print("Interactive TTS client ready! Type text and press Enter.")
    print("Type 'quit' or 'exit' to stop.\n")

    while not rospy.is_shutdown():
        try:
            text = input("> ").strip()
            if text.lower() in ["quit", "exit"]:
                print("Goodbye!")
                break
            if text:
                tts_client(text)
        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            break

    print("Type 'quit' or 'exit' to stop.\n")


while not rospy.is_shutdown():
    try:
        text = input("> ").strip()
        if text.lower() in ["quit", "exit"]:
            print("Goodbye!")
            break
        if text:
            tts_client(text)
    except (EOFError, KeyboardInterrupt):
        print("\nExiting...")
        break

