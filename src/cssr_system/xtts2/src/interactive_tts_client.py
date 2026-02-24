#!/usr/bin/env python3.10

import rospy
from xtts2.srv import RealtimeTTS

class InteractiveTTSClient:
    def __init__(self):
        rospy.init_node('interactive_tts_client')
        rospy.wait_for_service('realtime_tts')
        self.tts_service = rospy.ServiceProxy('realtime_tts', RealtimeTTS)
        rospy.loginfo("Interactive TTS Client ready")
    
    def run(self):
        """Run interactive TTS client"""
        print("\n=== Interactive RealtimeTTS Client ===")
        print("Type text to convert to speech, or 'quit' to exit")
        print("=========================================\n")
        
        while not rospy.is_shutdown():
            try:
                text = input("Enter text to speak: ").strip()
                
                if text.lower() in ['quit', 'exit', 'q']:
                    print("Goodbye!")
                    break
                
                if not text:
                    print("Please enter some text.")
                    continue
                
                # Call the TTS service
                response = self.tts_service(text)
                
                if response.success:
                    print(f"✓ {response.message}")
                else:
                    print(f"✗ {response.message}")
                    
            except KeyboardInterrupt:
                print("\nGoodbye!")
                break
            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    try:
        client = InteractiveTTSClient()
        client.run()
    except rospy.ROSInterruptException:
        pass
