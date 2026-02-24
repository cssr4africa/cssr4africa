#! /usr/bin/env python3

import rospy
import actionlib
from xtts2.msg import pizza_orderAction, pizza_orderFeedback, pizza_orderResult

class pizza_hut_server():
    def __init__(self):
        rospy.init_node('pizza_order_shop')
        self.a_server = actionlib.SimpleActionServer('pizza_ready', pizza_orderAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()
        rospy.loginfo("Pizza Order Server is ready to receive orders.")

    def execute_cb(self, goal):
        success = True
        last_pizza_ready = ''
        feedback = pizza_orderFeedback()
        result = pizza_orderResult()
        # rospy.loginfo(f"Received order for {goal.pizza_type} pizza.")
        rate = rospy.Rate(1)

        for i in range(0, goal.number_of_pizza_order):
            if self.a_server.is_preempt_requested():
                rospy.loginfo("Order preempted.")
                self.a_server.set_preempted()
                success = False
                break

            last_pizza_ready = 'Pizza' + str(i)
            print('Last pizza Ready is', last_pizza_ready)
            feedback.last_pizza_ready = last_pizza_ready
            result.pizza_ready.append(last_pizza_ready)
            self.a_server.publish_feedback(feedback)
            rospy.loginfo(last_pizza_ready)
            rate.sleep()
            
        if success:
            rospy.loginfo("All pizzas are ready.")
            self.a_server.set_succeeded(result)

if __name__ == '__main__':
    s = pizza_hut_server()
    rospy.spin()


