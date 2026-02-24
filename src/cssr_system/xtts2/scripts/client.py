#! /usr/bin/env python3

import rospy
import actionlib
from xtts2.msg import pizza_orderAction, pizza_orderGoal

class pizza_customer():
    def __init__(self):
        print("initialized class")

    def goal_result(self):
        client = actionlib.SimpleActionClient('pizza_ready', pizza_orderAction)
        client.wait_for_server()

        goal = pizza_orderGoal()
        goal.number_of_pizza_order = 8

        client.send_goal(goal, feedback_cb=self.feedback_cb)
        client.wait_for_result()
        result = client.get_result()

        return result

    def feedback_cb(self, msg):
        print('feedback Received', msg)



if __name__ == '__main__':
    pz = pizza_customer()

    try:
        rospy.init_node('action_client')

        result = pz.goal_result()
        print('The result is:', result)
    except rospy.ROSInterruptException as e:
        print('Something went wrong:', e)