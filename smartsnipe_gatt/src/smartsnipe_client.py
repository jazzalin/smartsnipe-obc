#! /usr/bin/env python

import rospy
import actionlib
import actionlib_msgs
import actionlib_tutorials.msg
from std_msgs.msg import String

class SmartsnipeClient:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('smartsnipe_action', actionlib_tutorials.msg.FibonacciAction)
        self.client.wait_for_server()
        rospy.Subscriber('uart_rx', String, self.process_goal)
        self.pub = rospy.Publisher('uart_tx', String, queue_size=100)
        self.action = actionlib_tutorials.msg.FibonacciGoal()
        self.rate = rospy.Rate(10)

    def process_goal(self, goal):
        self.action.order = goal.data
        self.client.send_goal(self.action, done_cb=action_complete)
    
    def action_complete(self, status, result):
        rospy.loginfo(result.sequence)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('smartsnipe_client')
        client = SmartsnipeClient()
        client.run()
    except rospy.ROSInterruptException:
        print("Action client interrupted")
    except KeyboardInterrupt:
        exit()