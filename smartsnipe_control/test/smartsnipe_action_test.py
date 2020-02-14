#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import json

rospy.init_node("test_node")
pub = rospy.Publisher("uart_rx", String, queue_size=10)

session = dict(session_in_progress=True)
# pub.publish(json.dumps(session))

action = dict(time_between_openings=0.5, time_slot_is_open=1.0, slots=[1, 1, 1, 1, 1], duration=180.0)
# pub.publish(json.dumps(action))