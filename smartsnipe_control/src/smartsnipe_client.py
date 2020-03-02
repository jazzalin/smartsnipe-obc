#! /usr/bin/env python
import rospy
import actionlib
import actionlib_msgs
import actionlib_tutorials.msg
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from smartsnipe_msgs.msg import SmartsnipeDrillAction, SmartsnipeDrillGoal
from smartsnipe_session import Session, SessionStatistics
import json
#TODO: add cloud DB storage client interface

class SmartsnipeClient:

    def __init__(self):
        # ROS
        # self.client = actionlib.SimpleActionClient('smartsnipe_action', actionlib_tutorials.msg.FibonacciAction)
        self.client = actionlib.SimpleActionClient('smartsnipe_action', SmartsnipeDrillAction)
        self.client.wait_for_server()
        rospy.Subscriber('uart_rx', String, self.process_rx)
        self.pub = rospy.Publisher('uart_tx', String, queue_size=100)
        self.action = SmartsnipeDrillGoal()
        self.rate = rospy.Rate(10)

        # Session
        self.session = Session()

    def process_rx(self, msg):
        """
        Decode stringified json on RX
        """
        data = json.loads(msg.data)
        rospy.logdebug(msg.data)
        # ACK/NACK payload
        resp = {"success": False, "error": ""}

        if "session_in_progress" in data.keys():
            rospy.loginfo("INIT: initializing session")
            self.session.set_status(data["session_in_progress"])
            resp["success"] = True
        elif "time_between_openings" in data.keys():
            if not self.session.in_progress:
                resp["error"] = "Inconsistent state: session not in progress"
                resp["success"] = False
            else:
                rospy.loginfo("CONFIG: configuring session")
                self.session.set_params(data)
                resp["success"] = True
        elif "slot_to_open_next" in data.keys():
            rospy.loginfo("OVERRIDE: setting session in override")
            self.session.set_override(data)
            resp["success"] = True
            # NOTE: probably need to override current goal request
        else:
            rospy.logdebug("Message on RX not recognized")
            resp["error"] = "Message on RX not recognized"

        # ACK/NACK
        self.pub.publish(json.dumps(resp))
    
    def drill_in_progress_cb(self):
        rospy.loginfo("Drill is now active")

    def drill_feedback_cb(self, feedback):
        rospy.loginfo(feedback.current_state)
    
    def drill_results_cb(self, status, result):
        rospy.logdebug("Drill results received with status {}; msg: {}".format(status, result))
        msg = ""
        if status == GoalStatus.SUCCEEDED:
            self.session.set_statistics(result.final_state)
        # elif status == GoalStatus.ABORTED:
            # rospy.logdebug("ABORTED")
            
        # NOTE: the __repr__ is used to support intermediate stats
        data = self.session.stats.__repr__()
        data["status"] = result.final_state.status
        self.pub.publish(json.dumps(data))

    def run(self):
        while not rospy.is_shutdown():
            if self.session.in_progress:
                rospy.loginfo("Session in progress")
                if self.session.requested:
                    rospy.loginfo("Requesting new board configuration")
                    # Request new drill
                    # Example drill
                    self.action.drill.time_between = self.session.time_between
                    self.action.drill.time_open = self.session.time_open
                    self.action.drill.slots = self.session.slots
                    self.action.drill.duration = self.session.duration
                    self.client.send_goal(self.action,
                                          active_cb=self.drill_in_progress_cb,
                                          done_cb=self.drill_results_cb, 
                                          feedback_cb=self.drill_feedback_cb)
                    self.session.requested = False
                else:
                    rospy.logdebug("Session not configured yet")
            else:
                rospy.logdebug("Session not in progress")
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('smartsnipe_client', log_level=rospy.DEBUG)
        client = SmartsnipeClient()
        client.run()
    except rospy.ROSInterruptException:
        print("Action client interrupted")
    except KeyboardInterrupt:
        exit()