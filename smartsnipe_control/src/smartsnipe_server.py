#! /usr/bin/env python
import rospy
import actionlib
import actionlib_tutorials.msg
from smartsnipe_msgs.srv import ActuateDoor, ActuateDoorRequest, ActuateDoorResponse
from smartsnipe_msgs.msg import Session, Statistics
from smartsnipe_msgs.msg import SmartsnipeDrillAction, SmartsnipeDrillFeedback, SmartsnipeDrillResult
# from std_msgs import UInt8

class Door:
    def __init__(self, label):
        self.id = label
        self.open = False
        self.count = 0
        # TODO: add field to control speed of door
        # self.speed = 127

    def increment_count(self):
        self.count += 1


class BoardController:

    def __init__(self, doors=[1, 1, 1, 1, 1]):
        """
        Control interface to actuate and monitor doors independently
            doors: mask list indicating which doors to control (default: all) e.g. [0, 1, 1, 0, 1]
        """
        self.doors = []
        for index in doors:
            if doors[index]:
                self.doors.append(Door(index))


    def actuate_door(self, door, open):
        """
        Blocking call to open/close doors
        """
        rospy.wait_for_service('doors')
        try:
            actuator = rospy.ServiceProxy('doors', ActuateDoor)
            req = ActuateDoorRequest(door=door, open=open)
            resp = actuator(req)
            rospy.loginfo(resp.message)
        except rospy.ServiceException as e:
            print("Call to ActuateDoor service failed: %s", e)


    def shot_cb(self, data):
        """Update internal state of targets for analytics"""
        # TODO: record "shot" msg in json/dict format for db storage
        self.doors[data.door].increment_count()


class SmartsnipeAction(object):
    _feedback = SmartsnipeDrillFeedback()
    _result = SmartsnipeDrillResult()
    # _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    # _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        # ROS
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, SmartsnipeDrillAction, execute_cb=self.execute, auto_start=False)
        # self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Control
        self.board_ctrl = BoardController()

    def execute_cb(self, goal):
        r = rospy.Rate(10)
        success = True

        self._feedback = "Drill in progress"
        rospy.loginfo("{}: Starting requested drill".format(self._action_name))
        
        # TODO: -call door service to apply request board configuration
        #       -if duration is specified, enter time loop | catch any preemption (and return existing stats) |
        #        use feedback to update stats | use return to give final stats
        #       -if duration is not specified, catch any preemption | return current stats

        # if goal.duration >= 0.0:
        # for i in range(1, goal.order):
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     self._as.publish_feedback(self._feedback)
        #     r.sleep()
        
        # if success:
        #     self._result.sequence = self._feedback.sequence
        #     rospy.loginfo('%s: Succeeded' % self._action_name)
        #     self._as.set_succeeded(self._result)


if __name__ == "__main__":
    try:
        rospy.init_node('smartsnipe_action', log_level=rospy.DEBUG)
        # door_mask = [0, 0, 0, 1, 1] # e.g.
        server = SmartsnipeAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Door actuation node interrupted before completion")
    except KeyboardInterrupt:
        exit()