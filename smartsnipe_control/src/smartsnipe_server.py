#! /usr/bin/env python
import rospy
import actionlib
import actionlib_tutorials.msg
from smartsnipe_msgs.srv import ActuateDoor, ActuateDoorRequest, ActuateDoorResponse
from smartsnipe_msgs.msg import Shot, BoardState
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

        # Subscribers
        rospy.Subscriber('shot', Shot, self.shot_cb)
        rospy.Subscriber('board_state', BoardState, self.board_state_update)


    def actuate_door(self, door, open):
        """Blocking call to open/close door(s)"""
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
    
    def board_state_update(self, msg):
        pass

class SmartsnipeAction(object):
    # _feedback = SmartsnipeDrillFeedback()
    # _result = SmartsnipeDrillResult()
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        # self._as = actionlib.SimpleActionServer(self._action_name, SmartsnipeDrillAction, execute_cb=self.execute, auto_start=False)
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        # start executing the action
        for i in range(1, goal.order):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            self._as.publish_feedback(self._feedback)
            r.sleep()
        
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    try:
        rospy.init_node('smartsnipe_action')
        # door_mask = [0, 0, 0, 1, 1] # e.g.
        client = BoardController()
        server = SmartsnipeAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Door actuation node interrupted before completion")
    except KeyboardInterrupt:
        exit()