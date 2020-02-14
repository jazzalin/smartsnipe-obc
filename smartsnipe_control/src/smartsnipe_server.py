#! /usr/bin/env python
import rospy
import actionlib
import actionlib_tutorials.msg
from smartsnipe_msgs.srv import ActuateDoor, ActuateDoorRequest, ActuateDoorResponse
from smartsnipe_msgs.msg import Session, Statistics, Shot
from smartsnipe_msgs.msg import SmartsnipeDrillAction, SmartsnipeDrillFeedback, SmartsnipeDrillResult
# from std_msgs.msg import UInt8, Float32

class Door:
    def __init__(self, label):
        self.id = label
        self.open = False
        self.count = 0
        # TODO: add field to control speed of door
        # self.speed = 127

    def increment_count(self):
        self.count += 1


class BoardMonitor:

    def __init__(self):
        """
        Control interface to actuate and monitor doors independently
        Keeps an internal state of the board statistics
        """
        self.prev_state = 0 # NOTE: for now, just monitor the total number of shots taken

        # ROS
        rospy.Subscriber("shot_stats", Shot, self.shot_stats_cb)
    
    def reset_state(self):
        # Reset internal state of the board
        self.doors = []
        self.shot_count = 0
        self.shot_missed = 0

    def set_doors(self, doors=[0, 0, 0, 0, 0]):
        for index in range(len(doors)):
            if doors[index]:
                self.doors.append(Door(index))
        
    def actuate_door(self):
        """
        Blocking call to open/close doors
        """
        rospy.wait_for_service('doors')
        try:
            actuator = rospy.ServiceProxy('doors', ActuateDoor)
            req = ActuateDoorRequest(doors=self.doors)
            resp = actuator(req)
            rospy.loginfo(resp.message)
        except rospy.ServiceException as e:
            print("Call to ActuateDoor service failed: %s", e)


    def shot_stats_cb(self, data):
        """
        Update internal state of targets for analytics
        """
        self.shot_count += 1
        if data.goal:
            self.doors[data.door].increment_count()
        else:
            self.shot_missed += 1


class SmartsnipeAction(object):
    _feedback = SmartsnipeDrillFeedback()
    _result = SmartsnipeDrillResult()
    # _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    # _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        # ROS
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, SmartsnipeDrillAction, execute_cb=self.execute_cb, auto_start=False)
        # self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Control + monitor of shooting board
        self.board = BoardMonitor()
        self.timeout = rospy.Time(60) # timeout after a minute of inactivity
    
    def stats_change(self):
        """
        Monitor internal state of the board for any changes (e.g. shots taken)
        """
        return True if self.board.prev_state != self.board.shot_count else False


    def execute_cb(self, goal):
        r = rospy.Rate(10)
        success = True
        self.board.reset_state()

        rospy.loginfo("{}: Starting requested drill".format(self._action_name))
        
        # TODO: -call door service to apply request board configuration
        #       -if duration is specified, enter time loop | catch any preemption (and return existing stats) |
        #        use feedback to update stats | use return to give final stats
        #       -if duration is not specified, catch any preemption (necessary???) | return current stats

        self.board.set_doors(goal.slots)
        self.board.actuate_door()

        start = rospy.Time.now()
        if goal.duration > 0.0:
            while rospy.Time.now() - start < goal.duration:
                if self._as.is_preempt_requested():  # handle preemption request;
                    rospy.loginfo("Current drill preempted. Ending current drill")
                    success = False
                    self._as.set_preempted()
                    break
                # TODO: periodically send statistics (internally kept up2d8) according to rospy.Rate specified
                self._feedback.status = "Useful feedback"
                self._as.publish_feedback(self._feedback)
                r.sleep()
        else:
            while not self.stats_change() and (rospy.Time.now() - start) < self.timeout:
                if self._as.is_preempt_requested():  # handle preemption request;
                    rospy.loginfo("Current drill preempted. Ending current drill")
                    success = False
                    self._as.set_preempted()
                    break
                self._feedback.status = "Waiting for shot to be taken..."
                self._as.publish_feedback(self._feedback)
                r.sleep()   
        
        self.prev_state = self.shot_count
        # Return results if drill completed successfully    
        if success:
            self._result.status = "Useful results"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    try:
        rospy.init_node('smartsnipe_action', log_level=rospy.DEBUG)
        server = SmartsnipeAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Door actuation node interrupted before completion")
    except KeyboardInterrupt:
        exit()