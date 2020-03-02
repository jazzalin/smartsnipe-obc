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

    def set_doors(self, doors=[0, 0, 0, 0]):
        self.doors = doors
        # FIXME
        # for index in range(len(doors)):
        #     if doors[index]:
        #         self.doors.append(Door(index))
        
    def actuate_door(self):
        """
        Blocking call to open/close doors
        """
        success = True
        try:
            rospy.wait_for_service('doors', timeout=5.0)
            actuator = rospy.ServiceProxy('doors', ActuateDoor)
            req = ActuateDoorRequest(doors=self.doors)
            resp = actuator(req)
            rospy.loginfo(resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Call to ActuateDoor service failed: %s", e)
            success = False
        except rospy.ROSException as e:
            rospy.logerr("Call to ActuateDoor service timed out")
            success = False
        return success


    def shot_stats_cb(self, data):
        """
        Update internal state of targets for analytics
        """
        pass
        # rospy.logdebug("shot stat received")
        # self.shot_count += 1
        # if data.goal:
        #     self.doors[data.door].increment_count()
        # else:
        #     self.shot_missed += 1


class SmartsnipeAction(object):
    _feedback = SmartsnipeDrillFeedback()
    _result = SmartsnipeDrillResult()

    def __init__(self, name):
        # ROS
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, SmartsnipeDrillAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # Control + monitor of shooting board
        self.board = BoardMonitor()
        self.timeout = rospy.Time(10).to_sec() # timeout after a minute of inactivity
    
    def stats_change(self):
        """
        Monitor internal state of the board for any changes (e.g. shots taken)
        """
        return True if self.board.prev_state != self.board.shot_count else False


    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True
        self.board.reset_state()

        rospy.loginfo("{}: Received new drill".format(self._action_name))
        
        # TODO: -call door service to apply request board configuration
        #       -if duration is specified, enter time loop | catch any preemption (and return existing stats) |
        #        use feedback to update stats | use return to give final stats
        #       -if duration is not specified, catch any preemption (necessary???) | return current stats

        self.board.set_doors(goal.drill.slots)
        ready = self.board.actuate_door()
        if not ready:
            self._result.final_state.status = "FAILED: Could not actuate doors"
            self._as.set_aborted(self._result)
        else:
            start = rospy.Time.now().to_sec()
            if goal.drill.duration > 0.0:
                rospy.logdebug("Starting requested {} second drill".format(goal.drill.duration))
                duration = rospy.Duration.from_sec(goal.drill.duration).to_sec()

                while (rospy.Time.now().to_sec() - start) < duration:
                    if self._as.is_preempt_requested():  # handle preemption request;
                        rospy.loginfo("Current drill preempted. Ending current drill")
                        success = False
                        self._as.set_preempted()
                        break
                    # TODO: periodically send statistics (internally kept up2d8) according to rospy.Rate specified
                    self._feedback.current_state.status = "Useful feedback"
                    self._as.publish_feedback(self._feedback)
                    r.sleep()
            else:
                rospy.logdebug("Starting requested drill")

                while not self.stats_change() and (rospy.Time.now().to_sec() - start) < self.timeout:
                    if self._as.is_preempt_requested():  # handle preemption request;
                        rospy.loginfo("Current drill preempted. Ending current drill")
                        success = False
                        self._as.set_preempted()
                        break
                    self._feedback.current_state.status = "Waiting for shot to be taken..."
                    self._as.publish_feedback(self._feedback)
                    r.sleep()   
            rospy.loginfo("DONE")
            self.prev_state = self.board.shot_count
            # Return results if drill completed successfully    
            if success:
                self._result.final_state.status = "Useful results"
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