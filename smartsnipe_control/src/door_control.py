import rospy
from smartsnipe_msgs.srv import ActuateDoor, ActuateDoorRequest, ActuateDoorResponse
from smartsnipe_msgs.msg import Shot, BoardState
from std_msgs import UInt8

class Door:
    def __init__(self, label):
        self.id = label
        self.open = False
        self.count = 0
        # TODO: add field to control speed of door
        # self.speed = 127

    def increment_count(self):
        self.count += 1


class DoorMonitor:

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


if __name__ == "__main__":
    try:
        rospy.init_node('door_actuator')
        # door_mask = [0, 0, 0, 1, 1] # e.g.
        client = DoorMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Door actuation node interrupted before completion")
    except KeyboardInterrupt:
        exit()