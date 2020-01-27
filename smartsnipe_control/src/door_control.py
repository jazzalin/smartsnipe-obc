import rospy
from actuate_door import ActuateDoor

class DoorActuator:

    def __init__(self):
        self.d1

    def actuate_door(self, door):
        request = AcuateDoorRequest()
        response = ActuateDoorResponse()

        






if __name__ == "__main__":
    try:
        rospy.init_node('door_actuator')
        client = DoorActuator()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Door actuation node interrupted before completion")
    except KeyboardInterrupt:
        exit()