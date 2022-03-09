import rospy
import roscpp
from std_msgs import UInt8

# This script listens to messages on the Alerts topic.
# It prints to console the action that would be taken by an external system

def callback(data):
    #
    if data == 0:
        rospy.logdebug("To Family: Ron was not found, contact family with the message \"Ron has forgotten to take his medication, and could not be found within the house.\"")
    elif data == 1:
        rospy.logdebug("To Emergency Services: Elderly patient has fallen down at <address> and cannot get up. They say they are in pain.")
    elif data == 2:
        rospy.logdebug("To Emergency Services: Elderly patient has fallen down at <address> and cannot get up. They say they are in pain.")
    elif data == 3:
        rospy.logdebug("To Emergency Services: Elderly patient has fallen down at <address> and is not responding")
    else:
        rospy.logerr("Invalid outcome code")


def setup():
    rospy.init_node('AlertListener')
    rospy.Subscriber("Alerts", UInt8, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        control = setup()
    except rospy.ROSInterruptException:
        pass