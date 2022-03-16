#!/usr/bin/env python

from email import message
import rospy
import random
from std_msgs.msg import String
import time 

# possible responses that can be given, or we could change this 
# to return ints for different cases 
responses = ['I am alright',
             'I need help']

message_recieved = String()

def callback(data):
    # print what the Turtlebot said
    rospy.loginfo(rospy.get_caller_id() + "Turtlebot said something: %s", data.data)
    # store the recieved message
    message_recieved.data = data.data

def respond():
    pub = rospy.Publisher('ron_response', String, queue_size=10)
    rospy.Subscriber('ask_ron', String, callback)
    rospy.init_node('ron_node', anonymous=True)

    while not rospy.is_shutdown():
	# read the message from 'ask_ron' subscriber and return a random response if Ron is asked "Are you alright Ron?"
	# extra cases can be added if the Turtlebot says something different 
        if message_recieved.data == "Are you alright Ron?":
            response = "%s" % responses[random.randint(0, len(responses)-1)]
            rospy.loginfo(response)
            pub.publish(response)
            message_recieved.data = None
            

if __name__ == '__main__':

    try:
        respond()
    except rospy.ROSInterruptException:
        pass
