#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String

# possible responses that can be given, or we could change this 
# to return ints for different cases 
responses = ['I am alright',
             'I need help']

def respond():
    pub = rospy.Publisher('ron_response', String, queue_size=10)
    rospy.init_node('ron_node', anonymous=True)

    # rate that messages are published (Hz) (will be changed to trigger when 
    # Ron is asked a question)
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        response = "%s" % responses[random.randint(0, len(responses)-1)]
        rospy.loginfo(response)
        pub.publish(response)
        rate.sleep()

if __name__ == '__main__':
    try:
        respond()
    except rospy.ROSInterruptException:
        pass