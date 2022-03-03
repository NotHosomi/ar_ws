#!/usr/bin/env python
import rospy
import math
import copy
from std_msgs.msg import Bool
from std_msgs.msg import String 

from geometry_msgs.msg import PoseStamped

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal






class sim_nav_goals():

    
    def callback(self, data):
        self.signal = data

    def callback_ron_response(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Ron responded with:  %s", data.data)

   

    
    def __init__(self):
        self.signal = False
        rospy.init_node('sim_nav_goals', anonymous=True)

        self.counter = 0

        rospy.Subscriber("interrupt_pub",Bool, self.callback)
        rospy.Subscriber("ron_response", String, self.callback_ron_response)


        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        
        self.begin = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

        
            self.counter += 1
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.z = 0
            msg.pose.orientation.w = 1
            if self.signal == False:
                if self.counter % 2 == 0:
                    msg.pose.position.x = 2
                    msg.pose.position.y = 2
                else:
                    msg.pose.position.x = 2.1
                    msg.pose.position.y = 2.1

            elif self.signal == True:
            
                print("changing")
                msg.pose.position.x = -3
                msg.pose.position.y = -3



                        
            msg.header.stamp = rospy.Time.now()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose = msg
            client.send_goal(goal)
            wait = client.wait_for_result()
    
            if not wait:
                rospy.logerr("Action server is not available.")
                rospy.signal_shutdown("Action server is not available.")
            else:
                x = client.get_result()
                
            rate.sleep()

if __name__ == '__main__':
    try:
        control = sim_nav_goals()
    except rospy.ROSInterruptException:
        pass