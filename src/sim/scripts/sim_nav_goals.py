#!/usr/bin/env python
import rospy
import math
import copy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class sim_nav_goals():
    
    
    def callback_is_in_sight(self, data):
        self.seen = data
        # print(self.seen)
       

        
    def __init__(self):
        
        rospy.init_node('sim_nav_goals', anonymous=True)

        self.counter = 0
        rospy.Subscriber("is_in_sight", Bool, self.callback_is_in_sight)
       
        
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        self.begin = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
   
            
                       
        
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.z = 0
            msg.pose.orientation.w = 1#
        
            
            if self.seen.data == True:
                print("ron is seen")
                print(self.seen)
                msg.pose.position.x = 6
                msg.pose.position.y = 6
            else:
                print("waiting to see ron")
                print(self.seen)
                msg.pose.position.x = -0.5
                msg.pose.position.y = 0.5


                            
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