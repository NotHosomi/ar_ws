#!/usr/bin/env python
import rospy
import math
import copy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class sim_nav_goals():

    waypoints = [[4,4], [3,3], [0,0]] #etc etc (last should be [0,0])

    
    
    def callback_is_in_sight(self, data):
        self.seen = data
        # print(self.seen)
       

        
    def __init__(self):
        self.seen = False
        rospy.init_node('sim_nav_goals', anonymous=True)

        self.counter = 0
        rospy.Subscriber("is_in_sight", Bool, self.callback_is_in_sight)
       
        
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        self.begin = 0  # This variable is never used
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
   
            
                       
        
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.z = 0
            msg.pose.orientation.w = 1
            if self.counter  < len(self.waypoints):
                msg.pose.position.x = self.waypoints[self.counter][0]
                msg.pose.position.y = self.waypoints[self.counter][1]
            else:
                rospy.logdebug("Reached end of path!")
            if self.seen == True:
                print("ron is seen")
                print(self.seen)
            msg.header.stamp = rospy.Time.now()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose = msg
            client.send_goal(goal)
            print('counter:' + str(self.counter))
            wait = client.wait_for_result() # This line blocks until the goal is reached, so interupt may be more difficult.
            #Need to find a way to break out of the loop early
            self.counter +=1 
            if not wait:
                rospy.logerr("Action server is not available.")
                rospy.signal_shutdown("Action server is not available.")
            else:
                rospy.logdebug(client.get_result())
                x = client.get_result()
                
            rate.sleep()
        #end

if __name__ == '__main__':
    try:
        
        control = sim_nav_goals()
    except rospy.ROSInterruptException:
        pass