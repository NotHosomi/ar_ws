#!/usr/bin/env python
import rospy
import math
import copy

from geometry_msgs.msg import PoseStamped

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class sim_nav_goals():

    waypoints = [[1,2], [3.5,4.3], [0,0]] #etc etc (last should be [0,0])

    def __init__(self):
        rospy.init_node('sim_nav_goals', anonymous=True)

        self.counter = 0
        
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        self.begin = 0  # This variable is never used
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():            
        
            self.counter += 1
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.z = 0
            msg.pose.orientation.w = 1
            #if self.counter % 2 == 0:
                # First pose
            #    msg.pose.position.x = 0.0
            #    msg.pose.position.y = 0.5
            #else:
                # Second pose
            #    msg.pose.position.x = -3.0
            #    msg.pose.position.y = 1.0
            
            # GOTO waypoint i
            #msg.pose.position.x = self.waypoints[i][0]
            #msg.pose.position.y = self.waypoints[i][1]
            if self.counter  > len(self.waypoints):
                msg.pose.position.x = self.waypoints[self.counter][0]
                msg.pose.position.y = self.waypoints[self.counter][0]
            else:
                rospy.logdebug("Reached end of path!")
                #Send alert to family (Ron is missing)

                        
            msg.header.stamp = rospy.Time.now()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose = msg
            client.send_goal(goal)
            wait = client.wait_for_result() # This line blocks until the goal is reached, so interupt may be more difficult.
            #Need to find a way to break out of the loop early
    
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
