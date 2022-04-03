#!/usr/bin/env python
from cmath import cos
from cmath import sin
from typing import List
import rospy
import math
import copy
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class sim_nav_goals():

    waypoints = [[4,4], [3,3], [0,0]] #etc etc (last should be [0,0])

    
    
    def callback_is_in_sight(self, data):
        self.seen = data
        # print(self.seen)
    
    pose_msg = PoseStamped()
    def setCurrentPose(self, pose): # (self, header, pose)
        pose_msg = pose

        
    def __init__(self):
        self.seen = False
        self.stopped = False
        rospy.init_node('sim_nav_goals', anonymous=True)

        self.counter = 0
        rospy.Subscriber("is_in_sight", Bool, self.callback_is_in_sight)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.setCurrentPose) # Check msg formatting, callback params, Get from amcl_pose topic
                #PoseWithCovarianceStamped > PoseWithCovariance > Pose 
        alert_pub = rospy.Publisher("Alerts", UInt8, queze_size = 1)
       
        
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        self.begin = 0  # This variable is never used
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.wish_return):
                counter = len(self.waypoints)-1
                
            # Don't do anything is Ron has been spotted
            if(self.seen):
                #set a goal of the current position
                if not self.stopped:
                    self.stopped = True
                    pose = PoseStamped()
                    pose.pose = self.pose_msg.pose.pose

                rospy.sleep(500)
                continue


            # Check if at waypoint yet
            # Could this have been done easier using a callback triggered by the client?
            pos = self.pose_msg.pose.pose.position      # fetch the position and orientation from the latest pose message (topic: amcl_pose)
            quat = self.pose_msg.pose.pose.orientation
            diff_x = self.waypoints[self.counter][0] - pos.x    # find the different between the waypoint and current position in each axis
            diff_y = self.waypoints[self.counter][1] - pos.y
            diff_z = sin(self.waypoints[self.counter][2]/2) - quat.z
            diff_w = cos(self.waypoints[self.counter][2]/2) - quat.w
            # rotational difference?
            diff_x *= diff_x    # squaring to get the abs squared. This should be computationally cheaper than abs() as no sqrt is needed
            diff_y *= diff_y
            diff_z *= diff_z
            diff_w *= diff_w
            if diff_x > 0.0001 or diff_y > 0.0001 or diff_z > 0.1 or diff_w > 0.1: # if both pos diffs are less than 0.01 (remember, they are squared above to discard negatives)
                rospy.sleep(200)                                                   # and both quat diffs are less that ~0.3
                continue    # we're not at the waypoint yet, don't create a new goal
            print("Reached waypoint")
                       

            # Build a new pose from the waypoint information
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.z = 0
            if self.counter  < len(self.waypoints):
                msg.pose.position.x = self.waypoints[self.counter][0]
                msg.pose.position.y = self.waypoints[self.counter][1]
                msg.pose.orientation.z = sin(self.waypoints[self.counter][2]/2)
                msg.pose.orientation.w = cos(self.waypoints[self.counter][2]/2)
            else:
                rospy.logdebug("Reached end of path!")
                message = UInt8()
                message.data = 0
                alert_pub.publish(message)
                break
            msg.header.stamp = rospy.Time.now()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose = msg
            client.send_goal(goal)
            print('counter:' + str(self.counter))
            #wait = client.wait_for_result() # This line blocks until the goal is reached, so interupt may be more difficult.
            #Need to find a way to break out of the loop early

            self.counter +=1 
            #if not wait:
            #    rospy.logerr("Action server is not available.")
            #    rospy.signal_shutdown("Action server is not available.")
            #else:
            #    rospy.logdebug(client.get_result())
            #    x = client.get_result()
                
            rate.sleep()
        #end

if __name__ == '__main__':
    try:
        
        control = sim_nav_goals()
    except rospy.ROSInterruptException:
        pass