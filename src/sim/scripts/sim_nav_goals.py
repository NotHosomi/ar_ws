#!/usr/bin/env python
#from cmath import cos
#from cmath import sin
#from typing import List
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

    #waypoints = [[-1,2.5,3.14/2],[-3,4,3.14], [-6.25, 2, 3*3.14/2], [-6, 3, 0], [1.5,4.5,1], [5,4.5,0], [6,1,-3.14/2], [-3,1,0]] #etc etc (last should be home pos)
    waypoints = [[1.5,4.5,1], [5,4,0], [6,2,-3.14/2], [-3,1,0]] 
    
    
    def callback_is_in_sight(self, data):
        self.seen = data
        # print(self.seen)
    
    pose_msg = PoseStamped()
    def setCurrentPose(self, pose): # (self, header, pose)
        self.pose_msg = pose

        
    def __init__(self):
        self.seen = False
        self.stopped = False
        rospy.init_node('sim_nav_goals', anonymous=True)

        self.counter = 0
        rospy.Subscriber("is_in_sight", Bool, self.callback_is_in_sight)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.setCurrentPose) # Check msg formatting, callback params, Get from amcl_pose topic
                #PoseWithCovarianceStamped > PoseWithCovariance > Pose 
        alert_pub = rospy.Publisher("Alerts", UInt8)
       
        
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        self.begin = 0  # This variable is never used
        rate = rospy.Rate(20)

        wish_return = False
        first = True
        while not rospy.is_shutdown():
            # MAIN LOOP ---------------------------------------------------------------------
            if wish_return:
                counter = len(self.waypoints)-1
                
            # Don't do anything is Ron has been spotted
            elif self.seen:
                #set a goal of the current position
                if not self.stopped:
                    self.stopped = True
                    pose = PoseStamped()
                    pose.pose = self.pose_msg.pose.pose

                rate.sleep()
                continue


            # Check if at waypoint yet
            # Could this have been done easier using a callback triggered by the client?
            if not first:
                pos = self.pose_msg.pose.pose.position      # fetch the position and orientation from the latest pose message (topic: amcl_pose)
                quat = self.pose_msg.pose.pose.orientation
                diff_x = abs(self.waypoints[self.counter-1][0] - pos.x)    # find the different between the waypoint and current position in each axis
                diff_y = abs(self.waypoints[self.counter-1][1] - pos.y)
                diff_z = abs(math.sin(self.waypoints[self.counter-1][2]/2) - quat.z)
                diff_w = abs(math.cos(self.waypoints[self.counter-1][2]/2) - quat.w)
                if diff_x > 0.25 or diff_y > 0.25 or diff_z > 3 or diff_w > 3: # if both pos diffs are less than 0.01 (remember, they are squared above to discard negatives)
                    # print("X: " + str(diff_x) + "\tY: " + str(diff_y) + "\tZ: " + str(diff_z) + "\tW: " + str(diff_w))
                    #print("Xm: " + str(pos.x) + "  \tXw: " + str(self.waypoints[self.counter-1][0]) + " \tXd: " + str(diff_x))
                    #print("Ym: " + str(pos.y) + "  \tYw: " + str(self.waypoints[self.counter-1][1]) + " \tYd: " + str(diff_y))
                    #print("Zm: " + str(quat.z) + "  \tZw: " + str(math.sin(self.waypoints[self.counter-1][2]/2)) + " \tZd: " + str(diff_z))
                    #print("Wm: " + str(quat.z) + "  \tZw: " + str(math.cos(self.waypoints[self.counter-1][2]/2)) + " \tZd: " + str(diff_w))
                    rate.sleep()                                                        # and both quat diffs are less that ~0.3
                    continue    # we're not at the waypoint yet, don't create a new goal
            print("Reached waypoint")
            first = False
                       

            # Build a new pose from the waypoint information
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.z = 0
            if self.counter  < len(self.waypoints):
                msg.pose.position.x = self.waypoints[self.counter][0]
                msg.pose.position.y = self.waypoints[self.counter][1]
                msg.pose.orientation.z = float(math.sin(self.waypoints[self.counter][2]/2))
                msg.pose.orientation.w = float(math.cos(self.waypoints[self.counter][2]/2))
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