#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from realrobot_pkg.msg import OdomRecordAction, OdomRecordFeedback
import actionlib
from std_msgs.msg import Empty

import rospkg
from realrobot_pkg.srv import FindWall, FindWallRequest

class FollowingWallActionClientClass(object):

    def __init__(self):

        # Important Variables
        self.wall_found = False
        self.current_distance = 0
        self.regions = {
            'right': 0,
            'front': 0,
            'left': 0
        }
        
        # Publisher/Subscriber Initialization
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(2)
        self.var = Twist()
        
        # Service to find Wall
        rospy.wait_for_service('/move_nearest_wall')
        self.find_nearest_wall_service_client = rospy.ServiceProxy('/move_nearest_wall', FindWall)
        rospy.loginfo('The service "/move_nearest_wall" found')
        self.find_nearest_wall_request_object = FindWallRequest()
        self.service_result = self.find_nearest_wall_service_client(self.find_nearest_wall_request_object)
        # Was the nearest wall found?        
        if self.service_result.wallfound == True:
            rospy.loginfo("Ready to follow the nearest wall...")
            self.wall_found = True

        # Create the connection to the action server
        self.action_client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
        rospy.loginfo('Waiting for the action client to send the goal...')
        self.action_client.wait_for_server()
        rospy.loginfo('Connection is ready...')
        # Create a gold to send to the action service
        self.goal = Empty()
        self.action_client.send_goal(self.goal, feedback_cb=self.feedback_callback)
        

        # Wait to get the laser readings 
        self.rate.sleep()

        

    def feedback_callback(self, feedback):
        self.current_distance = feedback.current_total
        rospy.loginfo("Distance travelled so far: " + str(feedback.current_total))

    def callback(self, msg):    
        # Regions
        self.regions = {
            'right': msg.ranges[180],
            'front': msg.ranges[360],
            'left': msg.ranges[540]
        }

        if self.wall_found == True: 

            self.var.linear.x = 0.1
            self.var.angular.z = 0

            if self.regions['front'] < 0.5:
                #print("Turn fast to left")
                self.var.linear.x = 0.0
                self.var.angular.z = 0.5
            elif self.regions['right'] > 0.25:
                #print("Find the wall")
                self.var.linear.x = 0.1
                self.var.angular.z = -0.1
            elif self.regions['right'] <= 0.18:
                #print("Move away from the wall")
                self.var.linear.x = 0.05
                self.var.angular.z = 0.1
                self.pub.publish(self.var)
                if self.regions['right'] > 0.18:
                    self.var.linear.x = 0
                    self.var.angular.z = 0
            elif self.regions['right'] > 0.18 and self.regions['right'] <= 0.25:
                #print("Follow the wall")
                self.var.linear.x = 0.1
                self.var.angular.z = 0
            elif self.regions['left'] < 0.3:
                #print("Move away from obstacle")
                self.var.linear.x = 0.0
                self.var.angular.z = -0.1
            
            self.pub.publish(self.var)
        
        if self.current_distance > 6:
            rospy.loginfo('One lap complete!')
            self.var.linear.x = 0
            self.var.angular.z = 0
            self.pub.publish(self.var)
            rospy.signal_shutdown('Stop the robot...')

if __name__ == "__main__":

    rospy.init_node('drive_along_wall')
    FollowingWallActionClientClass()
    rospy.spin()