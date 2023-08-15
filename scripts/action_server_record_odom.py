#! /usr/bin/env python

import rospy
import time
import actionlib

from geometry_msgs.msg import Point 
from realrobot_pkg.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

import numpy as np
import copy

class RecordOdomClass(object):
    
    # Create messages used to publish feedback/result
    _feedback = OdomRecordFeedback()
    _result = OdomRecordResult()

    def __init__(self):
        # Create a Subscriber 
        self.odomSub = rospy.Subscriber('/odom', Odometry, self.subs_callback)
        self.odometryData = Point()
    
        # Create the action server
        rospy.loginfo('Creating action server...')
        self._as = actionlib.SimpleActionServer("record_odom", OdomRecordAction, self.goal_callback, False)
        self._as.start()
        rospy.loginfo('The action server is ready...')

        # Parameters
        self.ctrl_c = False
        self.total_distance = 0
        self.x1 = 0
        self.y1 = 0
        self.list_trajectory = []
        
    def subs_callback(self, msg):
        # Listening the information from Subscriber
        # Record x, y, theta odometry of the robot 
        # rospy.loginfo('Recoding Odometry')
        self.odometryData.x = msg.pose.pose.position.x
        self.odometryData.y = msg.pose.pose.position.y
        self.odometryData.z = msg.pose.pose.orientation.z

    def calculate_distance(self, x1, x2, y1, y2):
        # Euclidean Distance 
        travelled_distance = round(np.sqrt((x1-x2)**2 + (y1-y2)**2), 3)
        # rospy.loginfo('travelled_distance: '+str(travelled_distance))
        return travelled_distance
    
    def goal_callback(self, goal):
        
        success = True 
        r = rospy.Rate(1)
        dist_one_lap = 6

        # Create Subscriber
        # self.odomSub = rospy.Subscriber('/odom', Odometry, self.call_record_odometry)

        # Initial values
        self.x1 = self.odometryData.x
        self.y1 = self.odometryData.y
        rospy.loginfo('Initial values trajectory: '+str(self.odometryData))

        while self.total_distance < dist_one_lap:

            # check that preempt (cancelation) has not been requested by the action cli
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled self._as.set_preempted()
                success = False
                break

            # Calculate travelled distance so far
            travelled_distance = self.calculate_distance(self.x1, self.odometryData.x, self.y1, self.odometryData.y)
            self.total_distance = self.total_distance + travelled_distance
            self._feedback.current_total = self.total_distance
            self._as.publish_feedback(self._feedback)
            
            self.x1 = self.odometryData.x
            self.y1 = self.odometryData.y
            # Append the odometries as result
            self.list_trajectory.append(copy.copy(self.odometryData))

            r.sleep()    

        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
        
        if success:
            # Save list of odometries as result
            self._result.list_of_odoms = self.list_trajectory
            rospy.loginfo('Printing odometries')
            self._as.set_succeeded(self._result)
            print(self._result)


if __name__ == "__main__":
    rospy.init_node('record_odom_node')
    RecordOdomClass()
    rospy.spin()

        