#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from realrobot_pkg.msg import OdomRecordAction, OdomRecordFeedback
import actionlib
from std_msgs.msg import Empty

regions = {
    'right': 0,
    'front': 0,
    'left': 0
}

def feedback_callback(feedback):
    rospy.loginfo("Distance travelled so far: " + str(feedback.current_total))

def callback(msg):    
    # Regions
    regions = {
        'right': msg.ranges[179],
        'front': msg.ranges[360],
        'left': msg.ranges[539]
    }
    
    if regions['front'] < 0.5:
        # print("Turn fast to left")
        var.linear.x = 0.05
        var.angular.z = 0.5
    elif regions['right'] > 0.3:
        # print("Find the wall")
        var.linear.x = 0.1
        var.angular.z = -0.1
    elif regions['right'] < 0.2:
        # print("Move away from the wall")
        var.linear.x = 0.05
        var.angular.z = 0.05
        if regions['right'] > 0.2:
            var.linear.x = 0
            var.angular.z = 0
    elif regions['right'] > 0.2 and regions['right'] < 0.3:
        # print("Follow the wall")
        var.linear.x = 0.1
        var.angular.z = 0
    elif regions['left'] < 0.3:
        # print("Move away from obstacle")
        var.linear.x = 0.0
        var.angular.z = -0.1
    else:
        # print("Stop")
        var.linear.x = 0
        var.angular.z = 0

# Publisher/Subscriber Initialization
rospy.init_node('drive_along_wall')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)
var = Twist()

# Create the connection to the action server
# rospy.init_node('record_odom_client')
client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
rospy.loginfo('Waiting for the action client to send the goal...')
client.wait_for_server()
rospy.loginfo('Connection is ready...')
# Create a gold to send to the action service
goal = Empty()
client.send_goal(goal, feedback_cb=feedback_callback)

while not rospy.is_shutdown():
    pub.publish(var)
    rate.sleep()

