#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

regions = {
    'right': 0,
    'front': 0,
    'left': 0
}


def callback(msg):
    
    # Regions

    regions = {
        'right': msg.ranges[179],
        'front': msg.ranges[360],
        'left': msg.ranges[539]
    }

    print(regions)
    

    if regions['front'] < 0.5:
        print("Turn fast to left")
        var.linear.x = 0.05
        var.angular.z = 0.5
        ray_angle = msg.angle_min + (regions['front'] * msg.angle_max)
        # print("ray angle front", ray_angle)
    elif regions['right'] > 0.3:
        print("Find the wall")
        var.linear.x = 0.1
        var.angular.z = -0.1
        ray_angle = msg.angle_min + (regions['right'] * msg.angle_max)
        # print("ray angle right", ray_angle)
    elif regions['right'] < 0.2:
        print("Move away from the wall")
        var.linear.x = 0.05
        var.angular.z = 0.05
        ray_angle = msg.angle_min + (regions['right'] * msg.angle_max)
        # print("ray angle right", ray_angle)
        if regions['right'] > 0.2:
            var.linear.x = 0
            var.angular.z = 0
    elif regions['right'] > 0.2 and regions['right'] < 0.3:
        print("Follow the wall")
        var.linear.x = 0.1
        var.angular.z = 0
        ray_angle = msg.angle_min + (regions['right'] * msg.angle_max)
        # print("ray angle right", ray_angle)
    elif regions['left'] < 0.3:
        print("Move away from obstacle")
        var.linear.x = 0.0
        var.angular.z = -0.1
        ray_angle = msg.angle_min + (regions['left'] * msg.angle_max)
        # print("ray angle left", ray_angle)
    else:
        print("Stop")
        var.linear.x = 0
        var.angular.z = 0

rospy.init_node('drive_along_wall')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

var = Twist()
# var.linear.x = 0.05
# var.angular.z = 0.3

while not rospy.is_shutdown():
    pub.publish(var)
    rate.sleep()

