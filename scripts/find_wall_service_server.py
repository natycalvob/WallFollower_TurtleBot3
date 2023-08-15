#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from realrobot_pkg.srv import FindWall, FindWallResponse

# Move the robot forward until the front ray is smaller than 30cm.
# Now rotate the robot again until ray number 270 of the laser ranges is pointing to the wall.
# At this point, consider that the robot is ready to start following the wall.
# Return the service message with True.

def get_min_value(ranges):
    ranges
    min_val = ranges[0]
    min_val_idx = 0
    for i in range(len(ranges)):
        if ranges[i] < min_val:
            min_val = ranges[i]
            min_val_idx = i
    
    return min_val, min_val_idx
    
def subscriber_callback(msg):
    global readings 
    readings = msg

def service_callback(request):
    global readings
    rospy.loginfo('The service move_nearest_wall has been called')
    
    min_val, min_val_idx = get_min_value(readings.ranges)

    front_value = round(readings.ranges[360], 3)
    right_value = round(readings.ranges[270], 3)
    print(min_val, min_val_idx, front_value, right_value)
    
    rospy.loginfo('Looking for the closest wall...')

    while not (min_val_idx <= 362 and min_val_idx >= 358):
        # print("Rotating to face the wall")
        move2wall.linear.x = 0.0
        move2wall.angular.z = 0.1
        _, min_val_idx = get_min_value(readings.ranges)
        # print("updated", min_val_idx)
        my_pub.publish(move2wall)
        rate.sleep()
    
    rospy.loginfo('Approaching to the closest wall...')

    while (front_value > 0.25):
        # print("Approaching the wall", front_value)
        move2wall.linear.x = 0.05
        move2wall.angular.z = 0.0
        front_value = round(readings.ranges[360], 3)
        my_pub.publish(move2wall)
        rate.sleep()

    rospy.loginfo('Turning to follow the wall...')

    while not (min_val_idx <= 181 and min_val_idx >= 179):
        # print("Turning to follow the wall")
        move2wall.linear.x = 0.0
        move2wall.angular.z = 0.1
        _, min_val_idx = get_min_value(readings.ranges)
        # print("updated right_value", min_val_idx)
        my_pub.publish(move2wall)
        rate.sleep()

    rospy.loginfo('Ready to follow the wall')
    move2wall.linear.x = 0.0
    move2wall.angular.z = 0.0
    my_pub.publish(move2wall)
    rate.sleep()

    result = FindWallResponse()
    result.wallfound = True
    return result

rospy.init_node('service_move_nearest_wall')
my_sub = rospy.Subscriber('/scan', LaserScan, subscriber_callback)
nearest_wall_service = rospy.Service('/move_nearest_wall', FindWall, service_callback)
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

move2wall = Twist()
readings = LaserScan()

rospy.spin()

