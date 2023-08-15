#! /usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from realrobot_pkg.srv import FindWall, FindWallRequest

rospy.init_node('call_move_nearest_wall')
rospy.wait_for_service('/move_nearest_wall')

find_nearest_wall_service_client = rospy.ServiceProxy('/move_nearest_wall', FindWall)
find_nearest_wall_request_object = FindWallRequest()

result = find_nearest_wall_service_client(find_nearest_wall_request_object)
print(result)
