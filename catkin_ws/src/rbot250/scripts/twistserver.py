#!/usr/bin/env python

from __future__ import print_function

from rbot250.srv import twist, twistResponse
from geometry_msgs.msg import Twist
import rospy

def handle_get_twist(req):
    print('dummy input:', req)
    
    twist = Twist()

    twist.linear.x = 1.2
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 1.571

    print("Returning twist, ", twist)
    return twistResponse(twist)

def handle_get_twist_server():
    rospy.init_node('get_twist_server')
    s = rospy.Service('get_twist', twist, handle_get_twist)
    print("Ready to get twist")
    rospy.spin()

if __name__ == "__main__":
    handle_get_twist_server()