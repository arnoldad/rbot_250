#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from rbot250.srv import *

def get_twist_client(msg):
    rospy.wait_for_service('get_twist')
    try:
        get_twist = rospy.ServiceProxy('get_twist', twist)
        resp1 = get_twist(msg)
        return resp1.twist
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        msg = str(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting Twist...")
    print("Got twist: ", get_twist_client(msg))