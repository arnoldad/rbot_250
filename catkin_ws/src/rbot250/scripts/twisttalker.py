#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def twisttalker():
    pub = rospy.Publisher('chatter', Twist, queue_size=10)
    rospy.init_node('twisttalker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist = Twist()

        twist.linear.x = 1.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 1.571

        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        twisttalker()
    except rospy.ROSInterruptException:
        pass
