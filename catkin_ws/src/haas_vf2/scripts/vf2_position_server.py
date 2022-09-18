#!/usr/bin/env python

#TODO This service should take in an x and y position and emit joint position messages that achieve that position.


# /y_axis_to_x_axis_controller/command std_msgs/Float64 10.0



from __future__ import print_function

from haas_vf2.srv import vf2, vf2Response
from std_msgs.msg import Float64
import rospy

class Vf2PositionHandler():
    def __init__(self):
        self.x_pub = rospy.Publisher('/y_axis_to_x_axis_controller/command', Float64, queue_size=1)
        self.y_pub = rospy.Publisher('/base_to_y_axis_controller/command', Float64, queue_size=1)

    def handle(self,req):
        # x_pos = Float64()
        x_pos  = float(req.x_pos)

        # y_pos = Float64()
        y_pos = float(req.y_pos)

        print('attempting to acheive position x,y: ', x_pos, y_pos)

        # TODO emit messages to other channels
        self.x_pub.publish(x_pos)
        self.y_pub.publish(y_pos)
        

        return vf2Response(x_pos, y_pos)

def handle_vf2_position(req):


    # x_pos = Float64()
    x_pos  = float(req.x_pos)

    # y_pos = Float64()
    y_pos = float(req.y_pos)

    print('attempting to acheive position x,y: ', x_pos, y_pos)

    # TODO emit messages to other channels
    

    return vf2Response(x_pos, y_pos)

def handle_get_twist_server():
    rospy.init_node('vf2_position_server')
    pos_handler = Vf2PositionHandler()
    s = rospy.Service('vf2_position', vf2, pos_handler.handle)
    # s = rospy.Service('vf2_position', vf2, handle_vf2_position)
    print("Ready to set vf2 position")
    rospy.spin()

if __name__ == "__main__":
    handle_get_twist_server()