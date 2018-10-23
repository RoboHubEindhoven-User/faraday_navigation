#!/usr/bin/env python

import rospy
import numpy
import socket
import struct
from geometry_msgs.msg import Twist


class Drivers(object):
    def __init__(self):
        self.settings = [0.2, 0.3, 0.075]
        self.shock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        target_host = rospy.get_param("hostname", default="192.168.2.103")
        self.shock.connect((target_host, 11001))

        rospy.init_node('socket_velocity', anonymous=False)
        rospy.Subscriber("/cmd_vel", Twist, self.twist_socket)
        rospy.spin()

    def twist_socket(self, data):
        twist_vx = data.linear.x
        twist_vy = data.linear.y
        twist_w0 = data.angular.z
        
        twist_vec = numpy.array([[twist_vx], [twist_vy], [twist_w0]])
        Lt = self.settings[0] + self.settings[1]/numpy.tan(45)
        A = 1 / numpy.tan(45)
        
        angular_vel_1 = [1, -A, -Lt]
        angular_vel_2 = [1, A, Lt]
        angular_vel_3 = [1, A, -Lt]
        angular_vel_4 = [1, -A, Lt]
        
        angular_vel = numpy.array([angular_vel_1, angular_vel_2, angular_vel_3, angular_vel_4])
        
        w = numpy.dot(angular_vel, twist_vec)
        angular_vel_1 = w[0].item() / self.settings[2]
        angular_vel_2 = w[1].item() / self.settings[2]
        angular_vel_3 = w[2].item() / self.settings[2]
        angular_vel_4 = w[3].item() / self.settings[2]

        end_data = struct.pack('dddd', angular_vel_1, angular_vel_2, angular_vel_3, angular_vel_4)
        self.shock.send(end_data)


if __name__ == '__main__':
    m = Drivers()