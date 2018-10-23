#! /usr/bin/env python

import rospy
import serial
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import struct
import math
from time import sleep

class Odom(object):
    def __init__(self):
        #Start node and load parameters
        rospy.init_node("positioning_node", anonymous=False)
        self.param_port = rospy.get_param("~encoder_port", default="/dev/ttyACM0")
        self.odom_ref = Odometry()
        self.odom_ref.header.frame_id = "odom"
        rot_z = 0.0
        linear_x = 0.0
        linear_y = 0.0

        #Reference ROS IO
        self.publisher_odom = rospy.Publisher("/odom", Odometry, queue_size=5)
        self.sub_cmd = rospy.Subscriber("/cmd_vel", Twist, self.loadTwist)
        self.broadcaster_tf = tf.TransformBroadcaster(queue_size=10)

        #Load Serial Port
        try:
            self.serial_obj = serial.Serial(self.param_port, 9600)
        except serial.SerialException as ex:
            rospy.logerr("Serial port fault")
            rospy.signal_shutdown(0)

        sleep(3.0)
        #Event Loop
        while(not rospy.is_shutdown()):
            wheel_sw = 0.40
            wheel_sl = 0.36
            linear_constant = 0.0000245633858171010
            angular_constant = 0.0002005 / (4*(wheel_sl + wheel_sw))
            
            encoder_data = self.loadSerial()
            fl = encoder_data[3]
            fr = -encoder_data[2]
            bl = encoder_data[0]
            br = -encoder_data[1]

            linear_x = (fl + fr + bl + br)*linear_constant
            linear_y = (-fl + fr + bl - br)*linear_constant
            rot_z += (-fl + fr - bl + br)*angular_constant
            qua = tf.transformations.quaternion_from_euler(0.0,0.0, rot_z)

            self.odom_ref.pose.pose.position.x += ((math.cos(rot_z)*linear_x) - (math.sin(rot_z)*linear_y))
            self.odom_ref.pose.pose.position.y += ((math.cos(rot_z)*linear_y) + (math.sin(rot_z)*linear_x))

            self.odom_ref.pose.pose.orientation.x = qua[0]
            self.odom_ref.pose.pose.orientation.y = qua[1]
            self.odom_ref.pose.pose.orientation.z = qua[2]
            self.odom_ref.pose.pose.orientation.w = qua[3]

            self.publisher_odom.publish(self.odom_ref)
            self.broadcaster_tf.sendTransform(
                (self.odom_ref.pose.pose.position.x, self.odom_ref.pose.pose.position.y, 0.0),
                qua,
                rospy.Time.now(),
                "base_link",
                "odom"
            )

    def loadSerial(self):
        self.serial_obj.write(chr(0x01))
        dat_raw = self.serial_obj.read(size=16)
        return struct.unpack('>llll', dat_raw)
    
    def loadTwist(self, cmdVel):
        self.odom_ref.twist.twist.linear.x = cmdVel.linear.x
        self.odom_ref.twist.twist.linear.y = cmdVel.linear.y
        self.odom_ref.twist.twist.angular.z = cmdVel.angular.z

if __name__ == '__main__':
    Odom()