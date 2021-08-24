#!/usr/bin/python3


from numpy.lib.function_base import _angle_dispatcher, angle
import rospy
from getkey import getkey
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Transform
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import time
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi

class KeyboardJoystick:
    def __init__(self):
        rospy.init_node('keyboard_node')
        self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.bridge = CvBridge()
        self.tf_husky_sub = rospy.Subscriber("/tf", tfMessage, self.tf_husky_callback)
        self.local_x = 0
        self.local_y = 0
        self.angular_z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    '''
    Husky callback to get transform and rotation
    '''
    def tf_husky_callback(self, msg):
        if msg.transforms[0].child_frame_id == 'base_link':
            # print(msg.transforms[0].transform.translation.x)
            self.local_x = msg.transforms[0].transform.translation.x
            self.local_y = msg.transforms[0].transform.translation.y
            self.angular_z = None
            # print(msg.transforms[0].transform.rotation)
            orientation_list = [msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y,
                                msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w]
            (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

            self.roll = self.roll * 180.0 / pi
            self.pitch = self.pitch * 180.0 / pi
            self.yaw = self.yaw * 180.0 / pi

            # x = cos(self.yaw)*cos(self.pitch)
            # y = sin(self.yaw)*cos(self.pitch)
            # z = sin(self.pitch)
            # print(x, y, z)
            # crx = self.roll # Roll
            # cry = self.pitch # Pitch
            # crz = self.yaw # Yaw


            # print(crx, cry, crz)
            # x = sin(crz)
            # y = -(sin(crx) * cos(crz))
            # z = cos(crx) * cos(cry)
            # print(x, y, z)


    def keyboard_control(self, key_d):

        k = key_d
        if k == 'a':
            self.msg.linear.x = 0.2
            self.msg.angular.z = 0.3
        elif k == 'w':
            self.msg.linear.x = 0.5
            self.msg.angular.z = 0
        elif k == 'd':
            self.msg.linear.x = 0.2
            self.msg.angular.z = -0.3
        elif k == 's':
            self.msg.linear.x = -0.5
            self.msg.angular.z = 0
        elif k == 'q':
            rospy.signal_shutdown("Shutdown!!!")
            exit(1)
        else:
            self.msg.linear.x = 0
            self.msg.angular.z = 0
            print(k ,"invalid")
        
        self.pub.publish(self.msg)
        self.rate.sleep()
    
    def auto_run(self):
        self.msg.linear.x = 0.2
        if self.local_x >= 4:
            if self.yaw >= 90:
                self.msg.linear.y = 0.2
                self.msg.angular.z = 0
            else:
                self.msg.linear.x = 0.2
                self.msg.angular.z = 0.4
        if self.local_y >= 4:
            self.msg.linear.x = 0
            self.msg.linear.y = 0

            # if self.yaw >= 180:
            #     self.msg.linear.y = 0.3
            #     self.msg.angular.z = 0
            # else:
            #     self.msg.linear.y = 0.1
            #     self.msg.angular.z = 0.3

        self.pub.publish(self.msg)
        self.rate.sleep()

if __name__ == '__main__':
    print("\tScript Control Husky Car")
    kt = KeyboardJoystick()
    try:
        while(not rospy.is_shutdown()):
            # key = getkey()
            # kt.keyboard_control(key)
            kt.auto_run()

    except rospy.ROSInitException as e:
        print(e)

