#!/usr/bin/python3

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Transform, PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
import tf
import rospy
import cv2 as cv
import numpy as np
import time
from EKF import ExtendedKalmanFilter
from common import Q_discrete_white_noise


class TrackerEKF:
    def __init__(self):
        self.pose = PoseStamped()
        self.dt = 0.2
        rospy.init_node("EKF_node")
        rate = rospy.Rate(20)
        rospy.Timer(rospy.Duration(self.dt), self.my_callback)
        self.pose_sub = rospy.Subscriber('measurement/pose', PoseStamped, self.poseCallback)
        self.state_pub = rospy.Publisher('kf/estimate', PoseWithCovarianceStamped, queue_size=10)
        '''
        Init for EKF.
        '''
        # Tracking position and velocity of car (x, y, Vx, Vy)
        self.kf = ExtendedKalmanFilter(dim_x=4, dim_z=4)
        # Initial state
        self.kf.x = np.array([[1.],     # position
                                [2.],
                                [0.5],  # velocity
                                [0.5]])
        # State transition matrix
        self.kf.F = np.array([[1.,1.], [0.,1.]])
        # Measurement function
        self.kf.H = np.array([[1.,0.]])
        # Covariance matrix
        self.kf.P *= 1000.
        # State uncertainty
        self.kf.R = 5
        # Process uncertaint
        self.kf.Q = Q_discrete_white_noise(2, self.dt, .1)

        while not rospy.is_shutdown():
            rate.sleep()

    def poseCallback(self, msg):
        rospy.loginfo("KF state estimate is initialized")
        self.pose.pose.position.x = msg.pose.position.x
        self.pose.pose.position.y = msg.pose.position.y
        self.pose.pose.position.z = msg.pose.position.z

    '''
    This function is called every dt sec
    to run calculation EKF again.
        - predict state
        - update state
        - publish to topic to controller get info
    '''
    def my_callback(self, event):
        rospy.loginfo("my callback")
        # self.kf.predict()
        self.pub_state()

    def pub_state(self):
        rospy.loginfo("Publish state")
        msg = PoseWithCovarianceStamped()
        self.state_pub.publish(msg)

if __name__ == "__main__":
    ekf = TrackerEKF()