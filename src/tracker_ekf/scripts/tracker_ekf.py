#!/usr/bin/python3

from _typeshed import Self
from numpy.ma.core import set_fill_value
import tf
import rospy
import cv2 as cv
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Transform, PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, euler_from_matrix
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from numpy import dot, zeros, eye
from EKF import ExtendedKalmanFilter
from common import Q_discrete_white_noise
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

def HJacobian(x):
    H = np.array([[1., 0., 0.],
                    [0., 1., 0.],
                    [0., 0., 1.]])

    return H

def Hx(x):
    H = np.array([[1., 0., 0.],
                    [0., 1., 0.],
                    [0., 0., 1.]])

    return dot(H, x)

class PoseMarker():
    def __init__(self):
        self.cam2imu_rotation = np.array ([[-0.0001 , -1 , 0],
                                            [-1 , 0 , 0 ],
                                            [-0.0001 , 0 , -1]])
        self.imu2neu_rotation = eye(3)
        self.position_cam = zeros((3,1))

        rospy.init_node("GetPose_node")
        rate = rospy.Rate(10)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_Callback)
        self.mav_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_Callback)
        self.marker_pose_sub = rospy.Subscriber('/tf_list', TFMessage, self.marker_pose_Callback)
        self.local_set_pos_sub = rospy.Subscriber('mavros/setpoint_position/local', PoseStamped, self.local_set_pose_Callback)

    def imu_Callback(self, msg):
        # making a quaternion of position
        quat = Quaternion(msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w)
        # making rotation matrix from quaternion
        imu2neu_rotation = quaternion_matrix(quat)
        yaw, pitch, roll = euler_from_matrix(imu2neu_rotation, 'rxyz')
        roll  = roll*(180/3.14)
        pitch = pitch*(180/3.14)
        yaw   = yaw*(180/3.14)

    def mavros_pose_Callback(self):
        pass

    def marker_pose_Callback(self):
        pass

    def local_set_pose_Callback(self):
        pass

class TrackerEKF():
    def __init__(self):
        self.pose = PoseStamped()
        self.dt = 0.2
        self.run = False
        self.Inputcontrol = np.array([[0.0],
                                        [0.]])

        rospy.init_node("EKF_node")
        rate = rospy.Rate(20)
        rospy.Timer(rospy.Duration(self.dt), self.EKF_callback)
        self.pose_sub = rospy.Subscriber('tag/pose', PoseStamped, self.poseCallback)
        self.state_pub = rospy.Publisher('kf/estimate', PoseWithCovarianceStamped, queue_size=10)


        """
        Init for EKF.
        """
        # Tracking position and velocity of car (x, y, Vx, Vy)
        self.kf = ExtendedKalmanFilter(dim_x=3, dim_z=3)
        # Initial state
        rospy.loginfo("EKF state estimate is initialized")
        self.kf.x = np.array([[1.],     # position
                                [0.],
                                [0.]])  # yaw 
        # State transition matrix
        self.kf.F = np.array([[1., 0., 0.],
                                [0., 1., 0.],
                                [0., 0., 1.]])
        # Measurement function (3x3)
        self.kf.H = np.array([[1., 0., 0.],
                                [0., 1., 0.],
                                [0., 0., 1.]])
        # Covariance matrix
        self.kf.P = np.array([[1., 0., 0.],
                                [0., 1., 0.],
                                [0., 0., 1.]])
        # State uncertainty
        # Sensor measurement noise covariance matrix
        self.kf.R = np.array([[0.1, 0., 0.],
                                [ 0., 0.1, 0.],
                                [ 0., 0., 0.1]])
        # Process uncertaint
        # self.kf.Q = Q_discrete_white_noise(3, self.dt, var=0.13)
        self.kf.Q = np.array([[1., 0., 0.],
                                [0., 1., 0.],
                                [0., 0., 1.]])

        self.kf.B = np.array([[np.cos(0)*self.dt, 0.],
                            [np.sin(0)*self.dt, 0.],
                            [0, self.dt]])

    def poseCallback(self, msg):
        self.pose.header.stamp = msg.header.stamp
        self.pose.pose.position.x = msg.pose.position.x
        self.pose.pose.position.y = msg.pose.position.y
        self.pose.pose.position.z = msg.pose.position.z

    def EKF_callback(self, event):
        """
        This function is called every dt sec
        to run calculation EKF again.
            - predict state
            - update state
            - publish to topic to controller get info
        """
        if self.run is True:
            self.kf.predict(self.Inputcontrol)
            z = np.array([[self.pose.pose.position.x],
                            [self.pose.pose.position.y],
                            [0.]])  # yaw 
            self.kf.update(z, HJacobian, Hx)
            # Prepare data
            rospy.loginfo("Publish state")
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = self.kf.x[0, 0]
            msg.pose.pose.position.y = self.kf.x[1, 0]
            msg.pose.pose.position.z = self.pose.pose.position.z
            print(msg)
            # self.state_pub.publish(msg)
        else:
            pass

    def setControlVector(self, v, yaw):
        """
        Calculates the B matrix 3x2 matix -> number of 
        states of control inputs, The control inputs are
        the speed and the rotation rate around the z axis
        in the counterclockwise direction.
        [V, yaw_rate]
        Expresses how the state of the system [x, y, yaw] changes
        from k-1 to k due to the control commands (i.e. control input).
        """
        self.Inputcontrol = np.array([[v],
                                        [yaw]])
        self.kf.B = np.array([ [np.cos(yaw)*self.dt, 0.],
                            [np.sin(yaw)*self.dt, 0.],
                            [0, self.dt]])

    def start(self):
        self.run = True
        rospy.loginfo("EKF start")
    
    def stop(self):
        self.run = False
        rospy.loginfo("EKF stop")

def main():
    ekf = TrackerEKF()
    ekf.start()
    ekf.setControlVector(0, 0)
    rospy.spin()
    
    return 0

if __name__ == "__main__":
    main()
