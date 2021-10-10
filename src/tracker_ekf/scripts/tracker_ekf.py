#!/usr/bin/python3

from numpy.core.fromnumeric import cumprod
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

class TrackerEKF():
    def __init__(self):
        # pose of target
        self.pose = PoseStamped()
        self.cur_pose = PoseStamped()
        self.dt = 0.2
        self.run = False
        self.Inputcontrol = np.array([[0.0],
                                        [0.]])

        self.imu2neu_rotation = np.eye(3)

        rospy.init_node("EKF_node")
        rate = rospy.Rate(20)
        rospy.Timer(rospy.Duration(self.dt), self.EKF_callback)
        self.state_pub = rospy.Publisher('kf/estimate', PoseWithCovarianceStamped, queue_size=10)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_Callback)
        self.mav_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_Callback)
        self.marker_pose_sub = rospy.Subscriber('/tf_list', TFMessage, self.marker_pose_Callback)

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
            # rospy.loginfo("Publish state")
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = self.kf.x[0, 0]
            msg.pose.pose.position.y = self.kf.x[1, 0]
            msg.pose.pose.position.z = self.pose.pose.position.z
            msg.pose.pose.orientation.x = self.pose.pose.orientation.x
            msg.pose.pose.orientation.y = self.pose.pose.orientation.y
            msg.pose.pose.orientation.z = self.pose.pose.orientation.z
            msg.pose.pose.orientation.w = self.pose.pose.orientation.w
            print(msg)
            self.state_pub.publish(msg)
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

    def imu_Callback(self, msg):
        # making a quaternion of position
        orientation_list =[msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w]

        self.imu2neu_rotation = quaternion_matrix(orientation_list)
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        yaw = yaw*(180/3.14)
        # print(self.imu2neu_rotation[:3, :3])
        # print(yaw)

    def mavros_pose_Callback(self, msg):
        self.cur_pose.header.stamp = msg.header.stamp
        self.cur_pose.pose.position.x = msg.pose.position.x
        self.cur_pose.pose.position.y = msg.pose.position.y
        self.cur_pose.pose.position.z = msg.pose.position.z
        # print(self.cur_pose)

    def marker_pose_Callback(self, msg):
        cam2imu_rotation = np.array ([[-0.0001 , -1 , 0],
                                        [-1 , 0 , 0 ],
                                        [-0.0001 , 0 , -1]])
        # position_marker_to_cam = zeros((3,1))
        # Get data 
        position_marker_to_cam = np.array([[msg.transforms[0].transform.translation.x],
                                            [msg.transforms[0].transform.translation.y],
                                            [msg.transforms[0].transform.translation.z]])

        orientation_list = [msg.transforms[0].transform.rotation.x,
                            msg.transforms[0].transform.rotation.y,
                            msg.transforms[0].transform.rotation.z,
                            msg.transforms[0].transform.rotation.w]
        # Skip calculation to stable param

        # Marker ----> Drone
        position_marker_to_drone = dot(cam2imu_rotation, position_marker_to_cam)
        # Marker ----> NEU
        position_marker_to_NEU = dot(self.imu2neu_rotation[:3, :3], position_marker_to_drone)
        # Update the position
        pose_marker = [ position_marker_to_NEU[0, 0] + self.cur_pose.pose.position.x,
                        position_marker_to_NEU[1, 0] + self.cur_pose.pose.position.y,
                        position_marker_to_NEU[2, 0] + self.cur_pose.pose.position.z ]

        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = pose_marker[0]
        self.pose.pose.position.y = pose_marker[1]
        self.pose.pose.position.z = pose_marker[2]

        self.pose.pose.orientation.x = orientation_list[0]
        self.pose.pose.orientation.y = orientation_list[1]
        self.pose.pose.orientation.z = orientation_list[2]
        self.pose.pose.orientation.w = orientation_list[3]
        # print(self.pose)


def main():
    ekf = TrackerEKF()
    ekf.start()
    ekf.setControlVector(0.5, 0)
    rospy.spin()
    
    return 0

if __name__ == "__main__":
    main()
