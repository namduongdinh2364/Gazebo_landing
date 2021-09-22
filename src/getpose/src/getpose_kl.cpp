#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <mavros_msgs/CommandBool.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "Kalmanfilter.h"
/******************************************************************************* 
 *                               Definitions
 ******************************************************************************/ 
#define PRECISION(x)    round(x * 100) / 100
#define DISTANCE        0.3

/******************************************************************************* 
 *                                Namespace
 ******************************************************************************/ 

using namespace std;
using namespace Eigen;

/******************************************************************************* 
 *                                  Object
 ******************************************************************************/
KalmanPID kalman_x = KalmanPID(0, 5, 1.5);
KalmanPID kalman_y = KalmanPID(0, 5, 1.5);
KalmanPID kalman_z = KalmanPID(0, 5, 1.5);

/******************************************************************************* 
 *                                 Variables
 ******************************************************************************/

static double var_offset_pose[3] = {0.0, 0.0, 0.0};
static char var_active_status[20];
static double x, y, z;
static bool detect = false;
ofstream outfile0, outfile1, outfile2;
Matrix3f R, cv_rotation, cam2imu_rotation;
Vector3f positionbe, position_cam, positionaf, offset_marker;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped vlocal_pose;
static int number_check = 0;
static int LOCK = 10;

/******************************************************************************* 
 *                                  Code
 ******************************************************************************/ 

/* storing gps data in pointer */
void mavros_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    vlocal_pose=*msg;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    double x,y,z,w;
    Quaternionf quat;

    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;
    /*making a quaternion of position*/
    quat = Eigen::Quaternionf(w,x,y,z);
    /*making rotation matrix from quaternion*/
    R = quat.toRotationMatrix();
    tf2::Quaternion q;
    q.setValue(x, y, z, w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll  = roll*(180/3.14);
    pitch = pitch*(180/3.14);
    yaw   = yaw*(180/3.14);
}

static void get_params_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    double xq,yq,zq,wq;
    Quaternionf quat;
    cam2imu_rotation << -0.0001 , -1 , 0 , -1 , 0 , 0 ,-0.0001 , 0 , -1;

    position_cam[0] = (msg->transforms[0].transform.translation.x);
    position_cam[1] = (msg->transforms[0].transform.translation.y);
    position_cam[2] = (msg->transforms[0].transform.translation.z);

    xq = msg->transforms[0].transform.rotation.x;
    yq = msg->transforms[0].transform.rotation.y;
    zq = msg->transforms[0].transform.rotation.z;
    wq = msg->transforms[0].transform.rotation.w;

    if (LOCK > 0) {
        /* Aruco ----> Drone */
        positionbe = cam2imu_rotation*position_cam;
        /* Drone ----> NEU */
        positionaf = R*positionbe;
        /* update the position */

        x = positionaf[0] + vlocal_pose.pose.position.x;
        y = positionaf[1] + vlocal_pose.pose.position.y;
        z = positionaf[2] + vlocal_pose.pose.position.z;
    }

    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = xq;
    pose.pose.orientation.y = yq;
    pose.pose.orientation.z = zq;
    pose.pose.orientation.w = wq;

    detect = true;
    LOCK = 0;
    number_check ++;
    if(number_check == 3) {
        LOCK = 1;
        number_check = 0;
    }

    cout<<"Marker2Cam  : " << PRECISION(position_cam[0]) <<'\t'<< PRECISION(position_cam[1]) << '\t' << PRECISION(position_cam[2]) << endl;
    cout<<"Marker2Drone: " << PRECISION(positionbe[0]) <<'\t'<< PRECISION(positionbe[1]) << '\t' << PRECISION(positionbe[2]) << endl;
    cout<<"Marker2NEU  : " << PRECISION(x) <<'\t'<< PRECISION(y) << '\t' << PRECISION(z) << endl;
    cout<<"Drone      : " << PRECISION(vlocal_pose.pose.position.x) << "\t" << PRECISION(vlocal_pose.pose.position.y) << "\t" << PRECISION(vlocal_pose.pose.position.z) << endl;
}

void local_set_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!detect) {
        pose.pose.position.x= msg->pose.position.x;
        pose.pose.position.y= msg->pose.position.y;
        pose.pose.position.z= msg->pose.position.z;
    }

    else {
        pose.pose.position.x= x;
        pose.pose.position.y= y;
        pose.pose.position.z= z;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "getpose_node");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe
                            ("/mavros/imu/data",10, imuCallback);
    ros::Subscriber gps_sub = n.subscribe
                            ("/mavros/local_position/pose",10, mavros_Pose_Callback);
    ros::Subscriber pose_sub = n.subscribe
                            ("/tf_list", 10, get_params_cb);
    ros::Subscriber local_set_pos_sub = n.subscribe
                            ("mavros/setpoint_position/local", 10, local_set_pose_callback);
    ros::Publisher pose_to_kl_pb = n.advertise<geometry_msgs::PoseStamped>
                            ("tag/pose", 10);
    ros::Rate rate(10.0);

    for(int i = 5; ros::ok() && i > 0; --i) {
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()) {
        pose_to_kl_pb.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
