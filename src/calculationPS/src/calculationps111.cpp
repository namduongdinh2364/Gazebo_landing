#include "ros/ros.h"
#include <math.h>
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
#include <csignal>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/******************************************************************************* 
 *                               Definitions 
 ******************************************************************************/ 
#define PRECISION(x)    round(x * 100) / 100
#define DISTANCE        0.3
#define NSTEP           4
#define PI              3.14159265
#define HeightChangeAngle           7
#define IncreaseHeightNotDetect     1
#define MaxRepeatDetections         2
/******************************************************************************* 
 *                                Namespace
 ******************************************************************************/ 
using namespace std;
using namespace Eigen;
/******************************************************************************* 
 *                                  Topic
 ******************************************************************************/ 
ros::Publisher custom_activity_pub;

/******************************************************************************* 
 *                                 Variables 
 ******************************************************************************/
int detectfailedrepeat = 0;
bool active_2s_ago = true;
static int number_check          = 0;
bool semaphore_b = true;
static bool LOCK_LAND            = false;
static bool vLand                = false;
static bool vend                 = false;
static int LOCK                  = 10;
time_t baygio                    = time(0);
tm *ltime                        = localtime(&baygio);
static double var_offset_pose[3] = {0.0, 0.0, 0.0};
static char var_active_status[20];
static double x, y, z;
static double x_ = 0, y_ = 0, z_ = 0;
ofstream outfile0, outfile1, outfile2;
Matrix3f R, cv_rotation, cam2imu_rotation, cam2imu_rotation_april;
Vector3f positionbe, position_cam, positionaf, point_change, positionaf_change;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped vlocal_pose;
int vbegin    = 2;
float minutes = 0;
float seconds = 0;
ros::Time begin_request, reset_request;
static bool detect = false;

/******************************************************************************* 
 *                                  Code 
 ******************************************************************************/ 
void turn_off_motors(void) {
    std_msgs::String msg;
    std::stringstream ss;

    ss << "LAND";
    msg.data = ss.str();
    custom_activity_pub.publish(msg);
}

/*storing gps data in pointer*/
void mavrosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
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
    yaw   = yaw*(180/3.14);;
}

/**
 * @brief 
 * 
 * @param 
 *
 * @return 
 */
// static void get_params_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
void set_target_position_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // begin_request = ros::Time::now();
    // active_2s_ago = true;
    // if( ros::Time::now() - reset_request > ros::Duration(3.0) ) {
    //     detectfailedrepeat = 0;
    // }
    // float radius;
    // if (LOCK_LAND == false) {
        // if (vbegin == 1) {
        //     baygio = time(0);
        //     ltime = localtime(&baygio);
        //     cout << "Start:" << ltime->tm_min << ":";
        //     cout << ltime->tm_sec << endl;
        //     minutes = ltime->tm_min;
        //     seconds = ltime->tm_sec;
        //     vbegin = 0;
        // }

        // double xq,yq,zq,wq;
        // Quaternionf quat;
        // cam2imu_rotation << -0.0001 , -1 , 0 , -1 , 0 , 0 ,-0.0001 , 0 , -1;

        // cam2imu_rotation_april << 0 , 1 , 0 , -1 , 0 , 0 ,0 , 0 , 1;
        position_cam[0] = msg->pose.pose.position.x;
        position_cam[1] = msg->pose.pose.position.y;
        position_cam[2] = msg->pose.pose.position.z;
        /* Drone ----> NEU */
        positionaf = position_cam;
        // positionaf = position_cam;
        // position_cam[0] = (msg->transforms[0].transform.translation.x);
        // position_cam[1] = (msg->transforms[0].transform.translation.y);
        // position_cam[2] = (msg->transforms[0].transform.translation.z);
        // xq = msg->transforms[0].transform.rotation.x;
        // yq = msg->transforms[0].transform.rotation.y;
        // zq = msg->transforms[0].transform.rotation.z;
        // wq = msg->transforms[0].transform.rotation.w;

        // position_cam = cam2imu_rotation_april*position_cam;
        /* Aruco ----> Drone */
            // positionbe = cam2imu_rotation*position_cam;
            // /* Drone ----> NEU */
            // positionaf = R*positionbe;
            // /* update the position */
            // double alpha, OR, A1E, Y2, X2, Z2;
            // OR = (float)sqrt(pow(positionbe[0],2) + pow(positionbe[1],2));
            // alpha = atan (OR/positionbe[2]) * 180 / PI;
            // A1E = (OR *( abs(positionbe[2]) - HeightChangeAngle )) / abs(positionbe[2]);
            // point_change[1] = (positionbe[1] * A1E) / OR;
            // point_change[0] = (positionbe[0] * A1E) / OR;
            // point_change[2] = positionbe[2] + HeightChangeAngle;
            // positionaf_change = R*point_change;
            // if (LOCK > 0)
            // {
        if (LOCK > 0)
        {
            x = position_cam[0];
            y = position_cam[1];
            // x = positionaf[0];
            // y = positionaf[1];
            z = vlocal_pose.pose.position.z;
            /* Convert float round 2 */
            x = PRECISION(x);
            y = PRECISION(y);
            z = PRECISION(z);
        }

        //     x_ = positionaf_change[0]+vlocal_pose.pose.position.x;
        //     y_ = positionaf_change[1]+vlocal_pose.pose.position.y;
        //     z_ = positionaf_change[2]+vlocal_pose.pose.position.z;
        //     x_ = PRECISION(x_);
        //     y_ = PRECISION(y_);
        //     z_ = PRECISION(z_);
        // }

                // if (30 >= abs(alpha) && vlocal_pose.pose.position.z > (HeightChangeAngle + 1))
                // // if (vlocal_pose.pose.position.z > (HeightChangeAngle + 1)) {
                //     // cout <<"----------------------------" << endl;
                //     // cout << positionaf_change[0]<< "--" << positionaf_change[1] << "--"<< positionaf_change[2] << endl;
                //     // cout << x_<< "--"<< y_<< "--"<< z_<< endl;
                //     // cout <<"----------------------------" << endl;
                //     pose.pose.position.x = x;
                //     pose.pose.position.y = y;
                //     pose.pose.position.z = HeightChangeAngle;
                // }
                // else if (10 >= abs(alpha) && vlocal_pose.pose.position.z <= (HeightChangeAngle + 1))
                // // else if (vlocal_pose.pose.position.z <= (HeightChangeAngle + 1)) {
                //     pose.pose.position.x = x;
                //     pose.pose.position.y = y;
                //     if (vlocal_pose.pose.position.z >= PRECISION(z) + 0.9) {
                //         if (pose.pose.position.z <= PRECISION(z) + 0.5) {
                //             pose.pose.position.z = PRECISION(z) + 0.5;
                //         }
                //         else {
                //             pose.pose.position.z = vlocal_pose.pose.position.z - 2.0;
                //             // pose.pose.position.z = abs(z);
                //         }
                //     }
                //     else {
                //         vLand = true;
                //     }
                // }
                // else {
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = vlocal_pose.pose.position.z;
            detect = true;
            ROS_INFO("Aligning........!");
            LOCK = 0;
        // }
        number_check ++;
        if(number_check == 3) {
            LOCK = 1;
            number_check = 0;
        }

        // cout<<"Marker2Cam  : " << PRECISION(position_cam[0]) <<'\t'<< PRECISION(position_cam[1]) << '\t' << PRECISION(position_cam[2]) << endl;
        cout<<"Marker2Drone: " << PRECISION(positionaf[0]) <<'\t'<< PRECISION(positionaf[1]) << '\t' << PRECISION(positionaf[2]) << endl;
        // cout<<"Marker2NEU  : " << x <<'\t'<< y << '\t' << z << endl;
        // cout<<"Drone      : " << PRECISION(vlocal_pose.pose.position.x) << "\t" << PRECISION(vlocal_pose.pose.position.y) << "\t" << PRECISION(vlocal_pose.pose.position.z) << endl;
        // cout << "===================================================" << endl;
    // }
}

void sig_handler( int sig ) {
    cout << "\nInterrupt signal (" << sig << ") received.\n";
    std_msgs::String msgs;
    std::stringstream ss;
    ss << "LAND";
    msgs.data = ss.str();
    cout << "Give mode AUTO.LAND" << endl;
    custom_activity_pub.publish(msgs);

    exit(sig);
}

void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
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

void landing_start() {
    LOCK_LAND = true;
    vLand     = false;
    vend      = true;
    baygio    = time(0);
    ltime     = localtime(&baygio);
    turn_off_motors();
    ROS_INFO("AUTO LANDING MODE is request");
    cout << "\n\x1B[36m========================================\033[0m"<< endl;
    cout << "\x1B[34m|-----------Time AUTO LANDING-----------\033[0m"<< endl;
    cout << "\x1B[36m========================================\033[0m"<< endl;
    cout << "| Start :" << minutes <<" : "<< seconds << endl;
    cout << "| END   :" << ltime->tm_min <<" : "<< ltime->tm_sec << endl;
    cout << "| Total :" << ltime->tm_min - minutes <<" minute "<< ltime->tm_sec - seconds << " Second" << endl;
    cout << "========================================"<< endl;
    cout<<"Drone : x = " << vlocal_pose.pose.position.x << " y = " << vlocal_pose.pose.position.y << " z = " << vlocal_pose.pose.position.z << endl;
    exit(0);
}

int main(int argc, char **argv) {
    signal(SIGTSTP, sig_handler);
    int sizeof_queue     = 10;

    cout << "\x1B[93mYear\033[0m  : "<< 1900 + ltime->tm_year << endl;
    cout << "\x1B[93mMonth\033[0m : "<< 1 + ltime->tm_mon<< endl;
    cout << "\x1B[93mDay\033[0m   : "<< ltime->tm_mday << endl;
    cout << "\x1B[93mTime\033[0m  : "<< ltime->tm_hour << ":";
    cout << ltime->tm_min << ":";
    cout << ltime->tm_sec << endl;

    ros::init(argc, argv, "subpose_node");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe
                            ("/mavros/imu/data",10,imuCallback);
    ros::Subscriber gps_sub = n.subscribe
                            ("/mavros/local_position/pose",10,mavrosPoseCallback);
    // ros::Subscriber pose_sub = n.subscribe
    //                         ("/tf_list", 10, get_params_cb);
    ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local", 10, local_pose_callback);
    ros::Publisher local_pos_pub1 = n.advertise<geometry_msgs::PoseStamped>
                            ("cmd/set_pose/position1", 10);
    custom_activity_pub = n.advertise<std_msgs::String>
                            ("cmd/set_activity/type",10);
    ros::Subscriber position_target_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>
                            ("kf/estimate",10, set_target_position_callback);
                        
    ros::Rate rate(10.0);

    // for(int i = 20; ros::ok() && i > 0; --i)
    // {
    //     LOCK = 1;
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // vbegin = 1;

    // begin_request = ros::Time::now();
    while(ros::ok()) {
        // if (vLand == true) {
        //     landing_start();
        // }
        // if (vend == false) {
            local_pos_pub1.publish(pose);
        // }
        // if( ros::Time::now() - begin_request > ros::Duration(2.0) ) {
        //     if (active_2s_ago == true) {
        //         detectfailedrepeat ++;
        //         active_2s_ago = false;
        //     }
        //     reset_request = ros::Time::now();
        //     cout << "Can't detect Marker" << endl;
        //     pose.pose.position.x = vlocal_pose.pose.position.x;
        //     pose.pose.position.y = vlocal_pose.pose.position.y;
        //     pose.pose.position.z = vlocal_pose.pose.position.z + IncreaseHeightNotDetect;
        //     if( (ros::Time::now() - begin_request > ros::Duration(6.0)) || detectfailedrepeat == MaxRepeatDetections ) {
        //         cout << "\x1B[31mTime Out or Maximum number of detection failed\033[0m\t" << detectfailedrepeat <<endl;
        //         landing_start();
        //     }
        //     local_pos_pub1.publish(pose);
        // }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
