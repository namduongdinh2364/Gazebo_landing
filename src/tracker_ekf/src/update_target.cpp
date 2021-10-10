#include "ros/ros.h"
#include <math.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <eigen3/Eigen/Geometry>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <csignal>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define PRECISION(x)    round(x * 100) / 100
#define PI              3.14159265
#define IncreaseHeightNotDetect     1
#define MaxRepeatDetections         2

using namespace std;
using namespace Eigen;

ros::Publisher custom_activity_pub;

int detectfailedrepeat = 0;
bool active_2s_ago = true;
static int number_check          = 0;
static bool LOCK_LAND            = false;
static bool vLand                = false;
static bool vend                 = false;
static int LOCK                  = 10;
time_t baygio                    = time(0);
tm *ltime                        = localtime(&baygio);
static double x, y, z;
Vector3f position_cam, positionaf;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped vlocal_pose;
int vbegin    = 2;
float minutes = 0;
float seconds = 0;
ros::Time begin_request, reset_request;
static bool detect = false;

/**
 * @brief 
 * 
 * @param 
 *
 * @return 
 */
void turn_off_motors(void) {
    std_msgs::String msg;
    std::stringstream ss;

    ss << "LAND";
    msg.data = ss.str();
    custom_activity_pub.publish(msg);
}

/*storing gps data in pointer*/
void mavrosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    vlocal_pose=*msg;
}

/**
 * @brief 
 * 
 * @param 
 *
 * @return 
 */
void set_target_position_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    begin_request = ros::Time::now();
    active_2s_ago = true;
    if( ros::Time::now() - reset_request > ros::Duration(3.0) ) {
        detectfailedrepeat = 0;
    }
    float radius;
    if (LOCK_LAND == false) {
        if (vbegin == 1) {
            baygio = time(0);
            ltime = localtime(&baygio);
            cout << "Start:" << ltime->tm_min << ":";
            cout << ltime->tm_sec << endl;
            minutes = ltime->tm_min;
            seconds = ltime->tm_sec;
            vbegin = 0;
        }

        double xq,yq,zq,wq;
        position_cam[0] = msg->pose.pose.position.x;
        position_cam[1] = msg->pose.pose.position.y;
        position_cam[2] = msg->pose.pose.position.z;

        xq = msg->pose.pose.orientation.x;
        yq = msg->pose.pose.orientation.y;
        zq = msg->pose.pose.orientation.z;
        wq = msg->pose.pose.orientation.w;

        positionaf = position_cam;
        if (LOCK > 0) {
            x = positionaf[0];
            y = positionaf[1];
            z = positionaf[2];
            /* Convert float round 2 */
            x = PRECISION(x);
            y = PRECISION(y);
            z = PRECISION(z);
        }
        /** used for control with pose */
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        if (vlocal_pose.pose.position.z >= PRECISION(z) + 0.9) {
            if (pose.pose.position.z <= PRECISION(z) + 0.5) {
                pose.pose.position.z = PRECISION(z) + 0.5;
            } else {
                pose.pose.position.z = vlocal_pose.pose.position.z - 2.0;
            }
        } else {
            vLand = true;
        }

        LOCK = 0;
        pose.pose.orientation.x = xq;
        pose.pose.orientation.y = yq;
        pose.pose.orientation.z = zq;
        pose.pose.orientation.w = wq;

        detect = true;
        /**
         * stable param
        */
        number_check ++;
        if(number_check == 10) {
            LOCK = 1;
            number_check = 0;
        }

        cout<< "Marker2Drone : " << PRECISION(position_cam[0]) <<'\t'
                                << PRECISION(position_cam[1]) << '\t'
                                << PRECISION(position_cam[2]) << endl;
        cout<< "Marker2NEU : " << x <<'\t'<< y << '\t' << z << endl;
        cout<< "Drone : " << PRECISION(vlocal_pose.pose.position.x) << "\t"
                                << PRECISION(vlocal_pose.pose.position.y) << "\t"
                                << PRECISION(vlocal_pose.pose.position.z) << endl;
        cout << "===================================================" << endl;
    }
}

/**
 * @brief 
 * 
 * @param 
 *
 * @return 
 */
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
    } else {
        pose.pose.position.x= x;
        pose.pose.position.y= y;
        pose.pose.position.z= z;
    }
}

/**
 * @brief 
 * 
 * @param 
 *
 * @return 
 */
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

    exit(-1);
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

    ros::init(argc, argv, "update_target_node");
    ros::NodeHandle n;

    ros::Subscriber gps_sub = n.subscribe
                            ("/mavros/local_position/pose",10,mavrosPoseCallback);
    ros::Subscriber position_target_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>
                            ("kf/estimate",10, set_target_position_callback);
    ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local", 10, local_pose_callback);
    ros::Publisher local_pos_pub1 = n.advertise<geometry_msgs::PoseStamped>
                            ("cmd/set_pose/position1", 10);
    custom_activity_pub = n.advertise<std_msgs::String>
                            ("cmd/set_activity/type",10);
                        
    ros::Rate rate(20.0);

    for(int i = 10; ros::ok() && i > 0; --i) {
        LOCK = 1;
        ros::spinOnce();
        rate.sleep();
    }
    vbegin = 1;

    begin_request = ros::Time::now();
    while(ros::ok()) {
        if (vLand == true) {
            landing_start();
        }
        if (vend == false) {
            local_pos_pub1.publish(pose);
        }
        /**
         * If can't detect marker during 2 second.
         * if number of detect failed more than maxium, 
         * Landing will be called.
         */
        if( ros::Time::now() - begin_request > ros::Duration(2.0) ) {
            if (active_2s_ago == true) {
                detectfailedrepeat ++;
                active_2s_ago = false;
            }
            reset_request = ros::Time::now();
            cout << "Can't detect Marker" << endl;
            pose.pose.position.x = vlocal_pose.pose.position.x;
            pose.pose.position.y = vlocal_pose.pose.position.y;
            pose.pose.position.z = vlocal_pose.pose.position.z + IncreaseHeightNotDetect;
            if( (ros::Time::now() - begin_request > ros::Duration(6.0)) || detectfailedrepeat == MaxRepeatDetections ) {
                cout << "\x1B[31mTime Out or Maximum number of detection failed\033[0m\t" << detectfailedrepeat <<endl;
                landing_start();
            }
            local_pos_pub1.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

