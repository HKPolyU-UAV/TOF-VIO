#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <stdio.h>
#include <math.h>
#include <deque>
#include "visualization_msgs/Marker.h"
#include <std_msgs/Float64MultiArray.h>

#include <include/euler_q_rmatrix.h>
#include <include/eskf_model.h>

ros::Publisher odom_pub;

using namespace std;
using namespace Eigen;
const double PI = 3.14159265358;

ros::Time last_frame_time;
ros::Time last_imu_time;


Mat12x12 Q_imu;
Mat6x6   R_vo;
deque<MatrixXd> state_q;
deque<MatrixXd> covariance_q;
deque<sensor_msgs::Imu> imu_msg_q;
int vio_state_initialized = 0;
int got_first_imu = 0;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!vio_state_initialized){// if initialized start computing
        last_imu_time = msg->header.stamp;
        return;
    }
    if (!got_first_imu){
        cout << "inited and got imu data" << endl << "at time :" << msg->header.stamp.toSec() << endl;
        imu_msg_q.push_back(*msg);
        last_imu_time = msg->header.stamp;
        got_first_imu = 1;
        return;
    }


}


void odom_callback_vo(const nav_msgs::Odometry::ConstPtr &msg)
{
    ros::Time t1, t2;
    t1 = ros::Time::now();
    if (msg->pose.pose.position.x == 0.012345)
    {
        return;
    }
    Vec6 z_vo;
    if ((!vio_state_initialized))
    {//initialized the state

    }


    //    cout << "Cost: " << (t2 - t1).toSec() * 1000 << "ms" << endl;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 20, imu_callback);
    ros::Subscriber s2 = n.subscribe("vo", 1, odom_callback_vo);

    odom_pub = n.advertise<nav_msgs::Odometry>("eskf_odom", 10);

    // You should also tune these parameters
    double ng, na, nbg, nba, n_vo_p, n_vo_q;
    // Q imu covariance matrix;
    n.getParam("eskf/ng", ng);
    n.getParam("eskf/na", na);
    n.getParam("eskf/nbg", nbg);
    n.getParam("eskf/nba", nba);
    /* pnp position and orientation noise */
    n.getParam("eskf/vo_p", n_vo_p);
    n.getParam("eskf/vo_q", n_vo_q);
    /* optical flow noise */

    cout << "ng     :" << ng << endl;
    cout << "na     :" << na << endl;
    cout << "nbg    :" << nbg << endl;
    cout << "nba    :" << nba << endl;
    cout << "n_vo_p :"  << n_vo_p << endl;
    cout << "n_vo_q :"  << n_vo_q << endl;


    Q_imu.setIdentity();
    R_vo.setIdentity();

    Q_imu.block<3, 3>(0, 0) = ng * Q_imu.block<3, 3>(0, 0);
    Q_imu.block<3, 3>(3, 3) = na * Q_imu.block<3, 3>(0, 0);
    Q_imu.block<3, 3>(6, 6) = nbg * Q_imu.block<3, 3>(6, 6);
    Q_imu.block<3, 3>(9, 9) = nba * Q_imu.block<3, 3>(9, 9);

    //Rt pnp
    R_vo.topLeftCorner(3, 3) = n_vo_p * R_vo.topLeftCorner(3, 3);         //pox_xyz
    R_vo.bottomRightCorner(3, 3) = n_vo_q * R_vo.bottomRightCorner(3, 3); //pitch roll yaw

    cout << "Qt_imu:" << endl
         << Q_imu << endl;
    cout << "R_vo:" << endl
         << R_vo << endl;

    ros::spin();
}
