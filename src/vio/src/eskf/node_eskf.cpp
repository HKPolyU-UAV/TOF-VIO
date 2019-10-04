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
#include <include/eskf_imu.h>



using namespace std;
using namespace Eigen;

#define PI (3.14159265358)


extern ESKF_IMU *eskf_imu;
extern ros::Publisher odom_pub;
extern bool initialized;

ESKF_IMU *eskf_imu;
ros::Publisher odom_pub;
bool initialized=false;




void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  eskf_imu->read_imu_msg(msg->header.stamp.toSec(),
                         msg->linear_acceleration.x,
                         msg->linear_acceleration.y,
                         msg->linear_acceleration.z,
                         msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
  if(!initialized)
  {
    //use the first vo input at the init value
    eskf_imu->states.push_back(eskf_imu->curr_state);
    if(eskf_imu->states.size()>=200)
    {
      eskf_imu->states.pop_front();
    }
  }
  else {//initialzed
    eskf_imu->update_Nominal_Error_Cov();
    nav_msgs::Odometry eskf_odom;
    eskf_odom.header.stamp = msg->header.stamp;
    eskf_odom.header.frame_id = "world";
    SYS_STATE xt=eskf_imu->states.back();
    eskf_odom.pose.pose.position.x = xt.n_state[4];
    eskf_odom.pose.pose.position.y = xt.n_state[5];
    eskf_odom.pose.pose.position.z = xt.n_state[6];
    eskf_odom.twist.twist.linear.x = xt.n_state[7];
    eskf_odom.twist.twist.linear.y = xt.n_state[8];
    eskf_odom.twist.twist.linear.z = xt.n_state[9];
    Quaterniond odom_q;
    eskf_odom.pose.pose.orientation.w = xt.n_state[0];
    eskf_odom.pose.pose.orientation.x = xt.n_state[1];
    eskf_odom.pose.pose.orientation.y = xt.n_state[2];
    eskf_odom.pose.pose.orientation.z = xt.n_state[3];
    odom_pub.publish(eskf_odom);
    //cout << "imu update process" << endl;
  }
}


void odom_callback_vo(const nav_msgs::Odometry::ConstPtr &msg)
{
  cout << endl << "in vo callback:" << endl;
  if (msg->pose.pose.position.x == 0.012345)
  {
    return;
  }
  else
  {
    if(!initialized)
    {
      //use the first vo input at the init value
      eskf_imu->init_from_vo(msg->header.stamp.toSec(),
                             msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z,
                             msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
      initialized = true;
    }
    else {//initialzed
      static int count = 0;
      eskf_imu->read_vo_msg(msg->header.stamp.toSec(),
                            msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
      eskf_imu->innovate_ErrorState();
      eskf_imu->innovate_Inject_Reset();
      eskf_imu->innovate_reintegrate();
      nav_msgs::Odometry eskf_odom;
      eskf_odom.header.stamp = msg->header.stamp;
      eskf_odom.header.frame_id = "world";
      SYS_STATE xt=eskf_imu->states.back();
      eskf_odom.pose.pose.position.x = xt.n_state[4];
      eskf_odom.pose.pose.position.y = xt.n_state[5];
      eskf_odom.pose.pose.position.z = xt.n_state[6];
      eskf_odom.twist.twist.linear.x = xt.n_state[7];
      eskf_odom.twist.twist.linear.y = xt.n_state[8];
      eskf_odom.twist.twist.linear.z = xt.n_state[9];
      Quaterniond odom_q;
      eskf_odom.pose.pose.orientation.w = xt.n_state[0];
      eskf_odom.pose.pose.orientation.x = xt.n_state[1];
      eskf_odom.pose.pose.orientation.y = xt.n_state[2];
      eskf_odom.pose.pose.orientation.z = xt.n_state[3];
      odom_pub.publish(eskf_odom);
      count++;
    }
  }

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "eskf_node");
  ros::NodeHandle n("~");


  odom_pub = n.advertise<nav_msgs::Odometry>("eskf_odom", 10);

  // You should also tune these parameters
  double ng, na, nbg, nba, n_vo_p, n_vo_q, vo_delay;
  // Q imu covariance matrix;
  n.getParam("eskf/ng", ng);
  n.getParam("eskf/na", na);
  n.getParam("eskf/nbg", nbg);
  n.getParam("eskf/nba", nba);
  /* pnp position and orientation noise */
  n.getParam("eskf/vo_p", n_vo_p);
  n.getParam("eskf/vo_q", n_vo_q);
  n.getParam("eskf/vo_delay", vo_delay);
  /* optical flow noise */

  cout << "ng     :" << ng << endl;
  cout << "na     :" << na << endl;
  cout << "nbg    :" << nbg << endl;
  cout << "nba    :" << nba << endl;
  cout << "n_vo_p :"  << n_vo_p << endl;
  cout << "n_vo_q :"  << n_vo_q << endl;

  eskf_imu = new ESKF_IMU(na,
                          ng,
                          nba,
                          nbg,
                          n_vo_q,
                          n_vo_q,
                          vo_delay);

  ros::Subscriber s1 = n.subscribe("imu", 30, imu_callback);
  ros::Subscriber s2 = n.subscribe("vo", 1, odom_callback_vo);


  ros::spin();
}
