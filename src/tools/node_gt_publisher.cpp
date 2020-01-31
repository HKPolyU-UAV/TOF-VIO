#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
extern ros::Publisher odom_pub;

ros::Publisher odom_pub;

//Call Back Function of motion captrure system
void MC_callback(const geometry_msgs::PoseStampedConstPtr & msg)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "world";
    odom.pose.pose.orientation.w = msg->pose.orientation.w;
    odom.pose.pose.orientation.x = msg->pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.orientation.z;
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.position.z = msg->pose.position.z;
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n("~");

    odom_pub = n.advertise<nav_msgs::Odometry>("odom_gt", 1000);

    ros::Subscriber sub = n.subscribe("vicon", 1000, MC_callback);


    ros::spin();

    return 0;
}
