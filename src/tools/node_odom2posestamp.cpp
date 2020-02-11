#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <fstream>

using namespace  std;

geometry_msgs::PoseStamped latest_pose;
bool first_msg=1;
ros::Time lastMsgTime;
double time_gap=0.1;
string odom_topic_in;
string pose_topic_out;
int frequency=10;

ros::Publisher pose_pub;

//Call Back Function of motion captrure system
void callback(const nav_msgs::OdometryConstPtr& msg)
{
    if(first_msg)
    {
        lastMsgTime = msg->header.stamp;
        latest_pose.header.frame_id = "world";
        latest_pose.header.stamp=msg->header.stamp;
        latest_pose.pose.orientation.w = msg->pose.pose.orientation.w;
        latest_pose.pose.orientation.x = msg->pose.pose.orientation.x;
        latest_pose.pose.orientation.y = msg->pose.pose.orientation.y;
        latest_pose.pose.orientation.z = msg->pose.pose.orientation.z;
        latest_pose.pose.position.x = msg->pose.pose.position.x;
        latest_pose.pose.position.y = msg->pose.pose.position.y;
        latest_pose.pose.position.z = msg->pose.pose.position.z;
        pose_pub.publish(latest_pose);
        first_msg = false;
    }
    else
    {
        if((msg->header.stamp.toSec()-lastMsgTime.toSec())>time_gap)
        {
            lastMsgTime = msg->header.stamp;
            latest_pose.header.frame_id = "world";
            latest_pose.header.stamp=msg->header.stamp;
            latest_pose.pose.orientation.w = msg->pose.pose.orientation.w;
            latest_pose.pose.orientation.x = msg->pose.pose.orientation.x;
            latest_pose.pose.orientation.y = msg->pose.pose.orientation.y;
            latest_pose.pose.orientation.z = msg->pose.pose.orientation.z;
            latest_pose.pose.position.x = msg->pose.pose.position.x;
            latest_pose.pose.position.y = msg->pose.pose.position.y;
            latest_pose.pose.position.z = msg->pose.pose.position.z;
            pose_pub.publish(latest_pose);
        }
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom2posestamp");
    ros::NodeHandle n("~");


    n.getParam("odom_in", odom_topic_in);
    n.getParam("pose_out", pose_topic_out);

    ros::Subscriber sub = n.subscribe(odom_topic_in, 2, callback);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(pose_topic_out, 2);
    ros::spin();

    return 0;
}
