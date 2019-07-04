#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <fstream>

using namespace  std;

std::ofstream fd;

//Call Back Function of motion captrure system
void callback(const nav_msgs::OdometryConstPtr& msg)
{
    fd << setprecision(6)
         << msg->header.stamp << " "
         << setprecision(9)
         << msg->pose.pose.position.x << " "
         << msg->pose.pose.position.y << " "
         << msg->pose.pose.position.z << " "
         << msg->pose.pose.orientation.w << " "
         << msg->pose.pose.orientation.x << " "
         << msg->pose.pose.orientation.y << " "
         << msg->pose.pose.orientation.z << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic2file");
    ros::NodeHandle n("~");
    std::string filepath;

    n.getParam("filepath", filepath);
    fd.open(filepath.c_str());

    ros::Subscriber sub = n.subscribe("odom", 10, callback);
    ros::spin();

    return 0;
}
