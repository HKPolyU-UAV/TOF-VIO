#include <ros/ros.h>
#include "node_vio.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "VIO"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type NODE_VIO");
    NODE_VIO node_vio(&nh);  //instantiate an NODE_VIO object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin");

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
