#include <ros/ros.h>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <iomanip>
#include <iostream>

using namespace std;
long seq = 0;
ros::Time last_update;

geometry_msgs::PoseStamped imu_pose;
static ros::Time t_pose;

void imuposeCallback(const sensor_msgs::ImuConstPtr& msg){
  imu_pose.pose.orientation.x = msg->orientation.x;
  imu_pose.pose.orientation.y = msg->orientation.y;
  imu_pose.pose.orientation.z = msg->orientation.z;
  imu_pose.pose.orientation.w = msg->orientation.w;
  t_pose = msg->header.stamp;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "vio_broadcaster");
  ros::NodeHandle node;

  ros::Rate loop_rate(100);
  ros::Subscriber sub = node.subscribe("/mavros/imu/data", 10, &imuposeCallback);

  tf2_ros::TransformBroadcaster br_world_sensors_link;
  imu_pose.pose.orientation.x = 0;
  imu_pose.pose.orientation.y = 0;
  imu_pose.pose.orientation.z = 0;
  imu_pose.pose.orientation.w = 1;

  while (ros::ok())
  {
    geometry_msgs::TransformStamped transformStamped_world_sensors_link;
    transformStamped_world_sensors_link.header.stamp = ros::Time::now();
    transformStamped_world_sensors_link.header.frame_id = "world";
    transformStamped_world_sensors_link.child_frame_id = "imu_link";
    transformStamped_world_sensors_link.transform.translation.x = 0.0;
    transformStamped_world_sensors_link.transform.translation.y = 0.0;
    transformStamped_world_sensors_link.transform.translation.z = 1.0;

    transformStamped_world_sensors_link.transform.rotation.x = imu_pose.pose.orientation.x;
    transformStamped_world_sensors_link.transform.rotation.y = imu_pose.pose.orientation.y;
    transformStamped_world_sensors_link.transform.rotation.z = imu_pose.pose.orientation.z;
    transformStamped_world_sensors_link.transform.rotation.w = imu_pose.pose.orientation.w;
    br_world_sensors_link.sendTransform(transformStamped_world_sensors_link);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
