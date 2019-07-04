#ifndef NODE_VIO_H
#define NODE_VIO_H


#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//vio
#include "icp_estimator.h"
#include "eskf_model.h"
#include <deque>
#include <mutex>

// define a class, including a constructor, member variables and member functions
class NODE_VIO
{
public:
    NODE_VIO(ros::NodeHandle* nodehandle);

private:
    //ROS_IO
    ros::NodeHandle nh_;

    ros::Subscriber imu_sub;
    //ros::Subscriber pc_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> origin_pc_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> salientpts_pc_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Publisher  vio_ros_publisher_pose;
    ros::Publisher  vio_ros_publisher_path;
    ros::Publisher  vio_ros_publisher_pc;
    ros::Publisher  vio_ros_publisher_cus_pose;

    void initializeSubscribers();
    void initializePublishers();

//    std::deque<IMU_DATA> imu_queue; //save 300 set
//    std::mutex mutex_imu_q;

    void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_clounin_msg,
                     const sensor_msgs::PointCloud2ConstPtr& pc_sailent_msg);

    //VIO
    ICP_EST* icp;    //icp(Frame to KeyFrame) provide the position and rotation
    //ICP_EST* icp_f2f;//icp(Frame to Frame) provide linear velocity and angular velocity
    ESKF_Fusion* eskf;

    //Start Pose in the map
    
    //IMU
    IMU_DATA curr_imu;

    //Vision
    int frameIdx;//current frame
    double dT;//time between 2 frame in second

    CloudTPtr newFrame;
    CloudTPtr salientPtsFromNewFrame;
    Affine3d tf_key;
    double keyFrame_tstamp;

    Affine3d tf_curr;
    double currFrame_tstamp;

    int keyFrameIdx;
    CloudTPtr keyFrame;
    Affine3d tf_last;


    //Sensor State Value
    unsigned char camera_working;
    unsigned char imu_working;
    void start_the_filter(void);


    void init_value();
    void pose_publish(Vector3d trans, Quaterniond rot_q);
    void pc_publish(CloudTPtr pcptr);
    void cus_pose_publish(Vector3d trans, Vector3d euler);


};


#endif // NODE_VIO_H
