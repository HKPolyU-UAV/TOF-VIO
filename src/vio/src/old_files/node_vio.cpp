#include <iomanip>
#include <iostream>

//ROS
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


//EIGEN
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
//vio
#include "common.h"
#include "utils/euler_q_rmatrix.h"
#include "utils/tic_toc_ros.h"
#include "node_vio.h"

#include <mutex>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace pcl;



void NODE_VIO::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    static double last_imu_time;
    if(!imu_working)
    {
        imu_working=1;//set system state
        last_imu_time = imu_msg->header.stamp.toSec();
        return;
    }
    if(imu_working)
    {
//        IMU_DATA imu_data;
//        imu_data.tstamp = imu_msg->header.stamp.toSec();
//        imu_data.dT =  imu_data.tstamp-last_imu_time;
//        imu_data.q.x() = imu_msg->orientation.x;
//        imu_data.q.y() = imu_msg->orientation.y;
//        imu_data.q.z() = imu_msg->orientation.z;
//        imu_data.q.w() = imu_msg->orientation.w;
//        imu_data.acc_bf[0] = imu_msg->linear_acceleration.x;
//        imu_data.acc_bf[1] = imu_msg->linear_acceleration.y;
//        imu_data.acc_bf[2] = imu_msg->linear_acceleration.z;
//        imu_data.gyro_bf[0] = imu_msg->angular_velocity.x;
//        imu_data.gyro_bf[1] = imu_msg->angular_velocity.y;
//        imu_data.gyro_bf[2] = imu_msg->angular_velocity.z;
//        mutex_imu_q.lock();
//        imu_queue.push_back(imu_data);
//        if(imu_queue.size()>200)
//        {
//            imu_queue.pop_front();
//        }
//        mutex_imu_q.unlock();
        //VIO->imu_feed(imu_data);
    }
}

void NODE_VIO::pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_clounin_msg,
                           const sensor_msgs::PointCloud2ConstPtr& pc_sailent_msg)
{
    tic_toc_ros tt;
    //read in the cloud from the message
    CloudTPtr full_cloud(new CloudT);
    CloudTPtr sailent_cloud(new CloudT);
    pcl::PCLPointCloud2 tmp1,tmp2;
    pcl_conversions::toPCL(*pc_clounin_msg, tmp1);
    pcl_conversions::toPCL(*pc_sailent_msg, tmp2);
    pcl::fromPCLPointCloud2(tmp1, *full_cloud);
    pcl::fromPCLPointCloud2(tmp2, *sailent_cloud);
    ros::Time cloud_time = pc_clounin_msg->header.stamp;
    static ros::Time last_cloud_time;
    if(!camera_working)
    {
        camera_working=1;//set system state
        last_cloud_time = pc_clounin_msg->header.stamp;
        cout << "got First Cloud" << endl;
        icp->setSource(full_cloud);
        start_the_filter();//trigger the filter
        frameIdx = 1;
        return;
    }else
    {
        //newFrame = cloud_in;
        //EKF Estimator
        //Time synchronizer
        //Update pridiction

        Affine3d diff_trans;
        Affine3f tmp;
        icp->setTarget(full_cloud);
        auto dis = icp->integrate();//icp process
        tmp = icp->getTansfromation();
        diff_trans = tmp.cast<double>();

        tf_curr = tf_curr*diff_trans.inverse();


        if(1)//update the keyframe
        {
            cout << "xxxxx" << endl;
            keyFrame = full_cloud;
            keyFrameIdx = frameIdx;
            icp->setSource(keyFrame);
        }
    }
    frameIdx++;

    pc_publish(icp->get_sailent_src());
    Matrix3d mat_tmp = tf_curr.rotation();
    Eigen::Quaterniond q_tmp(mat_tmp);
    pose_publish(tf_curr.translation(),q_tmp);
    cout << "Time Consumption:  " << tt.dT_ms() << "ms"<< endl;
}

void NODE_VIO::start_the_filter()
{
    //The init position will be set to x=0.0 y=0.0 z=0.5
    //The init pose will be set to the imu->q
    if(imu_working&&camera_working)
    {
        Vector3d init_pos(0.0,0.0,0.5);
        Vector3d init_euler(0.0,0.0,0.0);
        //
//        eskf->init(
//                   Quaterniond(1.0,0,0,0));
        //frame variable
    }
}


//int NODE_VIO::update_key_criteria()
//{
//    if((frameCount-keyFrameCount)>=100)//every 15 frame
//        return 1;
//    //    trans > 0.5m
//    //    diff_euler > 20 degree
//    Vector3f d_t=tf_curr-tf_key;
//    if((abs(d_t(0))+abs(d_t(1))+abs(d_t(2)))  > 0.3)
//        return 1;
//    Quaternionf d_q=rot_q_curr*rot_q_key.inverse();
//    Vector3f d_euler = euler_from_quaternion(d_q);
//    if((abs(d_euler(0))+abs(d_euler(1))+abs(d_euler(2)))  > 0.2)
//        return 1;

//    return 0;
//}

//std::deque<IMU_DATA> NODE_VIO::imu_cam_time_sync(double tprev, double tcurr)
//{
//    std::deque<IMU_DATA> ret;
//    ret.clear();
//    mutex_imu_q.lock();
//    for(int cnt;1;cnt++)
//    {
//        IMU_DATA result = imu_queue.front();
//        {
//            if(result.tstamp<tprev)
//            {
//                imu_queue.pop_front();
//                continue;
//            }
//            if(result.tstamp<=tcurr)
//            {
//                ret.push_back(result);
//                imu_queue.pop_front();
//                continue;
//            }
//            if(result.tstamp>tcurr)
//            {
//                break;
//            }
//            if(cnt>=200)
//            {
//                break;
//            }
//        }
//    }
//    mutex_imu_q.unlock();
//    return ret;
//}




void NODE_VIO::pose_publish(Vector3d trans, Quaterniond rot_q)
{
    geometry_msgs::PoseStamped pose_stamped;
    ros::Time now = ros::Time::now();

    pose_stamped.header.stamp = now;
    pose_stamped.header.frame_id="world";
    pose_stamped.pose.orientation.w = rot_q.w();
    pose_stamped.pose.orientation.x = rot_q.x();
    pose_stamped.pose.orientation.y = rot_q.y();
    pose_stamped.pose.orientation.z = rot_q.z();
    pose_stamped.pose.position.x = trans(0);
    pose_stamped.pose.position.y = trans(1);
    pose_stamped.pose.position.z = trans(2);
    vio_ros_publisher_pose.publish(pose_stamped);

    static nav_msgs::Path path;
    path.header.stamp=now;
    path.header.frame_id="world";
    path.poses.push_back(pose_stamped);
    if(path.poses.size()>= 550)
    {
        path.poses.erase(path.poses.begin());
    }
    vio_ros_publisher_path.publish(path);
}

void NODE_VIO::pc_publish(CloudTPtr pcptr)
{
    sensor_msgs::PointCloud2 output;
    ros::Time now = ros::Time::now();

    //cout << pcptr->size() << endl;
    toROSMsg(*pcptr,output);
    output.header.frame_id="opti_link";
    output.header.stamp = now;
    vio_ros_publisher_pc.publish(output);
}



NODE_VIO::NODE_VIO(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of NODE_VIO");

    init_value();
    initializePublishers();
    initializeSubscribers();
}

void NODE_VIO::init_value()
{
    icp = new ICP_EST;
    eskf = new ESKF_Fusion;

    tf_curr.setIdentity();
    tf_curr.translation() = Vector3d(0,0,0.5);
    tf_key = tf_curr;
    tf_last = tf_curr;

    frameIdx = 0;
    keyFrame.reset(new CloudT);
    newFrame.reset(new CloudT);
    salientPtsFromNewFrame.reset(new CloudT);

    camera_working = 0;
    imu_working    = 0;
}

void NODE_VIO::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    imu_sub = nh_.subscribe("/mavros/imu/data", 100, &NODE_VIO::imu_callback,this);
    //pc_sub = nh_.subscribe("/vo_input_cloud", 10, &NODE_VIO::pc_callback,this);

    origin_pc_sub.subscribe(nh_,     "/vo_input_cloud", 2);
    salientpts_pc_sub.subscribe(nh_, "/vo_input_sailent_cloud", 2);
    sync.reset(new Sync(MySyncPolicy(2), origin_pc_sub, salientpts_pc_sub));
    sync->registerCallback(boost::bind(&NODE_VIO::pc_callback, this, _1, _2));
}

//member helper function to set up publishers;
void NODE_VIO::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    vio_ros_publisher_pose = nh_.advertise<geometry_msgs::PoseStamped> ("/vio_pose",10);
    vio_ros_publisher_path = nh_.advertise<nav_msgs::Path> ("/vio_path",10);
    vio_ros_publisher_pc   = nh_.advertise<sensor_msgs::PointCloud2> ("/vio_pts",10);
}
