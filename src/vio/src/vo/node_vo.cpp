//ROS
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigen>
//CV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
//PCL
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
//USER
#include <include/common.h>
#include <include/tic_toc_ros.h>
#include <include/euler_q_rmatrix.h>
#include <include/salientpts.h>
#include <include/icp.h>
#include <include/tof_frame.h>


using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace Eigen;

namespace nodelet_ns
{



class NICP : public nodelet::Nodelet
{
public:
  NICP()  {;}
  ~NICP() {;}

private:
  TOF_Frame::Ptr curr_frame,prev_frame,key_frame;
  //    Eigen::Affine3d T_cw_curr,  T_cw_prev,  T_cw_key;
  //    CloudTPtr       P_curr,     P_prev,     P_key;
  //    Mat             i_img_curr, i_img_prev, i_img_key;

  Eigen::Affine3d T_cw_init;
  Eigen::Affine3d T_ic,T_ci;//T_ic: camera to imu

  //Salient Pts extractor and ICP alignment
  SalientPts*    salient_pts_extractor;
  ICP_ALIGNMENT* icp_alignment;
  //Flag
  int init_by_IMU, init_by_MC;
  int icp_init;
  int receive_mc_data;
  int receive_imu_data;
  int FrameCount;
  int updateKeyFrame;

  //PUB
  ros::Publisher pub_cloudin;
  ros::Publisher pub_sailentpts;
  ros::Publisher pub_keypts;
  ros::Publisher pub_tf_array;
  ros::Publisher pub_odom;
  //SUB
  ros::Subscriber mc_sub;
  ros::Subscriber imu_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
  message_filters::Subscriber<sensor_msgs::Image> grey_sub;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MyExactSyncPolicy;
  message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

  virtual void onInit()
  {
    ros::NodeHandle& node = getPrivateNodeHandle();

    string cam_cal_file_path;
    cv::Mat cameraMatrix, distCoeffs;
    double fx,fy,cx,cy;
    int pc_height,pc_width;
    int target_sailentpts_num;
    int use_depth_grad,use_intensity_grad,use_edge_detector,use_canny;
    float sh_backfloor, sh_depth, sh_grey, sh_edge;

    node.getParam("icp/target_sailentpts_num",   target_sailentpts_num);
    node.getParam("icp/use_depth_grad",          use_depth_grad);
    node.getParam("icp/use_intensity_grad",      use_intensity_grad);
    node.getParam("icp/use_edge_detector",       use_edge_detector);
    node.getParam("icp/use_canny",               use_canny);
    node.getParam("icp/sh_backfloor",            sh_backfloor);
    node.getParam("icp/sh_depth",                sh_depth);
    node.getParam("icp/sh_grey",                 sh_grey);
    node.getParam("icp/sh_edge",                 sh_edge);
    node.getParam("icp/init_by_IMU",             init_by_IMU);
    node.getParam("icp/init_by_MC",              init_by_MC);
    node.getParam("cam_cal_file", cam_cal_file_path);

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs   = cv::Mat::zeros(4, 1, CV_64F);
    cout << "get all parameter done" << endl;
    cout  << cam_cal_file_path << endl;
    cv::FileStorage param_reader(cam_cal_file_path, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> cameraMatrix;
    param_reader["distortion_coefficients"] >> distCoeffs;
    param_reader["image_height"] >> pc_height;
    param_reader["image_width"] >> pc_width;
    cout << "Camera Matrix:" << endl <<cameraMatrix << endl;
    cout << "Distortion coefficient" << endl << distCoeffs << endl;
    cout << "pc_height  : " << pc_height << endl;
    cout << "pc_width: " << pc_width << endl;
    fx=cameraMatrix.at<double>(0,0);
    fy=cameraMatrix.at<double>(1,1);
    cx=cameraMatrix.at<double>(0,2);
    cy=cameraMatrix.at<double>(1,2);


    salient_pts_extractor = new SalientPts(pc_width,
                                           pc_height,
                                           target_sailentpts_num,
                                           use_canny,
                                           use_depth_grad,
                                           use_intensity_grad,
                                           use_edge_detector,
                                           sh_depth,
                                           sh_grey,
                                           sh_backfloor,
                                           sh_edge
                                           );
    icp_alignment = new ICP_ALIGNMENT(pc_width,
                                      pc_height,
                                      fx,
                                      fy,
                                      cx,
                                      cy,

                                      30);

    curr_frame = std::make_shared<TOF_Frame>();
    prev_frame = std::make_shared<TOF_Frame>();
    key_frame  = std::make_shared<TOF_Frame>();


    //Camera to IMU Conversion
    //  0  0  1  0.1
    // -1  0  0  0
    //  0 -1  0  0.01
    //  0  0  0  1
    T_ic.matrix() << 0,0,1,0.1,-1,0,0,0,0,-1,0,0.01,0,0,0,1;
    T_ci = T_ic.inverse();
    cout << "Transformation from camera frame to imu frame :" << endl << T_ic.matrix() << endl;
    //set flags
    icp_init = 0;
    receive_mc_data = 0;
    receive_imu_data = 0;
    FrameCount = 0;
    updateKeyFrame = 0;

    //Pub
    pub_cloudin      = node.advertise<sensor_msgs::PointCloud2>   ("icp_cloudin", 1);
    pub_sailentpts   = node.advertise<sensor_msgs::PointCloud2>   ("icp_sailent_pts", 1);
    pub_keypts       = node.advertise<sensor_msgs::PointCloud2>   ("icp_keypts", 1);
    pub_tf_array     = node.advertise<std_msgs::Float32MultiArray>("/arrayxyz", 1);
    pub_odom         = node.advertise<nav_msgs::Odometry>         ("icp_odom", 1);

    //Sub
    mc_sub  = node.subscribe("/vicon", 1, &NICP::mc_callback, this);
    imu_sub = node.subscribe("/mavros/imu/data", 1, &NICP::imu_callback, this);
    //Sync Sub
    pc_sub.subscribe(node,   "/points", 2);
    grey_sub.subscribe(node, "/image_nir", 2);
    exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(1), pc_sub, grey_sub);
    exactSync_->registerCallback(boost::bind(&NICP::callback, this, _1, _2));
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr & pcPtr,
                const sensor_msgs::ImageConstPtr & mono8Ptr)
  {
    tic_toc_ros tt_cb;
    curr_frame->read_PC_Iimg_FromROSMsg(pcPtr,mono8Ptr);
    salient_pts_extractor->select_salient_from_pc(curr_frame->sailent_cloud,curr_frame->cloud,curr_frame->i_img);

    if(!icp_init)//first time
    {
      cout << "init icp" << endl;
      if((receive_mc_data && init_by_MC)||(receive_imu_data && init_by_IMU))
      {
        mc_sub.shutdown();
        imu_sub.shutdown();
        curr_frame->T_cw = T_cw_init;
        TOF_Frame::copy(*curr_frame,*prev_frame);
        TOF_Frame::copy(*curr_frame,*key_frame);
        FrameCount = 1;
        icp_init = 1;
        publish_pose(T_cw_init,pcPtr->header.stamp);
        publish_tf(T_cw_init);
        if(receive_mc_data && init_by_MC){
          cout << "init with motion capture system data" << endl;
        }
        if(receive_imu_data && init_by_IMU){
          cout << "init with imu data" << endl;
        }
      }
      tt_cb.toc("inti icp pose finished");
      return;
    }
    else
    {
      cout << "For Frame:" << FrameCount << endl;
      publishPC(curr_frame->cloud,"pico_flexx_optical_frame",pub_cloudin);
      publishPC(key_frame->cloud, "pico_flexx_optical_frame",pub_keypts);
      publishPC(prev_frame->sailent_cloud,"pico_flexx_optical_frame",pub_sailentpts);
      Eigen::Affine3d T_key_curr_guess = key_frame->T_cw * prev_frame->T_cw.inverse();
      Eigen::Affine3d T_key_curr_est;
      double mean_error;
      //      cout << "keyframe pose " << endl << key_frame->T_cw.matrix() << endl;
      //      cout << "prevframe pose" << endl << prev_frame->T_cw.matrix() << endl;
      //      cout << "initial guess " << endl << T_key_curr_guess.matrix() << endl;

      icp_alignment->alignment(curr_frame->sailent_cloud,
                               key_frame->cloud,
                               T_key_curr_guess,
                               T_key_curr_est,
                               mean_error);
      FrameCount++;

      if(mean_error<0.1)
      {
        curr_frame->T_cw = T_key_curr_est.inverse() * key_frame->T_cw;
        publish_pose(curr_frame->T_cw , pcPtr->header.stamp );
        publish_tf(curr_frame->T_cw);

        //if(FrameCount%15==0)
        //{
        //}

        Vector3d r,t;
        ICP_ALIGNMENT::getAngleandTrans(T_key_curr_est.inverse(),r,t);
        double r_norm=fabs(r[0])+fabs(r[1])+fabs(r[2]);
        double t_norm=fabs(t[0])+fabs(t[1])+fabs(t[2]);
//        if(r_norm>=0.1 || t_norm>0.3)
//        {
//          updateKeyFrame=1;
//          cout << "Update the new key Frame" << endl;
//          TOF_Frame::copy(*curr_frame,*key_frame);
//        }

        TOF_Frame::copy(*curr_frame,*prev_frame);
        curr_frame->clear();
      }
      else
      {
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = pcPtr->header.stamp;
        odom.pose.pose.position.x = 0.012345; odom.pose.pose.position.y = 0; odom.pose.pose.position.z = 0;
        pub_odom.publish(odom);
      }
      tt_cb.toc("Total Processing ");
    }
  }

  void mc_callback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    Eigen::Affine3d T_wi,T_iw;
    Vec3 translation_wi = Vec3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    Eigen::Quaterniond rot_q_wi(msg->pose.orientation.w,msg->pose.orientation.x,
                                msg->pose.orientation.y,msg->pose.orientation.z);
    Mat3x3 rotation_wi = rot_q_wi.toRotationMatrix();
    T_wi.translation() = translation_wi;
    T_wi.linear() = rotation_wi;
    T_iw = T_wi.inverse();
    if((receive_mc_data == 0) && init_by_MC)
    {
      cout << "set T_cw_init with MC" << endl;
      T_cw_init = T_ci*T_iw;
      receive_mc_data = 1;
    }
  }

  void imu_callback(const sensor_msgs::ImuConstPtr& msg)
  {
    Eigen::Affine3d T_wi,T_iw;
    Eigen::Quaterniond q_WI(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
    T_wi.translation() = Vec3(0,0,0);
    T_wi.linear() = q_WI.toRotationMatrix();
    T_iw = T_wi.inverse();
    if(receive_imu_data==0 && init_by_IMU)
    {
      cout << "set T_cw_init with IMU" << endl;
      T_cw_init = T_ci*T_iw;
      receive_imu_data = 1;
    }
  }

  void publish_tf(Eigen::Affine3d T_cw)
  {
    Eigen::Affine3d tf_WC = T_cw.inverse();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion tf_q;
    Vec3 trans_tmp       = tf_WC.translation();
    Matrix3d rot_tmp     = tf_WC.linear();
    Quaterniond q_tmp(rot_tmp);
    transform.setOrigin(tf::Vector3(trans_tmp(0),trans_tmp(1),trans_tmp(2)));
    tf_q.setW(q_tmp.w());  tf_q.setX(q_tmp.x());   tf_q.setY(q_tmp.y());  tf_q.setZ(q_tmp.z());
    transform.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pico_flexx_optical_frame"));
  }

  void publish_pose(Eigen::Affine3d T_cw, ros::Time pose_stamp)
  {
    //print the result
    //publish odom
    //publish result as array
    Eigen::Affine3d tf_WC = T_cw.inverse();
    Eigen::Affine3d T_wi = tf_WC*T_ci;

    Vec3 translation = T_wi.translation();
    Matrix3d rot     = T_wi.linear();
    Quaterniond q(rot);
    Vec3 euler = euler_from_rotation_matrix(rot);
    cout << "Ф:" << std::fixed << euler(0)*57.32 << "  θ:" << euler(1)*57.32 << "  ψ:" << euler(2)*57.32 << " "
         << "x:" << translation(0) << "  y:" << translation(1) << "  z:" << translation(2) << endl;

    std_msgs::Float32MultiArray array;
    array.data.clear();
    array.data.push_back(translation(0)); array.data.push_back(translation(1)); array.data.push_back(translation(2));
    array.data.push_back(euler(0)*57.32); array.data.push_back(euler(1)*57.32); array.data.push_back(euler(2)*57.32);
    pub_tf_array.publish(array);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.header.stamp = pose_stamp;
    odom.pose.pose.position.x = translation(0); odom.pose.pose.position.y = translation(1); odom.pose.pose.position.z = translation(2);
    odom.pose.pose.orientation.w = q.w();        odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();        odom.pose.pose.orientation.z = q.z();
    pub_odom.publish(odom);
  }

  inline void publishPC(CloudTPtr PCptr, string frame_id, ros::Publisher& pub)
  {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*PCptr,output);
    output.header.frame_id=frame_id;
    pub.publish(output);
  }

};//class NICP
}//namespace nodelet_ns



PLUGINLIB_EXPORT_CLASS(nodelet_ns::NICP, nodelet::Nodelet)










