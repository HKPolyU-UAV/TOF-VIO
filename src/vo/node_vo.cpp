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
#include <image_transport/image_transport.h>
//PCL
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
//USER
#include <include/common.h>
#include <include/tic_toc_ros.h>
#include <include/euler_q_rmatrix.h>
#include <vo/salientpts.h>
#include <vo/icp.h>
#include <vo/tof_frame.h>


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
    std::vector<TOF_Frame> keyframes;
    //map
    CloudTPtr mapcloud;

    Eigen::Affine3d T_cw_init;
    Eigen::Affine3d T_cw_imu_last;
    Eigen::Affine3d T_cw_gt_last;
    Eigen::Affine3d T_ic,T_ci;//T_ic: camera to imu

    //Salient Pts extractor and ICP alignment
    SalientPts*    salient_pts_extractor;
    ICP_ALIGNMENT* icp_alignment;


    //Flag
    bool init_by_IMU, init_by_MC, vo_mode;
    int icp_init;
    int receive_mc_data;
    int receive_imu_data;
    int FrameCount;
    int kf_criteria;
    int use_other_icp;
    bool use_orig_pts;
    bool use_ransomdownsample_pts;
    bool use_salient_pts;
    bool use_robust_w;

    //stastic
    double time_sum;

    //PUB
    ros::Publisher pub_cloudin;
    ros::Publisher pub_keyframe;
    ros::Publisher pub_sailentpts;
    ros::Publisher pub_mapcloud;
    ros::Publisher pub_tf_array;
    ros::Publisher pub_odom;
    image_transport::Publisher pub_colored_dimg;

    //SUB
    ros::Subscriber mc_sub;
    ros::Subscriber imu_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Subscriber<sensor_msgs::Image>       grey_sub;
    message_filters::Subscriber<sensor_msgs::Image>       depth_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image> MyExactSyncPolicy;
    message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

    virtual void onInit()
    {
        ros::NodeHandle& node = getPrivateNodeHandle();

        string  cam_cal_file_path;
        cv::Mat cameraMatrix, distCoeffs;
        double  fx,fy,cx,cy;
        int     pc_height,pc_width;
        int     target_sailentpts_num;
        int     use_depth_grad,use_intensity_grad,use_edge_detector,use_canny;
        float   sh_backfloor, sh_depth, sh_grey, sh_edge;

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
        node.getParam("icp/vo_mode",                 vo_mode);
        node.getParam("icp/kf_criteria",             kf_criteria);
        node.getParam("icp/use_other_icp",           use_other_icp);
        node.getParam("icp/use_orig_pts",            use_orig_pts);
        node.getParam("icp/use_salient_pts",         use_salient_pts);
        node.getParam("icp/use_ransomdownsample_pts",use_ransomdownsample_pts);
        node.getParam("icp/use_robust_w",            use_robust_w);
        node.getParam("cam_cal_file", cam_cal_file_path);
        if(use_other_icp==0)
        {
            int method=0;
            if(use_orig_pts) method++;
            if(use_salient_pts) method++;
            if(use_ransomdownsample_pts) method++;
            if(method!=1)
            {
                cout << "error: please check the launch file" << endl;
                while(1);
            }
        }

        time_sum = 0;

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
        mapcloud   = CloudTPtr(new CloudT);
        keyframes.clear();
        //Camera to IMU Conversion
        //  0  0  1  0.1
        // -1  0  0  0
        //  0 -1  0  0.01
        //  0  0  0  1
        T_ic.matrix() <<  0, 0, 1, 0.03,
                -1, 0, 0,   0,
                0,-1, 0,0.01,
                0, 0, 0,   1;
        T_ci = T_ic.inverse();
        cout << "Transformation from camera frame to imu frame :" << endl << T_ic.matrix() << endl;
        //set flags
        icp_init = 0;
        receive_mc_data = 0;
        receive_imu_data = 0;
        FrameCount = 0;

        //Pub
        pub_cloudin      = node.advertise<sensor_msgs::PointCloud2>   ("/icp_cloudin", 1);
        pub_sailentpts   = node.advertise<sensor_msgs::PointCloud2>   ("/icp_sailent_pts", 1);
        pub_keyframe     = node.advertise<sensor_msgs::PointCloud2>   ("/icp_key_frame", 1);
        pub_mapcloud     = node.advertise<sensor_msgs::PointCloud2>   ("/map", 1);
        pub_tf_array     = node.advertise<std_msgs::Float32MultiArray>("/arrayxyz", 1);
        pub_odom         = node.advertise<nav_msgs::Odometry>         ("/icp_odom", 1);
        image_transport::ImageTransport it(node);
        pub_colored_dimg = it.advertise("/colored_dimg", 1);

        //Sub
        mc_sub  = node.subscribe("/input_gt", 1, &NICP::mc_callback, this);
        imu_sub = node.subscribe("/input_imu", 1, &NICP::imu_callback, this);
        //Sync Sub
        pc_sub.subscribe   (node,   "/input_tof_pc",   2);
        grey_sub.subscribe (node,   "/input_tof_nir", 2);
        depth_sub.subscribe(node,   "/input_tof_depth", 2);
        exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(2), pc_sub, grey_sub,depth_sub);
        exactSync_->registerCallback(boost::bind(&NICP::tof_callback, this, _1, _2, _3));
        cout << "start the thread" << endl;
    }

    void tof_callback(const sensor_msgs::PointCloud2ConstPtr& pcPtr,
                      const sensor_msgs::ImageConstPtr& mono8Ptr,
                      const sensor_msgs::ImageConstPtr& depthPtr)
    {
        //tic_toc_ros tt_cb;
        FrameCount++;
        Mat dimg = cv_bridge::toCvCopy(depthPtr,  depthPtr->encoding)->image;
        Mat colored_dimg;
        visualizeDepthImg(colored_dimg,dimg);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colored_dimg).toImageMsg();
        pub_colored_dimg.publish(msg);
        //if(icp_init) findNearestKeyframe();

        tic_toc_ros tt_cnt;
        curr_frame->read_PC_Iimg_FromROSMsg(pcPtr,mono8Ptr);
        if(use_orig_pts)
            curr_frame->sailent_cloud = curr_frame->cloud;
        if(use_salient_pts)
            salient_pts_extractor->select_salient_from_pc(curr_frame->sailent_cloud,curr_frame->cloud,curr_frame->i_img);
        if(use_ransomdownsample_pts)
            salient_pts_extractor->select_random_from_pc(curr_frame->sailent_cloud,curr_frame->cloud,curr_frame->i_img);
        //cout << tt_cnt.dT_ms() << endl;
        if(!icp_init)//first time
        {
            if((receive_mc_data && init_by_MC)||(receive_imu_data && init_by_IMU)||vo_mode)
            {
                cout << "init icp" << endl;
                Eigen::Affine3d T_wc;
                //mc_sub.shutdown();
                //imu_sub.shutdown();
                if(vo_mode) T_cw_init=T_ci;
                curr_frame->T_cw = T_cw_init;
                T_wc = T_cw_init.inverse();
                curr_frame->T_wc = T_wc;
                curr_frame->T_cw = T_wc.inverse();
                TOF_Frame::copy(*curr_frame,*prev_frame);
                TOF_Frame::copy(*curr_frame,*key_frame);
                keyframes.push_back(*curr_frame);
                addFrameToMap(key_frame);
                FrameCount = 1;
                icp_init = 1;
                publish_pose(curr_frame->T_cw,pcPtr->header.stamp);
                publish_tf(curr_frame->T_cw);
                if(receive_mc_data && init_by_MC){
                    cout << "init with motion capture system data" << endl;
                }else if (receive_imu_data && init_by_IMU) {
                    cout << "init with imu data" << endl;
                }else
                {
                    cout << "run in vo mode" << endl;
                }
            }
            return;
        }
        else
        {
            publishPC(curr_frame->cloud,"pico_flexx_optical_frame",pub_cloudin);
            publishPC(curr_frame->sailent_cloud,"pico_flexx_optical_frame",pub_sailentpts);

            pcl::PointCloud<PointT>::Ptr keyframinworld (new pcl::PointCloud<PointT> ());
            pcl::transformPointCloud(*key_frame->cloud, *keyframinworld, key_frame->T_wc.cast<float>());
            publishPC(keyframinworld, "world",pub_keyframe);
            publishPC(mapcloud,"world",pub_mapcloud);

            Eigen::Affine3d T_key_curr_guess = key_frame->T_cw * prev_frame->T_wc;
            Eigen::Affine3d T_key_curr_est;
            double mean_error= 100;
            int inlier_count = 0;
            int loop_count = 0;

            if(this->use_other_icp)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz_curr (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz_key (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                cloudxyz_curr->points.resize(curr_frame->cloud->size());
                cloudxyz_key->points.resize(key_frame->cloud->size());
                for (size_t i = 0; i < curr_frame->cloud->points.size(); i++) {
                    cloudxyz_curr->points[i].x = curr_frame->cloud->points[i].x;
                    cloudxyz_curr->points[i].y = curr_frame->cloud->points[i].y;
                    cloudxyz_curr->points[i].z = curr_frame->cloud->points[i].z;
                }
                for (size_t i = 0; i < key_frame->cloud->points.size(); i++) {
                    cloudxyz_key->points[i].x = key_frame->cloud->points[i].x;
                    cloudxyz_key->points[i].y = key_frame->cloud->points[i].y;
                    cloudxyz_key->points[i].z = key_frame->cloud->points[i].z;
                }
                std::vector<int> indices;
                cloudxyz_curr->is_dense = false;
                cloudxyz_key->is_dense = false;
                pcl::removeNaNFromPointCloud(*cloudxyz_curr, *cloudxyz_curr, indices);
                pcl::removeNaNFromPointCloud(*cloudxyz_key,  *cloudxyz_key,  indices);
                if(use_other_icp==1)
                {
                    cout << "use conventional icp+RANSAC (PCL)" << endl;
                    icp.setInputSource(cloudxyz_curr);
                    icp.setInputTarget(cloudxyz_key);
                    icp.setMaximumIterations (30);
                    icp.setTransformationEpsilon (1e-5);
                    icp.setMaxCorrespondenceDistance (0.1);
                    icp.setEuclideanFitnessEpsilon (0.0001);
                    icp.setRANSACOutlierRejectionThreshold (0.01);
                    pcl::PointCloud<pcl::PointXYZ> Final;
                    icp.align (Final,T_key_curr_guess.matrix().cast<float>());
                    std::cout << "has converged:" << icp.hasConverged() << endl;
                    mean_error = icp.getFitnessScore();
                    inlier_count = Final.points.size();
                    T_key_curr_est = Eigen::Affine3d(icp.getFinalTransformation().cast<double>());
                    loop_count = 10;
                }if(use_other_icp==2)
                {
                    //                    cout << "use Normal Distribution Based  (PCL)" << endl;
                    //                    // Initializing Normal Distributions Transform (NDT).
                    //                    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
                    //                    ndt.setTransformationEpsilon (0.001);
                    //                    // Setting maximum step size for More-Thuente line search.
                    //                    ndt.setStepSize (0.1);
                    //                    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
                    //                    ndt.setResolution (0.05);
                    //                    // Setting max number of registration iterations.
                    //                    ndt.setMaximumIterations (20);
                    //                    // Setting point cloud to be aligned.
                    //                    ndt.setInputSource (cloudxyz_curr);
                    //                    // Setting point cloud to be aligned to.
                    //                    ndt.setInputTarget (cloudxyz_key);
                    //                    pcl::PointCloud<pcl::PointXYZ> Final;
                    //                    ndt.align (Final,T_key_curr_guess.matrix().cast<float>());
                    //                    std::cout << "has converged:" << ndt.hasConverged() << endl;
                    //                    mean_error = ndt.getFitnessScore();
                    //                    inlier_count = Final.points.size();
                    //                    T_key_curr_est = Eigen::Affine3d(icp.getFinalTransformation().cast<double>());
                    //                    loop_count = 10;
                }

            }else
            {

                icp_alignment->alignment(curr_frame->sailent_cloud,
                                         key_frame->cloud,
                                         use_robust_w,
                                         T_key_curr_guess,
                                         T_key_curr_est,
                                         mean_error,
                                         loop_count,
                                         inlier_count,
                                         false);


            }

            time_sum+=tt_cnt.dT_ms();
            //cout << "mean error " << mean_error << " with " << loop_count << " loops and contain " << inlier_count << " inliers" << endl;
            //cout << "average processing time" << time_sum/((double)FrameCount) << endl;
            //publish_tf(curr_frame->T_cw);
            if(mean_error<0.1 && inlier_count>200)//alignment success
            {
                curr_frame->T_cw = T_key_curr_est.inverse() * key_frame->T_cw;
                curr_frame->T_wc = curr_frame->T_cw.inverse();
                if(kf_criteria==1)//distance based criteria
                {
                    if(FrameCount%20==0)
                    {
                        //cout << "Update the new key Frame" << endl;
                        TOF_Frame::copy(*curr_frame,*key_frame);
                        keyframes.push_back(*key_frame);
                        addFrameToMap(key_frame);
                    }
                }
                if(kf_criteria==2)//distance based criteria
                {
                    Vector3d r_curr,t_curr,r_key,t_key;
                    Vector3d r_diff,t_diff;
                    ICP_ALIGNMENT::getAngleandTrans(curr_frame->T_wc*this->T_ci,r_curr,t_curr);
                    ICP_ALIGNMENT::getAngleandTrans(key_frame->T_wc*this->T_ci,r_key,t_key);
                    t_diff = t_curr-t_key;
                    r_diff = r_curr-r_key;
                    double r_norm=fabs(r_diff[0]+r_diff[1]+r_diff[2]);
                    double t_norm=t_diff.norm();
                    if(t_norm>0.3 || r_norm>0.2)
                    {
                        //cout << "Update the new key Frame" << endl;
                        TOF_Frame::copy(*curr_frame,*key_frame);
                        keyframes.push_back(*key_frame);
                        addFrameToMap(key_frame);
                    }
                    ICP_ALIGNMENT::getAngleandTrans(curr_frame->T_wc*this->T_ci,r_curr,t_curr);
                    //                    cout << "r:" << r_curr[0]*57
                    //                         << " p:" << r_curr[1]*57
                    //                         << " y:" <<  r_curr[2]*57 << endl;
                }

                publish_pose(curr_frame->T_cw , pcPtr->header.stamp );
                publish_tf(curr_frame->T_cw);
                TOF_Frame::copy(*curr_frame,*prev_frame);
                curr_frame->clear();
            }
            else
            {
                nav_msgs::Odometry odom;
                odom.header.frame_id = "world";
                odom.header.stamp = pcPtr->header.stamp;
                odom.pose.pose.position.x = 0.012345; odom.pose.pose.position.y = 0; odom.pose.pose.position.z = 0;
                //pub_odom.publish(odom);
            }
            //tt_cb.toc("Total Processing ");
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
        T_cw_gt_last = T_ci*T_iw;
        if((receive_mc_data == 0) && init_by_MC)
        {
            cout << "set T_cw_init with MC" << endl;
            T_cw_init = T_cw_gt_last;
            receive_mc_data = 1;
        }
        T_cw_init = T_cw_gt_last;
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& msg)
    {
        Eigen::Affine3d T_wi,T_iw;
        Eigen::Quaterniond q_WI(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
        T_wi.translation() = Vec3(0,0,0);
        T_wi.linear() = q_WI.toRotationMatrix();
        T_iw = T_wi.inverse();
        T_cw_imu_last = T_ci*T_iw;
        if(receive_imu_data==0 && init_by_IMU)
        {
            cout << "set T_cw_init with IMU" << endl;
            T_cw_init = T_cw_imu_last;
            receive_imu_data = 1;
        }
    }

    void findNearestKeyframe(void)
    {
        //cout << "Search nearest keyframe:" << endl;
        Vec3 t_guess = prev_frame->T_wc.translation();
        size_t min_idx=0;
        double min_rtnome=10;

        Vector3d r_prev,t_prev,r_key,t_key;
        Vector3d r_diff,t_diff;
        ICP_ALIGNMENT::getAngleandTrans(prev_frame->T_wc*this->T_ci,r_prev,t_prev);
        for(size_t i=0; i<this->keyframes.size(); i++)
        {
            ICP_ALIGNMENT::getAngleandTrans(keyframes.at(i).T_wc*this->T_ci,r_key,t_key);
            t_diff = t_prev-t_key;
            r_diff = r_prev-r_key;
            double r_norm=fabs(r_diff[0])+fabs(r_diff[1])+fabs(r_diff[2]);

            //cout << "r_nome" << r_norm << endl;
            double t_norm=t_diff.norm();
            if(r_norm<0.15)
            {
                if(t_norm<min_rtnome)
                {
                    min_rtnome = t_norm;
                    min_idx = i;
                    TOF_Frame::copy(keyframes.at(min_idx),*key_frame);
                }
            }
        }
        //cout << "cloest idx " << min_idx << " with distance" << min_distance << endl;
        //TOF_Frame::copy(keyframes.at(min_idx),*key_frame);
    }

    void addFrameToMap(TOF_Frame::Ptr frame)
    {
        pcl::PointCloud<PointT>::Ptr pc_in_world (new pcl::PointCloud<PointT> ());
        pcl::transformPointCloud(*frame->cloud, *pc_in_world, frame->T_wc.cast<float>());
        std::vector<int> indices;
        pc_in_world->is_dense=false;
        pcl::removeNaNFromPointCloud(*pc_in_world, *pc_in_world, indices);
        //      CloudXYZPtr pcxyz_in_world (new CloudXYZ);
        //      pcxyz_in_world->points.resize(pc_in_world->size());
        //      for (size_t i = 0; i < pc_in_world->size(); i++) {
        //        pcxyz_in_world->points[i].x = pc_in_world->points[i].x;
        //        pcxyz_in_world->points[i].y = pc_in_world->points[i].y;
        //        pcxyz_in_world->points[i].z = pc_in_world->points[i].z;
        //      }
        *mapcloud += *pc_in_world;
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (mapcloud);
        sor.setLeafSize (0.08f, 0.08f, 0.08f);
        sor.filter (*mapcloud);
    }

    inline void visualizeDepthImg(cv::Mat& visualized_depth, cv::Mat d_img_in)
    {
        Mat d_img = d_img_in;
        int size=d_img.cols*d_img.rows;
        for(int i=0; i<size; i++)
        {
            if(isnan(d_img.at<float>(i)))
            {
                d_img.at<float>(i)=0;
            }
            if(d_img.at<float>(i)>6||d_img.at<float>(i)<0.5)
            {
                d_img.at<float>(i)=0;
            }
        }
        cv::Mat adjMap;
        d_img.convertTo(adjMap,CV_8UC1, (255/6), 0);
        cv::applyColorMap(adjMap, visualized_depth, cv::COLORMAP_RAINBOW);
        for(int i=0; i<size; i++)
        {
            if(d_img.at<float>(i)==0)
            {
                cv::Vec3b color = visualized_depth.at<cv::Vec3b>(i);
                color[0]=255;
                color[1]=255;
                color[2]=255;
                visualized_depth.at<cv::Vec3b>(i)=color;
            }
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
        //    cout << "Ф:" << std::fixed << euler(0)*57.32 << "  θ:" << euler(1)*57.32 << "  ψ:" << euler(2)*57.32 << " "
        //         << "x:" << translation(0) << "  y:" << translation(1) << "  z:" << translation(2) << endl;

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










