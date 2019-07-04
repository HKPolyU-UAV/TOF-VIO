#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/io/io.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utils/tic_toc_ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <utils/euler_q_rmatrix.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <icp_test.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace Eigen;

inline bool sortbythird(const tuple<PointT, PointT, float, float, float>& a,
                        const tuple<PointT, PointT, float, float, float>& b)
{
    return (get<2>(a) < get<2>(b));
}
inline bool sortby5th(const tuple<PointT, PointT, float, float, float>& a,
                      const tuple<PointT, PointT, float, float, float>& b)
{
    return (get<4>(a) < get<4>(b));
}

namespace nodelet_ns
{


class NICP : public nodelet::Nodelet
{
public:
    NICP()  {;}
    ~NICP() {;}

private:

    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat new_cameraMatrix;
    Mat4x4  cameraMatrix4x4;
    cv::Size imageSize;

    //curr prev key
    Eigen::Affine3d curr_pose;
    Eigen::Affine3d prev_pose;
    Eigen::Affine3d key_pose;
    CloudTPtr currCloud;
    CloudTPtr prevCloud;
    CloudTPtr keyCloud;
    Mat curr_i_img;
    Mat prev_i_img;
    Mat key_i_img;

    Eigen::Affine3d init_pose;
    CloudTPtr sailentCloud;
    CloudTPtr localMap;
    Mat i_canny_dst;
    double fx,fy,cx,cy;
    int pc_height,pc_width;
    Eigen::Affine3d tf_CW;
    //parameters
    double sh_backfloor, sh_depth, sh_grey, sh_edge;
    int target_sailentpts_num;
    int use_depth_grad,use_grey_grad,use_edge_detector,use_canny;
    int init_pose_with_IMU, init_pose_with_MC;
    //from/to/dis/intensity_dis/weight
    std::vector< std::tuple<PointT, PointT, float, float, float> > pairs;
    double med_dp;
    double med_di;
    double sd_dp;
    double sd_di;
    Eigen::Affine3d tf_IC;//Camera to IMU Frame Conversion
    Eigen::Affine3d tf_CI;//IMU to Camera Frame COnversion
    //Flag
    int icp_init;
    int receive_mc_data;
    int receive_imu_data;
    int FrameCount;


    //ROS
    ros::Publisher pub_cloudin;
    ros::Publisher pub_sailentpts;
    ros::Publisher pub_icp_target;
    ros::Publisher pub_icp_process1;
    ros::Publisher pub_icp_process2;
    ros::Publisher pub_icp_reject;
    ros::Publisher pub_tf_array;
    ros::Publisher pub_odom;
    ros::Subscriber mc_sub;
    ros::Subscriber imu_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Subscriber<sensor_msgs::Image> grey_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MyExactSyncPolicy;
    message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;



    virtual void onInit()
    {
        ros::NodeHandle& node = getPrivateNodeHandle();
        //Pub
        pub_cloudin      = node.advertise<sensor_msgs::PointCloud2>   ("icp_cloudin", 1);
        pub_sailentpts   = node.advertise<sensor_msgs::PointCloud2>   ("icp_sailent_pts", 1);
        pub_icp_target   = node.advertise<sensor_msgs::PointCloud2>   ("icp_tgt", 1);
        pub_icp_process1 = node.advertise<sensor_msgs::PointCloud2>   ("icp_pair_from", 1);
        pub_icp_process2 = node.advertise<sensor_msgs::PointCloud2>   ("icp_pair_to", 1);
        pub_icp_reject   = node.advertise<sensor_msgs::PointCloud2>   ("icp_reject", 1);
        pub_tf_array     = node.advertise<std_msgs::Float32MultiArray>("/arrayxyz", 1);
        pub_odom         = node.advertise<nav_msgs::Odometry>         ("icp_odom", 1);


        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        new_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
        string cam_cal_file_path;
        node.getParam("cam_cal_file", cam_cal_file_path);
        cout  << cam_cal_file_path << endl;
        cv::Mat K, D;
        cv::FileStorage param_reader(cam_cal_file_path, cv::FileStorage::READ);
        param_reader["camera_matrix"] >> cameraMatrix;
        param_reader["distortion_coefficients"] >> distCoeffs;
        param_reader["image_height"] >> pc_height;
        param_reader["image_width"] >> pc_width;
        cout << "Camera Matrix:" << endl <<cameraMatrix << endl;
        cout << "Distortion coefficient" << endl << distCoeffs << endl;
        cout << "pc_height  : " << pc_height << endl;
        cout << "pc_width: " << pc_width << endl;
        new_cameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,cv::Size(pc_width,pc_height),1,cv::Size(pc_width,pc_height));
        fx=new_cameraMatrix.at<double>(0,0);
        fy=new_cameraMatrix.at<double>(1,1);
        cx=new_cameraMatrix.at<double>(0,2);
        cy=new_cameraMatrix.at<double>(1,2);
        cameraMatrix4x4 << fx,0,cx,0,
                0,fy,cy,0,
                0,0,1,0,
                0,0,0,1;
        cout << "Undistort PC Camera Matrix 4x4:" << endl << cameraMatrix4x4 << endl;

        node.getParam("icp/target_sailentpts_num",   target_sailentpts_num);
        node.getParam("icp/use_depth_grad",          use_depth_grad);
        node.getParam("icp/use_grey_grad",           use_grey_grad);
        node.getParam("icp/use_edge_detector",       use_edge_detector);
        node.getParam("icp/use_canny",               use_canny);
        node.getParam("icp/sh_backfloor",            sh_backfloor);
        node.getParam("icp/sh_depth",                sh_depth);
        node.getParam("icp/sh_grey",                 sh_grey);
        node.getParam("icp/sh_edge",                 sh_edge);
        node.getParam("icp/init_pose_with_IMU",      init_pose_with_IMU);
        node.getParam("icp/init_pose_with_MC",       init_pose_with_MC);

        cout << "Parameter: target_sailentpts_num  :" << target_sailentpts_num << endl;
        cout << "Parameter: use_depth_grad         :" << use_depth_grad << endl;
        cout << "Parameter: use_grey_grad          :" << use_grey_grad << endl;
        cout << "Parameter: use_edge_detector      :" << use_edge_detector << endl;
        cout << "Parameter: sh_backfloor           :" << sh_backfloor << endl;
        cout << "Parameter: sh_depth               :" << sh_depth << endl;
        cout << "Parameter: sh_grey                :" << sh_grey << endl;
        cout << "Parameter: sh_edge                :" << sh_edge << endl;
        cout << "Parameter: init_pose_with_IMU     :" << init_pose_with_IMU << endl;
        cout << "Parameter: init_pose_with_MC      :" << init_pose_with_MC << endl;
        //Camera to World Conversion
        //  0  0  1  0
        // -1  0  0  0
        //  0 -1  0  0
        //  0  0  0  1
        tf_IC.matrix() << 0,0,1,0.1,-1,0,0,0,0,-1,0,0,0,0,0,1;
        tf_CI = tf_IC.inverse();
        cout << "Transformation from camera frame to imu frame :" << endl << tf_IC.matrix() << endl;
        //set flags
        icp_init = 0;
        receive_mc_data = 0;
        receive_imu_data = 0;
        FrameCount=0;

        //Sub
        mc_sub  = node.subscribe("/vicon", 1, &NICP::mc_callback, this);
        imu_sub = node.subscribe("/mavros/imu/data", 1, &NICP::imu_callback, this);
        //Sync Sub
        pc_sub.subscribe(node, "/pico_flexx/points", 2);
        grey_sub.subscribe(node, "/pico_flexx/image_mono8", 2);
        exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(3), pc_sub, grey_sub);
        exactSync_->registerCallback(boost::bind(&NICP::callback, this, _1, _2));
    }



    void callback(const sensor_msgs::PointCloud2ConstPtr & pcPtr, const sensor_msgs::ImageConstPtr & mono8Ptr)
    {
        tic_toc_ros tt_cb;

        cv_bridge::CvImagePtr cvbridge_image  = cv_bridge::toCvCopy(mono8Ptr,  mono8Ptr->encoding);
        Mat image_raw=cvbridge_image->image;
        cv::equalizeHist(image_raw,image_raw);
        cv::GaussianBlur(image_raw,image_raw, Size(3,3), 0);
        curr_i_img.release();
        cv::GaussianBlur(image_raw,curr_i_img, Size(3,3), 0);
        cv::imshow("i_udistort", curr_i_img);
        cv::waitKey(2);
        size_t size = pcPtr->height * pcPtr->width;
        currCloud = CloudT::Ptr(new CloudT);
        currCloud->width=pcPtr->width;
        currCloud->height=pcPtr->height;
        for(size_t i = 0; i < size; ++i)
        {
            float *itCX = (float *)&pcPtr->data[i * pcPtr->point_step];
            float *itCY = itCX + 1;
            float *itCZ = itCY + 1;
            float *itCN = itCZ + 1;                    // Noise float
            uint16_t *itCM = (uint16_t *)(itCN + 1);   // Mono16 uint16_t
            uint8_t *itGREY = (uint8_t *)(itCM + 1);   // Mono8  uint8_t
            PointT pt;
            pt.x=*itCX;  pt.y=*itCY;  pt.z=*itCZ;  pt.intensity=*itGREY;
            currCloud->push_back(pt);
        }

        if(!icp_init)//first time
        {
            cout << "init icp" << endl;
            if(receive_mc_data && init_pose_with_MC)
            {
                prevCloud    = currCloud;
                keyCloud     = currCloud;
                curr_pose    = init_pose;
                prev_pose    = curr_pose;
                key_pose     = curr_pose;
                prev_i_img   = curr_i_img;
                key_i_img    = curr_i_img;
                sailentCloud = CloudT::Ptr(new CloudT);
                FrameCount = 1;
                icp_init = 1;
                publish_pose(init_pose,pcPtr->header.stamp);
                publish_tf(init_pose);
                cout << "init with motion capture system data" << endl;
            }
            if(receive_imu_data && init_pose_with_IMU)
            {
                prevCloud    = currCloud;
                keyCloud     = currCloud;
                curr_pose    = init_pose;
                prev_pose    = curr_pose;
                key_pose     = curr_pose;
                prev_i_img   = curr_i_img;
                key_i_img    = curr_i_img;
                sailentCloud = CloudT::Ptr(new CloudT);
                FrameCount = 1;
                icp_init = 1;
                cout << init_pose.matrix()<< endl;
                publish_pose(init_pose,pcPtr->header.stamp);
                publish_tf(init_pose);
                cout << "init with imu data" << endl;
            }
            return;
        }
        //select sailentPts from key_Frame
        select_sailent_from_pc(keyCloud, sailentCloud);
        CloudTPtr pc_tgt = CloudT::Ptr(new CloudT);
        CloudTPtr pc_sailent_src = CloudT::Ptr(new CloudT);
        Eigen::Affine3d debug_tf;
        if(0)
        {
            debug_tf.setIdentity();
            debug_tf.linear() = rotation_matrix_from_euler(Vec3(0,0,0));
            debug_tf.translation() =  Vec3(0,0,0.1);
            // Display in terminal the transformation matrix
            std::cout << "Transformation from src to tgt should be " << std::endl;
            cout << debug_tf.matrix() << endl;
            pcl::transformPointCloud (*sailentCloud, *pc_sailent_src, debug_tf.inverse());
        }

        publishPC(sailentCloud,"pico_flexx_optical_frame",pub_sailentpts);
        publishPC(currCloud,"pico_flexx_optical_frame",pub_icp_target);
        publishPC(keyCloud,"pico_flexx_optical_frame",pub_cloudin);
        Eigen::Affine3d tf_key2curr;
        tf_key2curr = prev_pose*(key_pose.inverse());
        sd_dp=0.02;
        sd_di=45;
        double dis;
        int count=0;

        icp_loop( 15,  5 , 400 ,  2e-2, sailentCloud, currCloud, tf_key2curr, dis, count);
        icp_loop( 11,  10 , 400 ,  2e-2, sailentCloud, currCloud, tf_key2curr, dis, count);
        icp_loop( 7,   20 , 400 ,  2e-2, sailentCloud, currCloud, tf_key2curr, dis, count);
        //        CloudTPtr resultpc = CloudT::Ptr(new CloudT);
        //        pcl::transformPointCloud (*pc_sailent_src, *resultpc, tf_key2curr);
        //        publishPC(resultpc,"pico_flexx_optical_frame",pub_icp_reject);
        cout << "med_dp" << med_dp << endl;
        cout << "sd_dp"  << sd_dp << endl;
        cout << "med_di" << med_di << endl;
        cout << "sd_di"  << sd_di << endl;
        cout << "ICP RESULT:  " << count << " loops " << " with the mean distance " << dis << endl;
        cout << tf_key2curr.matrix() << endl;

        curr_pose = tf_key2curr*key_pose;
        //        cout << "cal_velocity" << endl;
        //        cout << curr_pose.matrix() << endl;
        //        cout << prev_pose.matrix() << endl;
        //        Eigen::Affine3d tf_prev2curr = curr_pose*(prev_pose.inverse());
        //        Eigen::Affine3d tf_prev2curr_WC = tf_prev2curr.inverse();
        //        Vec3 trans = tf_prev2curr_WC.translation();
        //        Vec3 euler = euler_from_rotation_matrix(tf_prev2curr_WC.linear());
        //        double t_trans = sqrt(trans(0)*trans(0)+trans(1)*trans(1)+trans(2)*trans(2));
        //        double t_euler = euler(0)+euler(1)+euler(2);
        //        if(t_trans > 0.5 || t_euler >= 1.0)
        //        {
        //            tt_cb.toc("Erroro----- ");
        //            return;
        //        }

        publish_pose(curr_pose , pcPtr->header.stamp );
        publish_tf(curr_pose);
        cout << "For Frame " << FrameCount << " ";
        tt_cb.toc("Total Processing ");
        FrameCount++;

        if(0)
        {
            cout << "Update the new key Frame" << endl;
            keyCloud = currCloud;
            key_pose = curr_pose;
            key_i_img    = curr_i_img;
        }
        prevCloud  = currCloud;
        prev_pose  = curr_pose;
        prev_i_img = curr_i_img;

    }//end of callback function

    void icp_loop(double nns_radius, int max_loop_count, int sample_num, double dis_threshold,
                  const CloudTPtr from, const CloudTPtr to, Eigen::Affine3d& icp_tf, double& mean_dis, int& count)
    {
        for(int loop_count=0; loop_count<max_loop_count;loop_count++) //Loop STEP 2 to STEP6
        {
            //SETP 1: downsample from to ds_from
            CloudTPtr ds_from = CloudT::Ptr(new CloudT);
            for(int i=0; i<sample_num; i++){
                int idx=rand()%(from->size());
                PointT T_pt = pcl::transformPoint(from->at(idx),icp_tf);
                ds_from->push_back(T_pt);
            }
            //SETP 2: NNS radius search and make pairs
            pairs.clear();
            for(int i=0; i<sample_num; i++){
                PointT pt=ds_from->at(i);
                int    u,v;
                reprojection(pt,u,v);
                if(u<0 || u>=223 || v<0 || v>170){ continue;}//search point inside of boundary

                //search range for pt
                int su_min=floor(u-nns_radius);
                int su_max=floor(u+nns_radius);
                if(su_min<=0) su_min = 0;
                if(su_max>=223) su_max = 223;
                int sv_min=floor(v-nns_radius);
                int sv_max=floor(v+nns_radius);
                if(sv_min<=0) sv_min = 0;
                if(sv_max>=170) sv_max = 170;

                int found=0;
                double min_dis = 999.0;
                PointT pt_nn;//neareat neiborhood
                for(int su = su_min; su<=su_max; su++)
                {
                    for(int sv = sv_min; sv<=sv_max; sv++)
                    {
                        PointT pt_serach = to->at(su+sv*pc_width);
                        if(pt_serach.z != pt_serach.z) {continue;}//invalid pts
                        if(sqrt((su-u)*(su-u)+(sv-v)*(sv-v)) > nns_radius) {continue;}//out of search range
                        double dis_p = sqrt((pt_serach.x-pt.x)*(pt_serach.x-pt.x)
                                            +(pt_serach.y-pt.y)*(pt_serach.y-pt.y)
                                            +(pt_serach.z-pt.z)*(pt_serach.z-pt.z));
                        if(dis_p<min_dis){
                            min_dis = dis_p;
                            pt_nn=pt_serach;
                            found=1;
                        }
                    }
                }
                if(found){
                    std::tuple<PointT, PointT, float, float, float> pair;
                    pair = std::make_tuple(pt,pt_nn,min_dis,fabs(pt.intensity-pt_nn.intensity),0);
                    pairs.push_back(pair);
                }
            }
            if(1)//visualize the point
            {
                CloudTPtr pc_from = CloudT::Ptr(new CloudT);
                CloudTPtr pc_to = CloudT::Ptr(new CloudT);
                for (size_t i = 0; i<pairs.size(); i++)
                {
                    pc_from->push_back(get<0>(pairs[i]));
                    pc_to->push_back(get<1>(pairs[i]));
                }
                publishPC(pc_from,"pico_flexx_optical_frame",pub_icp_process1);
                publishPC(pc_to,  "pico_flexx_optical_frame",pub_icp_process2);
            }


            //SETP 3: Weight
            //calculate μ(med_dp,med_di) and σ(sd_dp,sd_di) for the distribution
            //check the pairs distance
            vector<float> tmp_p;
            tmp_p.clear();
            for (size_t i=0; i<pairs.size();i++) {tmp_p.push_back(get<2>(pairs[i]));}
            sort(tmp_p.begin(), tmp_p.end());
            med_dp = tmp_p[tmp_p.size()-tmp_p.size()/2];
            for (size_t i=0; i<tmp_p.size();i++) {tmp_p[i] = fabs(tmp_p[i]-med_dp);}
            sort(tmp_p.begin(), tmp_p.end());
            sd_dp = 1.f*1.4826* tmp_p[tmp_p.size()-tmp_p.size()/2]+1e-11;

            vector<float> tmp_i;
            tmp_i.clear();
            for (size_t i=0; i<pairs.size();i++) {tmp_i.push_back(get<3>(pairs[i]));}
            sort(tmp_i.begin(), tmp_i.end());
            med_di = tmp_i[tmp_i.size()-tmp_i.size()/2];
            for (size_t i=0; i<tmp_i.size();i++) {tmp_i[i] = fabs(tmp_i[i]-med_di);}
            sort(tmp_i.begin(), tmp_i.end());
            sd_di = 1.f*1.4826 * tmp_i[tmp_i.size()-tmp_i.size()/2]+1e-11;
            //            cout << "μ_p:" << med_dp << " σ_p:" << sd_dp << endl;
            //            cout << "μ_i:" << med_di << " σ_i:" << sd_di << endl;
            for (size_t i=0; i<pairs.size();i++){
                double weight_dis = (6.f/(5.f+ pow((get<2>(pairs[i])-med_dp)/sd_dp, 2)));
                double weight_intensity = (6.f/(5.f+ pow((get<3>(pairs[i])-med_di)/sd_di, 2)));
                get<2>(pairs[i]) = weight_dis;
                get<3>(pairs[i]) = weight_intensity;
                get<4>(pairs[i]) = weight_dis*weight_intensity;
            }
            mean_dis = med_dp;


            //STEP 4 Check whether ICP is needed
            if(count==0){
                cout << "init mean_dis is: " << mean_dis << endl;
            }
            if(mean_dis<dis_threshold){
                return;
            }

            //SETP 5: Reject
            sort(pairs.begin(), pairs.end(), sortby5th);
            int cut_off = floor(pairs.size()*0.9);
            pairs.erase(pairs.begin() + cut_off, pairs.end());

            //SETP 6: 3D-3D Alignment
            // Executing the transformation
            pcl::TransformationFromCorrespondences icp_trans;
            icp_trans.reset();
            for (size_t i = 0; i<pairs.size(); i++)
            {
                PointT pt_from, pt_to;
                pt_from = get<0>(pairs[i]);
                pt_to   = get<1>(pairs[i]);
                Vector3f source(pt_from.x,pt_from.y,pt_from.z);
                Vector3f target(pt_to.x,pt_to.y,pt_to.z);
                icp_trans.add(source, target, get<4>(pairs[i]));
            }
            Eigen::Affine3f tmp= icp_trans.getTransformation();
            Eigen::Affine3d increTrans=tmp.cast<double>();
            //cout << "increTrans: " << endl << increTrans.matrix() << endl;

            Vec3 euler,trans;
            getAngleandTrans(increTrans,euler,trans);
            double sum_euler = fabs(euler(0))+fabs(euler(1))+fabs(euler(2));
            double sum_trans = fabs(trans(0))+fabs(trans(1))+fabs(trans(2));
            if(sum_euler<0.01 && sum_trans<0.01)
            {
                cout << "converge with small change in increTrans with "
                     << "angle: " << sum_euler << " translation: " << sum_trans << endl;
                return;
            }

            count++;
            icp_tf = increTrans*icp_tf;

        }
        //tt_icploop.toc("One icp loop");
    }


    inline void select_sailent_from_pc(CloudTPtr pc, CloudTPtr& sailent_pc)
    {
        sailent_pc->clear();
        int sailentpts_cnt = 0;
        if(use_canny)
        {
            Canny( key_i_img, i_canny_dst , 230, 300, 3 );
            cv::imshow("canny", i_canny_dst);
            cv::waitKey(2);
        }
        for(int u=15; u<(pc_width-16); u++)// u v range = 90% full range
        {
            for(int v=15; v<(pc_height-16); v++)//
            {
                PointT pt = pc->at(u+v*pc_width);
                if(pt.z==pt.z)//valid pts
                {
                    //reject rules
                    //background rejection
                    float dis = pt.z;
                    float diff1=dis - pc->at((u+4)+v*pc_width).z;
                    float diff2=dis - pc->at((u-4)+v*pc_width).z;
                    float diff3=dis - pc->at(u+(v+4)*pc_width).z;
                    float diff4=dis - pc->at(u+(v-4)*pc_width).z;
                    float thres= sh_backfloor*dis;
                    if ( diff1>thres && diff1==diff1){continue;}
                    if ( diff2>thres && diff2==diff2){continue;}
                    if ( diff3>thres && diff3==diff3){continue;}
                    if ( diff4>thres && diff4==diff4){continue;}
                    //accept rules
                    if(use_canny)
                    {

                        int idx=v*pc_width+u;
                        if((i_canny_dst.at<uint8_t>(idx)) == 255)
                        {
                            sailentpts_cnt++;
                            sailent_pc->push_back(pt);
                            continue;
                        }
                    }
                    //depthe grad
                    if(use_depth_grad){
                        float fabs_d_gradx = fabs(pc->at((u-4)+v*pc_width).z - pc->at((u+4)+v*pc_width).z);
                        float fabs_d_grady = fabs(pc->at(u+(v-4)*pc_width).z - pc->at(u+(v+4)*pc_width).z);
                        if(fabs_d_gradx>(sh_depth*dis) || fabs_d_grady>(sh_depth*dis)){
                            sailentpts_cnt++;
                            sailent_pc->push_back(pt);
                            continue;
                        }
                    }
                    //intensity grad
                    if(use_grey_grad){
                        float fabs_g_gradx = fabs(pc->at((u-4)+v*pc_width).intensity - pc->at((u+4)+v*pc_width).intensity);
                        float fabs_g_grady = fabs(pc->at(u+(v-4)*pc_width).intensity - pc->at(u+(v+4)*pc_width).intensity);
                        if(fabs_g_gradx>sh_grey || fabs_g_grady>sh_grey){
                            sailent_pc->push_back(pt);
                            sailentpts_cnt++;
                            continue;
                        }
                    }
                    //edge_detector
                    if(use_edge_detector){
                        float gx[6],gy[6],gmaxx[2],gmaxy[2];
                        gx[0]= pc->at((u-3)+v*pc_width).z - pc->at((u-2)+v*pc_width).z;
                        gx[1]= pc->at((u-2)+v*pc_width).z - pc->at((u-1)+v*pc_width).z;
                        gx[2]= pc->at((u-1)+v*pc_width).z - pc->at((u)+v*pc_width).z;
                        gx[3]= pc->at((u)+v*pc_width).z   - pc->at((u+1)+v*pc_width).z;
                        gx[4]= pc->at((u+1)+v*pc_width).z - pc->at((u+2)+v*pc_width).z;
                        gx[5]= pc->at((u+2)+v*pc_width).z - pc->at((u+3)+v*pc_width).z;

                        gy[0]= pc->at(u+(v-3)*pc_width).z - pc->at(u+(v-2)*pc_width).z;
                        gy[1]= pc->at(u+(v-2)*pc_width).z - pc->at(u+(v-1)*pc_width).z;
                        gy[2]= pc->at(u+(v-1)*pc_width).z - pc->at(u+(v)*pc_width).z;
                        gy[3]= pc->at(u+(v)*pc_width).z   - pc->at(u+(v+1)*pc_width).z;
                        gy[4]= pc->at(u+(v+1)*pc_width).z - pc->at(u+(v+2)*pc_width).z;
                        gy[5]= pc->at(u+(v+2)*pc_width).z - pc->at(u+(v+3)*pc_width).z;

                        gmaxx[0]=fabs( pc->at((u-3)+v*pc_width).z-pc->at((u)+v*pc_width).z);
                        gmaxx[1]=fabs( pc->at((u)+v*pc_width).z-pc->at((u+3)+v*pc_width).z);

                        gmaxy[0]=fabs( pc->at(u+(v-3)*pc_width).z-pc->at(u+(v)*pc_width).z);
                        gmaxy[1]=fabs( pc->at(u+(v)*pc_width).z-pc->at(u+(v+3)*pc_width).z);


                        if((gx[0]>0 && gx[1]>0 && gx[2]>0 && gx[3]<0 && gx[4]<0 && gx[5]<0 && (gmaxx[0]>sh_edge*dis || gmaxx[1]>sh_edge*dis))
                                ||(gx[0]<0 && gx[1]<0 && gx[2]<0 && gx[3]>0 && gx[4]>0 && gx[5]>0 && (gmaxx[0]>sh_edge*dis || gmaxx[1]>sh_edge*dis))
                                ||(gy[0]>0 && gy[1]>0 && gy[2]>0 && gy[3]<0 && gy[4]<0 && gy[5]<0 && (gmaxy[0]>sh_edge*dis || gmaxy[1]>sh_edge*dis))
                                ||(gy[0]<0 && gy[1]<0 && gy[2]<0 && gy[3]>0 && gy[4]>0 && gy[5]>0 && (gmaxy[0]>sh_edge*dis || gmaxy[1]>sh_edge*dis)))
                        {
                            sailent_pc->push_back(pt);
                            sailentpts_cnt++;
                            continue;
                        }
                    }
                }//valid pts
            }
        }
        if(sailentpts_cnt < target_sailentpts_num)
        {
            cout << "sailentpts_cnt "<< sailentpts_cnt << " less than" << target_sailentpts_num
                 << " add random pts" << endl;
            //            for(int sample_count=0; sailentpts_cnt<target_sailentpts_num && sample_count<5000 ; sample_count++)//sample 5000
            //            {
            //                int u=rand()%(pc_width-6)+3;
            //                int v=rand()%(pc_height-6)+3;
            //                PointT pt = pc->at(u+v*pc_width);
            //                if(pt.z==pt.z)//valid pts
            //                {
            //                    sailentpts_cnt++;
            //                    sailent_pc->push_back(pt);
            //                    continue;
            //                }
            //            }
        }
        //        if(sailentpts_cnt < target_sailentpts_num)
        //        {
        //            cout << "Error, Please check the input cloud" << endl;
        //            return;

        //        }
    }

    inline void random_sailent_from_pc(CloudTPtr pc, CloudTPtr& sailent_pc)
    {
        sailent_pc->clear();
        int sailentpts_cnt = 0;
        while(1)
        {
            int idx = rand()%(pc->size());
            PointT pt=pc->at(idx);
            if(pt.x==pt.x){
                sailent_pc->push_back(pt);
                sailentpts_cnt++;
            }
            if(sailentpts_cnt >= target_sailentpts_num)
                break;
        }
    }



    inline void reprojection(const PointT pt, int& u, int& v)
    {
        u=int(floor(fx/pt.z * pt.x + cx));
        v=int(floor(fy/pt.z * pt.y + cy));
    }

    void mc_callback(const geometry_msgs::TransformStampedConstPtr& msg)
    {
        Eigen::Affine3d tf_WI,tf_IW;
        Vec3 translation_WI = Vec3(msg->transform.translation.x,msg->transform.translation.y,msg->transform.translation.z);
        Eigen::Quaterniond rot_q_WI(msg->transform.rotation.w,msg->transform.rotation.x,msg->transform.rotation.y,msg->transform.rotation.z);
        Mat3x3 rotation_WI = rot_q_WI.toRotationMatrix();
        tf_WI.translation() = translation_WI;
        tf_WI.linear() = rotation_WI;
        tf_IW = tf_WI.inverse();
        if((receive_mc_data == 0) && init_pose_with_MC)
        {
            cout << "set init_pose with MC" << endl;
            init_pose = tf_CI*tf_IW;
            receive_mc_data = 1;
        }
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& msg)
    {
        Eigen::Affine3d tf_WI,tf_IW;
        Eigen::Quaterniond q_WI(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
        tf_WI.translation() = Vec3(0,0,0);
        tf_WI.linear() = q_WI.toRotationMatrix();
        tf_IW = tf_WI.inverse();
        if(receive_imu_data==0 && init_pose_with_IMU)
        {
            cout << "set init_pose with IMU" << endl;
            init_pose = tf_CI*tf_IW;
            receive_imu_data = 1;
        }
    }

    void publish_tf(Eigen::Affine3d tf_CW)
    {
        Eigen::Affine3d tf_WC = tf_CW.inverse();
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

    void publish_pose(Eigen::Affine3d tf_CW, ros::Time pose_stamp)
    {
        //print the result
        //publish odom
        //publish result as array
        Eigen::Affine3d tf_WC = tf_CW.inverse();
        Eigen::Affine3d tf_WI = tf_WC*tf_CI;

        Vec3 translation = tf_WI.translation();
        Matrix3d rot     = tf_WI.linear();
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


    void remove_nan(CloudTPtr& in, CloudTPtr& out)
    {
        out.reset();
        int count=0;
        for(int i=0; i< in->size(); i++)
        {
            PointT pt=in->at(i);
            if(pt.x==pt.x && pt.y==pt.y && pt.z==pt.z && pt.intensity==pt.intensity)
            {
                out->push_back(pt);
                count++;
            }
        }
        out->width = count;
        out->height = 1;
    }

    inline void getAngleandTrans(Eigen::Affine3d tf, Vec3& angle, Vec3& trans)
    {
        trans = tf.translation();
        angle = euler_from_rotation_matrix(tf.linear());
    }
};//class NICP
}//namespace nodelet_ns



PLUGINLIB_EXPORT_CLASS(nodelet_ns::NICP, nodelet::Nodelet)










