#include <iomanip>
#include <iostream>
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigen>
#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs.hpp"
#include <common.h>

using namespace std;
using namespace message_filters;
using namespace cv;
using namespace pcl;

//Publisher
ros::Publisher pub_d;
ros::Publisher pub_i;
ros::Publisher pub_pc1;
ros::Publisher pub_pc2;


const Eigen::Vector3d trans_opti2enu(0,0,0);//x y z
const Eigen::Quaterniond rot_opti2enu(-0.5, 0.5, -0.5, 0.5);//w x y z

void buildUnorganizedPointCloudENU(const Mat& depth_img, const Mat& intensity_img, const Mat& intr_rect_ir, CloudT& cloud);

int gotCameraInfo=0;
Mat map1,map2;
Mat new_camera_matrix;
cv::Size image_size;
void getCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info);


float para_depth_grad_threshold = 0.1;    //0.01~0.08
float para_itensity_grad_threshold = 0.3; //20~90
float para_itensity_residual = 100.0;       //2~30
Mat prev_i_image;
unsigned int frame_cnt=0;
int width=0;
int height=0;

inline int get55patch_dx(const int i)//i=0~24
{
    return (i%5)-2;
}
inline int get55patch_dy(const int i)//i=0~24
{
    return ((int)(i/5))-2;
}


void callback(const sensor_msgs::ImageConstPtr& imagePtr_depth, const sensor_msgs::ImageConstPtr& imagePtr_intensity)
{
    //For PMD Flexx camera
    //depth image encoding 32FC1
    //intensity image encoding mono16 CV_16UC1
    ros::Time begin = ros::Time::now();

    ros::Time msg_stamp = imagePtr_depth->header.stamp;
    cv_bridge::CvImagePtr d_raw_image = cv_bridge::toCvCopy(imagePtr_depth, imagePtr_depth->encoding);
    cv_bridge::CvImagePtr i_raw_image  = cv_bridge::toCvCopy(imagePtr_intensity,  imagePtr_intensity->encoding);

    Mat d_image,i_image;
    cv::remap(d_raw_image->image, d_image, map1, map2, cv::INTER_NEAREST,cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(i_raw_image->image, i_image, map1, map2, cv::INTER_NEAREST,cv::BORDER_CONSTANT, cv::Scalar());
    i_image.convertTo(i_image,CV_8UC1);
    //cv::equalizeHist(i_image,i_image);
    //cv::GaussianBlur(i_image,i_image, Size(5,5), 0);
    //cv::bilateralFilter(i_image,i_image,5,100,100);
    cv::medianBlur(i_image,i_image,5);
    //BrightnessAndContrastAuto(i_image,i_image,0);
    frame_cnt++;
    if(frame_cnt == 1)
    {
        cout << "Got First Image" << endl;
        width = imagePtr_depth->width;
        height = imagePtr_depth->height;//224 x 171
        prev_i_image.release();
        prev_i_image = i_image;
        return;
    }

    //select sailent pts
    int count=0;
    Mat feature_image;
    cv::Size size;
    size.height = height;
    size.width = width;


    feature_image = Mat( size , d_image.type());
    feature_image = Scalar::all(0);
    //Filters
    for(int y = 5; y< (height-6) ; y++)//for every row
    {
        for(int x = 5; x< (width-6); x++)//for every col
        {
            float dis = d_image.at<float>(Point(x,y));
            if(dis==0 || isnan(dis)) continue;

            //background rejection
            float diff1=dis - d_image.at<float>(Point(x,y+2));
            float diff2=dis - d_image.at<float>(Point(x+2,y));
            float diff3=dis - d_image.at<float>(Point(x,y-2));
            float diff4=dis - d_image.at<float>(Point(x-2,y));
            if ( diff1!=diff1 || diff2!=diff2 || diff3!=diff3 || diff4!=diff4){
                continue;
            }
            float thres= 0.03*dis;
            if ( diff1>thres || diff2>thres || diff3>thres || diff4>thres ){
                continue;
            }

            //            if(1)
            //            {
            //                feature_image.at<float>(Point(x,y)) = d_image.at<float>(Point(x,y));
            //                count++;
            //                continue;
            //            }


            //            if(Canny_feature.at<unsigned char>(Point(x,y))!=0)
            //            {
            //                feature_image.at<float>(Point(x,y)) = d_image.at<float>(Point(x,y));
            //                count++;
            //                continue;
            //            }
            //assess pixel using Point(x,y)
            // ---------------> x (width)                     27              Patch
            // |                                              26
            // |      (0,0)  (1,0)  (2,0)  |           0   1   2   3   4
            // |                           |           5   6   7   8   9
            // |      (1,0)  (1,1)  (2,1)  |  29  28  10  11  12  13  14  32  33
            // |                           |          15  16  17  18  19
            // |      (2,0)  (1,2)  (2,2)  |          20  21  22  23  24
            // v                                              30
            // y(height)                                      31
            float z_patch[25];
            float i_patch[25];
            for(int k=0; k<25; k++)
            {
                z_patch[k] = d_image.at<float>(Point(x+get55patch_dx(k),y+get55patch_dy(k)));
                i_patch[k] = i_image.at<unsigned char>(Point(x+get55patch_dx(k),y+get55patch_dy(k)));
            }
            z_patch[26] = d_image.at<float>(Point(x,y-3));
            z_patch[27] = d_image.at<float>(Point(x,y-4));
            z_patch[28] = d_image.at<float>(Point(x-3,y));
            z_patch[29] = d_image.at<float>(Point(x-4,y));
            z_patch[30] = d_image.at<float>(Point(x,y+3));
            z_patch[31] = d_image.at<float>(Point(x,y+4));
            z_patch[32] = d_image.at<float>(Point(x+3,y));
            z_patch[33] = d_image.at<float>(Point(x+4,y));

            if(0)//intensity residual
            {
                float intensity= i_image.at<unsigned char>(Point(x,y));
                float prev_intensity = prev_i_image.at<unsigned char>(Point(x,y));
                if(intensity-prev_intensity>50)
                {
                    {
                        feature_image.at<float>(Point(x,y)) = d_image.at<float>(Point(x,y));
                        count++;
                        continue;
                    }
                }
            }

            if(1)//intensity grad
            {
                if(  fabs(i_patch[0]-i_patch[7])>=50
                     ||fabs(i_patch[1]-i_patch[6])>=50)
                {
                    {
                        feature_image.at<float>(Point(x,y)) = d_image.at<float>(Point(x,y));
                        count++;
                        continue;
                    }
                }
            }

            if(1)//depthe grad
            {
                float diffx=z_patch[10]-z_patch[14];
                float diffy=z_patch[2]-z_patch[22];
                if((fabs(diffx)>=0.05f*dis)||(fabs(diffy)>=0.05f*dis))
                {
                    feature_image.at<float>(Point(x,y)) = d_image.at<float>(Point(x,y));
                    count++;
                    continue;
                }
            }

            if(1)//edge finder
            {
                float gx[8],gy[8];
                //
                gx[0]= z_patch[28]-z_patch[29];
                gx[1]= z_patch[10]-z_patch[28];
                gx[2]= z_patch[11]-z_patch[10];
                gx[3]= z_patch[12]-z_patch[11];
                gx[4]= z_patch[13]-z_patch[12];
                gx[5]= z_patch[14]-z_patch[13];
                gx[6]= z_patch[32]-z_patch[14];
                gx[7]= z_patch[33]-z_patch[32];
                float gmaxx1=fabs(z_patch[12]-z_patch[29]);
                float gmaxx2=fabs(z_patch[33]-z_patch[12]);
                //assess pixel using Point(x,y)
                // ---------------> x (width)                     27              Patch
                // |                                              26
                // |      (0,0)  (1,0)  (2,0)  |           0   1   2   3   4
                // |                           |           5   6   7   8   9
                // |      (1,0)  (1,1)  (2,1)  |  29  28  10  11  12  13  14  32  33
                // |                           |          15  16  17  18  19
                // |      (2,0)  (1,2)  (2,2)  |          20  21  22  23  24
                // v                                              30
                // y(height)                                      31
                gy[0]= z_patch[26]-z_patch[27];
                gy[1]= z_patch[2]-z_patch[26];
                gy[2]= z_patch[7]-z_patch[2];
                gy[3]= z_patch[12]-z_patch[7];
                gy[4]= z_patch[17]-z_patch[12];
                gy[5]= z_patch[22]-z_patch[17];
                gy[6]= z_patch[30]-z_patch[22];
                gy[7]= z_patch[31]-z_patch[30];
                float gmaxy1=fabs(z_patch[12]-z_patch[27]);
                float gmaxy2=fabs(z_patch[31]-z_patch[12]);

                for(int i=0; i<8; i++)
                {
                    if ( gx[i]!=gx[i] || gy[i]!=gy[i])
                        continue;
                }
                if((gx[0]>0&&gx[1]>0&&gx[2]>0&&gx[3]>0&&gx[4]<0&&gx[5]<0&&gx[6]<0&&gx[7]<0
                    &&gmaxx1>0.01f*dis&&gmaxx2>0.01f*dis
                    &&gx[0]>gx[3]&&gx[7]<gx[4])
                        ||(gx[0]<0&&gx[1]<0&&gx[2]<0&&gx[3]<0&&gx[4]>0&&gx[5]>0&&gx[6]>0&&gx[7]>0
                           &&gmaxx1>0.01f*dis&&gmaxx2>0.01f*dis
                           &&gx[0]<gx[3]&&gx[7]>gx[4])
                        ||(gy[0]>0&&gy[1]>0&&gy[2]>0&&gy[3]>0&&gy[4]<0&&gy[5]<0&&gy[6]<0&&gy[7]<0
                           &&gmaxy1>0.01f*dis&&gmaxy2>0.01f*dis
                           &&gy[0]>gy[3]&&gy[7]<gy[4])
                        ||(gy[0]<0&&gy[1]<0&&gy[2]<0&&gy[3]<0&&gy[4]>0&&gy[5]>0&&gy[6]>0&&gy[7]>0
                           &&gmaxy1>0.01f*dis&&gmaxy2>0.01f*dis
                           &&gy[0]<gy[3]&&gy[7]>gy[4]))
                {
                    feature_image.at<float>(Point(x,y)) = d_image.at<float>(Point(x,y));
                    count++;
                    continue;
                }
            }
        }
    }

    prev_i_image.release();
    prev_i_image = i_image;

    CloudTPtr pc_undistort (new CloudT());
    CloudTPtr pc_sailent (new CloudT());
    buildUnorganizedPointCloudENU(d_image,i_image,new_camera_matrix,*pc_undistort);
    buildUnorganizedPointCloudENU(feature_image,i_image,new_camera_matrix,*pc_sailent);

    //    pcl::VoxelGrid<PointT> sor;

    //    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    //    sor.setInputCloud (pc_undistort);
    //    sor.filter (*pc_undistort);
    //    sor.setInputCloud (pc_sailent);
    //    sor.filter (*pc_sailent);

    //        pcl::StatisticalOutlierRemoval<PointT> sor2;
    //        sor2.setMeanK(30);
    //        sor2.setStddevMulThresh(1);
    //        sor2.setInputCloud (pc_sailent);
    //        sor2.filter (*pc_sailent);


    //publisth topics
    //ros::Time curr_stamp= ros::Time::now();
    ros::Time curr_stamp = d_raw_image->header.stamp;
    //image

    cv_bridge::CvImage d_result(d_raw_image->header,d_raw_image->encoding,d_image);
    pub_d.publish(d_result.toImageMsg());

    cv_bridge::CvImage i_result(i_raw_image->header,"mono8",i_image);
    pub_i.publish(i_result.toImageMsg());


    //undistort pc
    sensor_msgs::PointCloud2 pc2_msg;
    toROSMsg(*pc_undistort,pc2_msg);
    pc2_msg.header.frame_id="opti_link";
    pc2_msg.header.stamp = curr_stamp;
    pub_pc1.publish(pc2_msg);

    //sailent_pts
    sensor_msgs::PointCloud2 pc2_msg2;
    toROSMsg(*pc_sailent,pc2_msg2);
    pc2_msg2.header.frame_id="opti_link";
    pc2_msg2.header.stamp = curr_stamp;
    pub_pc2.publish(pc2_msg2);

    cout << "Frame: " << setw(6) << frame_cnt
         << " Sailent Pts: " << setw(6) << count
         <<" Cost Time: " << (ros::Time::now() - begin).toSec()*1000 << "ms" << endl;
    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Preprocessor");
    ros::NodeHandle nh;

    pub_d = nh.advertise<sensor_msgs::Image>    ("/undistorted_d_image", 5);
    pub_i = nh.advertise<sensor_msgs::Image>    ("/undistorted_i_image", 5);
    pub_pc1 = nh.advertise<sensor_msgs::PointCloud2> ("/vo_input_cloud", 5);
    pub_pc2 = nh.advertise<sensor_msgs::PointCloud2> ("/vo_input_sailent_cloud", 5);
    ros::Subscriber caminfo = nh.subscribe("/royale_camera_driver/camera_info", 1, getCameraInfo);

    ros::Rate rate(5);
    while(ros::ok())
    {
        ros::spinOnce();
        if(gotCameraInfo)
        {
            cout << "Got Camera Info:" << endl;
            cout << new_camera_matrix << endl;
            caminfo.shutdown();
            break;
        }
        rate.sleep();
    }

    message_filters::Subscriber<sensor_msgs::Image> depth_sub (nh, "/royale_camera_driver/depth_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> grey_sub  (nh, "/royale_camera_driver/gray_image", 1);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(depth_sub, grey_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
}


/**
 *  \brief Build the Pointcloud
 *  \param [in]depth_img_rect Input image
 *  \param [in]intensity_img  Input image
 *  \param [in]intr_rect_ir   Projection matrix
 *  \param [out]cloud Output cloud
*/
void buildUnorganizedPointCloudENU(const Mat& depth_img, const Mat& intensity_img, const Mat& intr_rect_ir, CloudT& cloud)
{
    int w = depth_img.cols;
    int h = depth_img.rows;

    double cx = intr_rect_ir.at<double>(0,2);
    double cy = intr_rect_ir.at<double>(1,2);
    double fx_inv = 1.0 / intr_rect_ir.at<double>(0,0);
    double fy_inv = 1.0 / intr_rect_ir.at<double>(1,1);

    cloud.clear();
    int count = 0;
    PointT pt;
    for (int u = 0; u < w; u++)
        for (int v = 0; v < h; v++)
        {
            float z = depth_img.at<float>(v, u);
            if(z!=0) {
                //                pt.x = (float)(z * ((u - cx) * fx_inv));
                //                pt.y = (float)(z * ((v - cy) * fy_inv));
                //                pt.z = z;
                //CAMERA TO ENU
                pt.y = -(float)(z * ((u - cx) * fx_inv));
                pt.z = -(float)(z * ((v - cy) * fy_inv));
                pt.x = z;
                pt.intensity = (float)intensity_img.at<unsigned char>(v,u);
                cloud.push_back(pt);
                count++;
            }
            else
            {
                continue;
            }

        }
    cloud.width = count;
    cloud.height = 1;
    cloud.is_dense = true;
}



void getCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    //    Mat P_matrix;
    Mat K_matrix;
    Mat R_matrix;
    Mat distCoeffs;
    K_matrix = (Mat1d(3, 3)  << cam_info->K.at(0),cam_info->K.at(1),cam_info->K.at(2),
                cam_info->K.at(3),cam_info->K.at(4),cam_info->K.at(5),
                cam_info->K.at(6),cam_info->K.at(7),cam_info->K.at(8));
    R_matrix = (Mat1d(3, 3)  << cam_info->R.at(0),cam_info->R.at(1),cam_info->R.at(2),
                cam_info->R.at(3),cam_info->R.at(4),cam_info->R.at(5),
                cam_info->R.at(6),cam_info->R.at(7),cam_info->R.at(8));
    distCoeffs = (Mat1d(5, 1) << cam_info->D.at(0),cam_info->D.at(1),cam_info->D.at(2),cam_info->D.at(3),cam_info->D.at(4));

    //https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
    image_size.height = cam_info->height;
    image_size.width = cam_info->width;
    cout << image_size << endl;
    cout << K_matrix << endl;
    cout << distCoeffs << endl;
    cout << R_matrix << endl;
    new_camera_matrix = getOptimalNewCameraMatrix(K_matrix, distCoeffs, image_size, 0, image_size);
    cout << "new_camera_matrix: " << new_camera_matrix << endl;
    cv::initUndistortRectifyMap(K_matrix,distCoeffs,R_matrix,new_camera_matrix,image_size,CV_32FC1,map1,map2);
    gotCameraInfo = 1;
}

