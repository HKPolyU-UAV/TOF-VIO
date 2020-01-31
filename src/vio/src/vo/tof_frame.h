#ifndef TOF_FRAME_H
#define TOF_FRAME_H

#include "include/common.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

using namespace cv;

class TOF_Frame
{
public:
  typedef std::shared_ptr<TOF_Frame> Ptr;

  Eigen::Affine3d T_cw;
  Eigen::Affine3d T_wc;
  CloudTPtr       cloud;
  CloudTPtr       sailent_cloud;
  Mat             i_img;

  TOF_Frame();

  void clear();
  void read_PC_Iimg_FromROSMsg(const sensor_msgs::PointCloud2ConstPtr & pcPtr,
                               const sensor_msgs::ImageConstPtr & mono8Ptr);
  static void copy(TOF_Frame &frome, TOF_Frame &to);

};



#endif // TOF_FRAME_H
