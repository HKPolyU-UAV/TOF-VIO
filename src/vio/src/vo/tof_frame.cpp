#include "vo/tof_frame.h"


TOF_Frame::TOF_Frame()
{
  this->T_cw.matrix()<< 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  this->cloud = CloudT::Ptr(new CloudT);
  this->sailent_cloud = CloudT::Ptr(new CloudT);
}

void TOF_Frame::clear()
{
  this->cloud->clear();
  this->sailent_cloud->clear();
  this->i_img.release();
  this->T_cw.matrix()<< 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  this->T_wc=T_cw.inverse();
  /*1,0,0,0,
      0,1,0,0,
      0,0,1,0,
      0,0,0,1;
    */
}


void TOF_Frame::read_PC_Iimg_FromROSMsg(const sensor_msgs::PointCloud2ConstPtr &pcPtr, const sensor_msgs::ImageConstPtr &mono8Ptr)
{
  cv_bridge::CvImagePtr cvbridge_image  = cv_bridge::toCvCopy(mono8Ptr,  mono8Ptr->encoding);
  Mat img=cvbridge_image->image;
  cv::equalizeHist(img,img);
  cv::GaussianBlur(img,this->i_img, Size(3,3), 0);
  size_t size = pcPtr->height * pcPtr->width;
  this->cloud = CloudT::Ptr(new CloudT);
  this->cloud->width=pcPtr->width;
  this->cloud->height=pcPtr->height;
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
    this->cloud->push_back(pt);
  }
}

void TOF_Frame::copy(TOF_Frame &from, TOF_Frame &to)
{
  to.cloud = CloudT::Ptr(new CloudT);
  to.sailent_cloud = CloudT::Ptr(new CloudT);
  to.i_img.release();
  pcl::copyPointCloud(*(from.cloud),*(to.cloud));
  pcl::copyPointCloud(*(from.sailent_cloud),*(to.sailent_cloud));
  to.i_img = from.i_img;
  to.T_cw = from.T_cw;
  to.T_wc = from.T_wc;
}
