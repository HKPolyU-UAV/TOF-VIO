#ifndef SALIENTPTS_H
#define SALIENTPTS_H

#include <common.h>
#include <opencv2/opencv.hpp>
#include <include/tic_toc_ros.h>

using namespace cv;
using namespace std;

class SalientPts
{
    int pc_width;
    int pc_height;
    int use_canny;
    int use_depth_grad;
    int use_intensity_grad;
    int use_edge_detector;
    float sh_depth;
    float sh_intensity;
    float sh_background;
    float sh_edge;
    int min_pts_num;


public:

    SalientPts(int pc_width_in,
               int pc_height_in,
               int min_pts_num_in,
               int canny_switch,
               int depth_grad_switch,
               int intensity_grad_switch,
               int edge_detector_switch,
               float sh_depth_in,
               float sh_intensity_in,
               float sh_background_in,
               float sh_edge_in);
    void select_salient_from_pc(CloudTPtr& salient_pc, CloudTPtr pc, const Mat& i_img );
    void select_random_from_pc(CloudTPtr& salient_pc, CloudTPtr pc, const Mat& i_img);
};

#endif // SALIENTPTS_H
