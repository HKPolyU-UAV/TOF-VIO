#ifndef ICP_EST_H
#define ICP_EST_H


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <tuple>
#include "common.h"
#include <utils/euler_q_rmatrix.h>

using namespace std;

class ICP_EST
{
public:
    ICP_EST()
    {
        numPts = 600;
        icp_dis_threshold = 0.001;
        icp_max_loopCount = 30;
        pridict_pose.setIdentity();
    }
    void set_est_icp_para(int num_icp_pts=600, float icp_dis_threshold=0.001, int max_int_num=30)
    {
        numPts = num_icp_pts;
        icp_dis_threshold = icp_dis_threshold;
        icp_max_loopCount = max_int_num;
    }
    void setSource(CloudTPtr &source){
        std::vector<int> indices;
        pc_source.reset(new CloudT);
        pcl::copyPointCloud(*source,*pc_source);
    }
    void setTarget(CloudTPtr &target){
        std::vector<int> indices;
        pc_target.reset(new CloudT);
        pcl::copyPointCloud(*target,*pc_target);
    }
    void setThreshold(float threshold){
        icp_dis_threshold = threshold;
    }
    Affine3f getTansfromation(){
        return src2tgt_trans;
    }
    CloudTPtr get_sailent_src()
    {
        return sailent_src;
    }

    float integrate(Vector3f predict_tans=Vector3f(0.0,0.0,0.0),
                    Quaternionf predict_rot=Quaternionf(1.0,0.0,0.0,0.0));

private:

    CloudTPtr pc_source;
    CloudTPtr pc_target;

    Quaternionf rot_source;
    Quaternionf rot_target;

    CloudTPtr sailent_src;

    pcl::KdTreeFLANN<PointT> kdtree;
    CloudTPtr inliers;
    //ICP pairs <inlier, targetpt, distance>
    std::vector< std::tuple<PointT, PointT, float> > pairs;

    int numPts;//number of points for icp allignment
    float icp_dis_threshold;
    int icp_max_loopCount;
    int loop_count;
    int no_motion;

    float icp_avg_dis;
    Vector3f    icp_trans;
    Quaternionf icp_rot_q;

    Affine3f  src2tgt_trans;
    Affine3f  pridict_pose;

    void select_sailent_pts();
    void intLoop(int num_of_pts, int num_of_loop);
    void predict();

};


#endif // ICP_EST_H
