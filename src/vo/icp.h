#ifndef ICP_H
#define ICP_H

#include <include/common.h>
#include <include/euler_q_rmatrix.h>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>


using namespace Eigen;
using namespace pcl;
using namespace std;

class ICP_ALIGNMENT
{
    //align the src to tgt
    CloudT src;
    CloudT tgt;
    int loop_count;
    int max_loop;
    int inlier_count;
    Eigen::Affine3f T_ts_est; //transformation for src to tgt;
    std::vector<std::tuple<PointT, PointT, float, float, float>> pairs;
    //from/to/dis/intensity_dis/weight
    double fx,fy,cx,cy;//camera parameters
    int pc_width;
    int pc_height;

    double mean_dp;
    double sigma_dp;
    //    double mean_di;
    //    double sigma_di;

public:


    ICP_ALIGNMENT(int pc_width_in,
                  int pc_height_in,
                  double fx_in=0,
                  double fy_in=0,
                  double cx_in=0,
                  double cy_in=0,
                  int max_loop_in=50
            );

    //workflow of ICP
    void alignment(const CloudTPtr src_in,
                   const CloudTPtr tgt_in,
                   bool            use_weight_factor,
                   Eigen::Affine3d T_ts_guess_in,
                   Eigen::Affine3d &T_ts_out,
                   double          &mean_dis_error,
                   int             &loop_count_out,
                   int             &inlier_count_out,
                   bool            output_info
                   );
    static void getAngleandTrans(Eigen::Affine3d tf,
                                 Vector3d& angle,
                                 Vector3d& trans);


private:
    //single ICP loop
    void icp_single_loop(bool   use_robust_kernel,
                         int    &converged,
                         double &mean_dis
                         );
    void icp_make_pairs(int nns_radius);
    bool icp_loops(int             times,
                   bool            output_info,
                   int             nns_radius,
                   double          sh_distance,
                   bool            use_robust_kernel,
                   int             &converged_flag,
                   double          &mean_dis_out,
                   int             &loop_count_out,
                   int             &inlier_count_out,
                   Eigen::Affine3d &T_ts_out);


    void reprojection(const PointT pt_c, int& u, int& v);
    void remove_motion_induced_outlier();

};

#endif // ICP_H
