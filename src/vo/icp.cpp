#include <vo/icp.h>
#include <pcl/common/transforms.h>

using namespace pcl;

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

ICP_ALIGNMENT::ICP_ALIGNMENT(int pc_width_in,
                             int pc_height_in,
                             double fx_in,
                             double fy_in,
                             double cx_in,
                             double cy_in,
                             int max_loop_in)
{
    this->loop_count = 0;
    this->fx = fx_in;
    this->fy = fy_in;
    this->cx = cx_in;
    this->cy = cy_in;
    this->pc_height = pc_height_in;
    this->pc_width  = pc_width_in;
    this->max_loop = max_loop_in;
    cout << "width" << pc_width << "height" << pc_height << endl;

    src = CloudT();
    tgt = CloudT();
    sigma_dp=1.0;
}

void ICP_ALIGNMENT::reprojection(const PointT pt_c, int& u, int& v)
{
    u=int(floor(fx/pt_c.z * pt_c.x + cx));
    v=int(floor(fy/pt_c.z * pt_c.y + cy));
}

bool ICP_ALIGNMENT::icp_loops(int             times,
                              bool            output_info,
                              int             nns_radius,
                              double          sh_distance,
                              bool            use_robust_kernel,
                              int             &converged_flag,
                              double          &mean_dis_out,
                              int             &loop_count_out,
                              int             &inlier_count_out,
                              Eigen::Affine3d &T_ts_out)
{
    for(int i=0; i<times; i++)
    {
        this->icp_make_pairs(nns_radius);
        sort(pairs.begin(), pairs.end(), sortbythird);
        if(output_info)
        {
            double sum=0;
            if(pairs.size()>400)
            {
//                for(int j=0; j<100; j++)
//                {
//                    sum+=get<2>(pairs[j]);
//                }
                cout << get<2>(pairs[floor(pairs.size()/2)]) << ",";
            }else
            {
//                for(int j=0; j<pairs.size(); j++)
//                {
//                    sum+=get<2>(pairs[j]);
//                }
                cout << get<2>(pairs[floor(pairs.size()/2)]) << ",";
            }
        }

        this->icp_single_loop(use_robust_kernel,converged_flag,mean_dis_out);
        if(output_info)
        {
            //cout << "Loop:"<< loop_count << "mean_dis_error is " << mean_dis_out << endl;
            //cout << mean_dis_out << endl;
            //cout << mean_dis_out << ",";
        }
        if(converged_flag || mean_dis_out <= sh_distance)
        {
            T_ts_out = this->T_ts_est.cast<double>();
            loop_count_out = this->loop_count;
            inlier_count_out = this->inlier_count;
            //return true;
        }
    }
    return false;
}

void ICP_ALIGNMENT::alignment(const CloudTPtr src_in,
                              const CloudTPtr tgt_in,
                              bool            use_weight_factor,
                              Eigen::Affine3d T_ts_guess_in,
                              Eigen::Affine3d &T_ts_out,
                              double          &mean_dis_error,
                              int             &loop_count_out,
                              int             &inlier_count_out,
                              bool            output_info
                              )
{
    src.clear();
    tgt.clear();
    copyPointCloud(*src_in, src);
    copyPointCloud(*tgt_in, tgt);
    this->inlier_count = 0;
    this->loop_count = 0;

    this->T_ts_est=T_ts_guess_in.cast<float>();
    int converged = 0;

    if(icp_loops(1,output_info,21,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(1,output_info,19,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(1,output_info,17,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(1,output_info,15,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(1,output_info,13,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(1,output_info,11,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(2,output_info,10,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(4,output_info,9,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;
    if(icp_loops(3,output_info,9,0.005,use_weight_factor,converged,mean_dis_error,loop_count_out,inlier_count_out,T_ts_out)) return;

    T_ts_out = this->T_ts_est.cast<double>();
    loop_count_out = this->loop_count;
    inlier_count_out = this->inlier_count;
    if(output_info)
    {
        cout << endl;
    }
    return;
}

void ICP_ALIGNMENT::getAngleandTrans(Eigen::Affine3d tf, Vector3d& angle, Vector3d& trans)
{
    trans = tf.translation();
    angle = euler_from_rotation_matrix(tf.linear());
}


void ICP_ALIGNMENT::icp_make_pairs(int nns_radius)
{
    pairs.clear();
    int umax = pc_width-1;
    int vmax = pc_height-1;
    int umin,vmin;
    umin=vmin=0;
    //  cout << "src cloud size " << src.size() << endl;
    //  cout << "tgt cloud size " << tgt.size() << endl;
    for(int i=0; i<src.size(); i++){
        PointT pt_in_src=src.at(i);
        PointT pt = transformPoint(src.at(i),T_ts_est);

        int    u,v;
        this->reprojection(pt,u,v);
        if(u<0 || u>umax || v<0 || v>vmax){continue;}//search point inside of boundary

        //search range
        int su_min=floor(u-nns_radius);
        int su_max=floor(u+nns_radius);
        if(su_min<=umin) su_min = umin;
        if(su_max>=umax) su_max = umax;
        int sv_min=floor(v-nns_radius);
        int sv_max=floor(v+nns_radius);
        if(sv_min<=vmin) sv_min = vmin;
        if(sv_max>=vmax) sv_max = vmax;

        int found=0;
        int step = floor(nns_radius/10)+1;
        double min_sqrdis = 999.0;

        PointT pt_nn;//neareat neiborhood
        for(int su = su_min+(rand()%step); su<=su_max; su+=step)
        {
            for(int sv = sv_min+(rand()%step); sv<=sv_max; sv+=step)
            {
                PointT pt_serach = this->tgt.at(su+sv*pc_width);
                if(pt_serach.z != pt_serach.z) {continue;}//invalid pts
                if(sqrt((su-u)*(su-u)+(sv-v)*(sv-v)) > nns_radius) {continue;}//out of search range
                double squdis_p = (pt_serach.x-pt.x)*(pt_serach.x-pt.x)
                        +(pt_serach.y-pt.y)*(pt_serach.y-pt.y)
                        +(pt_serach.z-pt.z)*(pt_serach.z-pt.z);
                if(squdis_p<min_sqrdis){
                    min_sqrdis = squdis_p;
                    pt_nn=pt_serach;
                    found=1;
                }
            }
        }
        if(found){
            std::tuple<PointT, PointT, float, float, float> pair;
            pair = std::make_tuple(pt,pt_nn,sqrt(min_sqrdis),pt.intensity-pt_nn.intensity,0);
            pairs.push_back(pair);
        }else {
            std::tuple<PointT, PointT, float, float, float> pair;
            pair = std::make_tuple(pt,pt,0,0,0);
            //pairs.push_back(pair);
        }
    }//

}

void ICP_ALIGNMENT::icp_single_loop(bool   use_robust_kernel,
                                    int    &converged,
                                    double &mean_dis
                                    )
{
    this->loop_count++;

    //Robust Weight Kernel: Calculation of sigma_dp
    for(int count=0; count<5; count++) {
        double sum=0;
        double sqrsigma_dp = sigma_dp*sigma_dp;
        for(size_t i=0; i<pairs.size();i++)
        {
            double sqdis = get<2>(pairs[i])*get<2>(pairs[i]);
            sum += sqdis*5.0/(4.0+(sqdis/sqrsigma_dp));
        }
        sigma_dp = sqrt(sum/pairs.size());
    }
    //here we use position error to creat error matrix
    for (size_t i=0; i<pairs.size();i++){
        double sqdis_p = get<2>(pairs[i])*get<2>(pairs[i]);
        double weight_p = 1;
        if(use_robust_kernel)
        {
            weight_p = 5.0/(4.0+ sqdis_p / (sigma_dp*sigma_dp));
        }
        get<4>(pairs[i]) = weight_p;
    }

    sort(pairs.begin(), pairs.end(), sortbythird);
    mean_dp=get<2>(pairs[floor(pairs.size()/2)]);
    mean_dis = mean_dp;

    //Reject and Remove from source
    //cout << endl << "remove outlier from pairs " << pairs.size() << " to ";

    if(!use_robust_kernel)
    {
        int cut_off=pairs.size()-1;
        if(get<2>(pairs[cut_off]) > 0.02)
        {
            int cut_off = floor(pairs.size()*0.8);
            //pairs.erase(pairs.begin() + cut_off, pairs.end());
        }
    }
    //cout << pairs.size() << endl;
    this->inlier_count =  pairs.size();

    this->src.clear();
    Eigen::Affine3f T_ts_est_inv = T_ts_est.inverse();
    for(int i=0; i<pairs.size(); i++)
    {
        this->src.push_back(transformPoint(get<0>(pairs[i]),T_ts_est_inv));
    }


    //3D-3D Alignment
    pcl::TransformationFromCorrespondences icp_trans;
    icp_trans.reset();
    for (size_t i = 0; i<pairs.size(); i++)
    {
        PointT pt_src, pt_tgt;
        pt_src = get<0>(pairs[i]);
        pt_tgt = get<1>(pairs[i]);
        Vector3f source(pt_src.x,pt_src.y,pt_src.z);
        Vector3f target(pt_tgt.x,pt_tgt.y,pt_tgt.z);
        icp_trans.add(source, target, get<4>(pairs[i]));
    }
    Eigen::Affine3f increTrans= icp_trans.getTransformation();
    //cout <<"increTrans:" << endl << increTrans.matrix() << endl;


    T_ts_est = increTrans*T_ts_est;

    //check whether it has converged
    Vec3 euler,trans;
    getAngleandTrans(increTrans.cast<double>(),euler,trans);
    double sum_euler = fabs(euler(0))+fabs(euler(1))+fabs(euler(2));
    double sum_trans = fabs(trans(0))+fabs(trans(1))+fabs(trans(2));
    if(sum_euler<0.0005 && sum_trans<0.0001)
    {
        //        cout << "converge with small change in increTrans with "
        //             << "angle: " << sum_euler << " translation: " << sum_trans << endl;
        converged = 1;
        return;
    }

}
//tt_icploop.toc("One icp loop");
