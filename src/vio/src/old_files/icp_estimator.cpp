#include "icp_estimator.h"
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <tuple>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <ros/time.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

using namespace std;

bool sortbythird(const tuple<PointT, PointT, float>& a,
                 const tuple<PointT, PointT, float>& b)
{
    return (get<2>(a) < get<2>(b));
}


void ICP_EST::intLoop(int num_of_pts, int num_of_loop)
{
    //loop
    //Sample pc and creat initial inliers cloud

    for (size_t count=0; count<num_of_loop; count++)
    {
        inliers.reset(new CloudT);
        for(int i=0; i< num_of_pts; i++)
        {
            int rand_num = rand() % ((int)sailent_src->size());
            PointT pt = sailent_src->points[rand_num];
            PointT pt_trans=pcl::transformPoint(pt, src2tgt_trans);   //warped source point
            //transfomed
            inliers->points.push_back(pt_trans);
        }
        //search nearest points of inliers and creat matching pair
        pairs.clear();
        PointT sp;
        for (size_t i = 0; i< inliers->size(); i++)
        {
            sp = inliers->points[i];
            float sp_intensity = sp.intensity;
            vector<int> pointIdx(4);
            vector<float> SquaredDis(4);
            if ( kdtree.nearestKSearch (sp, 4, pointIdx, SquaredDis) > 0 )
            {
                vector<float> w_dis(4);
                for(int j=0; j<4; j++)
                {
                    float intensity_diff = sp_intensity-pc_target->points[pointIdx[j]].intensity;
                    w_dis[j]=SquaredDis[j]*intensity_diff*intensity_diff;
                }
                int min_idx = std::min_element(w_dis.begin(),w_dis.end()) - w_dis.begin();
                std::tuple<PointT, PointT, float> pair;
                pair = std::make_tuple(inliers->points[i],pc_target->points[pointIdx[min_idx]],w_dis[min_idx]);
                pairs.push_back(pair);
            }
        }
        sort(pairs.begin(), pairs.end(), sortbythird);

        float mean_distance=0;
        for (size_t i=0; i<pairs.size();i++)
        {
            mean_distance += get<2>(pairs[i]);
        }
        mean_distance/=(float)(pairs.size());
        icp_avg_dis = mean_distance;
        if(loop_count==0 && icp_avg_dis<=0.00008)//non-motion
        {
            //src2tgt_trans.setIdentity();
            no_motion = 1;
            return ;
        }
        if(icp_avg_dis<icp_dis_threshold)//check if distance small than thredhold
        {
            return ;
        }

        //remove outlier from pair
        float variance = 0;
        for(size_t i=0; i<pairs.size();i++)
        {
            variance += pow((get<2>(pairs[i]) - mean_distance), 2);
        }
        variance = variance/(float)(pairs.size()-1);
        float stdDeviation = sqrt(variance);

        sort(pairs.begin(), pairs.end(), sortbythird);
        size_t cut_off;
        for(cut_off=0; cut_off<pairs.size(); cut_off++)
        {
            if(get<2>(pairs[cut_off]) > mean_distance + stdDeviation)
                break;
        }
        pairs.erase(pairs.begin() + cut_off, pairs.end());

        //estimate the transformation
        pcl::TransformationFromCorrespondences trans;
        for (size_t i = 0; i<pairs.size(); i++)
        {
            PointT pt_inlier, pt_target;
            pt_inlier = get<0>(pairs[i]);
            pt_target = get<1>(pairs[i]);
            Vector3f source(pt_inlier.x,pt_inlier.y,pt_inlier.z);
            Vector3f target(pt_target.x,pt_target.y,pt_target.z);
            trans.add(source, target);
        }
        Affine3f increTrans= trans.getTransformation();
        src2tgt_trans = increTrans*src2tgt_trans;
        loop_count ++;
    }

}


float ICP_EST::integrate(Vector3f predict_tans, Quaternionf predict_rot)
{
    icp_rot_q = predict_rot;
    icp_trans = predict_tans;

    src2tgt_trans.setIdentity();
    src2tgt_trans.translation() = predict_tans;
    src2tgt_trans.rotate(predict_rot);
    //create kd-tree for target frame(keyFrame)
    kdtree.setInputCloud(pc_target);

    //sample sailentpc from source(down sample)
    sailent_src.reset(new CloudT());
    int src_size = pc_source->size();
    for(int ptidx=0; ptidx<500; ptidx++)
    {
        int idx= rand()%src_size;
        sailent_src->push_back(pc_source->at(idx));
    }
    loop_count = 0;
    no_motion = 0;
    intLoop(250, 50);
    if(no_motion)
    {
        cout << "No motion" << endl;
    }else
    {
        cout << "Ili_Pts: " << inliers->size() <<" Loop: " << loop_count << " Avgdis:" << icp_avg_dis<< endl;
    }

    return icp_avg_dis;
}


