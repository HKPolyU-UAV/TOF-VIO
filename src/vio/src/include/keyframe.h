#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "common.h"
#include "include/common.h"
//store 30 key frames in the keyFrame Store

struct KeyFrameItem{
  CloudTPtr Cloud;
  double pose;
};

class KeyFrameStore
{
public:
    KeyFrameStore()
    {
        warehouseSize=30;
    }

    addItem(CloudTPtr& P_key, Eigen::Affine3d& T_cw_key);
    searchCloestKeyFrame(Eigen::Affine3d T_cw_curr, CloudTPtr& P_key, Eigen::Affine3d& T_cw_key);

private:
    int warehouseSize;
    std::vector<KeyFrameItem> warehouse;
};

#endif // KEYFRAME_H
