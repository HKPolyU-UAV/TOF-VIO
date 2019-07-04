#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "common.h"
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

    addItem(CloudTPtr& keyCloud, Eigen::Affine3d& key_pose);
    searchCloestKeyFrame(Eigen::Affine3d curr_pose, CloudTPtr& keyCloud, Eigen::Affine3d& key_pose);

private:
    int warehouseSize;
    std::vector<KeyFrameItem> warehouse;
};

#endif // KEYFRAME_H
