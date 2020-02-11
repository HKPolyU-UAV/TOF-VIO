#include "vo/salientpts.h"



SalientPts::SalientPts(int pc_width_in,
                       int pc_height_in,
                       int min_pts_num_in,
                       int canny_switch,
                       int depth_grad_switch,
                       int intensity_grad_switch,
                       int edge_detector_switch,
                       float sh_depth_in,
                       float sh_intensity_in,
                       float sh_background_in,
                       float sh_edge_in)
{
  pc_width = pc_width_in;
  pc_height = pc_height_in;

  min_pts_num = min_pts_num_in;

  use_canny = canny_switch;
  use_depth_grad = depth_grad_switch;
  use_intensity_grad = intensity_grad_switch;
  use_edge_detector = edge_detector_switch;

  sh_depth = sh_depth_in;
  sh_intensity = sh_intensity_in;
  sh_background= sh_background_in;
  sh_edge = sh_edge_in;
}

void SalientPts::select_random_from_pc(CloudTPtr &salient_pc, CloudTPtr pc, const Mat &i_img)
{
    salient_pc->clear();
    int sailentpts_cnt = 0;
    for(int sample_count=0; (sailentpts_cnt<min_pts_num && sample_count<5000) ; sample_count++)//sample 5000
    {
      int u=rand()%(pc_width-6)+3;
      int v=rand()%(pc_height-6)+3;
      PointT pt = pc->at(u+v*pc_width);
      if(pt.z==pt.z)//valid pts
      {
        sailentpts_cnt++;
        salient_pc->push_back(pt);
        continue;
      }
    }
}

void SalientPts::select_salient_from_pc(CloudTPtr& salient_pc, CloudTPtr pc, const Mat& i_img )
{
  salient_pc->clear();
  int sailentpts_cnt = 0;
  int inputpts_cnt = 0;
  Mat i_canny_dst;
  if(use_canny)
  {
    Canny( i_img, i_canny_dst , 230, 300, 3 );
  }
  for(int u=10; u<(pc_width-10); u++)// u v range = 90% full range
  {
    for(int v=10; v<(pc_height-10); v++)//
    {
      PointT pt = pc->at(u+v*pc_width);
      if(pt.z==pt.z)//valid pts
      {
        inputpts_cnt++;
        //reject rules
        //background rejection
        float dis = pt.z;
        float diff1=dis - pc->at((u+8)+v*pc_width).z;
        float diff2=dis - pc->at((u-8)+v*pc_width).z;
        float diff3=dis - pc->at(u+(v+8)*pc_width).z;
        float diff4=dis - pc->at(u+(v-8)*pc_width).z;
        float diff5=dis - pc->at((u+4)+v*pc_width).z;
        float diff6=dis - pc->at((u-4)+v*pc_width).z;
        float diff7=dis - pc->at(u+(v+4)*pc_width).z;
        float diff8=dis - pc->at(u+(v-4)*pc_width).z;
        float thres= sh_background*dis;
        if ( diff1>thres && diff1==diff1){continue;}
        if ( diff2>thres && diff2==diff2){continue;}
        if ( diff3>thres && diff3==diff3){continue;}
        if ( diff4>thres && diff4==diff4){continue;}
        if ( diff5>thres && diff5==diff5){continue;}
        if ( diff6>thres && diff6==diff6){continue;}
        if ( diff7>thres && diff7==diff7){continue;}
        if ( diff8>thres && diff8==diff8){continue;}

        //accept rules
        if(use_canny)
        {
          int idx=v*pc_width+u;
          if((i_canny_dst.at<uint8_t>(idx)) == 255)
          {
            sailentpts_cnt++;
            salient_pc->push_back(pt);
            continue;
          }
        }
        //depth gradient
        if(use_depth_grad){
          float fabs_d_gradx = fabs(pc->at((u-8)+v*pc_width).z - pc->at((u+8)+v*pc_width).z);
          float fabs_d_grady = fabs(pc->at(u+(v-8)*pc_width).z - pc->at(u+(v+8)*pc_width).z);
          if(fabs_d_gradx>(sh_depth*dis) || fabs_d_grady>(sh_depth*dis)){
            sailentpts_cnt++;
            salient_pc->push_back(pt);
            continue;
          }
        }
        //intensity gragradient
        if(use_intensity_grad){
          float fabs_g_gradx = fabs(pc->at((u-4)+v*pc_width).intensity - pc->at((u+4)+v*pc_width).intensity);
          float fabs_g_grady = fabs(pc->at(u+(v-4)*pc_width).intensity - pc->at(u+(v+4)*pc_width).intensity);
          if(fabs_g_gradx>sh_intensity || fabs_g_grady>sh_intensity){
            salient_pc->push_back(pt);
            sailentpts_cnt++;
            continue;
          }
        }
        //edge_detector
        if(use_edge_detector){
          float gx[6],gy[6],gmaxx[2],gmaxy[2];
          gx[0]= pc->at((u-3)+v*pc_width).z - pc->at((u-2)+v*pc_width).z;
          gx[1]= pc->at((u-2)+v*pc_width).z - pc->at((u-1)+v*pc_width).z;
          gx[2]= pc->at((u-1)+v*pc_width).z - pc->at((u)+v*pc_width).z;
          gx[3]= pc->at((u)+v*pc_width).z   - pc->at((u+1)+v*pc_width).z;
          gx[4]= pc->at((u+1)+v*pc_width).z - pc->at((u+2)+v*pc_width).z;
          gx[5]= pc->at((u+2)+v*pc_width).z - pc->at((u+3)+v*pc_width).z;

          gy[0]= pc->at(u+(v-3)*pc_width).z - pc->at(u+(v-2)*pc_width).z;
          gy[1]= pc->at(u+(v-2)*pc_width).z - pc->at(u+(v-1)*pc_width).z;
          gy[2]= pc->at(u+(v-1)*pc_width).z - pc->at(u+(v)*pc_width).z;
          gy[3]= pc->at(u+(v)*pc_width).z   - pc->at(u+(v+1)*pc_width).z;
          gy[4]= pc->at(u+(v+1)*pc_width).z - pc->at(u+(v+2)*pc_width).z;
          gy[5]= pc->at(u+(v+2)*pc_width).z - pc->at(u+(v+3)*pc_width).z;

          gmaxx[0]=fabs( pc->at((u-3)+v*pc_width).z-pc->at((u)+v*pc_width).z);
          gmaxx[1]=fabs( pc->at((u)+v*pc_width).z-pc->at((u+3)+v*pc_width).z);

          gmaxy[0]=fabs( pc->at(u+(v-3)*pc_width).z-pc->at(u+(v)*pc_width).z);
          gmaxy[1]=fabs( pc->at(u+(v)*pc_width).z-pc->at(u+(v+3)*pc_width).z);


          if((gx[0]>0 && gx[1]>0 && gx[2]>0 && gx[3]<0 && gx[4]<0 && gx[5]<0 && (gmaxx[0]>sh_edge*dis || gmaxx[1]>sh_edge*dis))
             ||(gx[0]<0 && gx[1]<0 && gx[2]<0 && gx[3]>0 && gx[4]>0 && gx[5]>0 && (gmaxx[0]>sh_edge*dis || gmaxx[1]>sh_edge*dis))
             ||(gy[0]>0 && gy[1]>0 && gy[2]>0 && gy[3]<0 && gy[4]<0 && gy[5]<0 && (gmaxy[0]>sh_edge*dis || gmaxy[1]>sh_edge*dis))
             ||(gy[0]<0 && gy[1]<0 && gy[2]<0 && gy[3]>0 && gy[4]>0 && gy[5]>0 && (gmaxy[0]>sh_edge*dis || gmaxy[1]>sh_edge*dis)))
          {
            salient_pc->push_back(pt);
            sailentpts_cnt++;
            continue;
          }
        }
      }//valid pts
    }
  }
  //cout << inputpts_cnt << ","<< sailentpts_cnt << ",";
  //cout << sailentpts_cnt << endl;
  if(sailentpts_cnt < min_pts_num)
  {
    //cout << sailentpts_cnt << endl;
    //cout << "sailentpts_cnt "<< sailentpts_cnt << " less than " << min_pts_num  << ", add random pts to ";
    for(int sample_count=0; (sailentpts_cnt<min_pts_num && sample_count<5000) ; sample_count++)//sample 5000
    {
      int u=rand()%(pc_width-6)+3;
      int v=rand()%(pc_height-6)+3;
      PointT pt = pc->at(u+v*pc_width);
      if(pt.z==pt.z)//valid pts
      {
        sailentpts_cnt++;
        salient_pc->push_back(pt);
        continue;
      }
    }
     //cout << sailentpts_cnt << endl;
  }
}
