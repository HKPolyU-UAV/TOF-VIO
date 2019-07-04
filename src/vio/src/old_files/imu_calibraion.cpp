#include "imu_calibraion.h"
#include <eigen3/Eigen/Eigen>


imu_calibraion::imu_calibraion()
{
    sd_acc[0] = 0;
    sd_acc[1] = 0;
    sd_acc[2] = 0;
    sd_gyro[0] = 0;
    sd_gyro[1] = 0;
    sd_gyro[2] = 0;
}


void imu_calibraion:: get_parameter(float& sd_ax,float& sd_ay,float& sd_az, float& sd_wx,float& sd_wy,float& sd_wz)
{
    sd_ax = sd_acc[0];
    sd_ay = sd_acc[1];
    sd_az = sd_acc[2];
    sd_wx = sd_gyro[0];
    sd_wx = sd_gyro[1];
    sd_wx = sd_gyro[2];
}

int imu_calibraion::calibrate(float wx,float wy,float wz,float ax,float ay,float az)
{
#define NUM_SAMPLE_CALIBRATION (1000)
  static int count=0;
  static float imu_data_set[NUM_SAMPLE_CALIBRATION][6];

  if(count < NUM_SAMPLE_CALIBRATION)
  {
    imu_data_set[count][0] = ax;
    imu_data_set[count][1] = ay;
    imu_data_set[count][2] = az;
    imu_data_set[count][3] = wx;
    imu_data_set[count][4] = wy;
    imu_data_set[count][5] = wz;
    count++;
    cout << count << endl;
    return 0;
  }
  float sum[6]= {0,0,0,0,0,0};
  float avg[6]= {0,0,0,0,0,0};
  float sd[6]= {0,0,0,0,0,0};
  //sum
  for(int i =0; i<6; i++)
  {
    for (count=0;count<NUM_SAMPLE_CALIBRATION;count++)
    {
      sum[i] += imu_data_set[count][i];
    }
    avg[i] += sum[i]/((float)NUM_SAMPLE_CALIBRATION);
    for (count=0;count<NUM_SAMPLE_CALIBRATION;count++)
    {
      sd[i] += pow(imu_data_set[count][i] - avg[i], 2);
    }
    sd[i] = sqrt(sd[i]/((float)NUM_SAMPLE_CALIBRATION));
  }
  sd_acc[0] = sd[0];
  sd_acc[1] = sd[1];
  sd_acc[2] = sd[2];
  sd_gyro[0] = sd[3];
  sd_gyro[1] = sd[4];
  sd_gyro[2] = sd[5];
  cout << sd_acc[0] << "  " << sd_acc[1] << "   " << sd_acc[2] << endl;
  cout << sd_gyro[0] << "  " << sd_gyro[1] << "   " << sd_gyro[2] << endl;
  return 1;
}
