#ifndef IMU_CALIBRAION_H
#define IMU_CALIBRAION_H


class imu_calibraion
{
public:
    imu_calibraion();
private:

    float sd_acc[3];
    float sd_gyro[3];
};

#endif // IMU_CALIBRAION_H
