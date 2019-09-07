#ifndef GY521_H
#define GY521_H
#include "mbed.h"

/*GY521を使う。
 *I2Cオブジェクトを渡す
 *GY521(I2C &i2c,int bit = 2,int calibration = 1000,double user_reg = 1.0);
 *example
 *L432KC : SDA = PB_7 , SCL = PB_6
 *F446RE : SDA = PB_3 , SCL = PB_10
 */
//I2C i2c(SDA,SCL);
//GY521 gyro(i2c);

enum GY521RegisterMap {
  WHO_AM_I = 0x75,
  PWR_MGMT_1 = 0x6B,
  LPF = 0x1A,
  FS_SEL = 0x1B,
  AFS_SEL = 0x1C,
  ACCEL_XOUT_H = 0x3B,
  ACCEL_YOUT_H = 0x3D,
  ACCEL_ZOUT_H = 0x3F,
  //TEMPERATURE  = 0x41,
  //GYRO_XOUT_H = 0x43,
  //GYRO_YOUT_H = 0x45,
  GYRO_ZOUT_H = 0x47
};

class GY521{
public:
    GY521(I2C &i2c,int bit = 2,int calibration = 1000,double user_reg = 1.0);
    double yaw;
    //double temp;
    void updata();
    void reset(int user);
    void start(double start = 0){
        yaw = start;
    }
    double checkStatus(int mode);
private:
    I2C &i2c_;
    Timer timer_;
    int16_t gyroRead2(enum GY521RegisterMap reg);
    double gyro_z_aver_;
    double gyro_z_now_;
    double gyro_z_prev_;
    double gyro_LSB_;
    double diff_yaw_;
    int bit_;
    int calibration_;
    bool flag_;
};

#endif /* GY521_H */