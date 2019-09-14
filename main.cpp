#include "mbed.h"
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "library/scrp_master.hpp"
#include "library/rotary_inc.hpp"
#include "library/gy521.hpp"

#define Motor_NUM 4

constexpr int PERIOD = 64;
constexpr double MAXPWM = 0.98;

constexpr double PI2_3 = M_PI * 2 / 3;
constexpr double PI_3  = M_PI / 3;
constexpr double PI_180 = M_PI / 180;
constexpr double PI_4 = M_PI / 4;
constexpr double Motor_L = 857;
constexpr double Rotary_L = 610;

double kp = 0.0003; //モーター
double ki = 0.005;
double kd = 0.0000007;

double Kp = 1.6; //自動移動
double Ki = 2.2;
double Kd = 0.001;

double tKp = 10.0;//回転
double tKi = 0.8;
double tKd = 20;

constexpr PinName pwm_pin[7][3] = {
    {PB_1 ,PA_11,PC_6 },
    {PB_13,PB_14,PB_2 },
    {PB_4 ,PB_5 ,PB_15},
    {PC_8 ,PC_9 ,PA_10},
    {PB_6 ,PB_7 ,PB_12},
    {PB_8 ,PB_9 ,PC_7 },
    {PA_0 ,PA_1 ,PB_0 }
};

constexpr PinName rotary_pin[8][2] = {
    {PC_10,PC_11},
    {PC_4 ,PA_13},
    {PA_14,PA_15},
    {PC_2 ,PC_3 },
    {PA_12,PC_5 },
    {PC_0 ,PC_1 },
    {PA_6 ,PA_7 },
    {PA_8 ,PA_9 }
};

//ScrpMaster master(PC_12,PD_2,PH_1);

DigitalOut led(PA_5);
DigitalIn button(PC_13);
DigitalIn emergency(PA_4);//非常停止を読む
Timer motortimer;

PwmOut* Motor[Motor_NUM][2];
DigitalOut* Led[Motor_NUM];
RotaryInc* Speed[Motor_NUM];
RotaryInc* Place[Motor_NUM];
DigitalIn* Limit[2][2];
GY521 *gy;
bool Ok = false;
double X = 0,Y = 0,T = 0,Yaw = 0;//オドメトリ
double Vx = 0,Vy = 0,Omega = 0;//速度
double driveS[Motor_NUM],nowV[Motor_NUM];
double goal_x,goal_y,goal_yaw;
double VMAX = 1200,AMAX = 300;

bool ablemove = true;
bool automove = false;
int limit_move = -1;
Timer autotimer;

ros::NodeHandle nh;
/*
void sendSerial(const std_msgs::Int32 &msg){
    uint8_t id = (msg.data >> 24) & 0xff;
    uint8_t cmd = (msg.data >> 16) & 0xff;
    int16_t data = (msg.data) & 0xffff;
    master.send(id,cmd,data);
}*/

void safe(){
    Vx = 0;
    Vy = 0;
    Omega = 0;
    automove = false;
    ablemove = true;
    for(int i = 0;i < Motor_NUM;i++){
        Motor[i][0]->write(0);
        Motor[i][1]->write(0);
        Led[i]->write(0);
    }
}

void Reset(){
    if(!Ok){
        Ok = true;
    }else{
        led.write(0);
        safe();
        gy->reset(0);
        X = 0;
        Y = 0;
        T = 0;
        led.write(1);
    }
}    

inline bool Drive(int id,double pwm){//モーターを回す
    pwm = constrain(pwm,-MAXPWM,MAXPWM);
    if(!pwm){
        Motor[id][0]->write(0);
        Motor[id][1]->write(0);
        Led[id]->write(0);
    }else if(0 < pwm){
        Motor[id][0]->write(pwm);
        Motor[id][1]->write(0);
        Led[id]->write(1);
    }else{
        Motor[id][0]->write(0);
        Motor[id][1]->write(-pwm);
        Led[id]->write(1);
    }
    return true;
}

inline void move(){
    static double diff[Motor_NUM],errer[Motor_NUM],diffV[Motor_NUM],lastV[Motor_NUM],driveV[Motor_NUM],now_t,cos_yaw,sin_yaw;
    static int j;
#if Motor_NUM == 3
    driveV[0] = Vx*cos(Yaw)         + Vy*sin(Yaw)         + Omega*Motor_L;
    driveV[1] = Vx*cos(Yaw + PI2_3) + Vy*sin(Yaw + PI2_3) + Omega*Motor_L;
    driveV[2] = Vx*cos(Yaw - PI2_3) + Vy*sin(Yaw - PI2_3) + Omega*Motor_L;
#elif Motor_NUM == 4
    cos_yaw = cos(Yaw+PI_4);
    sin_yaw = sin(Yaw+PI_4);
    driveV[0] =  Vx*cos_yaw + Vy*sin_yaw + Omega*Motor_L;
    driveV[1] = -Vx*sin_yaw + Vy*cos_yaw + Omega*Motor_L;
    driveV[2] = -Vx*cos_yaw - Vy*sin_yaw + Omega*Motor_L;
    driveV[3] =  Vx*sin_yaw - Vy*cos_yaw + Omega*Motor_L;
#endif
    now_t = motortimer.read();
    motortimer.reset();
    for(j = 0;j < Motor_NUM;j++){
        nowV[j] = Speed[j]->getSpeed();
        diff[j] = driveV[j] - nowV[j];
        if(nowV[j] == 0 && driveV[j] == 0 && errer[j] != 0){
            errer[j] = 0;
        }else{
            errer[j] += diff[j] * now_t;
        }
        diffV[j] = (nowV[j] - lastV[j]) / now_t;
        lastV[j] = nowV[j];
        driveS[j] = 0.0004 * driveV[j] + diff[j] * kp + errer[j] * ki - diffV[j] * kd;
        Drive(j,-driveS[j]);
    }
}

void getData(const std_msgs::Float32MultiArray &msgs){
    if(!Ok && (int)msgs.data[0] != -1)return;
    switch((int)msgs.data[0]){
        case -2:
            safe();
            break;
        case -1:
            ablemove = true;
            Reset();
            break;
        case 0:
            ablemove = true;
            automove = false;
            Vx = msgs.data[1];
            Vy = msgs.data[2];
            Omega = msgs.data[3];
            break;
        case 1:
            ablemove = true;
            goal_x = msgs.data[1];
            goal_y = msgs.data[2];
            goal_yaw = msgs.data[3];
            if(!automove){
                automove = true;
                autotimer.reset();
                autotimer.start();
            }
            break;
        case 2:
            ablemove = false;
            automove = false;
            Drive(0,msgs.data[1]/255);
            Drive(1,msgs.data[2]/255);
            break;
        case 3:
            ablemove = false;
            automove = false;
            Drive(2,msgs.data[1]/255);
            Drive(3,msgs.data[2]/255);
            break;
        case 4:
            led.write(0);
            safe();
            X = msgs.data[1];
            Y = msgs.data[2];
            T = (double)msgs.data[3]/180.0*M_PI;
            gy->reset(msgs.data[3]);
            led.write(1);
            break;
        case 5:
            limit_move = (int)msgs.data[1];
            VMAX = msgs.data[2];
            AMAX = msgs.data[3];
            break;
        case 6://移動PID
            Kp = msgs.data[1];
            Ki = msgs.data[2];
            Kd = msgs.data[3];
            break;
        case 7://回転PID
            tKp = msgs.data[1];
            tKi = msgs.data[2];
            tKd = msgs.data[3];
            break;
        case 8://モーターPID
            kp = msgs.data[1];
            ki = msgs.data[2];
            kd = msgs.data[3];
            break;
    }
}

std_msgs::Float32MultiArray now;
std_msgs::Bool emergency_msg;
ros::Publisher place("place",&now);
ros::Publisher emergency_pub("emergency",&emergency_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("motor",&getData);
//ros::Subscriber<std_msgs::Int32> mdd("Motor_Serial",&sendSerial);

int main(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(place);
    nh.advertise(emergency_pub);
    nh.subscribe(sub);
    //nh.subscribe(mdd);
    now.data_length = 8;
    now.data = (float*)malloc(sizeof(float)*now.data_length);
    button.mode(PullUp);
    emergency.mode(PullUp);
    int i,j;
    bool flag = false;
    bool emergency_last = !emergency;//最初に強制的に送信させる。
    double diff[Motor_NUM],Pspeed[Motor_NUM];
    double nowVx,nowVy,nowVt;
    double cos_yaw_2,sin_yaw_2;
    double diff_x,diff_y,diff_yaw;
    double now_t,use_amax;
    const double R = 2 * 25.4 * M_PI / 256;
    double pid_v_x,pid_v_y,pid_omega;
    double errer_x = 0;
    double errer_y = 0;
    double errer_omega = 0;
    for(i = 0;i < Motor_NUM;i++){
        Led[i] = new DigitalOut(pwm_pin[i][2]);
        Motor[i][0] = new PwmOut(pwm_pin[i][0]);
        Motor[i][1] = new PwmOut(pwm_pin[i][1]);
        Motor[i][0]->period_us(PERIOD);
        Motor[i][1]->period_us(PERIOD);
        Speed[i] = new RotaryInc(rotary_pin[i][0],rotary_pin[i][1],2*50.8*M_PI,256,2);
        Place[i] = new RotaryInc(rotary_pin[i+Motor_NUM][0],rotary_pin[i+Motor_NUM][1],1);
    }
    for(i = 0;i < 2;i++){
        for(j = 0;j < 2;j++){
            Limit[i][j] = new DigitalIn(pwm_pin[4+i][j]);
            Limit[i][j]->mode(PullUp);
        }
    }
    I2C i2c(PB_3,PB_10);
    Timer loop;
    loop.start();
    while(button && !Ok){
        nh.spinOnce();
        if(loop.read_ms() > 250){
            led = !led;
            loop.reset();
        }
    }
    led.write(0);
    Ok = true;
    GY521 gyro(i2c);
    //gyro.reset(0);
    gy = &gyro;
    motortimer.start();
    led.write(1);
    while(true){
        if(ablemove){
            move();
        }else{
            for(i = 0;i<4;i++){
                nowV[i] = Speed[i]->getSpeed();
            }
        }
        nh.spinOnce();
        gyro.updata();
        Yaw = gyro.yaw;
        if(loop.read_ms() > 20){//50msごとに通信する
            //データ送信
            now.data[0] = X;
            now.data[1] = Y;
            now.data[2] = T;
            now.data[3] = Yaw;
            now.data[4] = nowVx;
            now.data[5] = nowVy;
            now.data[6] = nowVt;
            now.data[7] = automove;
            place.publish(&now);
            loop.reset();
        }
        Yaw *= 0.01745329251994329576923690768489;//PI/180
        for(i = 0;i<Motor_NUM;++i){
            diff[i] = Place[i]->diff() * R;//Place
            Pspeed[i] = Speed[i]->getSpeed();//Place
        }
        //オドメトリ計算
#if Motor_NUM == 3
        X += -2.0/3.0*diff[0]*cos(Yaw) + 2.0/3.0*diff[1]*cos(Yaw-PI_3) + 2.0/3.0*diff[2]*cos(Yaw+PI_3);
        Y += -2.0/3.0*diff[0]*sin(Yaw) + 2.0/3.0*diff[1]*sin(Yaw-PI_3) + 2.0/3.0*diff[2]*sin(Yaw+PI_3);
        T +=  diff[0]/Rotary_L/3 + diff[1]/Rotary_L/3 + diff[2]/Rotary_L/3;
        nowVx = -2.0/3.0*Pspeed[0]*cos(Yaw) + 2.0/3.0*Pspeed[1]*cos(Yaw-PI_3) + 2.0/3.0*Pspeed[2]*cos(Yaw+PI_3);
        nowVy = -2.0/3.0*Pspeed[0]*sin(Yaw) + 2.0/3.0*Pspeed[1]*sin(Yaw-PI_3) + 2.0/3.0*Pspeed[2]*sin(Yaw+PI_3);
        nowVt =  Pspeed[0]/Rotary_L/3 + Pspeed[1]/Rotary_L/3 + Pspeed[2]/Rotary_L/3;
#elif Motor_NUM == 4
        cos_yaw_2 = cos(Yaw)/2.0;
        sin_yaw_2 = sin(Yaw)/2.0;
        X += diff[0]*cos_yaw_2 - diff[1]*sin_yaw_2 - diff[2]*cos_yaw_2 + diff[3]*sin_yaw_2;
        Y += diff[0]*sin_yaw_2 + diff[1]*cos_yaw_2 - diff[2]*sin_yaw_2 - diff[3]*cos_yaw_2;
        T += diff[0]/Rotary_L/4 + diff[1]/Rotary_L/4 + diff[2]/Rotary_L/4 + diff[3]/Rotary_L/4;
        cos_yaw_2 = cos(Yaw+PI_4)/2.0;
        sin_yaw_2 = sin(Yaw+PI_4)/2.0;
        nowVx = Pspeed[0]*cos_yaw_2 - Pspeed[1]*sin_yaw_2 - Pspeed[2]*cos_yaw_2 + Pspeed[3]*sin_yaw_2;
        nowVy = Pspeed[0]*sin_yaw_2 + Pspeed[1]*cos_yaw_2 - Pspeed[2]*sin_yaw_2 - Pspeed[3]*cos_yaw_2;
        nowVt = Pspeed[0]/Rotary_L/4 + Pspeed[1]/Rotary_L/4 + Pspeed[2]/Rotary_L/4 + Pspeed[3]/Rotary_L/4;
#endif
        if(!button && !flag){
            flag = true;
            Reset();
        }else if(button && flag){
            flag = false;
        }
        if(emergency != emergency_last){//非常停止監視
        	emergency_last = emergency;
        	emergency_msg.data = emergency;
        	emergency_pub.publish(&emergency_msg);
        	if(emergency){
        		//safe();//非常停止時にプログラムも停止する。
        	}
        }
        if(automove){
            now_t = autotimer.read();
            autotimer.reset();
            use_amax = AMAX * now_t;
            
            if(limit_move >= 0){
                if(Limit[limit_move][0]->read() && Limit[limit_move][1]->read()){//両方押された時 押された時が１//NCにつなぐ
                    diff_x = 0;
                    X = 0;
                }else{
                    if(Limit[limit_move][0]->read()){//赤前 青後
                        diff_yaw = 0.001;
                    }else if(Limit[limit_move][1]->read()){//赤後 青前
                        diff_yaw = -0.001;
                    }
                    if(limit_move == 0){
                        diff_x = -100;
                    }else{
                        diff_x = 100;
                    }
                }
            }else{
                diff_x = goal_x - X;
                diff_yaw = goal_yaw - Yaw;
            }
            diff_y = goal_y - Y;

            pid_v_x = constrain(Kp * diff_x + Ki * errer_x - Kd * nowVx,-VMAX,VMAX);
            if(fabs(pid_v_x) >= fabs(Vx)){
                if(fabs(Vx - pid_v_x) > use_amax){
                    Vx += (pid_v_x >= 0 ? 1 : -1)*use_amax;
                }else{
                    Vx = pid_v_x;
                }
            }else{
                Vx = pid_v_x;
                errer_x += diff_x * now_t;
            }
            pid_v_y = constrain(Kp * diff_y + Ki * errer_y - Kd * nowVy,-VMAX,VMAX);
            if(fabs(pid_v_y) >= fabs(Vy)){
                if(fabs(Vy - pid_v_y) > use_amax){
                    Vy += (pid_v_y >= 0 ? 1 : -1)*use_amax;
                }else{
                    Vy = pid_v_y;
                }
            }else{
                Vy = pid_v_y;
                errer_y += diff_y * now_t;
            }
            pid_omega = constrain(-diff_yaw / tKp + errer_omega / tKi - nowVt / tKd,-0.9,0.9);
            if(fabs(pid_omega) >= fabs(Omega)){
                if(fabs(Omega - pid_omega) > use_amax){
                    Omega += (pid_omega >= 0 ? 1 : -1)*0.02;
                }else{
                    Omega = pid_omega;
                }
            }else{
                Omega = pid_omega;
                errer_omega += -diff_yaw * now_t;
            }
            if(fabs(diff_x) < 5  && fabs(diff_y) < 5 && fabs(diff_yaw) < (PI_180)
                    && fabs(nowVx) < 15 && fabs(nowVy) < 15 && fabs(nowVt) < 0.018){
                Vx = 0;
                Vy = 0;
                Omega = 0;
                errer_x = 0;
                errer_y = 0;
                errer_omega = 0;
                automove = false;
                autotimer.stop();
                autotimer.reset();
            }
        }
    }
}
