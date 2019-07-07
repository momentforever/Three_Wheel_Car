#ifndef _CONTROL_H_
#define _CONTROL_H_
typedef struct PID{float P,pout,I,iout,D,dout,OUT;}PID;
extern  PID PID_ANGLE,PID_SPEED,PID_TURN,PID_AD_TURN;

extern float Angle,Angle_Speed,Car_Angle,Turn_Speed;
extern int16  Gyro_X_Offset,Gyro_Y_Offset,Acc_Offset; //传感器偏差

extern int Speed_Filter_Times;
extern int SpeedCount;
extern float CarSpeed,ControlSpeed,Hill_Slow_Ratio;
extern float SetSpeed;
extern float mycarspeed;

//extern uint8 Set_Angle;
extern float AverageSpeed;
extern float Distance;

extern float SpeedError_next;

extern float SpeedControlOutOld,SpeedControlOutNew;
extern float SpeedControlIntegral;
extern float LeftMotorOut,RightMotorOut;   //电机输出量
//模糊化相关
extern float  Delta_P;
extern float  Delta_D;
extern float  Fuzzy_Kp;
extern float  Fuzzy_Kd;
//方向控制相关
extern int    DirectionCount;
extern float  Delt_error,Middle_Err;
extern float  Turn_Speed,Turn_Out,Turn_Angle_Integral;

extern uint8 Protect,ForceStop,Starting,Stop,CarStopedJustNow;

extern int  AD1_Value;
extern int  AD2_Value;
extern int  AD3_Value;
extern int  AD1_Normalized;
extern int  AD2_Normalized;
extern int  AD3_Normalized;
extern char AD_Miss;
extern float AD_Error_Delta;
extern float AD_Error;
extern char CarmeraMiss;

extern int RedSan;
extern int RedZhi;
extern int Dutime;
extern int yanshi;


void Get_Attitude();
void Get_Speed();
void Strong_Turn();
void Angle_Calculate();
void Angle_Control();
void Moto_Out_Control();
void Moto_Out();
void Speed_Control();
void Speed_Control_Output();
void Direction_Control();
void Direction_ADControl_zl();
void Direction_ADControl_sl();
float My_Slope_Calculate(uint8 begin,uint8 end,float *p);
void My_Push_And_Pull(float *buff,int len,float newdata);
float  Middle_Err_Filter(float middle_err);  
float  Turn_Out_Filter(float turn_out);
char Red_Check();
char BiZhang();
char go_block();

void left_turn();
void right_turn();
void go_straight();
void go_stop();
int judge_ad();
#endif
