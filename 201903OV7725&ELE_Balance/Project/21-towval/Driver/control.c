#include "include.h"

#define AD_OUT_MAX 0.52
#define AD_OUT_MIN -0.52
#define AD1_MAX 4095
#define AD2_MAX 2400
#define AD3_MAX 4095
//#define AD4_MAX
//#define AD5_MAX
//#define AD6_MAX

#define AD1_MIN 0
#define AD2_MIN 0
#define AD3_MIN 0
//#define AD4_MIN
//#define AD5_MIN
//#define AD6_MIN

#define TIME 175
#define STOP_TIME 25

//角度类变量
int16 Acc_Z, Gyro_X, Gyro_Y;
int16 Acc_Max = 8192;
int16 Acc_Min = -8192;
int16 Acc_Offset;
float Angle, Angle_Speed, Car_Angle = 0;
int16 Gyro_X_Offset = -10, Gyro_Y_Offset = 1;
float Car_AngleIntegral = 0;
//速度类变量
float SpeedControlOutNew;
float SpeedControlOutOld;
float SpeedControlIntegral = 0, Hill_Slow_Ratio;
float Set_Angle; //加速前倾角度
int SpeedCount = 1;
int Speed_Filter_Times = 25; //速度平滑输出
float CarSpeed = 0, ControlSpeed = 0, AverageSpeed, SetSpeed = 0.4, Distance;
//方向类变量
float DirectionControlOutNew;
float DirectionControlOutOld;
float Turn_Speed = 0;
int DirectionCount;
float Delt_error, Middle_Err;
float Turn_Out;
float Turn_Angle_Integral;
float Pre1_Error[4];

int AD1_Value;
int AD2_Value;
int AD3_Value;
int AD4_Value;
int AD5_Value;
int AD6_Value;
int AD1_Normalized;
int AD2_Normalized;
int AD3_Normalized;
int AD4_Normalized;
int AD5_Normalized;
int AD6_Normalized;
char AD_Miss = false;
char CarmeraMiss = 1; //false;
char Dir_Fix;
float AD_Error;
float AD_Error_Delta;
float AD_Error_Filter[10];
int AD_Error_FilterCount = 0;
float AD_Turn_Out;
//模糊化系数
float Delta_P;
float Delta_D;
float Fuzzy_Kp;
float Fuzzy_Kd;
//PID控制类变量
PID PID_ANGLE, PID_SPEED, PID_TURN, PID_AD_TURN;

float LeftMotorOut, RightMotorOut; //电机输出量
float L_DeadValue = 0, R_DeadValue = 0;
uint8 Starting, Stop, CarStopedJustNow;
uint8 Encoder_Disable = 0;
//
float Speed_temp;
float mycarspeed; //设定速度

float Dis_Turn_Out;
float Turn_Out_tmp0, Turn_Out_tmp1, Turn_Out_tmp2;

float SpeedError_next;

extern unsigned int leftspeed, rightspeed;

extern uint8 runmode; //0: 直立跑  1：三轮跑
extern int time;
extern int zha;

int RedSan; //红外，三轮模式
int RedZhi;

int turn_num;

void Get_Attitude()
{
  Acc_Z = Get_Z_Acc();
  Gyro_X = Get_X_Gyro();
  Gyro_Y = Get_Y_Gyro();
}
//******Kalman滤波******//
//-------------------------------------------------------
static float Q_angle = 0.001, Q_gyro = 0.001, R_angle = 5, dt = 0.004;
//Q增大，动态响应增大
static float Pk[2][2] = {{1, 0}, {0, 1}};

static float Pdot[4] = {0, 0, 0, 0};

static float q_bias = 0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m, float gyro_m)
{
  Car_Angle += (gyro_m - q_bias) * dt; ///预测值
  Pdot[0] = Q_angle - Pk[0][1] - Pk[1][0];
  Pdot[1] = -Pk[1][1];
  Pdot[2] = -Pk[1][1];
  Pdot[3] = Q_gyro;

  Pk[0][0] += Pdot[0] * dt;
  Pk[0][1] += Pdot[1] * dt;
  Pk[1][0] += Pdot[2] * dt;
  Pk[1][1] += Pdot[3] * dt;

  angle_err = angle_m - Car_Angle; ///测量值-预测值

  PCt_0 = Pk[0][0];
  PCt_1 = Pk[1][0];

  E = R_angle + PCt_0;

  K_0 = PCt_0 / E; ///卡尔曼增益
  K_1 = PCt_1 / E;

  t_0 = PCt_0;
  t_1 = Pk[0][1];

  Pk[0][0] -= K_0 * t_0;
  Pk[0][1] -= K_0 * t_1;
  Pk[1][0] -= K_1 * t_0;
  Pk[1][1] -= K_1 * t_1;

  Car_Angle += K_0 * angle_err; ///最优角度=预测值+卡尔曼增益*(测量值-预测值)
  q_bias += K_1 * angle_err;
  Angle_Speed = gyro_m - q_bias;
}

//角度计算与滤波
void Angle_Calculate()
{

  float ratio = 0.048;

  if (runmode == 0)
  { //直立
    Angle = (Acc_Z - Acc_Offset) * 180.0 / (Acc_Max - Acc_Min) + Set_Angle;
  }
  else if (runmode == 1)
  { //三轮
    Angle = (Acc_Z - Acc_Offset) * 180.0 / (Acc_Max - Acc_Min) + 25;
  }
  Angle_Speed = (Gyro_Y - Gyro_Y_Offset) * ratio;

  Kalman_Filter(Angle, Angle_Speed); //调用卡尔曼滤波函数
}
//角度控制函数
void Angle_Control()
{
  PID_ANGLE.pout = PID_ANGLE.P * Car_Angle;
  PID_ANGLE.dout = PID_ANGLE.D * Angle_Speed;
  if (ABS(Angle_Speed) > 30 && ABS(Angle_Speed) < 80)
  {
    PID_ANGLE.dout *= (1 + (ABS(Angle_Speed) - 30.0) / 30.0);
  }
  PID_ANGLE.OUT = PID_ANGLE.pout + PID_ANGLE.dout;
}

void Get_Speed() //5ms执行一次
{
  static float Avg_Sp[10], Avg_Sp1[10];
  int qd1_result, qd2_result;

  if (gpio_get(PTE24) == 1)
  {
    qd1_result = leftspeed;
  }
  if (gpio_get(PTE24) != 1)
  {
    qd1_result = -leftspeed;
  }
  if (gpio_get(PTE25) == 1)
  {
    qd2_result = -rightspeed;
  }
  if (gpio_get(PTE25) != 1)
  {
    qd2_result = rightspeed;
  }

  rightspeed = 0;
  leftspeed = 0;

  Avg_Sp[9] = Avg_Sp[8];
  Avg_Sp[8] = Avg_Sp[7];
  Avg_Sp[7] = Avg_Sp[6];
  Avg_Sp[6] = Avg_Sp[5];
  Avg_Sp[5] = Avg_Sp[4];
  Avg_Sp[4] = Avg_Sp[3];
  Avg_Sp[3] = Avg_Sp[2];
  Avg_Sp[2] = Avg_Sp[1];
  Avg_Sp[1] = Avg_Sp[0];
  Avg_Sp[0] = qd1_result;

  Avg_Sp1[9] = Avg_Sp1[8];
  Avg_Sp1[8] = Avg_Sp1[7];
  Avg_Sp1[7] = Avg_Sp1[6];
  Avg_Sp1[6] = Avg_Sp1[5];
  Avg_Sp1[5] = Avg_Sp1[4];
  Avg_Sp1[4] = Avg_Sp1[3];
  Avg_Sp1[3] = Avg_Sp1[2];
  Avg_Sp1[2] = Avg_Sp1[1];
  Avg_Sp1[1] = Avg_Sp1[0];
  Avg_Sp1[0] = qd2_result;

  qd1_result = (Avg_Sp[0] + Avg_Sp[1] + Avg_Sp[2] + Avg_Sp[3] + Avg_Sp[4] + Avg_Sp[5] + Avg_Sp[6] + Avg_Sp[7] + Avg_Sp[8] + Avg_Sp[9]) / 10;
  qd2_result = (Avg_Sp1[0] + Avg_Sp1[1] + Avg_Sp1[2] + Avg_Sp1[3] + Avg_Sp1[4] + Avg_Sp1[5] + Avg_Sp1[6] + Avg_Sp1[7] + Avg_Sp1[8] + Avg_Sp1[9]) / 10;

  Distance += (qd1_result + qd2_result) / 5000.0; //转化为距离  N/2/500 *0.2 =
  CarSpeed = (qd1_result + qd2_result) * 0.05;    //*250.0/6100.0;    //求出车速转换为M/S   N/2 /500 * L * T = N/2 /500 *0.2 * 250 = N /1000 * 50 = N * 0.05
}

//速度控制量计算
void Speed_Control(void)
{

  static float PreError[20] = {0};
  float SpeedError; //,Speed_temp;
  uint8 i;

  //if(Starting||Stop)  setspeed=0; //启动的时候把速度置为零
  SpeedError = SetSpeed - CarSpeed;

  //求出最近20个偏差的总和作为积分项
  SpeedControlIntegral = 0;
  for (i = 0; i < 19; i++)
  {
    PreError[i] = PreError[i + 1];
    SpeedControlIntegral += PreError[i];
  }
  PreError[19] = SpeedError;
  SpeedControlIntegral += PreError[19];

  //速度更新
  SpeedControlOutOld = SpeedControlOutNew;

  //防止起步位移

  SpeedControlOutNew = PID_SPEED.P * SpeedError + PID_SPEED.I * SpeedControlIntegral; //PI控制

  // SpeedControlOutNew = SpeedControlOutOld*0.8 + SpeedControlOutNew*0.2;
}

//速度控制
void Speed_Control_Output(void)
{
  float fValue;
  fValue = (SpeedControlOutNew - SpeedControlOutOld);

  PID_SPEED.OUT = SpeedControlOutNew;

  //PID_SPEED.OUT = fValue * (SpeedCount)/Speed_Filter_Times+SpeedControlOutOld;

  if (SpeedCount == 25)
  {
    SpeedCount = 1;
  }
  else if (SpeedCount < 25)
  {
    SpeedCount = SpeedCount + 1;
  }
}

void Direction_ADControl_zl()
{
  int adc_sum[8], j, zy;
  float ratiot = 0.048;
  float ADC, ADH;
  static int Mh = 0;
  static int Ytime = 0; //用在判断欧拇环进环时  做为延时检测计时

  static char rightM = 0, leftM = 0;

  adc_sum[0] = 0;
  adc_sum[1] = 0;
  adc_sum[2] = 0;
  adc_sum[3] = 0;

  for (j = 0; j < 10; j++)
  {
    adc_sum[1] += adc_once(ADC1_SE4a, ADC_12bit);
    adc_sum[3] += adc_once(ADC1_SE9, ADC_12bit);
  }
  AD1_Value = adc_sum[1] / j; //Left
  AD3_Value = adc_sum[3] / j; //Right

  AD1_Value = ((AD1_Value < AD1_MIN) ? AD1_MIN : AD1_Value) > AD1_MAX ? AD1_MAX : AD1_Value;
  AD3_Value = ((AD3_Value < AD3_MIN) ? AD3_MIN : AD3_Value) > AD3_MAX ? AD3_MAX : AD3_Value;

  AD1_Normalized = (AD1_Value) / (AD1_MAX / 100);
  AD3_Normalized = (AD3_Value) / (AD3_MAX / 100);

  if (AD1_Normalized == 0 && AD3_Normalized == 0)
  {
    AD_Miss = true;
  }
  if (AD1_Normalized > 10 || AD3_Normalized > 10)
  {
    AD_Miss = false;
  }

  /*////////////////右欧姆换//////////////////
  
  if(AD1_Normalized >100)  //右欧姆换
  {
    ////////////////进环操作///////////////
    
    if( (80<AD3_Normalized) && ( rightM==0)) 
    {
       Ytime++;
    }
    
    if(( rightM==0) && (Ytime>=20))
    {
      rightM=2; Ytime=0;
    }
    
    if((AD3_Normalized >100) && ( rightM==2)){
    
       if(Mh==0){ Mh=0; rightM=1; }
    }
    ////////////////进环操作////////////////
    ////////////////出环操作////////////////
    if((AD3_Normalized ==102) && ( rightM==3))
    {
      rightM=4; Mh=0;
    }
    
    ////////////////出环操作////////////////
    
    //if(Mh==0){ Mh=1;}
  }
  
  if(AD1_Normalized<100) {
     Ytime=0;
  }
   ////////////////右欧姆换///////////////*/

  //*///////////////左欧姆换////////////////
  if (AD3_Normalized > 100) //右欧姆换
  {
    ////////////////进环操作///////////////

    if ((80 < AD1_Normalized) && (leftM == 0))
    {
      Ytime++;
    }

    if ((leftM == 0) && (Ytime >= 20))
    {
      leftM = 2;
      Ytime = 0;
    }

    if ((AD1_Normalized > 100) && (leftM == 2))
    {

      if (Mh == 0)
      {
        Mh = 0;
        leftM = 1;
      }
    }
    ////////////////进环操作////////////////
    ////////////////出环操作////////////////
    if ((AD1_Normalized == 102) && (leftM == 3))
    {
      leftM = 4;
      Mh = 0;
    }

    ////////////////出环操作////////////////

    //if(Mh==0){ Mh=1;}
  }

  if (AD3_Normalized < 100)
  {
    Ytime = 0;
  }

  ////////////////左欧姆换///////////////*/
  Turn_Speed = -(Gyro_X - Gyro_X_Offset) * ratiot;

  ADC = (float)AD3_Value - AD1_Value;
  ADH = (float)(AD3_Value + AD1_Value);

  if (runmode == 0)
  {
    zy = 1000;
  }
  if (runmode == 1)
  {
    zy = 300;
  } //三轮跑

  AD_Error = (ADC / ADH) * zy;

  //if( (AD1_Value<4) && ((AD3_Value - AD1_Value) < 450) && ((AD3_Value - AD1_Value) > 350) )  {Middle_Err = -16;}
  //if( (AD3_Value<4) && ((AD1_Value - AD3_Value) < 450) && ((AD1_Value - AD3_Value) > 350) )  {Middle_Err = 16;}

  PID_AD_TURN.pout = (PID_AD_TURN.P) * AD_Error;
  PID_AD_TURN.dout = (PID_AD_TURN.D) * Turn_Speed;

  PID_AD_TURN.OUT = PID_AD_TURN.pout + PID_AD_TURN.dout;

  /**********进环操作**********/
  if ((rightM == 1) || (leftM == 1))
  {

    Mh++;

    if ((rightM == 1) && (Mh > 70))
    {
      PID_AD_TURN.OUT = -0.17;
    } //100ms后转向
    //Mh延时时间
    if ((leftM == 1) && (Mh > 40))//70
    {
      PID_AD_TURN.OUT = 0.17;
    }
//mh转向结束时间，也适当减少
    if (Mh > 200)//250
    { //250ms后转向完成

      Mh = 0;
      if (rightM == 1)
      {
        rightM = 3;
      }
      if (leftM == 1)
      {
        leftM = 3;
      }
    }
  }
  /**********进环操作**********/
  /**********出环操作**********/
  if ((rightM == 4) || (leftM == 4))
  {

    Mh++;
//mh出环时间0-100
    if ((rightM == 4) && (Mh < 100))//100
    {
      PID_AD_TURN.OUT = -0.17;
    } //右拐100ms
    else if ((leftM == 4) && (Mh < 80))//100
    {
      PID_AD_TURN.OUT = 0.17;
    }
    else if ((80 <= Mh) && (Mh < 1200))
    { //按照正常直道走 200ms
    }
    else if (1200 <= Mh)
    { //300ms后恢复欧拇环判断初始标志
      Mh = 0;
      if (rightM == 4)
      {
        rightM = 0;
      }
      else if (leftM == 4)
      {
        leftM = 0;
      }
    }
  }

  /**********出环操作**********/
  if(rightM==0&&leftM==0){
    zha=0;
  }
  if(rightM!=0||leftM!=0){
    zha=3;
  }
}

/********************方向控制量计算***************/

//电机pwm值输出
void Moto_Out_Control()
{
  static float Forward_Safe_Angle = 5; //前倾的安全角度
  int Backward_Safe_Angle = 20;        //后倾的安全角度
  float Sum;

  //AD_Miss=false;  //测试直立
  //CarmeraMiss = 1;
  if (runmode == 0) //直立跑
  {
    if (CarmeraMiss == 1 && !AD_Miss) //电磁工作
    {

      // if((-0.5<Car_Angle) && (Car_Angle<0))  {Sum=PID_ANGLE.OUT-0.1;}
      // else   {Sum=PID_ANGLE.OUT ;}

      //  if((-1<Car_Angle) && (Car_Angle<0))  {Sum=PID_ANGLE.OUT-0.07;}
      // else if((0<Car_Angle) && (Car_Angle<-0.5))  {Sum=PID_ANGLE.OUT+0.05;}
      // else   {Sum=PID_ANGLE.OUT ;}

      Sum = PID_ANGLE.OUT + PID_SPEED.OUT;
      LeftMotorOut = Sum - PID_AD_TURN.OUT; //丢失了信号，靠电磁
      RightMotorOut = Sum + PID_AD_TURN.OUT;
    }

    else if (AD_Miss == 1) //出了赛道,停车
    {
      LeftMotorOut = 0.0f;
      RightMotorOut = 0.0f;
    }
  }
  if (runmode == 1) //runmode=1，三轮跑,不需要直立分量
  {
    if (CarmeraMiss == 1 && !AD_Miss) //电磁工作
    {

      LeftMotorOut = -PID_SPEED.OUT - PID_AD_TURN.OUT;  // + PID_ANGLE.OUT*0.2;  //电磁分量
      RightMotorOut = -PID_SPEED.OUT + PID_AD_TURN.OUT; // + PID_ANGLE.OUT*0.2;  //电磁分
    }

    if (AD_Miss == 1) //出了赛道
    {
      LeftMotorOut = 0.0f;
      RightMotorOut = 0.0f;
    }
  }

  //////      测试命令    //////////////////////////
  //    LeftMotorOut = PID_ANGLE.OUT;   // 纯直立
  //    RightMotorOut= PID_ANGLE.OUT;

  // LeftMotorOut  =  -PID_TURN.OUT;   // 纯转向
  // RightMotorOut =  +PID_TURN.OUT;
  /*
if(Stop)                                //如果停止则电机不输出
 {
      LeftMotorOut = 0.0f;
      RightMotorOut = 0.0f;
 }
*/
}

void Moto_Out()
{

  int L_Value, R_Value;
  static int Motor_Abnormal_Cnt = 0; //电机转速异常计数

  if (RightMotorOut > 0.99)
    RightMotorOut = 0.99;
  if (RightMotorOut < -0.99)
    RightMotorOut = -0.99;
  if (LeftMotorOut > 0.99)
    LeftMotorOut = 0.99;
  if (LeftMotorOut < -0.99)
    LeftMotorOut = -0.99;

  L_Value = (int)(10000 * LeftMotorOut);
  R_Value = (int)(10000 * RightMotorOut);

  /////////////////////保护////////////////////////////
  if (Car_Angle < -10 || Car_Angle > 30) //倒下
  {

    if (Stop == false && RunTime > 2)
    {
      Stop = true;
      CarStopedJustNow = true;
      Motor_Abnormal_Cnt = 0;
    }
  }
  if ((Distance > 1) && (Stop == false)) //              检测是否是速度异常
  {
    if ((CarSpeed < 0.5) || (CarSpeed > 3))
    {
      Motor_Abnormal_Cnt++;
      if (Motor_Abnormal_Cnt > 250) //电机异常的持续时间大于500ms秒
      {
        Stop = true;
        CarStopedJustNow = true;
      }
    }
    else
    {
      Motor_Abnormal_Cnt = 0;
    }
  }

  if (L_Value >= 0) //正转
  {
    FTM_PWM_Duty(FTM0, FTM_CH0, L_Value); //占空比精度为10000
    FTM_PWM_Duty(FTM0, FTM_CH1, 0);
  }
  else //反转
  {
    FTM_PWM_Duty(FTM0, FTM_CH0, 0);
    FTM_PWM_Duty(FTM0, FTM_CH1, -L_Value);
  }
  if (R_Value >= 0) //正转
  {
    FTM_PWM_Duty(FTM0, FTM_CH2, R_Value);
    FTM_PWM_Duty(FTM0, FTM_CH3, 0);
  }
  else //反转
  {
    FTM_PWM_Duty(FTM0, FTM_CH2, 0);
    FTM_PWM_Duty(FTM0, FTM_CH3, -R_Value);
  }
}

float My_Slope_Calculate(uint8 begin, uint8 end, float *p) //最小二乘法拟合斜率
{
  float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
  uint8 i = 0;
  float result = 0;
  static float resultlast;
  p = p + begin;
  for (i = begin; i < end; i++)
  {
    xsum += i;
    ysum += *p;
    xysum += i * (*p);
    x2sum += i * i;
    p = p + 1;
  }
  if ((end - begin) * x2sum - xsum * xsum) //判断除数是否为零
  {
    result = ((end - begin) * xysum - xsum * ysum) / ((end - begin) * x2sum - xsum * xsum);
    resultlast = result;
  }
  else
  {
    result = resultlast;
  }
  return result;
}

void My_Push_And_Pull(float *buff, int len, float newdata)
{
  int i;
  for (i = len - 1; i > 0; i--)
  {
    *(buff + i) = *(buff + i - 1);
  }
  *buff = newdata;
}

float Turn_Out_Filter(float turn_out) //转向控制输出滤波
{
  float Turn_Out_Filtered;
  Pre1_Error[3] = Pre1_Error[2];
  Pre1_Error[2] = Pre1_Error[1];
  Pre1_Error[1] = Pre1_Error[0];
  Pre1_Error[0] = turn_out;
  Turn_Out_Filtered = Pre1_Error[0] * 0.4 + Pre1_Error[1] * 0.3 + Pre1_Error[2] * 0.2 + Pre1_Error[3] * 0.1;
  return Turn_Out_Filtered;
}

float Middle_Err_Filter(float middle_err) //中心偏差滤波
{
  float Middle_Err_Fltered;
  static float Pre3_Error[4];
  Pre3_Error[3] = Pre3_Error[2];
  Pre3_Error[2] = Pre3_Error[1];
  Pre3_Error[1] = Pre3_Error[0];
  Pre3_Error[0] = middle_err;
  Middle_Err_Fltered = Pre3_Error[0] * 0.4 + Pre3_Error[1] * 0.3 + Pre3_Error[2] * 0.2 + Pre3_Error[3] * 0.1;
  return Middle_Err_Fltered;
}

int Red_Check()
{
  int red = 0;
  int j;
  int SL = 1600, ZL = 40;
  for (j = 0; j < 10; j++)
  {

    if (runmode)
    {
      red = red + adc_once(ADC1_SE5a, ADC_12bit);
    }
    else
    {
      red = red + adc_once(ADC1_SE6a, ADC_12bit);
    }
  }
  red = red / 10;
  // red=(6762/(red-9))-4;

  if (runmode)
  { //三轮

    RedSan = red;
    if (red > SL)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  { //直立

    RedZhi = red;
    return 0;
  }
}

int BiZhang()
{

  int redVlueflag = 1800;

  int red, j, flag = 0;

  Get_Speed();

  while (CarSpeed > 0) //判断停车
  {

    LeftMotorOut = 0.99;
    RightMotorOut = 0.99;
    Moto_Out();

    Get_Speed();
  }

  for (j = 0; j < 10; j++)
  {
    if (runmode)
    {
      red = red + adc_once(ADC1_SE5a, ADC_12bit);
    }
  }
  red = red / 10;

  while (red > redVlueflag)
  {
    Direction_ADControl_zl();
    Get_Speed();
    if (CarSpeed > 0)
    {

      RightMotorOut = 0.99; //+PID_AD_TURN.OUT;
      LeftMotorOut = 0.99;  // -PID_AD_TURN.OUT;
    }
    else
    {
      RightMotorOut = 0.1 + PID_AD_TURN.OUT * 0.3;
      LeftMotorOut = 0.1 - PID_AD_TURN.OUT * 0.3;
    }

    //RightMotorOut=0.1;
    //LeftMotorOut=0.1;
    Moto_Out();

    for (j = 0; j < 10; j++)
    {
      if (runmode)
      {
        red = red + adc_once(ADC1_SE5a, ADC_12bit);
      }
    }
    red = red / 10;
  }

  while (red <= redVlueflag)
  {

    if (CarSpeed < 0)
    {
      LeftMotorOut = -0.1;
      RightMotorOut = -0.1;
    }
    else
    {
      break;
    }

    for (j = 0; j < 10; j++)
    {
      if (runmode)
      {
        red = red + adc_once(ADC1_SE5a, ADC_12bit);
      }
    }
    red = red / 10;
    Get_Speed();
  }

  return 1;
}

int Ramp()
{
  switch (turn_num)
  {
  case 0:
  { //加速
    time++;
    PID_SPEED.OUT = 0.3f;
    if (Car_Angle < 0 || time >= 500)
    {
      time = 0;
      turn_num = 1;
    }
    return 2;
  }
  case 1:
  { //慢速2
    PID_SPEED.OUT = -0.2f;
    if (Car_Angle >= -5)
    {
      turn_num = 0;
      return 0;
    }
    return 2;
  }
  }
}

int go_block()
{
  time++;
  static char turn_num = 0;
/*
  if (judgeramp()==0)
  {
    return 2;
  }
*/
  switch (turn_num)
  {
  case 0:
  { //刹车
    brake_car();
    if (CarSpeed <= 0 || time >= 500)
    {
      turn_num = 1;
      time = 0;
    }
    return 1;
  }
  case 1:
  { //停车
    go_stop();
    if (time >= STOP_TIME)
    {
      turn_num = 2;
      time = 0;
    }
    return 1;
  }
  case 2:
  { //倒车
    go_back();
    if (Red_Check() == 0)
    {
      turn_num = 3;
      time = 0;
    }
    return 1;
  }
  case 3:
  { //停车
    go_stop();
    if (time >= STOP_TIME)
    {
      turn_num = 4;
      time = 0;
    }
    return 1;
  }
  case 4:
  { //左转
    Red_Check();
    left_turn();
    if (time >= TIME&&RedSan<=1000)
    {
      turn_num = 5;
      time = 0;
    }
    return 1;
  }
  case 5:
  { //停车
    go_stop();
    if (time >= STOP_TIME)
    {
      turn_num = 6;
      time = 0;
    }
    return 1;
  }
  case 6:
  { //直行
    go_straight();
    if (time >= TIME * 6)//5.5
    {
      turn_num = 7;
      time = 0;
    }
    return 1;
  }
  case 7:
  { //停车
    go_stop();
    if (time >= STOP_TIME)
    {
      turn_num = 8;
      time = 0;
    }
    return 1;
  }
  case 8:
  { //右转
    right_turn();
    if (time >= TIME * 1.5)
    {
      turn_num = 9;
      time = 0;
    }
    return 1;
  }
  case 9:
  { //停车
    go_stop();
    if (time >= STOP_TIME)
    {
      turn_num = 10;
      time = 0;
    }
    return 1;
  }
  case 10:
  { //直行
    go_straight();
    if ((judge_ad() == 1) && (time >= TIME * 3))
    {
      turn_num = 11;
      time = 0;
    }
    return 1;
  }
  case 11:
  { //停车
    go_stop();
    if (time >= STOP_TIME)
    {
      turn_num = 12;
      time = 0;
    }
    return 1;
  }
  case 12:
  { //左转
    left_turn();
    if (time >= TIME)
    {
      turn_num = 13;
      time = 0;
    }
    return 1;
  }
  case 13:
  { //停车
    go_stop();
    if (time >= STOP_TIME)
    {
      turn_num = 0;
      time = 0;
      return 0;
    }
    return 1;
  }
  }
}

void left_turn()
{
  LeftMotorOut = -0.15f;
  RightMotorOut = 0.15f;
}

void right_turn()
{
  LeftMotorOut = 0.18f;
  RightMotorOut = -0.18f;
}

void go_straight()
{
  LeftMotorOut = -0.15f;
  RightMotorOut = -0.15f;
}

void go_stop()
{
  LeftMotorOut = 0.0f;
  RightMotorOut = 0.0f;
}

int judge_ad()
{
  if (AD3_Value >= 4000 && AD1_Value <= 1000)
    return 1; //左值大于0
  else
    return 0;
}

void go_back()
{
  LeftMotorOut = 0.15f- PID_AD_TURN.OUT;
  RightMotorOut = 0.15f+ PID_AD_TURN.OUT;
}

void brake_car()
{
  LeftMotorOut = 0.99f;
  RightMotorOut = 0.99f;
}

/*
return 1 无斜坡
return 0 有斜坡
 */
int judge_ramp_num = 1;
int judgeramp()
{
  if (Car_Angle >= 15)
  {
    judge_ramp_num++;
    if (judge_ramp_num >= 500)
      return 1;
  }
  else
  {
    judge_ramp_num = 0;
    return 0;
  }
}