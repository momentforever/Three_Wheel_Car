#include "include.h"

#define AD_OUT_MAX 0.52
#define AD_OUT_MIN -0.52
#define AD1_MAX    5000
#define AD2_MAX    2400
#define AD3_MAX    5000
//#define AD4_MAX
//#define AD5_MAX
//#define AD6_MAX

#define AD1_MIN    0
#define AD2_MIN    0
#define AD3_MIN    0
//#define AD4_MIN
//#define AD5_MIN
//#define AD6_MIN

//�Ƕ������
int16  Acc_Z,Gyro_X,Gyro_Y;
int16  Acc_Max=8192;
int16  Acc_Min=-8192;
int16  Acc_Offset;
float Angle,Angle_Speed,Car_Angle=0;
int16  Gyro_X_Offset=-10,Gyro_Y_Offset=1;
float Car_AngleIntegral=0;
//�ٶ������
float SpeedControlOutNew;
float SpeedControlOutOld;
float SpeedControlIntegral=0,Hill_Slow_Ratio;
float Set_Angle;   //����ǰ��Ƕ�
int   SpeedCount;
int   Speed_Filter_Times=50;    //�ٶ�ƽ�����
float CarSpeed=0,ControlSpeed=0,AverageSpeed,SetSpeed=0.4,Distance;
//���������
float DirectionControlOutNew;
float DirectionControlOutOld;
float Turn_Speed=0;
int   DirectionCount;
float Delt_error,Middle_Err;
float Turn_Out;
float Turn_Angle_Integral;
float Pre1_Error[4];

int  AD1_Value;
int  AD2_Value;
int  AD3_Value;
int  AD4_Value;
int  AD5_Value;
int  AD6_Value;
int  AD1_Normalized;
int  AD2_Normalized;
int  AD3_Normalized;
int  AD4_Normalized;
int  AD5_Normalized;
int  AD6_Normalized;
char AD_Miss=false;
char CarmeraMiss=1;//false;
char Dir_Fix;
float AD_Error;
float AD_Error_Delta;
float AD_Error_Filter[10];
int   AD_Error_FilterCount=0;
float AD_Turn_Out;
//ģ����ϵ��
float  Delta_P;
float  Delta_D;
float  Fuzzy_Kp;
float  Fuzzy_Kd;
//PID���������
PID PID_ANGLE,PID_SPEED,PID_TURN,PID_AD_TURN;

float  LeftMotorOut,RightMotorOut;   //��������
float  L_DeadValue=0,R_DeadValue=0;
uint8   Starting,Stop,CarStopedJustNow;
uint8 Encoder_Disable=0;
//
float Speed_temp;
float mycarspeed; //�趨�ٶ�

float Dis_Turn_Out;
float Turn_Out_tmp0,Turn_Out_tmp1,Turn_Out_tmp2;

extern unsigned int leftspeed,rightspeed;

extern uint8 runmode;  //0: ֱ����  1��������
//extern uint8 lockrun;  //0:����ı�runmode   1:������ı�


void Get_Attitude()
{
  Acc_Z =Get_Z_Acc();
  Gyro_X= Get_X_Gyro();
  Gyro_Y= Get_Y_Gyro();
}
//******Kalman�˲�******//
//-------------------------------------------------------
static  float Q_angle=0.001, Q_gyro=0.001, R_angle=5, dt=0.004;
	//Q���󣬶�̬��Ӧ����
static float Pk[2][2] = { {1, 0}, {0, 1 }};

static float Pdot[4] ={0,0,0,0};

static float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)
{
	Car_Angle+=(gyro_m-q_bias) * dt; ///Ԥ��ֵ
	Pdot[0]=Q_angle - Pk[0][1] - Pk[1][0];
	Pdot[1]=- Pk[1][1];
	Pdot[2]=- Pk[1][1];
	Pdot[3]=Q_gyro;

	Pk[0][0] += Pdot[0] * dt;
	Pk[0][1] += Pdot[1] * dt;
	Pk[1][0] += Pdot[2] * dt;
	Pk[1][1] += Pdot[3] * dt;

	angle_err = angle_m -Car_Angle;///����ֵ-Ԥ��ֵ

	PCt_0 =  Pk[0][0];
	PCt_1 =  Pk[1][0];

	E = R_angle + PCt_0;

	K_0 = PCt_0 / E; ///����������
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = Pk[0][1];

	Pk[0][0] -= K_0 * t_0;
	Pk[0][1] -= K_0 * t_1;
	Pk[1][0] -= K_1 * t_0;
	Pk[1][1] -= K_1 * t_1;

	Car_Angle+= K_0 * angle_err; ///���ŽǶ�=Ԥ��ֵ+����������*(����ֵ-Ԥ��ֵ)
	q_bias	+= K_1 * angle_err;
	Angle_Speed = gyro_m-q_bias;
}


//�Ƕȼ������˲�
void Angle_Calculate()
{
  float ratio=0.048,ratiot=0.01;;
  Angle =Acc_Z*180.0/(Acc_Max-Acc_Min) - Set_Angle;

  Angle_Speed=(Gyro_Y-Gyro_Y_Offset) * ratio;
  Kalman_Filter(Angle,Angle_Speed);            //���ÿ������˲�����
}
//�Ƕȿ��ƺ���
void Angle_Control()
{
  static float Car_AngleI[10];
  int i;

  //PID_ANGLE.D = 0.0006;

  if( Car_Angle < 20.0f && Car_Angle > -20.0f )
  {
    //PID_ANGLE.D = 0.0025;
    //PID_SPEED.P=0.25;
    //PID_SPEED.I=0.02;
    Car_AngleIntegral -= Car_AngleI[0]/10.0f;
    for( i=0;i<9;i++ )
    {
      Car_AngleI[i]=Car_AngleI[i+1];
    }
    Car_AngleI[9]=Car_Angle;
    Car_AngleIntegral += Car_AngleI[9]/10.0f;
  }
  else
  {
    Car_AngleIntegral = 0.0f;
    for( i=0;i<10;i++ )
    {
      Car_AngleI[i]=0.0f;
    }
  }

  PID_ANGLE.pout=PID_ANGLE.P*Car_Angle;
  PID_ANGLE.iout=PID_ANGLE.I*Car_AngleIntegral;
  PID_ANGLE.dout=PID_ANGLE.D*Angle_Speed;

  if(ABS(Angle_Speed)>30&&ABS(Angle_Speed)<80)
  {
    PID_ANGLE.dout*=(1+(ABS(Angle_Speed)-30.0)/30.0);
  }
  PID_ANGLE.OUT= PID_ANGLE.pout+ PID_ANGLE.dout;
  
}

void Get_Speed()                     //5msִ��һ��
{
  int qd1_result,qd2_result;
  //qd1_result = -FTM_QUAD_get(FTM1);
  //qd2_result = FTM_QUAD_get(FTM2);
  if(gpio_get(PTE24)==1)
     qd1_result = leftspeed;
  else
     qd1_result = -leftspeed;
  if(gpio_get(PTE25)==1)
     qd2_result = - rightspeed;
  else
    qd2_result = rightspeed;

  rightspeed=0;
  leftspeed=0;

  Distance+=(qd1_result+qd2_result)/6100.0;  //ת��Ϊ����
  CarSpeed=(qd1_result+qd2_result)*250.0/6100.0;    //�������ת��ΪM/S
   if(CarSpeed>3) CarSpeed=3;
}

//�ٶȿ���������
void Speed_Control(void)
{

  static float PreError[20]={0};
  float  SpeedError ;

  uint8 i;
  float  SpeedFilterRatio=0.85;     //�ٶ��趨ֵ�˲�����ֹ�ٶȿ��Ʊ仯̫����

  //�趨�ٶ��˲�


  //�ٶ��˲�����ֹ��Ϊ�ٶȱ仯���������ζ�
  Speed_temp=SetSpeed;


  //if(Starting||Stop)  setspeed=0; //������ʱ����ٶ���Ϊ��
   SpeedError= SetSpeed - CarSpeed;


  //������20��ƫ����ܺ���Ϊ������
   SpeedControlIntegral=0;
   for(i=0;i<19;i++)
   {
     PreError[i]=PreError[i+1];
     SpeedControlIntegral+=PreError[i];
   }
    PreError[19]=SpeedError;
    SpeedControlIntegral+=PreError[19];

  //�ٶȸ���
  SpeedControlOutOld=SpeedControlOutNew;

  //��ֹ��λ��
  if(Starting)
  {
   // SpeedControlIntegral=-50*Distance;
    //SpeedError=0;
  }

  SpeedControlOutNew=PID_SPEED.P*SpeedError+PID_SPEED.I*SpeedControlIntegral;   //PI����

  SpeedControlOutNew= SpeedControlOutOld*0.7+SpeedControlOutNew*0.3;
  Speed_temp = SpeedControlOutNew;
}
//�ٶȿ���
void Speed_Control_Output(void)
{
  float fValue;

  fValue = SpeedControlOutNew - SpeedControlOutOld;
  PID_SPEED.OUT = fValue * (SpeedCount+1)/Speed_Filter_Times+SpeedControlOutOld;
  //Speed_temp = fValue;
}

void Direction_ADControl()
{
  float AD_Turn_P;
  float ratiot=0.048;
  //AD2_Value = adc_once(ADC1_SE5a,ADC_12bit);//Center
  AD1_Value = adc_once(ADC1_SE4a,ADC_12bit);//Left
  AD3_Value = adc_once(ADC1_SE9,ADC_12bit);//Right

  AD1_Value=((AD1_Value<AD1_MIN)?AD1_MIN:AD1_Value)>AD1_MAX?AD1_MAX:AD1_Value;
  //AD2_Value=((AD2_Value<AD2_MIN)?AD2_MIN:AD2_Value)>AD2_MAX?AD2_MAX:AD2_Value;
  AD3_Value=((AD3_Value<AD3_MIN)?AD3_MIN:AD3_Value)>AD3_MAX?AD3_MAX:AD3_Value;

  AD1_Normalized = 100*(AD1_Value)/AD1_MAX;
  //AD2_Normalized = 100*(AD2_Value)/AD2_MAX;
  AD3_Normalized = 100*(AD3_Value)/AD3_MAX;

  if( AD1_Normalized == 0 && AD3_Normalized == 0 )
  {
    AD_Miss = true;
  }
  else if( AD1_Normalized > 10 || AD3_Normalized > 10 )
  {
    AD_Miss = false;
  }

  if( Dir_Fix == 2 )//֮ǰ״̬��������
  {
    if( (AD1_Normalized<20 && (AD1_Normalized>AD3_Normalized)) || AD3_Normalized == 0 || (AD1_Normalized-AD3_Normalized)>30 )//Right Miss;
      Dir_Fix = 1;
    if( (AD3_Normalized<20 && (AD3_Normalized>AD1_Normalized)) || AD1_Normalized == 0 || (AD3_Normalized-AD1_Normalized)>30)//Left Miss
      Dir_Fix = 0;
  }

  Turn_Speed=(Gyro_X-Gyro_X_Offset)*ratiot;

  if( AD1_Normalized > 20 || AD3_Normalized > 20)
  {
    Dir_Fix=2;
    AD_Error=100*(AD3_Normalized-AD1_Normalized)/(AD3_Normalized+AD1_Normalized);
    AD_Error=AD_Error*(AD_Error*AD_Error/1250.0+2)/10;

    My_Push_And_Pull(AD_Error_Filter,8,AD_Error);

    if( AD_Error_FilterCount < 8 )
    {
      AD_Error_FilterCount++;
    }
    else
    {
      AD_Error_Delta = -10*My_Slope_Calculate(0,AD_Error_FilterCount,AD_Error_Filter);//�����ֵ��б��
    }

    AD_Turn_P=PID_AD_TURN.P*ABS(AD_Error);
    if( AD_Turn_P > 0.026 )
    {
      AD_Turn_P = 0.026;
    }

    PID_AD_TURN.pout=AD_Error*AD_Turn_P;
    PID_AD_TURN.dout=AD_Error_Delta*PID_AD_TURN.D + Turn_Speed*PID_TURN.D;

    AD_Turn_Out = PID_AD_TURN.pout+PID_AD_TURN.dout;
    AD_Turn_Out = Turn_Out_Filter(AD_Turn_Out);
    PID_AD_TURN.OUT = AD_Turn_Out;
    
    if(PID_AD_TURN.OUT<AD_OUT_MIN)  PID_AD_TURN.OUT=AD_OUT_MIN;
    if(PID_AD_TURN.OUT>AD_OUT_MAX)  PID_AD_TURN.OUT=AD_OUT_MAX;

  }
  if( Dir_Fix == 1 )//Right Miss
  {
    PID_AD_TURN.OUT = AD_OUT_MIN;
  }
  if( Dir_Fix == 0 )//Left Miss
  {
    PID_AD_TURN.OUT = AD_OUT_MAX;
  }
  //�޷�

  //PID_AD_TURN.OUT=((PID_AD_TURN.OUT<AD_OUT_MIN)?AD_OUT_MIN:PID_AD_TURN.OUT)>AD_OUT_MAX?AD_OUT_MAX:PID_AD_TURN.OUT;

}

/********************�������������***************/
void Direction_Control()
{
  float ratio=0.048;
  static int Calculate_Length=0;
  Turn_Speed=(Gyro_X-Gyro_X_Offset)*ratio;

//  if(RoadType==0)  //ֻ������ͨ�����͵�������ģ��
//  {
//    if(Calculate_Length<8)
//    {
//     Calculate_Length++;
//     Delta_P=0;
//     Delt_error=0;
//     Delta_D=0;
//    }
//    else
//    {
//      Delt_error=-10*Slope_Calculate(0,Calculate_Length,Previous_Error);
//      Delta_P=Fuzzy( Middle_Err,Delt_error)* Fuzzy_Kp*0.1;
//      Delta_D=Fuzzy( Middle_Err,Delt_error)* Fuzzy_Kd*0.1;
//    }
//  }
//  else
//  {
//    Delta_P=0;
//    Delta_D=0;
//    Calculate_Length=0;
//  }

  PID_TURN.pout=(PID_TURN.P)*Middle_Err;
  PID_TURN.dout=(PID_TURN.D)*Turn_Speed;
  Turn_Out= PID_TURN.pout+ PID_TURN.dout;
  Dis_Turn_Out = Turn_Out;
  //Turn_Out=Turn_Out_Filter(Turn_Out);         //ת������˲�
  Turn_Out_tmp2 = Turn_Out_tmp1;
  Turn_Out_tmp1 = Turn_Out_tmp0;
  Turn_Out_tmp0 = Dis_Turn_Out;
  Turn_Out = Turn_Out_tmp0*0.6 + Turn_Out_tmp1*0.3 + Turn_Out_tmp2*0.1;
  PID_TURN.OUT=Turn_Out;

}

//���pwmֵ���
void Moto_Out()
{
  int L_Value,R_Value;
  static float  Forward_Safe_Angle=35;//ǰ��İ�ȫ�Ƕ�
  static int Motor_Abnormal_Cnt=0;    //���ת���쳣����
  int Backward_Safe_Angle=20;          //����İ�ȫ�Ƕ�
  float Sum;
//#if    0
//  //�ٶȿ�������޷�
//  if(PID_SPEED.OUT>PID_ANGLE.P*Forward_Safe_Angle)//�������ǰ�㣬��ģ���ٶȿ������Ϊ������֮Ϊ��
//    PID_SPEED.OUT=PID_ANGLE.P*Forward_Safe_Angle;                       //�Ѿ���б������ȫ�Ƕ���
//  if(PID_SPEED.OUT<-PID_ANGLE.P*Backward_Safe_Angle)
//    PID_SPEED.OUT=-PID_ANGLE.P*Backward_Safe_Angle;
//
//  Sum=PID_ANGLE.OUT - PID_SPEED.OUT;
//
//  LeftMotorOut = Sum+PID_TURN.OUT;   //�������ֵ
//  RightMotorOut= Sum-PID_TURN.OUT;
//
// //��ֵ�޷�����ֹ���ٹ���
//#endif

  //AD_Miss=false;  //����ֱ��
  //CarmeraMiss = 1;
  if(runmode==0) //ֱ����
  {
    if( CarmeraMiss == 1 && !AD_Miss )   //��Ź���
    {
      //�ٶȿ�������޷�
      if(PID_SPEED.OUT>PID_ANGLE.P*Forward_Safe_Angle)//�������ǰ�㣬��ģ���ٶȿ������Ϊ������֮Ϊ��
        PID_SPEED.OUT=PID_ANGLE.P*Forward_Safe_Angle;                       //�Ѿ���б������ȫ�Ƕ���
      if(PID_SPEED.OUT<-PID_ANGLE.P*Backward_Safe_Angle)
        PID_SPEED.OUT=-PID_ANGLE.P*Backward_Safe_Angle;

      Sum=PID_ANGLE.OUT - PID_SPEED.OUT;   //ֱ�� + �ٶ�

      LeftMotorOut = Sum - PID_AD_TURN.OUT;   //��ʧ���źţ������
      RightMotorOut= Sum + PID_AD_TURN.OUT;
    }
    else if( !CarmeraMiss && !AD_Miss )   //����ͷ����
    {
      //�ٶȿ�������޷�
      if(PID_SPEED.OUT>PID_ANGLE.P*Forward_Safe_Angle)//�������ǰ�㣬��ģ���ٶȿ������Ϊ������֮Ϊ��
        PID_SPEED.OUT=PID_ANGLE.P*Forward_Safe_Angle;                       //�Ѿ���б������ȫ�Ƕ���
      if(PID_SPEED.OUT<-PID_ANGLE.P*Backward_Safe_Angle)
        PID_SPEED.OUT=-PID_ANGLE.P*Backward_Safe_Angle;

      Sum=PID_ANGLE.OUT - PID_SPEED.OUT;  //ֱ�� + �ٶ�

      LeftMotorOut  =  Sum  - PID_TURN.OUT;
      RightMotorOut =  Sum  + PID_TURN.OUT;
    }
    else if( AD_Miss == 1 ) //��������,ͣ��
    {
      LeftMotorOut = 0.0f;
      RightMotorOut = 0.0f;
    }
  }
  else  //runmode=1��������,����Ҫֱ������
  {
    if(CarmeraMiss == 1 && !AD_Miss )   //��Ź���
    { //�ٶȿ�������޷�
//      if(PID_SPEED.OUT>PID_ANGLE.P*Forward_Safe_Angle)//�������ǰ�㣬��ģ���ٶȿ������Ϊ������֮Ϊ��
//        PID_SPEED.OUT=PID_ANGLE.P*Forward_Safe_Angle;                       //�Ѿ���б������ȫ�Ƕ���
//      if(PID_SPEED.OUT<-PID_ANGLE.P*Backward_Safe_Angle)
//        PID_SPEED.OUT=-PID_ANGLE.P*Backward_Safe_Angle;
//
//      Sum=PID_ANGLE.OUT - PID_SPEED.OUT;

      if(PID_SPEED.OUT>0.6){ PID_SPEED.OUT=0.6;}
       LeftMotorOut  =   -PID_SPEED.OUT- PID_AD_TURN.OUT +0.1*PID_ANGLE.OUT;  //��ŷ���
       RightMotorOut =   -PID_SPEED.OUT+ PID_AD_TURN.OUT +0.1*PID_ANGLE.OUT;  //��ŷ���
    }
    else if( !CarmeraMiss && !AD_Miss )   //����ͷ����
    {
      LeftMotorOut  =  PID_SPEED.OUT - PID_TURN.OUT ;  //��ŷ���
      RightMotorOut =  PID_SPEED.OUT + PID_TURN.OUT;  //��ŷ���
    }

      if( AD_Miss == 1 )  //��������
    {
      LeftMotorOut = 0.0f;
      RightMotorOut = 0.0f;
    }
  }

  //////      ��������    //////////////////////////
//    LeftMotorOut = PID_ANGLE.OUT;   // ��ֱ��
//    RightMotorOut= PID_ANGLE.OUT;

   // LeftMotorOut  =  -PID_TURN.OUT;   // ��ת��
   // RightMotorOut =  +PID_TURN.OUT;



  if(RightMotorOut>0.99)RightMotorOut=0.99;
  if(RightMotorOut<-0.99)RightMotorOut=-0.99;
  if(LeftMotorOut>0.99)LeftMotorOut=0.99;
  if(LeftMotorOut<-0.99)LeftMotorOut=-0.99;

  L_Value=(int)(10000*LeftMotorOut);
  R_Value=(int)(10000*RightMotorOut);



 /////////////////////����////////////////////////////
  if(Car_Angle<-40||Car_Angle>30)             //����
  {
    if(Stop==false&&RunTime>2)
    {
      Stop=true;
      CarStopedJustNow=true;
      Motor_Abnormal_Cnt=0;
     }
  }
  if((Distance>1)&&(Stop==false))               //              ����Ƿ����ٶ��쳣
  {
    if((CarSpeed<0.5)||(CarSpeed>3))
    {
       Motor_Abnormal_Cnt++;
       if(Motor_Abnormal_Cnt>250) //����쳣�ĳ���ʱ�����500ms��
       {
          Stop=true;
          CarStopedJustNow=true;
       }
    }
    else
    {
       Motor_Abnormal_Cnt=0;
    }
 }
 if(Stop)                                //���ֹͣ���������
 {
    //  L_Value=0;
    //R_Value=0;
 }

   if(L_Value>=0) //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,L_Value);//ռ�ձȾ���Ϊ10000
     FTM_PWM_Duty(FTM0,FTM_CH1,0);
  }
  else   //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH0,0);
     FTM_PWM_Duty(FTM0,FTM_CH1,-L_Value);
  }
  if(R_Value>=0) //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH2,R_Value);
     FTM_PWM_Duty(FTM0,FTM_CH3,0);
  }
  else   //��ת
  {
     FTM_PWM_Duty(FTM0,FTM_CH2,0);
     FTM_PWM_Duty(FTM0,FTM_CH3,-R_Value);
  }
}

float My_Slope_Calculate(uint8 begin,uint8 end,float *p)    //��С���˷����б��
{
  float xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
	   xsum+=i;
	   ysum+=*p;
	   xysum+=i*(*p);
	   x2sum+=i*i;
	   p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}

void My_Push_And_Pull(float *buff,int len,float newdata)
{
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata;
}

float  Turn_Out_Filter(float turn_out)    //ת���������˲�
{
  float Turn_Out_Filtered;
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=Pre1_Error[0]*0.4+Pre1_Error[1]*0.3+Pre1_Error[2]*0.2+Pre1_Error[3]*0.1;
  return Turn_Out_Filtered;
}

float  Middle_Err_Filter(float middle_err)    //����ƫ���˲�
{
  float Middle_Err_Fltered;
  static float Pre3_Error[4];
  Pre3_Error[3]=Pre3_Error[2];
  Pre3_Error[2]=Pre3_Error[1];
  Pre3_Error[1]=Pre3_Error[0];
  Pre3_Error[0]=middle_err;
  Middle_Err_Fltered=Pre3_Error[0]*0.4+Pre3_Error[1]*0.3+Pre3_Error[2]*0.2+Pre3_Error[3]*0.1;
  return Middle_Err_Fltered;
}
