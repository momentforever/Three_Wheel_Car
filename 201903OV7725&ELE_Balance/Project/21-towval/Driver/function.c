#include "common.h"
#include "include.h"

//配置中断复位函数

void PORTC_IRQHandler();
void DMA0_IRQHandler();
void PIT_IRQHandler();
void FTM_IN_Init();
void ADC_Init();
void FTM1_IRQHandler();

unsigned int leftspeed,rightspeed;
extern float  Set_Angle;   //加速前倾角度
//float mycarspeed;

void init()
{   
   OLED_Init();
   button_init(); 
   //switch_init();
   BEEP_ON;
   DELAY_MS(70);
   BEEP_OFF;
   DELAY_MS(170);
   BEEP_ON;
   DELAY_MS(70);
   BEEP_OFF;
   //OLED_Draw_Logo();
   led_init(); 
   led_flash(); 
   DisableInterrupts;
   
   adc_init (ADC1_SE14);// 电池电压采样接口 装换比例为3.3/65535*5.7 
   adc_init (ADC1_SE9);//  PB1 电磁最右电感
   adc_init (ADC1_SE4a);// PE0  电磁最左电感
   
    adc_init(ADC1_SE5a);//E1  L3  E1红外采集
    adc_init(ADC1_SE6a);//E2  L4
    adc_init(ADC1_SE7a);//E3  
    adc_init(ADC1_SE8);//B0  L5
   
   //adc_init(); //红外初始化
   
   pit_init_ms(PIT0,2); //2ms定时中断
   
   set_vector_handler(PIT0_VECTORn ,PIT_IRQHandler);
   
   EEPROM_init();
   Para_Init();
   I2C_Init();
   
   //电机机PWM 频率为10khz 占空比精度为10000 
   FTM_PWM_init(FTM0,FTM_CH0,10*1000,0);   
   FTM_PWM_init(FTM0,FTM_CH1,10*1000,0);  
   FTM_PWM_init(FTM0,FTM_CH2,10*1000,0);  
   FTM_PWM_init(FTM0,FTM_CH3,10*1000,0);   
   
   //FTM_QUAD_Init(FTM1); //正交解码初始化; //
   //FTM_QUAD_Init(FTM2); //正交解码初始化; // 
   //FTM_Input_init(FTM1,FTM_CH0,Rising,1);
   //FTM_Input_init(FTM1,FTM_CH1,Rising,1);
   
   FTM_IN_Init();
   //set_vector_handler(FTM1_VECTORn ,FTM1_IRQHandler);  
   BEEP_OFF;  
   OLED_CLS();
   OLED_P6x8Str(0,0,"K60VLL OV7725&ELE");
   camera_init();
   set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);    
   set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);   
   enable_irq(PORTC_IRQn); 
   OLED_CLS();
   set_vector_handler(UART0_RX_TX_VECTORn,UART0_RX_IRQHandler);
   uart_rx_irq_en(UART0);
   enable_irq (PIT0_IRQn);
   NVIC_SetPriority(PIT0_IRQn,2);
   NVIC_SetPriority(DMA0_VECTORn,1);
   NVIC_SetPriority(PORTC_VECTORn ,0); 
   EnableInterrupts; //打开中断  
   OLED_Refresh=true;
}

void Para_Init()
{
  PID_ANGLE.P=0.09;//0.16; //0.172
  PID_ANGLE.I=0;//0.0008;
  PID_ANGLE.D=0.0021;//0.0015;
 
  PID_SPEED.P=0.020;//0.12//0.3;
  PID_SPEED.I=0.0018;//0.028;
 
  PID_TURN.P=0.004; //0.032;
  PID_TURN.D=0.011;//-0.0028;//0.00; 
 
  PID_AD_TURN.P=0.0020;//0.0031
  PID_AD_TURN.D=0.0021;//0.03
 
  Fuzzy_Kp=0.005;
  Fuzzy_Kd=0.0005;
 
  SetSpeed=2;//1.3;
  Set_Angle=-21;//16;  -10
  
  //Control_Para[0]=SetSpeed;
  Control_Para[0]=PID_ANGLE.P;
  Control_Para[1]=PID_ANGLE.I;
  Control_Para[2]=PID_ANGLE.D;
  Control_Para[3]=PID_SPEED.P;
  Control_Para[4]=PID_SPEED.I;
  Control_Para[5]=PID_TURN.P;
  Control_Para[6]=PID_TURN.D;
  
  Control_Para[7]=SetSpeed;
  Control_Para[8]=PID_AD_TURN.P;
  Control_Para[9]=PID_AD_TURN.D;
  //mycarspeed = 1.5;
}

void ADC_Init()
{
  //adc_init(ADC1_SE4a);//E0
  adc_init(ADC1_SE5a);//E1
  adc_init(ADC1_SE6a);//E2
  adc_init(ADC1_SE7a);//E3
  adc_init(ADC1_SE8);//B0
  //adc_init(ADC1_SE9);//B1
}

void FTM_IN_Init()
{  SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;
   SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
   
   PORTA_PCR12 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;   //PA12
   FTM1_C0SC |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK);  //上升沿触发
   FTM1_C0SC &= ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK);
   
   PORTA_PCR13 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;   //PA13
   FTM1_C1SC |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK);  //上升沿触发
   FTM1_C1SC &= ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK);
   
   FTM1_SC = FTM_SC_CLKS(1);
   FTM1_MODE |= FTM_MODE_WPDIS_MASK;
   FTM1_COMBINE = 0;
   FTM1_MODE &= ~FTM_MODE_FTMEN_MASK;
   FTM1_CNTIN = 0;
   FTM1_STATUS = 0x00; //清中断标志位
   set_vector_handler(FTM1_VECTORn ,FTM1_IRQHandler); 
   enable_irq(FTM1_IRQn);  //FTM1中断
}

float Slope_Calculate(uint8 begin,uint8 end,float *p)    //最小二乘法拟合斜率
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
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零 
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

void FTM1_IRQHandler()
{
   unsigned char curstatus = FTM1_STATUS;
   FTM1_STATUS = 0x00;  //清中断
   if(curstatus&(1<<0))  //CH0中断,左测速
   { leftspeed++;
   }
   if(curstatus&(1<<1))  //CH1中断，右测速
   { rightspeed++;
   }
}

