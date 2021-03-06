/********************************************
*  OV7725 & ELE 2019.3
* @file       main.c
* @brief      主函数
********************************************/

#include "common.h"
#include "include.h"

uint8 runmode; //0: 直立跑  1：三轮跑
uint8 lockrun; //0:允许改变runmode   1:不允许改变
int zha = 0;   //0,没有遇到障碍物， 1，遇到障碍物
int time;
int Dutime; //断路延时
int yanshi;

void main(void)
{
  uint8 cc, dd;
  runmode = 1; //开始时直立跑 //0: 直立跑  1：三轮跑
  lockrun = 1; //可以改变状态（当摄像头看到黑时 //0:允许改变runmode   1:不允许改变
  DELAY_MS(100);
  init();
  while (1)
  {
    if (beep)
    {
      BEEP_ON;
      DELAY_MS(50);
      BEEP_OFF;
      beep = 0;
    }
    Check_BottonPress();
    if (new_img) //此段不超过0.5ms 200ms主频  6ms 或8ms 执行一次
    {
      get_edge();

      new_img = 0;
      Variable_update();
      enable_irq(PORTC_IRQn);
      EnableInterrupts;
    }
    if (OLED_Refresh)
    {
      img_extract(img, imgbuff_process, CAMERA_SIZE);
      OLED_Draw_UI(); //显示图像
    }
  }
}

void PIT_IRQHandler() //2ms一次中断
{
  static uint8 flag_100ms, cnt = 0;
  static char STcar = 0; //三轮便直立刹车
  PIT_Flag_Clear(PIT0);  //清中断标志位
  RunTime = RunTime + 0.002;

  // if(lockrun==0){STcar=judgeblack(); }//切换

  if (RunTime < 2.0f)
  {
    PID_SPEED.I = 0.0f;
  }
  else
  {
    PID_SPEED.I = 0.01f;
  }

  flag_100ms++;
  if (flag_100ms > 50)
  {
    flag_100ms = 0;
    // Speed_Control();  //100ms进行一次速度控制
    SpeedCount = 0;
  }

  cnt++;
  if (cnt == 1) //4ms运行一次
  {
    Get_Attitude(); //
    Angle_Calculate();
    Angle_Control();
    Direction_ADControl_zl();
  }
  if (cnt >= 2)
  {
    cnt = 0;
  }

  SpeedCount++;
  Get_Speed();
  Speed_Control();
  Speed_Control_Output();

  if (zha == 0)   //正常赛道
  {
    if (Red_Check() && (Car_Angle < 10))   //红外检测到障碍物，且不是坡道
    {
      zha = 1;   //跳到避障
    }
    if (judgeramp() == 1)  //爬坡
    {
      zha = 2;
    }
    PID_SPEED.OUT = 0.23;  
    time = 0;
    Moto_Out_Control();
  }

  else if (zha == 1)  //避障
  {
    Red_Check();
    zha = go_block();
  }
  else if (zha == 2)  //爬坡
  {
    zha = Ramp();
    Moto_Out_Control();
  }
  else if (zha == 3)  //过欧姆环
  {
    PID_SPEED.OUT = 0.2f;  //过欧姆环速度
    Moto_Out_Control();
  }
  
  Moto_Out();
}
