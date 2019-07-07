/********************************************
*  OV7725 & ELE 2019.3
* @file       main.c
* @brief      ������
********************************************/

#include "common.h"
#include "include.h"

uint8 runmode;  //0: ֱ����  1��������
uint8 lockrun;  //0:����ı�runmode   1:������ı�
char zha=0;    //0,û�������ϰ�� 1�������ϰ���
int time;
int Dutime; //��·��ʱ
int yanshi;

void  main(void)
{  uint8 cc,dd;
   runmode  = 0; //��ʼʱֱ���� //0: ֱ����  1��������
   lockrun = 1; //���Ըı�״̬��������ͷ������ʱ //0:����ı�runmode   1:������ı�
   DELAY_MS(100);
   init();
   while(1)
    {
      if(beep)
      {
        BEEP_ON;
        DELAY_MS(50);
        BEEP_OFF;
        beep=0;
      }
      Check_BottonPress();
      if(new_img)  //�˶β�����0.5ms 200ms��Ƶ  6ms ��8ms ִ��һ��
      {
        get_edge();
        
       
        
        new_img=0;
        Variable_update();
        enable_irq(PORTC_IRQn);
        EnableInterrupts;
      }
      if(OLED_Refresh)
      {
        img_extract(img,imgbuff_process,CAMERA_SIZE);
        OLED_Draw_UI();//��ʾͼ��
      }
     }
}


void PIT_IRQHandler()  //2msһ���ж�
{
  static uint8 flag_100ms,cnt=0;
  static char STcar=0; //���ֱ�ֱ��ɲ��
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
  RunTime=RunTime+0.002;
  
  if(zha==0)
  {
    int redflag;
    redflag=Red_Check();
    
     if(lockrun==0){STcar=judgeblack(); }//�л�
     
          if(!redflag){ //����ϰ����ж�
           
                  if( RunTime < 2.0f )
                  {
                    PID_SPEED.I = 0.0f;
                  }
                  else
                  {
                    PID_SPEED.I = 0.01f;
                  }

                  flag_100ms++;
                  if(flag_100ms>50)
                  {
                   flag_100ms=0;
                  // Speed_Control();  //100ms����һ���ٶȿ���
                   SpeedCount=0;
                  }
                   cnt++;
                   if(cnt==1)      //4ms����һ��
                   {
                     Get_Attitude(); //
                     Angle_Calculate();
                     Angle_Control();
                     Direction_ADControl_zl(); 
                   }
                   if(cnt>=2)
                   {
                     cnt=0;
                   }
                   
                   SpeedCount++;
                   Get_Speed();
                   Speed_Control();
                   Speed_Control_Output();
                   
                   if(STcar==0){ Moto_Out_Control();}
          }
          
          else if(redflag) {
                 
            zha=BiZhang();
            time=0;
          }
  }
  
  else if(zha==1)
  {
        time++;
        zha=go_block();
      
  }
           Moto_Out();
}
