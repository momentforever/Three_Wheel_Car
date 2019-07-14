#include "common.h"
#include "include.h"
float sVariable[20]; //����������
float Variable[20];  //
float Control_Para[15];
float Voltage;
float RunTime;
int   Start_Cnt=0;
uint8 Para_Index_Limit=7;       //һҳ�����7���������
uint8 Page_Index=3,Para_Index=1,Light_Tower_Index=0,Para_Checked=0,OLED_Refresh=0;
uint8 Uart_Send=0,SendPara,stop_contorl,send_data_contorl=0,SendSD,SD_Save=0,beep=0,SD_Write_info=0;
float Step[6]={0.0001,0.001,0.01,0.1,1.0,10.0};   //Ĭ�ϵ��ڲ���Ϊ0.01
unsigned char Step_Index=2;
//��Ӧ��ͬ��ҳ��
char Para_Name[7][12]={"PID_ANGLE.P\0","PID_ANGLE.I\0","PID_ANGLE.D\0","PID_SPEED.P\0",
"PID_SPEED.I\0","PID_DIREC.P\0","PID_DIREC.D\0"};
char Debug_Mode[4][12]={"Normal  \0","UpRight\0","NoSpeed\0","NoDirec\0"};  //����ģʽ ���� ֱ�� û���ٶ� ��û�з���

char Para_Name1[7][12]={"SetSpeed\0","AD_TURN.P\0","AD_TURN.D\0","Fuzzy_kp",
"Fuzzy_kd","STurnSpeed\0","BTurnAngle\0"};


extern  float Speed_temp;
extern uint8 disyuzhi;
extern float Dis_Turn_Out;
//extern float Delt_error,Middle_Err;


void my_putchar(char temp)
{
      uart_putchar(UART0,temp); //����ʵ�ʵĴ��ں����޸�
}
/*����֪ͨ��λ���µ�һ�����ݿ�ʼ��Ҫ�������ݱ��뷢����*/
void Send_Begin()
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}






void Variable_update()
{
  Variable[0]=Angle;
  Variable[1]=Car_Angle;
  Variable[2]=Angle_Speed;
  Variable[3]=Middle_Err;
  Variable[4]=Delt_error;
  Variable[5]=RunTime;
  Variable[6]=Turn_Speed;
  Variable[7]=CarSpeed;
  Variable[8]=PID_ANGLE.OUT;
  Variable[9]= PID_SPEED.OUT;
  Variable[10]= PID_TURN.OUT;
  Variable[15]=RunTime;
  Variable[16]=turn_num;
}


void Para_Update()
{
  //SetSpeed=Control_Para[0];
  PID_ANGLE.P=Control_Para[0];
  PID_ANGLE.I=Control_Para[1];
  PID_ANGLE.D=Control_Para[2];
  PID_SPEED.P=Control_Para[3];
  PID_SPEED.I=Control_Para[4];
  PID_TURN.P=Control_Para[5];
  PID_TURN.D=Control_Para[6];

  SetSpeed = Control_Para[7];
  PID_AD_TURN.P=Control_Para[8];
  PID_AD_TURN.D=Control_Para[9];
}


void OLED_Draw_UI()  //��������
{
   uint8 i;
   if(Page_Index<=1)
   {
     Voltage=adc_once(ADC1_SE14,ADC_12bit);
     Voltage=Voltage*Vol_ratio; //ת��Ϊʵ�ʵ�ѹ
     OLED_P6x8Str(0,0,"Voltage=");                          //��ʾ��ص�ѹ
     OLED_PrintValueF(48, 0,Voltage,2);
     OLED_PrintValueF(72, 0,Step[Step_Index],5);            //��ʾ���ڲ���ֵ
     if(Para_Index==7)
     {
        reverse=1;
        OLED_P6x8Str(116,0,"EE");
        reverse=0;
     }
     else
     {
        OLED_P6x8Str(116,0,"EE");
     }
     OLED_Set_Pos(122,7);
     OLED_P6x8Char(Page_Index+48);                         //д��ҳ�����
   }
  /////////////////////////////////////////////////////////��0ҳ  PID����
  if(Page_Index==0)
  {
    for(i=0;i<7;i++)
    {
      if(i==Para_Index&&Para_Checked==false)
      {
       reverse=1;
       OLED_P6x8Str(0,i+1,Para_Name[i]);   //����������ת��ʾ
       reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Para_Name[i]);



      if(i==Para_Index&&Para_Checked)
      {
        reverse=1;
        OLED_PrintValueF(72, i+1,Control_Para[i],5);
        reverse=0;
      }
      else  OLED_PrintValueF(72, i+1,Control_Para[i],5);

      OLED_Set_Pos(116,i+1);

    }
  }
  /////////////////////////////////////////////////////////��1ҳ   ��������
    else if(Page_Index==1)
  {
     for(i=0;i<7;i++)
    {
     if(i==Para_Index&&Para_Checked==false)
      {
       reverse=1;
       OLED_P6x8Str(0,i+1,Para_Name1[i]);   //����������ת��ʾ
       reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Para_Name1[i]);

      if(i==Para_Index&&Para_Checked)
      {
        reverse=1;
        OLED_PrintValueF(72, i+1,Control_Para[i+7],5);
        reverse=0;
      }
      else  OLED_PrintValueF(72, i+1,Control_Para[i+7],5);

      OLED_Set_Pos(116,i+1);
    }



    }
    /////////////////////////////////////////////////////////��3ҳ ״̬��ʾ
  else if(Page_Index==2)
  {
    OLED_P6x8Str(0,0,"CarAngle");
    OLED_PrintValueF(72,0,Car_Angle,3);
    OLED_P6x8Str(0,1,"GyroSpeed");
    OLED_PrintValueF(72, 1,Angle_Speed,3);
    OLED_P6x8Str(0,2,"CarSpeed");
    OLED_PrintValueF(72, 2,CarSpeed,3);
    OLED_P6x8Str(0,3,"Distance");
    OLED_PrintValueF(72, 3,Distance,4);
    OLED_P6x8Str(0,4,"RunTime");
    OLED_PrintValueF(72, 4,RunTime,4);
    OLED_P6x8Str(0,5,"turnnum");
    OLED_PrintValueF(72, 5,turn_num,4);
    OLED_P6x8Str(0,6,"RedSan");
    OLED_PrintValueF(72, 6,RedSan,4); //��������
    OLED_P6x8Str(0,7,"CarmeraMiss");
    OLED_PrintValueIForAny(72, 7,CarmeraMiss,3);
    reverse=0;

  }
  ////////////////////////////////////////////////////////////��4ҳ ��������ʾ
  else if(Page_Index==3)
  {
    OLED_Draw_camera();
    //oledimg();
    OLED_PrintValueF(84, 2,Middle_Err,4);
    OLED_PrintValueF(84,4,Turn_Speed,4);
    OLED_PrintValueF(84,5,PID_TURN.OUT,4);
    OLED_PrintValueF(84,6,Dis_Turn_Out,4);
  }
  else if( Page_Index == 4 )
  {
    OLED_P6x8Str(0,0,"AD1:");
    OLED_PrintValueI_CLS(25,0,AD1_Value);
    OLED_P6x8Str(64,0,"AD3:");
    OLED_PrintValueI_CLS(89,0,AD3_Value);
    OLED_P6x8Str(0,1,"AD1_N:");
    OLED_PrintValueIForAny(35,1,AD1_Normalized,3);
    OLED_P6x8Str(64,1,"AD3_N:");
    OLED_PrintValueIForAny(99,1,AD3_Normalized,3);
    OLED_P6x8Str(0,2,"AD_Miss:");
    OLED_PrintValueIForAny(64,2,AD_Miss,3);
    OLED_P6x8Str(0,3,"SPEED.OUT:");
    OLED_PrintValueF(64,3,PID_SPEED.OUT,5);
    OLED_P6x8Str(0,4,"Del:");
    OLED_PrintValueF(25,4,AD_Error_Delta,5);
    OLED_P6x8Str(70,4,"Err:");
    OLED_PrintValueIForAny(99,4,AD_Error,3);
    OLED_P6x8Str(0,5,"TURN.OUT:");
    OLED_PrintValueF(64,5,PID_AD_TURN.OUT,5);
    OLED_P6x8Str(0,6,"LeftMotorOut:");
    OLED_PrintValueF(64,6,LeftMotorOut,5);
    OLED_P6x8Str(0,7,"RightMotorOut:");
    OLED_PrintValueF(64,7,RightMotorOut,5);
    reverse=0;
  }
  else if(Page_Index==5)
  {  OLED_P6x8Str(0,0,"Page 5");
    //OLED_PrintValueF(90, 2,Middle_Err,4);
    //OLED_PrintValueC(90,3,disyuzhi);
    //OLED_PrintValueF(90,4,Turn_Speed,4);
    //OLED_PrintValueF(90,5,PID_TURN.OUT,4);
    //OLED_PrintValueF(90,6,Dis_Turn_Out,4);
  }

}


/*
 * �����뿪�ص�ֵ
 */
void Read_Switch()
{
}

/*
 * ��ⰴ���Ƿ���
 */
void Check_BottonPress()
{

      //��ʾ����
     if(BT_SHOW_IN==0)
   {
      //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_SHOW_IN==0)
      {
          if(OLED_Refresh==false)
        {
         OLED_Init();
         OLED_Refresh=true;
        }
        else
        {
          OLED_Refresh=false;
          OLED_Fill(0x00);
        }
      }
      while(BT_SHOW_IN==0);  //ֱ�������ɿ�������
      DELAY_MS(30);
   }

   //����1 yes
   if(BT_YES_IN==0)
   {
     //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_YES_IN==0&&OLED_Refresh)
     {
       if(Para_Index==7)
       {
         EEPROM_Save();
         Para_Index=0;
       }
       else
       {

         if(Para_Checked==false&&((Page_Index==1)||(Page_Index==0))) Para_Checked=true;
         else Para_Checked=false;

       }
      }
      else
      {
        if(Stop==true)
       {
         Start_Cnt=1000;
         Starting=true;
         Stop=false;
         //�������ٶȿ��Ʊ�������
         SpeedControlOutOld=0;
         SpeedControlOutNew=0;
         SpeedControlIntegral=0;
         PID_SPEED.OUT=0;
         Distance=0;
         RunTime=0;
         ControlSpeed=0;
       }
       else
       {
        Stop=true;
        CarStopedJustNow=true;     //С����ֹͣ
       }

      }
      while(BT_YES_IN==0); //ֱ�������ɿ�������
   }
   //����2 Left_L
     if(BT_LEFT_IN==0&&OLED_Refresh)
   {
      //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_LEFT_IN==0)
      {
        if(Para_Checked)
        {
          if(Step_Index==5)
          Step_Index=5;   //���Ĳ���Ϊ10
          else Step_Index++;
        }
        else
        {
          Para_Index=0;
          if(Page_Index==0) Page_Index=5; //������û��ѡ�е�ʱ�򣬰����Ҽ���ҳ
          else Page_Index--;
          OLED_Fill(0);//����
        }
      }
      while(BT_LEFT_IN==0);//ֱ�������ɿ�������
   }
   //����6 Right_L
     if(BT_RIGHT_IN==0&&OLED_Refresh)
   {
      //ȥ��
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_RIGHT_IN==0)
      {
        if(Para_Checked)
        {
          if(Step_Index==0)
           Step_Index=0;//��С�Ĳ���Ϊ0.0001
          else
          {
            Step_Index--;
          }
        }
        else
        {
          Para_Index=0;
          if(Page_Index==5) Page_Index=0;
          else Page_Index++;
         OLED_Fill(0);//����
        }
      }
      while(BT_RIGHT_IN==0);      //ֱ�������ɿ�������
   }
   //����3 up
     if(BT_UP_IN==0&&OLED_Refresh)
   {
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_UP_IN==0)
      {

          if(Para_Checked==false)
          {
           if(Para_Index==0) Para_Index=Para_Index_Limit;
           else Para_Index-=1;
          }
          else
          {
              if(Page_Index==0&&Para_Index<=6)                    //�޸ĵ�0ҳ�Ĳ���
            {
              Control_Para[Para_Index]+=Step[Step_Index];
            }

            if(Page_Index==1&&Para_Index<=6)                    //�޸ĵ�1ҳ�Ĳ���
            {
              Control_Para[Para_Index+7]+=Step[Step_Index];
            }
            Para_Update();
          }
      }
      while(BT_UP_IN==0);//ֱ�������ɿ�������
   }
   //����4 down
     if(BT_DOWN_IN==0&&OLED_Refresh)
   {
       BEEP_ON;
       DELAY_MS(30);
       BEEP_OFF;
      if(BT_DOWN_IN==0)
      {
          if(Para_Checked==false)
          {
            if(Para_Index==Para_Index_Limit)Para_Index=0;   //��ֹ��ų�����Χ
            else  Para_Index+=1;
          }
           else
           {
              if(Page_Index==0&&Para_Index<=6)                    //�޸ĵ�0ҳ�Ĳ���
            {
              Control_Para[Para_Index]-=Step[Step_Index];
            }

             if(Page_Index==1&&Para_Index<=6)                    //�޸ĵ�1ҳ�Ĳ���
            {
              Control_Para[Para_Index+7]-=Step[Step_Index];
            }
            Para_Update();
          }
      }

      while(BT_DOWN_IN==0);  //ֱ�������ɿ�������
   }
}



void Send_Variable()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Variable_num=16;
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0x01);
  my_putchar(Variable_num);
 for(i=0;i<Variable_num;i++)
  {
    temp=Variable[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
     my_putchar(0x01);
}




void Modify_Parameter(uint8 *buff)
{
   uint8 i=0,addr=0;
   float temp;
   uint8 Parameter_num=14; //14���ɸĲ���
  /*          �޸Ĳ�������         */
   for(i=0;i<Parameter_num;i++)
  {
       BYTE0(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE1(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE2(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE3(temp)=*(uint8*)(buff+addr);
       addr++;
       Control_Para[i]=temp;
   }
    Para_Update();
}



void Send_Parameter()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Parameter_num=14;  //14���ɸĲ���


  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0x02);
  my_putchar(Parameter_num);
  for(i=0;i<Parameter_num;i++)
  {
     temp=Control_Para[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
    my_putchar(0X02);//֡β
}

void UART0_RX_IRQHandler()
{
  static uint8 recv;
  static uint8 data_cnt=0;
  static uint8 predata[10];
  static uint8 Recv_Buff[100];
  static uint8 Data_Receiving=false;

  if(uart_query(UART0)==1)  uart_getchar (UART0,(char*)(&recv));  //����ʵ�ʵĴ������޸�
  /**********�������ڽ���������λ���Ĳ�������*********/
  if(Data_Receiving)
  {
      if(data_cnt<56)
      {
       Recv_Buff[data_cnt]= recv;
       data_cnt++;
      }
      else
      {
        data_cnt=0;    //�ﵽ֡��
        Data_Receiving=false;
        if(recv==2)  //֡β
        {
           Modify_Parameter(Recv_Buff);
           SendPara=1;      //�����ش���ȷ�ϲ����޸����
            beep=1; //����������
        }
      }
  }



    if( predata[1]==0x55&&predata[0]==0xAA)
    {

        switch(recv)         //�жϹ�����
         {
            case 1:           //��ȡ����
               if(SendPara==0) SendPara=1;
                beep=1; //����������
              break;

            case 2:           //�޸Ĳ���
              data_cnt=0;
              Data_Receiving=true;
            case 3:           //�������
              //EEPROM_Save();
              beep=1; //����������
            break;

            case 4:           //���ܿ���1

            break;

            case 5:           //���ܿ���2

            break;

            case 6:           //���ܿ���3

            break;

            case 7:           //���ܿ���4

            break;

            default:           //
             break;
          }
    }
  predata[1]=predata[0];
  predata[0]=recv;
}
