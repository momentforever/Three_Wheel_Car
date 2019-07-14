#include "include.h"
uint8  RoadType=0;
float Previous_Error[12];


uint16 edgposition[CAMERA_H];


uint16 cont;
#define NORMAL_NUM 40

extern uint8 runmode;  //0: ֱ����  1��������
extern uint8 lockrun;  //0:����ı�runmode   1:������ı�

//extern float Delt_error,Middle_Err;

void Push_And_Pull(float *buff,int len,float newdata)
{
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata; 
}
   
float Slope_Calculate_Uint8(uint8 begin,uint8 end,uint8 *p)    //��С���˷����б��
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

void sendimg()
{
   uint8 ch=0;
  float temp=0;
  uint16 i=0,num;
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0xa2);
  my_putchar(0x1); //С��״̬
  
  num=cont+2+180+36;  
  //ͳ�ƽ�Ҫ����������� 2����ΪҪ����ؼ��ּ�0xf0��0xf2
  //180�Ǳ��ߵ�λ 36�Ǳ�����λ ���������Ͳ�Ҫ���ϣ�
  
  my_putchar(BYTE0(num)); 
  my_putchar(BYTE1(num));
 for(i=0;i< cont;i++)
 {
     my_putchar(img_edg[i]);
 }
 my_putchar(0xf0);  //����ͼ�����ݷ�����
 /******************�Ǻ�Χ�����Ŀ��Բ�����*******************/
 for(i=0;i<180;i++)
 {
  my_putchar( LMR[i/60][i%60]);
 }
  for(i=0;i<9;i++)
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
 /*****************************************************/
 my_putchar(0xf2); //�����������ݶ�������
 
}
void get_edge()   //�������ٳ˷�����
{
  
  static uint16 i=0,j=0,last=0,x=0,y=0;
  int16 n=0;
  uint8 temp=0,find=0;
  cont=0;
  for(i=0;i<60;i++)
  {
    last=0; 
    x=i*10;
    find=0;
    edgposition[i]=0;
    edgposition[i]=0;
    for(j=0;j<10;j++)
    {
      if(imgbuff_process[x+j]==0xff)
      {
        if(last==0)
        {
              y=j<<3;
              if(find==0)
              {
                edgposition[i]=cont;
              }
              img_edg[cont++]=y;   //���ƶ�5�൱�ڳ���32 ���ƶ�3�൱�ڳ���8 
              find=1;
        }
         last=1;
         continue;
      }
       if(imgbuff_process[x+j]==0)
      {
        if(last==1)
        {          
               y=j<<3;
              if(find==0)
              {
                edgposition[i]=cont;
              }
              img_edg[cont++]=y;   //���ƶ�5�൱�ڳ���32 ���ƶ�3�൱�ڳ���8  
              find=1;
        }
         last=0;
         continue;
      }
      
      for(n=7;n>=0;n--)
      {
            temp=(imgbuff_process[x+j]>>n)&1;// ��ȡ�õ�����ֵ ��0��1��     
            if(temp!=last) //����һ��ֵ����ͬ ������������            
            {
               y=j<<3;  
               if(find==0)
              {
                edgposition[i]=cont;
              }
               img_edg[cont++]=y+7-n;   //���ƶ�5�൱�ڳ���32 ���ƶ�3�൱�ڳ���8 
               find=1;
            }
              last=temp;                //�洢�õ��ֵ
      } 
    } 
    img_edg[cont++]=0xff;   //���ƶ�5�൱�ڳ���32 ���ƶ�3�൱�ڳ���8

  }
}
/*img_edg��һ��һά���� ��¼������ͷÿ�е������ص�����ֵ  ÿ�������� �������ؿ�ʼ���ɺڱ�ף�Ȼ����½��أ��ɰױ�ڣ� 
   0xff����ָʾ���е����������ˣ���ʼ��¼��һ��
   ���ÿ��ͼ���԰�ɫ���ֿ�ʼ����ô������������ʼλ��Ϊ0
   �������ȫ�� ��ô���м�¼Ϊ0xff
   �������Ϊȫ�� ��¼Ϊ 0 0xff
 
   
  oxff����������ֵ������������һ��
 //edgposition[i]�����i�е� ������ �� img_edg ���������

 */
void fix_break_line() //�޸��Ͽ�����
{
  
}
void Search()
{
  //�ӵײ���������
  float Middle_Err_Sum=0;
  static int i,j,n,m,find;
  //uint8 left_cont=0,right_cont=0;
  //uint8 *startedge;
  //startedge=img_edg;
  //static float slopeleft,sloperight,left_Pre,right_Pre;
  //uint8 Turning_Line_Left=0,Turning_Line_Right=0;
  //uint8 break_line_left=0,break_line_right=0;
  uint8 left_fix_begin=0,left_fix_end=0,right_fix_begin=0,right_fix_end=0;
  char BlackCount=0;
  for(i=0;i<60;i++)  //�������
  {   
    LMR[0][i]=0; //���������
    LMR[1][i]=0;  //��������
    LMR[2][i]=80; //�ұ�������
  }
  for(i=59;i>0;i--) //�ӵ�59�п�ʼ����
  {
    if(edgposition[i]==0&&(i!=0)) //ȫ���� ��Ϊ����
    {
      BlackCount++;
      if( BlackCount >= 10 )  
      {
        CarmeraMiss = true;  //��10������ȫ���У������·����ſ�ʼ����
        if(lockrun==0)
        { runmode++;//�л�С��״̬
          if(runmode>1) runmode=0; //0��ֱ��  1������
          lockrun=1; //������������ı�runmode,ֱ��С�����¿�������
        }
        break;
      }     
    }
    if( i == 30 )
    {
      CarmeraMiss =false;
      lockrun=0;  //�������ˣ����µȴ���һ��ȫ��
    }
    j=edgposition[i];//���������ؿ�ʼ��λ��  j����ڱ�� ������� j+1����ٱ�� ���ұ��� 

    if(i==59)  //�ײ���ʼ��
    {   
       while(img_edg[j]!=255)
       {  
         if((img_edg[j]<55)&&(img_edg[j+1]>25))  //�����С��55 �ұ��ش���25
         {
           if((img_edg[j+1]-img_edg[j])>25) //�ұ���-����ش���20
           {
             LMR[0][i]=img_edg[j];
             if(img_edg[j+1]==255)
             {
                LMR[2][i]=80;      
             }
             else
             {
                LMR[2][i]=img_edg[j+1];
             }
             break;//while
           }
         } 
         if(img_edg[j+1]==255)
         {
           break;//while
         }
         j=j+2;
       }
    }
    else   //���ǵײ���ʼ��
    { 
        find=0;
        while(img_edg[j]!=255)
       {  
        if((img_edg[j]<=LMR[2][i+1])&&(img_edg[j+1]>=LMR[0][i+1])&&(img_edg[j+1]-img_edg[j]>3))     //�����С����һ�е��ұ��� �ұ��ش�����һ�е����������ͨ��
        {
          find=1;
          if(LMR[0][i]==0)
          {
             LMR[0][i]=img_edg[j];
          }
          if(img_edg[j+1]!=255&&(LMR[2][i-1]==80))
          {
            LMR[2][i]=img_edg[j+1];
          }
        }
         
        if(img_edg[j+1]==255) //���е������ؽ�����
        {
          break;//while
        }
        j=j+2;
       }
       if(find==0)//û���ҵ���ͨ����
       {
         i++; 
         if((left_fix_begin>0)&&(left_fix_begin<=n))
         {
           for(n=left_fix_begin;n>0;n--)
           {
            LMR[0][n]=0; //���������
           }
         }
        if((right_fix_begin>0)&&(right_fix_begin<=n))
         {
           for(n=right_fix_begin;n>0;n--)
           {
            LMR[2][n]=0; //���������
           }
         }
         break;//for
        }
  } 
 } 
 for(;i<59;i++)
 {
    LMR[1][i]=(LMR[0][i]+LMR[2][i])/2;
    if(i>=35&&i<40)
    {
      Middle_Err_Sum=Middle_Err_Sum+ LMR[1][i]-40;
    }
 }
     
if(ABS(Middle_Err-Middle_Err_Sum/5)<2)
{
  Middle_Err= Middle_Err_Sum/5;//  for(i=59;i>0;i--) //�ӵ�59�п�ʼ����
}
else
{
  if(Middle_Err<Middle_Err_Sum/5) Middle_Err=Middle_Err+2;
  else Middle_Err=Middle_Err-2;
}


//Middle_Err = 2;


}

char judgeblack()  //����ͷȫ���ж������ȫ�ڣ��л�����Ź���
{  
//   ����ͷ �� ����л� ������Ǵ�����ͷ��ע����γ��� ////////////////////////////////////////////
//      �ж��м�һЩ���Ƿ�ȫ�ڣ����ȫ���л������    ////////////////////////////////////////
  uint16 m,n;
  uint16 sum[5];  //�ж��м�5���Ƿ�Ϊȫ�� 
  uint16 sum1[5]; //�ж����5���Ƿ�Ϊȫ�� 
  uint16 whitenum;
  
  static char flag=0;
  static char stopcar=0;
  
  static uint16 imgflag=0; // 0  �Ƕ�·�� 1 �Ƕ�·
  
   for(n=0;n<5;n++) sum[n]=0;
  
  for(n=18;n<23;n++) //ͼ��ǰ���ж�
  {  for(m=50;m<60;m++) 
     { //����ÿ���ֽ��а׵����
       whitenum = ((imgbuff_process[n*10+m]>>7)&0x01)+((imgbuff_process[n*10+m]>>6)&0x01)+((imgbuff_process[n*10+m]>>5)&0x01)+((imgbuff_process[n*10+m]>>4)&0x01)+((imgbuff_process[n*10+m]>>3)&0x01)+((imgbuff_process[n*10+m]>>2)&0x01)+((imgbuff_process[n*10+m]>>1)&0x01)+((imgbuff_process[n*10+m]>>0)&0x01);
       sum[n-20] = sum[n-20] + whitenum;
     }
  }
  
  for(n=0;n<5;n++) sum1[n]=0;
  for(n=18;n<23;n++) //ͼ�񿿺��ж�
  {  for(m=60;m<70;m++) 
     {  whitenum = ((imgbuff_process[n*10+m]>>7)&0x01)+((imgbuff_process[n*10+m]>>6)&0x01)+((imgbuff_process[n*10+m]>>5)&0x01)+((imgbuff_process[n*10+m]>>4)&0x01)+((imgbuff_process[n*10+m]>>3)&0x01)+((imgbuff_process[n*10+m]>>2)&0x01)+((imgbuff_process[n*10+m]>>1)&0x01)+((imgbuff_process[n*10+m]>>0)&0x01);
        sum1[n-45] = sum1[n-45] + whitenum;  //ÿ�а׵����
     }
  }
  
  switch(flag)
  {
  case 0:{
            if(( (sum1[0]<NORMAL_NUM) && (sum1[1]<NORMAL_NUM) && (sum1[2]<NORMAL_NUM) && (sum1[3]<NORMAL_NUM) && (sum1[4]<NORMAL_NUM) )&&( (sum[0]<NORMAL_NUM) && (sum[1]<NORMAL_NUM) && (sum[2]<NORMAL_NUM) && (sum[3]<NORMAL_NUM) && (sum[4]<NORMAL_NUM) ))
            {
               if(runmode==1){  flag=1; SBZ(); runmode = 0; } //���ֱ�ֱ��
               else { runmode = 1; flag=2; } //ֱ��������
               Dutime=0;
            }
            return 0;
         }
  case 1:{ //ͣ������
           Dutime++;
           if(Dutime>500){ flag=2;  Dutime=0; return 0;  }
           else { LeftMotorOut=0.1; RightMotorOut=0.1; return 1;}
          }
  case 2:{  //��������¼���·
            Dutime++;
           if(Dutime>2000){ flag=0; Dutime=0;}
          return 0;
         }
  }
  
  
/*  
  if(imgflag==0) //
  {
    if( (sum1[0]>60) && (sum1[1]>60) && (sum1[2]>60) && (sum1[3]>60) && (sum1[4]>60) ){//5������
       
        imgflag=0; 
        
    }
    else{   //��⵽��·
       
      imgflag=1; 
      if(runmode==1){ runmode = 0; }
      else { runmode = 1; }
    }
       
  }
  else if(imgflag==1)
  { 
    if( (sum1[0]>60) && (sum1[1]>60) && (sum1[2]>60) && (sum1[3]>60) && (sum1[4]>60) ) { //5������
      
      imgflag=0;
    }
    
    else {
      imgflag=1;
    }
  }*/
////////////  �ж�����  //////////////////////////////////////////  
  
}
void SBZ() //����
{
  
   Get_Speed();  
   
   while(CarSpeed>0) //�ж�ͣ��
  {
      
       LeftMotorOut=0.9;
       RightMotorOut=0.9;
        Moto_Out();
        
        Get_Speed();                                                                                                                                                                                                                   
  }
  
  
}
