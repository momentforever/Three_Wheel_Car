#include "include.h"
uint8  RoadType=0;
float Previous_Error[12];


uint16 edgposition[CAMERA_H];


uint16 cont;
#define NORMAL_NUM 40

extern uint8 runmode;  //0: 直立跑  1：三轮跑
extern uint8 lockrun;  //0:允许改变runmode   1:不允许改变

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
   
float Slope_Calculate_Uint8(uint8 begin,uint8 end,uint8 *p)    //最小二乘法拟合斜率
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

void sendimg()
{
   uint8 ch=0;
  float temp=0;
  uint16 i=0,num;
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xff);
  my_putchar(0xa2);
  my_putchar(0x1); //小车状态
  
  num=cont+2+180+36;  
  //统计将要传输的数据量 2是因为要传输关键字即0xf0和0xf2
  //180是边线的位 36是变量的位 如果不传输就不要加上！
  
  my_putchar(BYTE0(num)); 
  my_putchar(BYTE1(num));
 for(i=0;i< cont;i++)
 {
     my_putchar(img_edg[i]);
 }
 my_putchar(0xf0);  //代表图像数据发完了
 /******************星号围起来的可以不传输*******************/
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
 my_putchar(0xf2); //代表整个数据都发完了
 
}
void get_edge()   //尽量减少乘法运算
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
              img_edg[cont++]=y;   //左移动5相当于乘以32 左移动3相当于乘以8 
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
              img_edg[cont++]=y;   //左移动5相当于乘以32 左移动3相当于乘以8  
              find=1;
        }
         last=0;
         continue;
      }
      
      for(n=7;n>=0;n--)
      {
            temp=(imgbuff_process[x+j]>>n)&1;// 获取该点像素值 （0或1）     
            if(temp!=last) //与上一个值不相同 出现了跳变沿            
            {
               y=j<<3;  
               if(find==0)
              {
                edgposition[i]=cont;
              }
               img_edg[cont++]=y+7-n;   //左移动5相当于乘以32 左移动3相当于乘以8 
               find=1;
            }
              last=temp;                //存储该点的值
      } 
    } 
    img_edg[cont++]=0xff;   //左移动5相当于乘以32 左移动3相当于乘以8

  }
}
/*img_edg是一个一维数组 记录了摄像头每行的跳变沿的坐标值  每行跳变沿 由上升沿开始（由黑变白）然后接下降沿（由白变黑） 
   0xff用于指示该行的跳变沿完了，开始记录下一行
   如果每行图像以白色部分开始，那么该行跳变沿起始位置为0
   如果该行全黑 那么该行记录为0xff
   如果该行为全白 记录为 0 0xff
 
   
  oxff代表本行坐标值结束，进入下一行
 //edgposition[i]代表第i行的 跳变沿 在 img_edg 中坐标起点

 */
void fix_break_line() //修复断开的线
{
  
}
void Search()
{
  //从底部往上搜线
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
  for(i=0;i<60;i++)  //清空数组
  {   
    LMR[0][i]=0; //左边线数列
    LMR[1][i]=0;  //中线数列
    LMR[2][i]=80; //右边线数列
  }
  for(i=59;i>0;i--) //从第59行开始搜线
  {
    if(edgposition[i]==0&&(i!=0)) //全黑行 置为丢线
    {
      BlackCount++;
      if( BlackCount >= 10 )  
      {
        CarmeraMiss = true;  //有10行以上全黑行，进入断路，电磁开始工作
        if(lockrun==0)
        { runmode++;//切换小车状态
          if(runmode>1) runmode=0; //0：直立  1：三轮
          lockrun=1; //锁定，不允许改变runmode,直到小车重新看到白线
        }
        break;
      }     
    }
    if( i == 30 )
    {
      CarmeraMiss =false;
      lockrun=0;  //看到白了，重新等待下一次全黑
    }
    j=edgposition[i];//该行跳变沿开始的位置  j代表黑变白 即左边线 j+1代表百变黑 即右边线 

    if(i==59)  //底部开始行
    {   
       while(img_edg[j]!=255)
       {  
         if((img_edg[j]<55)&&(img_edg[j+1]>25))  //左边沿小于55 右边沿大于25
         {
           if((img_edg[j+1]-img_edg[j])>25) //右边沿-左边沿大于20
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
    else   //不是底部开始行
    { 
        find=0;
        while(img_edg[j]!=255)
       {  
        if((img_edg[j]<=LMR[2][i+1])&&(img_edg[j+1]>=LMR[0][i+1])&&(img_edg[j+1]-img_edg[j]>3))     //左边沿小于上一行的右边线 右边沿大于上一行的左边线是连通域；
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
         
        if(img_edg[j+1]==255) //该行的跳变沿结束了
        {
          break;//while
        }
        j=j+2;
       }
       if(find==0)//没有找到连通区域
       {
         i++; 
         if((left_fix_begin>0)&&(left_fix_begin<=n))
         {
           for(n=left_fix_begin;n>0;n--)
           {
            LMR[0][n]=0; //左边线数列
           }
         }
        if((right_fix_begin>0)&&(right_fix_begin<=n))
         {
           for(n=right_fix_begin;n>0;n--)
           {
            LMR[2][n]=0; //左边线数列
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
  Middle_Err= Middle_Err_Sum/5;//  for(i=59;i>0;i--) //从第59行开始搜线
}
else
{
  if(Middle_Err<Middle_Err_Sum/5) Middle_Err=Middle_Err+2;
  else Middle_Err=Middle_Err-2;
}


//Middle_Err = 2;


}

char judgeblack()  //摄像头全黑判定，如果全黑，切换至电磁工作
{  
//   摄像头 和 电磁切换 ，如果是纯摄像头，注释这段程序 ////////////////////////////////////////////
//      判定中间一些行是否全黑，如果全黑切换到电磁    ////////////////////////////////////////
  uint16 m,n;
  uint16 sum[5];  //判定中间5行是否为全黑 
  uint16 sum1[5]; //判定后端5行是否为全黑 
  uint16 whitenum;
  
  static char flag=0;
  static char stopcar=0;
  
  static uint16 imgflag=0; // 0  非断路， 1 是短路
  
   for(n=0;n<5;n++) sum[n]=0;
  
  for(n=18;n<23;n++) //图像靠前处判定
  {  for(m=50;m<60;m++) 
     { //计算每个字节中白点个数
       whitenum = ((imgbuff_process[n*10+m]>>7)&0x01)+((imgbuff_process[n*10+m]>>6)&0x01)+((imgbuff_process[n*10+m]>>5)&0x01)+((imgbuff_process[n*10+m]>>4)&0x01)+((imgbuff_process[n*10+m]>>3)&0x01)+((imgbuff_process[n*10+m]>>2)&0x01)+((imgbuff_process[n*10+m]>>1)&0x01)+((imgbuff_process[n*10+m]>>0)&0x01);
       sum[n-20] = sum[n-20] + whitenum;
     }
  }
  
  for(n=0;n<5;n++) sum1[n]=0;
  for(n=18;n<23;n++) //图像靠后处判定
  {  for(m=60;m<70;m++) 
     {  whitenum = ((imgbuff_process[n*10+m]>>7)&0x01)+((imgbuff_process[n*10+m]>>6)&0x01)+((imgbuff_process[n*10+m]>>5)&0x01)+((imgbuff_process[n*10+m]>>4)&0x01)+((imgbuff_process[n*10+m]>>3)&0x01)+((imgbuff_process[n*10+m]>>2)&0x01)+((imgbuff_process[n*10+m]>>1)&0x01)+((imgbuff_process[n*10+m]>>0)&0x01);
        sum1[n-45] = sum1[n-45] + whitenum;  //每行白点个数
     }
  }
  
  switch(flag)
  {
  case 0:{
            if(( (sum1[0]<NORMAL_NUM) && (sum1[1]<NORMAL_NUM) && (sum1[2]<NORMAL_NUM) && (sum1[3]<NORMAL_NUM) && (sum1[4]<NORMAL_NUM) )&&( (sum[0]<NORMAL_NUM) && (sum[1]<NORMAL_NUM) && (sum[2]<NORMAL_NUM) && (sum[3]<NORMAL_NUM) && (sum[4]<NORMAL_NUM) ))
            {
               if(runmode==1){  flag=1; SBZ(); runmode = 0; } //三轮便直立
               else { runmode = 1; flag=2; } //直立便三轮
               Dutime=0;
            }
            return 0;
         }
  case 1:{ //停车两秒
           Dutime++;
           if(Dutime>500){ flag=2;  Dutime=0; return 0;  }
           else { LeftMotorOut=0.1; RightMotorOut=0.1; return 1;}
          }
  case 2:{  //两秒后重新检测断路
            Dutime++;
           if(Dutime>2000){ flag=0; Dutime=0;}
          return 0;
         }
  }
  
  
/*  
  if(imgflag==0) //
  {
    if( (sum1[0]>60) && (sum1[1]>60) && (sum1[2]>60) && (sum1[3]>60) && (sum1[4]>60) ){//5行正常
       
        imgflag=0; 
        
    }
    else{   //检测到断路
       
      imgflag=1; 
      if(runmode==1){ runmode = 0; }
      else { runmode = 1; }
    }
       
  }
  else if(imgflag==1)
  { 
    if( (sum1[0]>60) && (sum1[1]>60) && (sum1[2]>60) && (sum1[3]>60) && (sum1[4]>60) ) { //5行正常
      
      imgflag=0;
    }
    
    else {
      imgflag=1;
    }
  }*/
////////////  判定结束  //////////////////////////////////////////  
  
}
void SBZ() //减速
{
  
   Get_Speed();  
   
   while(CarSpeed>0) //判断停车
  {
      
       LeftMotorOut=0.9;
       RightMotorOut=0.9;
        Moto_Out();
        
        Get_Speed();                                                                                                                                                                                                                   
  }
  
  
}
