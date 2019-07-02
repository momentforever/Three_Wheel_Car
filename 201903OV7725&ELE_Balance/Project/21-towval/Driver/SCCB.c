/*!
 * @brief      OV摄像头配置总线SCCB函数库
 */


#include "common.h"
#include "MK60_gpio.h"
#include "SCCB.h"

extern uint8 gainval;  //手动曝光时增益值 取值范围：16~64
extern uint8 expomode; //曝光模式设定 0： 手动曝光  1：自动曝光

static void SCCB_delay(uint16 i);
void sccb_wait(void);

/*!
 *  @brief      SCCB延迟函数
 *  @param      time    延时时间
 *  @since      v5.0
 */
static void SCCB_delay(volatile uint16 time)
{
    while(time)
    {
        time--;
    }
}

/*!
 *  @brief      SCCB管脚配置
 *  @since      v5.0
 */
void SCCB_GPIO_init(void)
{
    gpio_init  (SCCB_SCL, GPO, 1); //初始化SCL
    gpio_init  (SCCB_SDA, GPO, 1); //初始化SDA

    port_init_NoALT(SCCB_SCL,ODO | PULLUP);
    port_init_NoALT(SCCB_SDA,ODO | PULLUP);
}

/*!
 *  @brief      SCCB起始信号
 *  @since      v5.0
 */
static uint8 SCCB_Start(void)
{
//    SDA_H();
//    SCL_H();
//    sccb_wait();
//
//    SDA_DDR_IN();
//    if(!SDA_IN())
//    {
//        SDA_DDR_OUT();
//        return 0;   /* SDA线为低电平则总线忙,退出 */
//    }
//    SDA_DDR_OUT();
//    SDA_L();
//
//    sccb_wait();
//    SCL_L();
//
//    if(SDA_IN())
//    {
//        SDA_DDR_OUT();
//        return 0;   /* SDA线为高电平则总线出错,退出 */
//    }
//    //SDA_DDR_OUT();
//    //SDA_L();
//    //SCCB_delay();
    SCL_DDR_OUT();
    SDA_DDR_OUT();
    SDA_H();
    sccb_wait();
    SCL_H();
    sccb_wait();
    SDA_L();
    sccb_wait();
    SCL_L(); 
    return 1;
}

/*!
 *  @brief      SCCB停止信号
 *  @since      v5.0
 */
static void SCCB_Stop(void)
{
//    SCL_L();
//    //SCCB_DELAY();
//    SDA_L();
//    sccb_wait();
//    SCL_H();
//    sccb_wait();
//    SDA_H();
//    sccb_wait();
    SCL_DDR_OUT();
    SDA_DDR_OUT();
    SDA_L();
    sccb_wait();
    SCL_H();
    sccb_wait();
    SDA_H();
    sccb_wait();  
    
}

/*!
 *  @brief      SCCB应答信号
 *  @since      v5.0
 */
static void SCCB_Ack(void)
{
    SCL_L();
    sccb_wait();
    SDA_L();
    sccb_wait();
    SCL_H();
    sccb_wait();
    SCL_L();
    sccb_wait();
}

/*!
 *  @brief      SCCB无应答信号
 *  @since      v5.0
 */
static void SCCB_NoAck(void)
{
    SCL_L();
    sccb_wait();
    SDA_H();
    sccb_wait();
    SCL_H();
    sccb_wait();
    SCL_L();
    sccb_wait();
}

/*!
 *  @brief      SCCB 等待应答
 *  @return     应答结果（0表示无应答，1表示有应答）
 *  @since      v5.0
 */
static int SCCB_WaitAck(void)
{
    SCL_L();
    //SDA_H();
    SDA_DDR_IN();

    sccb_wait();
    SCL_H();

    sccb_wait();

    if(SDA_IN())           //应答为高电平，异常，通信失败
    {
        SDA_DDR_OUT();
        SCL_L();
        return 0;
    }
    SDA_DDR_OUT();
    SCL_L();
    return 1;
}

/*!
 *  @brief      SCCB 发送的数据
 *  @param      SendByte    需要发送的数据
 *  @since      v5.0
 */
static void SCCB_SendByte(uint8 SendByte)
{
    uint8 i = 8;
    while(i--)
    {

        if(SendByte & 0x80)     //SDA 输出数据
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        SendByte <<= 1;
        sccb_wait();
        SCL_H();                //SCL 拉高，采集信号
        sccb_wait();
        SCL_L();                //SCL 时钟线拉低
        //SCCB_DELAY();
    }
    //SCL_L();
}

/*!
 *  @brief      接收SCCB总线的数据
 *  @return     接收到的数据
 *  @since      v5.0
 */
static int SCCB_ReceiveByte(void)
{
    uint8 i = 8;
    uint8 ReceiveByte = 0;

    //SDA_H();
    //SCCB_DELAY();
    SDA_DDR_IN();

    while(i--)
    {
        ReceiveByte <<= 1;
        SCL_L();
        sccb_wait();
        SCL_H();
        sccb_wait();

        if(SDA_IN())
        {
            ReceiveByte |= 0x01;
        }


    }
    SDA_DDR_OUT();
    SCL_L();
    return ReceiveByte;
}

/*****************************************************************************************
* 函数名：SCCB_WriteByte
* 描述  ：写一字节数据
* 输入  ：- WriteAddress: 待写入地址    - SendByte: 待写入数据  - DeviceAddress: 器件类型
* 输出  ：返回为:=1成功写入,=0失败
* 注意  ：无
*****************************************************************************************/
static int SCCB_WriteByte_one( uint16 WriteAddress , uint8 SendByte );


int SCCB_WriteByte( uint16 WriteAddress , uint8 SendByte )            //考虑到用sccb的管脚模拟，比较容易失败，因此多试几次
{
    uint8 i = 0;
    while( 0 == SCCB_WriteByte_one ( WriteAddress, SendByte ) )
    {
        i++;
        if(i == 20)
        {
            return 0 ;
        }
    }
    return 1;
}

int SCCB_WriteByte_one( uint16 WriteAddress , uint8 SendByte )
{
    if(!SCCB_Start())
    {
        return 0;
    }
    SCCB_SendByte( DEV_ADR );                    /* 器件地址 */
    if( !SCCB_WaitAck() )
    {
        SCCB_Stop();
        return 0;
    }
    SCCB_SendByte((uint8)(WriteAddress & 0x00FF));   /* 设置低起始地址 */
    SCCB_WaitAck();
    SCCB_SendByte(SendByte);
    SCCB_WaitAck();
    SCCB_Stop();
    return 1;
}




/******************************************************************************************************************
 * 函数名：SCCB_ReadByte
 * 描述  ：读取一串数据
 * 输入  ：- pBuffer: 存放读出数据  - length: 待读出长度    - ReadAddress: 待读出地址        - DeviceAddress: 器件类型
 * 输出  ：返回为:=1成功读入,=0失败
 * 注意  ：无
 **********************************************************************************************************************/
static int SCCB_ReadByte_one(uint8 *pBuffer,   uint16 length,   uint8 ReadAddress);

int SCCB_ReadByte(uint8 *pBuffer,   uint16 length,   uint8 ReadAddress)
{
    uint8 i = 0;
    while( 0 == SCCB_ReadByte_one(pBuffer, length, ReadAddress) )
    {
        i++;
        if(i == 30)
        {
            return 0 ;
        }
    }
    return 1;
}

int SCCB_ReadByte_one(uint8 *pBuffer,   uint16 length,   uint8 ReadAddress)
{
    if(!SCCB_Start())
    {
        return 0;
    }
    SCCB_SendByte( DEV_ADR );         /* 器件地址 */
    if( !SCCB_WaitAck() )
    {
        SCCB_Stop();
        return 0;
    }
    SCCB_SendByte( ReadAddress );           /* 设置低起始地址 */
    SCCB_WaitAck();
    SCCB_Stop();

    if(!SCCB_Start())
    {
        return 0;
    }
    SCCB_SendByte( DEV_ADR + 1 );               /* 器件地址 */

    if(!SCCB_WaitAck())
    {
        SCCB_Stop();
        return 0;
    }
    while(length)
    {
        *pBuffer = SCCB_ReceiveByte();
        if(length == 1)
        {
            SCCB_NoAck();
        }
        else
        {
            SCCB_Ack();
        }
        pBuffer++;
        length--;
    }
    SCCB_Stop();
    return 1;
}

////////////////////////  MT9V032 MT9V034 //////////////////////////
/************************************************************************
*                             
*
*  函数名称：sccb_sendByte
*  功能说明：在SCCB总线上发送一个字节
*  参数说明：data 要发送的字节内容
*  函数返回：无
*
*************************************************************************/
uint8 sccb_sendregByte(uint8 data)
{
  uint8 i;
  uint8 ack;
  SDA_DDR_OUT();
  for( i=0; i<8; i++)
  {
    if(data & 0x80)
      SDA_H();
    else 
      SDA_L();
    data <<= 1;
    sccb_wait();
    SCL_H();
    sccb_wait();
    SCL_L();
    sccb_wait();
  }
  SDA_H();
  SDA_DDR_IN();
  sccb_wait();
  SCL_H();;
  sccb_wait();
  ack = SDA_DATA;
  SCL_L();;
  sccb_wait();
  return ack;
}
/************************************************************************
*                             
*
*  函数名称：sccb_regWrite
*  功能说明：通过SCCB总线向指定设备的指定地址发送指定内容
*  参数说明：device---设备号  读写有区别
*            address---写数据的寄存器
*            data---写的内容
*  函数返回：ack=1未收到应答(失败)    ack=0收到应答(成功)
*
*************************************************************************/
uint8 sccb_regWrite(uint8 device,uint8 address,uint8 data1,uint8 data2)
{
  uint8 i;
  uint8 ack;
  for( i=0; i<20; i++)
  {
    SCCB_Start();//sccb_start();
    ack = sccb_sendregByte( device ); //sccb_sendByte(device);
    if( ack == 1 )
    {
      SCCB_Stop();//sccb_stop();
      continue;
    }   
    ack = sccb_sendregByte( address );//sccb_sendByte(address);
    if( ack == 1 )
    {
      SCCB_Stop();//sccb_stop();
      continue;
    }   
    ack = sccb_sendregByte( data1 );//sccb_sendByte(data1);
    if( ack == 1 )
    {
      SCCB_Stop();//sccb_stop();
      continue;
    } 
    ack = sccb_sendregByte( data2 );//ccb_sendByte(data2);
    if( ack == 1 )
    {
      SCCB_Stop();//sccb_stop();
      continue;
    }
    SCCB_Stop();//sccb_stop();
    if( ack == 0 ) break;
  }
  return ack;
}

void sccb_wait(void)
{
  uint16 i;
  for( i=0; i<2000; i++)
  {
    asm ("nop");
  }
}


uint8 sccb_refresh()
{
    //-----------SCCB恢复默认出厂设置----------//
    //--PCLK:73ns   HREF:63.6us   VSYN:16.64ms--//
    //--默认隔行扫描，全分辨率为640*480，采集VSYN分辨率是640*240  
    //--在每两个HREF之间有640个PCLK
    //--在每两个VSYN之间有240个HREF    
    //sccb_regWrite(0x42,0x11,0x00);   
    //sccb_regWrite(0x42,0x14,0x04);
    //sccb_regWrite(0x42,0x28,0x20);
    //---------------------------------------//
    uint8 ack1,ack2,ack3;
    uint8 ACK = 1;
    for(uint8 sccb_time=0; sccb_time<10; sccb_time++)
   { 
     ack1 = sccb_regWrite(0xB8,0x0C,0x00,0x01);    // RESET
     sccb_wait();
     ack1 = sccb_regWrite(0xB8,0x0C,0x00,0x00);    // RESET
     sccb_wait();
     sccb_wait();
     sccb_wait();
                  
//     ack1 = sccb_regWrite(0xB8,0x07,0x03,0x88);    //  0X38不行
//     ack1 = sccb_regWrite(0xB8,0x0D,0x03,0x3A);    // 行列各压缩4倍
//     ack1 = sccb_regWrite(0xB8,0xA5,0x00,0x3A);    //  0X38不行
//     ack1 = sccb_regWrite(0xB8,0xAF,0x00,0x03);    //  
//     ack2 = sccb_regWrite(0xB8,0x2C,0x00,0x04);    // Vref ADC control
//     ack3 = sccb_regWrite(0xB8,0x35,0x20,0x20);    // Analog Gain : 16 - 64   
///////////////////  MT9V032 初始化程序 188*122  AGC AEC   ////////////////////////////////////////////////////////
//     ack1 = sccb_regWrite(0xB8,0x01,0x00,0x01);    // Column Start          图像开始列号   0x0001
//     ack3 = sccb_regWrite(0xB8,0x02,0x00,0x04);    // Row Start             图像开始行号   0x0004
//     ack3 = sccb_regWrite(0xB8,0x03,0x01,0xE0);    // Window Height         图像高度   0x01e0  (480)
//     ack3 = sccb_regWrite(0xB8,0x04,0x02,0xF0);    // Window Width          图像宽度   0x02f0  (752)
//     ack3 = sccb_regWrite(0xB8,0x05,0x00,0x5E);    // Horizontal Blanking   水平消隐 (61 ~ 1023)  0x005e (94)
//     ack3 = sccb_regWrite(0xB8,0x06,0x00,0x2D);    // Vertical Blanking    垂直消隐 ( 2 ~ 32288)  0x002d (45)
//     
//     ack1 = sccb_regWrite(0xB8,0x07,0x03,0x88);    // CHIP CONTROL 0x0388
//     
//     ack3 = sccb_regWrite(0xB8,0x08,0x01,0xBB);    // Coarse Shutter Width 1 Context A  粗快门时间1   0x01bb (433)
//     ack3 = sccb_regWrite(0xB8,0x09,0x01,0xD9);    // Coarse Shutter Width 2 Context A  粗快门时间2   0x01d9 (473)
//     ack3 = sccb_regWrite(0xB8,0x0A,0x01,0x64);    // Coarse Shutter Width Control      粗快门控制    0x0164
//     ack3 = sccb_regWrite(0xB8,0x0B,0x01,0xE0);    // Coarse Shutter Width Total        粗快门总计    0x01e0  (480)
//     
//     ack3 = sccb_regWrite(0xB8,0xD3,0x00,0x00);    // Fine Shutter Width 1      精密快门时间1      0x0000
//     ack3 = sccb_regWrite(0xB8,0xD4,0x00,0x00);    // Fine Shutter Width 2      精密快门时间2      0x0000
//     ack3 = sccb_regWrite(0xB8,0xD5,0x00,0x00);    // Fine Shutter Width Total   精密快门时间总计  0x0000
//     
//     
//     ack1 = sccb_regWrite(0xB8,0x0D,0x03,0x3A);    // Read Mode Context A   0x0300   行列输出设置：行列各压缩4倍
//     ack3 = sccb_regWrite(0xB8,0x0F,0x00,0x11);    // Sensor Type, HDR Enable   高动态范围图像使能  0x0040
//     
//     ack2 = sccb_regWrite(0xB8,0x1C,0x02,0x02);    // Companding  0x0302   
//
//     ack2 = sccb_regWrite(0xB8,0x2C,0x00,0x05);   // Vref ADC control 0x0000
//     
//     ack2 = sccb_regWrite(0xB8,0x31,0x00,0x1D);    // V1 Control （0~63）  0x001D （29）
//     ack2 = sccb_regWrite(0xB8,0x32,0x00,0x18);    // V2 Control （0~63）  0x0018 （24）
//     ack2 = sccb_regWrite(0xB8,0x33,0x00,0x15);    // V3 Control （0~63）  0x0015 （21）
//     ack2 = sccb_regWrite(0xB8,0x34,0x00,0x04);    // V4 Control （0~63）  0x0004 （4）
//     
//     ack3 = sccb_regWrite(0xB8,0x35,0x00,60);    // Analog Gain : 16 - 64   0x0010  自动曝光模式下无效
//     
//     ack3 = sccb_regWrite(0xB8,0x70,0x00,0x00);    // Row Noise Correction Control  0x00
//     
//     ack1 = sccb_regWrite( 0xB8,0x80, 0x04,0xF4 );  // TILE_X0_Y0    0x04f4  数字增益
//     ack1 = sccb_regWrite( 0xB8,0x81, 0x04,0xF4 );  // TILE_X1_Y0
//     ack1 = sccb_regWrite( 0xB8,0x82, 0x04,0xF4 );  // TILE_X2_Y0
//     ack1 = sccb_regWrite( 0xB8,0x83, 0x04,0xF4 );  // TILE_X3_Y0
//     ack1 = sccb_regWrite( 0xB8,0x84, 0x04,0xF4 );  // TILE_X4_Y0
//     ack1 = sccb_regWrite( 0xB8,0x85, 0x04,0xF4 );  // TILE_X0_Y1
//     ack1 = sccb_regWrite( 0xB8,0x86, 0x04,0xF4 );  // TILE_X1_Y1
//     ack1 = sccb_regWrite( 0xB8,0x87, 0x04,0xF4 );  // TILE_X2_Y1
//     ack1 = sccb_regWrite( 0xB8,0x88, 0x04,0xF4 );  // TILE_X3_Y1
//     ack1 = sccb_regWrite( 0xB8,0x89, 0x04,0xF4 );  // TILE_X4_Y1
//     ack1 = sccb_regWrite( 0xB8,0x8A, 0x04,0xF4 );  // TILE_X0_Y2
//     ack1 = sccb_regWrite( 0xB8,0x8B, 0x04,0xF4 );  // TILE_X1_Y2
//     ack1 = sccb_regWrite( 0xB8,0x8C, 0x04,0xF4 );  // TILE_X2_Y2
//     ack1 = sccb_regWrite( 0xB8,0x8D, 0x04,0xF4 );  // TILE_X3_Y2
//     ack1 = sccb_regWrite( 0xB8,0x8E, 0x04,0xF4 );  // TILE_X4_Y2
//     ack1 = sccb_regWrite( 0xB8,0x8F, 0x04,0xF4 );  // TILE_X0_Y3
//     ack1 = sccb_regWrite( 0xB8,0x90, 0x04,0xF4 );  // TILE_X1_Y3
//     ack1 = sccb_regWrite( 0xB8,0x91, 0x04,0xF4 );  // TILE_X2_Y3
//     ack1 = sccb_regWrite( 0xB8,0x92, 0x04,0xF4 );  // TILE_X3_Y3
//     ack1 = sccb_regWrite( 0xB8,0x93, 0x04,0xF4 );  // TILE_X4_Y3
//     ack1 = sccb_regWrite( 0xB8,0x94, 0x04,0xF4 );  // TILE_X0_Y4
//     ack1 = sccb_regWrite( 0xB8,0x95, 0x04,0xF4 );  // TILE_X1_Y4
//     ack1 = sccb_regWrite( 0xB8,0x96, 0x04,0xF4 );  // TILE_X2_Y4
//     ack1 = sccb_regWrite( 0xB8,0x97, 0x04,0xF4 );  // TILE_X3_Y4
//     ack1 = sccb_regWrite( 0xB8,0x98, 0x04,0xF4 );  // TILE_X4_Y4
//     
//     ack1 = sccb_regWrite(0xB8,0xA5,0x00,0x3A);    //   AEC/AGC Desired Bin   0x003a   
//     ack1 = sccb_regWrite(0xB8,0xA6,0x00,0x02);    //   AEC Update Frequency   0x0002
//     ack1 = sccb_regWrite(0xB8,0xA8,0x00,0x00);    //   AEC Low Pass Filter   0x0000
//       
//     ack1 = sccb_regWrite(0xB8,0xA9,0x00,0x02);    // AGC Output Update Frequency 0x0002
//     ack1 = sccb_regWrite(0xB8,0xAB,0x00,0x02);    // AGC Low Pass Filter 0x0002
//     ack1 = sccb_regWrite(0xB8,0xAF,0x00,0x00);    //  AEC/AGC Enable A/B     0x0003
//  
//////////  MT9V032 初始化完成  //////////////////////////////////////////////////////////////////////////////////////
 
///////////////////MT9V034  188 * 120  AGC开启 AEC开启 /////////////////////////////////////////////

     ack1 = sccb_regWrite(0xB8,0x01,0x00,0x01);    // Column Start          图像开始列号   0x0001
     ack3 = sccb_regWrite(0xB8,0x02,0x00,0x04);    // Row Start             图像开始行号   0x0004
     ack3 = sccb_regWrite(0xB8,0x03,0x01,0xE0);    // Window Height         图像高度   0x01e0  (480)
     ack3 = sccb_regWrite(0xB8,0x04,0x02,0xF0);    // Window Width          图像宽度   0x02f0  (752)
     ack3 = sccb_regWrite(0xB8,0x05,0x00,0x5E);    // Horizontal Blanking   水平消隐 (61 ~ 1023)  0x005e (94)
     ack3 = sccb_regWrite(0xB8,0x06,0x00,0x2D);    // Vertical Blanking    垂直消隐 ( 2 ~ 32288)  0x002d (45)
     
     ack1 = sccb_regWrite(0xB8,0x07,0x03,0x88);    // CHIP CONTROL 0x0388
     
     ack3 = sccb_regWrite(0xB8,0x08,0x01,0xBB);    // Coarse Shutter Width 1 Context A  粗快门时间1   0x01bb (433)
     ack3 = sccb_regWrite(0xB8,0x09,0x01,0xD9);    // Coarse Shutter Width 2 Context A  粗快门时间2   0x01d9 (473)
     ack3 = sccb_regWrite(0xB8,0x0A,0x01,0x64);    // Coarse Shutter Width Control      粗快门控制    0x0164
     ack3 = sccb_regWrite(0xB8,0x0B,0x01,0xE0);    // Coarse Shutter Width Total        粗快门总计    0x01e0  (480)
     
     ack3 = sccb_regWrite(0xB8,0xD3,0x00,0x00);    // Fine Shutter Width 1      精密快门时间1      0x0000
     ack3 = sccb_regWrite(0xB8,0xD4,0x00,0x00);    // Fine Shutter Width 2      精密快门时间2      0x0000
     ack3 = sccb_regWrite(0xB8,0xD5,0x00,0x00);    // Fine Shutter Width Total   精密快门时间总计  0x0000
     
     
     ack1 = sccb_regWrite(0xB8,0x0D,0x03,0x3A);    // Read Mode Context A   0x0300   行列输出设置：行列各压缩4倍
     ack3 = sccb_regWrite(0xB8,0x0F,0x00,0x11);    // Sensor Type, HDR Enable   高动态范围图像使能  0x0100
     
     ack2 = sccb_regWrite(0xB8,0x1C,0x02,0x02);    // Companding  0x0302   

     ack2 = sccb_regWrite(0xB8,0x2C,0x00,0x01);   // Vref ADC control 0x0000  0~7
     
     ack2 = sccb_regWrite(0xB8,0x31,0x00,0x27);    // V1 Control （0~63）  0x0027 （39）
     ack2 = sccb_regWrite(0xB8,0x32,0x00,0x1a);    // V2 Control （0~63）  0x001a （26）
     ack2 = sccb_regWrite(0xB8,0x33,0x00,0x05);    // V3 Control （0~63）  0x0005 （5）
     ack2 = sccb_regWrite(0xB8,0x34,0x00,0x03);    // V4 Control （0~63）  0x0003 （3）
     
     ack3 = sccb_regWrite(0xB8,0x35,0x00,gainval);    // Analog Gain : 16 - 64   0x0010  自动曝光模式下无效
     
     ack3 = sccb_regWrite(0xB8,0x70,0x00,0x00);    // Row Noise Correction Control  0x00
     
     ack1 = sccb_regWrite( 0xB8,0x80, 0x04,0xF4 );  // TILE_X0_Y0    0x04f4  数字增益
     ack1 = sccb_regWrite( 0xB8,0x81, 0x04,0xF4 );  // TILE_X1_Y0
     ack1 = sccb_regWrite( 0xB8,0x82, 0x04,0xF4 );  // TILE_X2_Y0
     ack1 = sccb_regWrite( 0xB8,0x83, 0x04,0xF4 );  // TILE_X3_Y0
     ack1 = sccb_regWrite( 0xB8,0x84, 0x04,0xF4 );  // TILE_X4_Y0
     ack1 = sccb_regWrite( 0xB8,0x85, 0x04,0xF4 );  // TILE_X0_Y1
     ack1 = sccb_regWrite( 0xB8,0x86, 0x04,0xF4 );  // TILE_X1_Y1
     ack1 = sccb_regWrite( 0xB8,0x87, 0x04,0xF4 );  // TILE_X2_Y1
     ack1 = sccb_regWrite( 0xB8,0x88, 0x04,0xF4 );  // TILE_X3_Y1
     ack1 = sccb_regWrite( 0xB8,0x89, 0x04,0xF4 );  // TILE_X4_Y1
     ack1 = sccb_regWrite( 0xB8,0x8A, 0x04,0xF4 );  // TILE_X0_Y2
     ack1 = sccb_regWrite( 0xB8,0x8B, 0x04,0xF4 );  // TILE_X1_Y2
     ack1 = sccb_regWrite( 0xB8,0x8C, 0x04,0xF4 );  // TILE_X2_Y2
     ack1 = sccb_regWrite( 0xB8,0x8D, 0x04,0xF4 );  // TILE_X3_Y2
     ack1 = sccb_regWrite( 0xB8,0x8E, 0x04,0xF4 );  // TILE_X4_Y2
     ack1 = sccb_regWrite( 0xB8,0x8F, 0x04,0xF4 );  // TILE_X0_Y3
     ack1 = sccb_regWrite( 0xB8,0x90, 0x04,0xF4 );  // TILE_X1_Y3
     ack1 = sccb_regWrite( 0xB8,0x91, 0x04,0xF4 );  // TILE_X2_Y3
     ack1 = sccb_regWrite( 0xB8,0x92, 0x04,0xF4 );  // TILE_X3_Y3
     ack1 = sccb_regWrite( 0xB8,0x93, 0x04,0xF4 );  // TILE_X4_Y3
     ack1 = sccb_regWrite( 0xB8,0x94, 0x04,0xF4 );  // TILE_X0_Y4
     ack1 = sccb_regWrite( 0xB8,0x95, 0x04,0xF4 );  // TILE_X1_Y4
     ack1 = sccb_regWrite( 0xB8,0x96, 0x04,0xF4 );  // TILE_X2_Y4
     ack1 = sccb_regWrite( 0xB8,0x97, 0x04,0xF4 );  // TILE_X3_Y4
     ack1 = sccb_regWrite( 0xB8,0x98, 0x04,0xF4 );  // TILE_X4_Y4
     
     ack1 = sccb_regWrite(0xB8,0xA5,0x00,0x3A);    //   AEC/AGC Desired Bin   0x003a   
     ack1 = sccb_regWrite(0xB8,0xA6,0x00,0x02);    //   AEC Update Frequency   0x0002
     ack1 = sccb_regWrite(0xB8,0xA8,0x00,0x00);    //   AEC Low Pass Filter   0x0000
     
     
     ack1 = sccb_regWrite(0xB8,0xA9,0x00,0x02);    // AGC Output Update Frequency 0x0002
     ack1 = sccb_regWrite(0xB8,0xAA,0x00,0x02);    // AGC Output Update Frequency 0x0002
     ack1 = sccb_regWrite(0xB8,0xAF,0x00,expomode);    //  AEC/AGC Enable A/B     0x0003 自动曝光  自动增益
 ////////////////  MT9V034  END /////////////////////////////////////////////////////
     
     sccb_wait();
     if( (ack1 == 0) && (ack2==0) && (ack3==0) )  //成功
     {
        sccb_wait();
        ACK = 0;
        break;
     }
     else
     {
        sccb_wait();
        ACK = 1;
        continue;
     }
   }
    return ACK;
}



