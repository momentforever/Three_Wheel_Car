/*!
 * @brief      OV����ͷ��������SCCB������
 */


#include "common.h"
#include "MK60_gpio.h"
#include "SCCB.h"

extern uint8 gainval;  //�ֶ��ع�ʱ����ֵ ȡֵ��Χ��16~64
extern uint8 expomode; //�ع�ģʽ�趨 0�� �ֶ��ع�  1���Զ��ع�

static void SCCB_delay(uint16 i);
void sccb_wait(void);

/*!
 *  @brief      SCCB�ӳٺ���
 *  @param      time    ��ʱʱ��
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
 *  @brief      SCCB�ܽ�����
 *  @since      v5.0
 */
void SCCB_GPIO_init(void)
{
    gpio_init  (SCCB_SCL, GPO, 1); //��ʼ��SCL
    gpio_init  (SCCB_SDA, GPO, 1); //��ʼ��SDA

    port_init_NoALT(SCCB_SCL,ODO | PULLUP);
    port_init_NoALT(SCCB_SDA,ODO | PULLUP);
}

/*!
 *  @brief      SCCB��ʼ�ź�
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
//        return 0;   /* SDA��Ϊ�͵�ƽ������æ,�˳� */
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
//        return 0;   /* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
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
 *  @brief      SCCBֹͣ�ź�
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
 *  @brief      SCCBӦ���ź�
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
 *  @brief      SCCB��Ӧ���ź�
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
 *  @brief      SCCB �ȴ�Ӧ��
 *  @return     Ӧ������0��ʾ��Ӧ��1��ʾ��Ӧ��
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

    if(SDA_IN())           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
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
 *  @brief      SCCB ���͵�����
 *  @param      SendByte    ��Ҫ���͵�����
 *  @since      v5.0
 */
static void SCCB_SendByte(uint8 SendByte)
{
    uint8 i = 8;
    while(i--)
    {

        if(SendByte & 0x80)     //SDA �������
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        SendByte <<= 1;
        sccb_wait();
        SCL_H();                //SCL ���ߣ��ɼ��ź�
        sccb_wait();
        SCL_L();                //SCL ʱ��������
        //SCCB_DELAY();
    }
    //SCL_L();
}

/*!
 *  @brief      ����SCCB���ߵ�����
 *  @return     ���յ�������
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
* ��������SCCB_WriteByte
* ����  ��дһ�ֽ�����
* ����  ��- WriteAddress: ��д���ַ    - SendByte: ��д������  - DeviceAddress: ��������
* ���  ������Ϊ:=1�ɹ�д��,=0ʧ��
* ע��  ����
*****************************************************************************************/
static int SCCB_WriteByte_one( uint16 WriteAddress , uint8 SendByte );


int SCCB_WriteByte( uint16 WriteAddress , uint8 SendByte )            //���ǵ���sccb�Ĺܽ�ģ�⣬�Ƚ�����ʧ�ܣ���˶��Լ���
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
    SCCB_SendByte( DEV_ADR );                    /* ������ַ */
    if( !SCCB_WaitAck() )
    {
        SCCB_Stop();
        return 0;
    }
    SCCB_SendByte((uint8)(WriteAddress & 0x00FF));   /* ���õ���ʼ��ַ */
    SCCB_WaitAck();
    SCCB_SendByte(SendByte);
    SCCB_WaitAck();
    SCCB_Stop();
    return 1;
}




/******************************************************************************************************************
 * ��������SCCB_ReadByte
 * ����  ����ȡһ������
 * ����  ��- pBuffer: ��Ŷ�������  - length: ����������    - ReadAddress: ��������ַ        - DeviceAddress: ��������
 * ���  ������Ϊ:=1�ɹ�����,=0ʧ��
 * ע��  ����
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
    SCCB_SendByte( DEV_ADR );         /* ������ַ */
    if( !SCCB_WaitAck() )
    {
        SCCB_Stop();
        return 0;
    }
    SCCB_SendByte( ReadAddress );           /* ���õ���ʼ��ַ */
    SCCB_WaitAck();
    SCCB_Stop();

    if(!SCCB_Start())
    {
        return 0;
    }
    SCCB_SendByte( DEV_ADR + 1 );               /* ������ַ */

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
*  �������ƣ�sccb_sendByte
*  ����˵������SCCB�����Ϸ���һ���ֽ�
*  ����˵����data Ҫ���͵��ֽ�����
*  �������أ���
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
*  �������ƣ�sccb_regWrite
*  ����˵����ͨ��SCCB������ָ���豸��ָ����ַ����ָ������
*  ����˵����device---�豸��  ��д������
*            address---д���ݵļĴ���
*            data---д������
*  �������أ�ack=1δ�յ�Ӧ��(ʧ��)    ack=0�յ�Ӧ��(�ɹ�)
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
    //-----------SCCB�ָ�Ĭ�ϳ�������----------//
    //--PCLK:73ns   HREF:63.6us   VSYN:16.64ms--//
    //--Ĭ�ϸ���ɨ�裬ȫ�ֱ���Ϊ640*480���ɼ�VSYN�ֱ�����640*240  
    //--��ÿ����HREF֮����640��PCLK
    //--��ÿ����VSYN֮����240��HREF    
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
                  
//     ack1 = sccb_regWrite(0xB8,0x07,0x03,0x88);    //  0X38����
//     ack1 = sccb_regWrite(0xB8,0x0D,0x03,0x3A);    // ���и�ѹ��4��
//     ack1 = sccb_regWrite(0xB8,0xA5,0x00,0x3A);    //  0X38����
//     ack1 = sccb_regWrite(0xB8,0xAF,0x00,0x03);    //  
//     ack2 = sccb_regWrite(0xB8,0x2C,0x00,0x04);    // Vref ADC control
//     ack3 = sccb_regWrite(0xB8,0x35,0x20,0x20);    // Analog Gain : 16 - 64   
///////////////////  MT9V032 ��ʼ������ 188*122  AGC AEC   ////////////////////////////////////////////////////////
//     ack1 = sccb_regWrite(0xB8,0x01,0x00,0x01);    // Column Start          ͼ��ʼ�к�   0x0001
//     ack3 = sccb_regWrite(0xB8,0x02,0x00,0x04);    // Row Start             ͼ��ʼ�к�   0x0004
//     ack3 = sccb_regWrite(0xB8,0x03,0x01,0xE0);    // Window Height         ͼ��߶�   0x01e0  (480)
//     ack3 = sccb_regWrite(0xB8,0x04,0x02,0xF0);    // Window Width          ͼ����   0x02f0  (752)
//     ack3 = sccb_regWrite(0xB8,0x05,0x00,0x5E);    // Horizontal Blanking   ˮƽ���� (61 ~ 1023)  0x005e (94)
//     ack3 = sccb_regWrite(0xB8,0x06,0x00,0x2D);    // Vertical Blanking    ��ֱ���� ( 2 ~ 32288)  0x002d (45)
//     
//     ack1 = sccb_regWrite(0xB8,0x07,0x03,0x88);    // CHIP CONTROL 0x0388
//     
//     ack3 = sccb_regWrite(0xB8,0x08,0x01,0xBB);    // Coarse Shutter Width 1 Context A  �ֿ���ʱ��1   0x01bb (433)
//     ack3 = sccb_regWrite(0xB8,0x09,0x01,0xD9);    // Coarse Shutter Width 2 Context A  �ֿ���ʱ��2   0x01d9 (473)
//     ack3 = sccb_regWrite(0xB8,0x0A,0x01,0x64);    // Coarse Shutter Width Control      �ֿ��ſ���    0x0164
//     ack3 = sccb_regWrite(0xB8,0x0B,0x01,0xE0);    // Coarse Shutter Width Total        �ֿ����ܼ�    0x01e0  (480)
//     
//     ack3 = sccb_regWrite(0xB8,0xD3,0x00,0x00);    // Fine Shutter Width 1      ���ܿ���ʱ��1      0x0000
//     ack3 = sccb_regWrite(0xB8,0xD4,0x00,0x00);    // Fine Shutter Width 2      ���ܿ���ʱ��2      0x0000
//     ack3 = sccb_regWrite(0xB8,0xD5,0x00,0x00);    // Fine Shutter Width Total   ���ܿ���ʱ���ܼ�  0x0000
//     
//     
//     ack1 = sccb_regWrite(0xB8,0x0D,0x03,0x3A);    // Read Mode Context A   0x0300   ����������ã����и�ѹ��4��
//     ack3 = sccb_regWrite(0xB8,0x0F,0x00,0x11);    // Sensor Type, HDR Enable   �߶�̬��Χͼ��ʹ��  0x0040
//     
//     ack2 = sccb_regWrite(0xB8,0x1C,0x02,0x02);    // Companding  0x0302   
//
//     ack2 = sccb_regWrite(0xB8,0x2C,0x00,0x05);   // Vref ADC control 0x0000
//     
//     ack2 = sccb_regWrite(0xB8,0x31,0x00,0x1D);    // V1 Control ��0~63��  0x001D ��29��
//     ack2 = sccb_regWrite(0xB8,0x32,0x00,0x18);    // V2 Control ��0~63��  0x0018 ��24��
//     ack2 = sccb_regWrite(0xB8,0x33,0x00,0x15);    // V3 Control ��0~63��  0x0015 ��21��
//     ack2 = sccb_regWrite(0xB8,0x34,0x00,0x04);    // V4 Control ��0~63��  0x0004 ��4��
//     
//     ack3 = sccb_regWrite(0xB8,0x35,0x00,60);    // Analog Gain : 16 - 64   0x0010  �Զ��ع�ģʽ����Ч
//     
//     ack3 = sccb_regWrite(0xB8,0x70,0x00,0x00);    // Row Noise Correction Control  0x00
//     
//     ack1 = sccb_regWrite( 0xB8,0x80, 0x04,0xF4 );  // TILE_X0_Y0    0x04f4  ��������
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
//////////  MT9V032 ��ʼ�����  //////////////////////////////////////////////////////////////////////////////////////
 
///////////////////MT9V034  188 * 120  AGC���� AEC���� /////////////////////////////////////////////

     ack1 = sccb_regWrite(0xB8,0x01,0x00,0x01);    // Column Start          ͼ��ʼ�к�   0x0001
     ack3 = sccb_regWrite(0xB8,0x02,0x00,0x04);    // Row Start             ͼ��ʼ�к�   0x0004
     ack3 = sccb_regWrite(0xB8,0x03,0x01,0xE0);    // Window Height         ͼ��߶�   0x01e0  (480)
     ack3 = sccb_regWrite(0xB8,0x04,0x02,0xF0);    // Window Width          ͼ����   0x02f0  (752)
     ack3 = sccb_regWrite(0xB8,0x05,0x00,0x5E);    // Horizontal Blanking   ˮƽ���� (61 ~ 1023)  0x005e (94)
     ack3 = sccb_regWrite(0xB8,0x06,0x00,0x2D);    // Vertical Blanking    ��ֱ���� ( 2 ~ 32288)  0x002d (45)
     
     ack1 = sccb_regWrite(0xB8,0x07,0x03,0x88);    // CHIP CONTROL 0x0388
     
     ack3 = sccb_regWrite(0xB8,0x08,0x01,0xBB);    // Coarse Shutter Width 1 Context A  �ֿ���ʱ��1   0x01bb (433)
     ack3 = sccb_regWrite(0xB8,0x09,0x01,0xD9);    // Coarse Shutter Width 2 Context A  �ֿ���ʱ��2   0x01d9 (473)
     ack3 = sccb_regWrite(0xB8,0x0A,0x01,0x64);    // Coarse Shutter Width Control      �ֿ��ſ���    0x0164
     ack3 = sccb_regWrite(0xB8,0x0B,0x01,0xE0);    // Coarse Shutter Width Total        �ֿ����ܼ�    0x01e0  (480)
     
     ack3 = sccb_regWrite(0xB8,0xD3,0x00,0x00);    // Fine Shutter Width 1      ���ܿ���ʱ��1      0x0000
     ack3 = sccb_regWrite(0xB8,0xD4,0x00,0x00);    // Fine Shutter Width 2      ���ܿ���ʱ��2      0x0000
     ack3 = sccb_regWrite(0xB8,0xD5,0x00,0x00);    // Fine Shutter Width Total   ���ܿ���ʱ���ܼ�  0x0000
     
     
     ack1 = sccb_regWrite(0xB8,0x0D,0x03,0x3A);    // Read Mode Context A   0x0300   ����������ã����и�ѹ��4��
     ack3 = sccb_regWrite(0xB8,0x0F,0x00,0x11);    // Sensor Type, HDR Enable   �߶�̬��Χͼ��ʹ��  0x0100
     
     ack2 = sccb_regWrite(0xB8,0x1C,0x02,0x02);    // Companding  0x0302   

     ack2 = sccb_regWrite(0xB8,0x2C,0x00,0x01);   // Vref ADC control 0x0000  0~7
     
     ack2 = sccb_regWrite(0xB8,0x31,0x00,0x27);    // V1 Control ��0~63��  0x0027 ��39��
     ack2 = sccb_regWrite(0xB8,0x32,0x00,0x1a);    // V2 Control ��0~63��  0x001a ��26��
     ack2 = sccb_regWrite(0xB8,0x33,0x00,0x05);    // V3 Control ��0~63��  0x0005 ��5��
     ack2 = sccb_regWrite(0xB8,0x34,0x00,0x03);    // V4 Control ��0~63��  0x0003 ��3��
     
     ack3 = sccb_regWrite(0xB8,0x35,0x00,gainval);    // Analog Gain : 16 - 64   0x0010  �Զ��ع�ģʽ����Ч
     
     ack3 = sccb_regWrite(0xB8,0x70,0x00,0x00);    // Row Noise Correction Control  0x00
     
     ack1 = sccb_regWrite( 0xB8,0x80, 0x04,0xF4 );  // TILE_X0_Y0    0x04f4  ��������
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
     ack1 = sccb_regWrite(0xB8,0xAF,0x00,expomode);    //  AEC/AGC Enable A/B     0x0003 �Զ��ع�  �Զ�����
 ////////////////  MT9V034  END /////////////////////////////////////////////////////
     
     sccb_wait();
     if( (ack1 == 0) && (ack2==0) && (ack3==0) )  //�ɹ�
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



