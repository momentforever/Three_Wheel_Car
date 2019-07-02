/*!
 * @file       VCAN_SCCB.h
 * @brief      OV摄像头配置总线SCCB函数库
 */


#ifndef _VCAN_SCCB_H_
#define _VCAN_SCCB_H_

//SCCB 管脚配置
#define SCCB_SCL        PTE5
#define SCCB_SDA        PTE4

   

#define SCL_H()         PTXn_T(SCCB_SCL,OUT) = 1
#define SCL_L()         PTXn_T(SCCB_SCL,OUT) = 0
#define SCL_DDR_OUT()   PTXn_T(SCCB_SCL,DDR) = 1
#define SCL_DDR_IN()    PTXn_T(SCCB_SCL,DDR) = 0

#define SDA_H()         PTXn_T(SCCB_SDA,OUT) = 1
#define SDA_L()         PTXn_T(SCCB_SDA,OUT) = 0
#define SDA_IN()        PTXn_T(SCCB_SDA,IN)
#define SDA_DDR_OUT()   PTXn_T(SCCB_SDA,DDR) = 1
#define SDA_DDR_IN()    PTXn_T(SCCB_SDA,DDR) = 0
   
#define SDA_DATA        gpio_get(PTE4) 

#define ADDR_OV7725   0x42
#define ADDR_OV7620   0x42

#define DEV_ADR  ADDR_OV7725             /*设备地址定义*/

#define SCCB_DELAY()    SCCB_delay(400)


extern void SCCB_GPIO_init(void);
extern int SCCB_WriteByte( uint16 WriteAddress , uint8 SendByte);
extern int SCCB_ReadByte(uint8 *pBuffer,   uint16 length,   uint8 ReadAddress);
extern uint8 sccb_regWrite(uint8 device,uint8 address,uint8 data1,uint8 data2);
extern uint8 sccb_refresh();


#endif      //_VCAN_SCCB_H_
