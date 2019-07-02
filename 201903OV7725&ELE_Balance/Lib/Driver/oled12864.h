#ifndef	_OLED12864_H_
#define _OLED12864_H_





#define RST    PTA16
#define DC     PTB2
#define D0     PTB9
#define D1     PTB3

#define DC_HIGH gpio_set (DC,1)
#define DC_LOW  gpio_set (DC,0)

#define D0_HIGH gpio_set (D0,1)
#define D0_LOW  gpio_set (D0,0)

#define D1_HIGH gpio_set (D1,1)
#define D1_LOW  gpio_set (D1,0)

#define RST_HIGH gpio_set (RST,1)
#define RST_LOW  gpio_set (RST,0)



 extern uint8  reverse; //��ת��ʾ
 
 void OLED_Set_Pos(uint8 x, uint8 y);
 void OLED_Init(void);
 void OLED_CLS(void);
 //����OLED_WrDat
 void OLED_WrDat(uint8 data, uint8 revs);
 //��ʾһ��6x8��׼ASCII�ַ�
void OLED_P6x8Char(char ch);
 void OLED_P6x8Str(uint8 x,uint8 y,char ch[]);
//��һ��char����ת����3λ��������ʾ 
void OLED_PrintValueC(unsigned char x, unsigned char y, int data);
//������������һ��int����ת����5λ��������ʾ
void OLED_PrintValueI(unsigned char x, unsigned char y, int data);
//��ʾunsigned int��
void OLED_PrintValueFP(unsigned int data, unsigned char num);
//������������һ��float����ת������������5λ��С���ͷ��ŵ����ݲ�������ʾ
void OLED_PrintValueF(unsigned char x, unsigned char y, float data, unsigned char num);
 void OLED_Fill(uint8 dat);
 void OLED_Draw_Logo(void);
 void OLED_Draw_camera(void);
#endif