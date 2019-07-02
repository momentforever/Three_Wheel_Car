#include  "common.h"
#include "include.h"

#define	Brightness	0xCF 

uint8  reverse=0;

//======================================
const uint8 mykc_logo[1024]=
{
      /*--  ������һ��ͼ��C:\Users\Lenovo\Desktop\����.bmp  --*/
    /*--  ����x�߶�=128x64  --*/
    /*--  ������һ��ͼ��C:\Users\Lenovo\Desktop\����.bmp  --*/
    /*--  ����x�߶�=128x64  --*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x03,0x07,0x0F,0x0C,0x18,0x31,0x23,0x07,
    0x1F,0x1F,0x3F,0x7F,0xFF,0x7F,0x3F,0x1F,0x1F,0x0F,0x07,0x01,0x01,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x07,
    0x0F,0x1F,0x3F,0x7F,0x7C,0xF8,0xF0,0xE0,0xC6,0x0F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x3F,
    0x1F,0x0F,0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0F,0x1F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFE,
    0xF8,0xE0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0xC0,
    0xC0,0xC0,0xC0,0xE0,0xE0,0xE0,0xF0,0xF8,0xF8,0xF8,0xFC,0xFC,0xFE,0xFE,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x0F,0x0F,0x03,0x01,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0F,
    0x1F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xF0,0x80,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
    0x80,0xC0,0xC0,0xE0,0xF0,0xF0,0xF8,0xFC,0xFC,0xFE,0xFF,0xFF,0xFF,0x7F,0x1F,0x0F,
    0x07,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x1F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x00,0x00,
    0x2F,0x6F,0x6E,0x7E,0x5E,0x4E,0x7E,0x77,0x67,0x00,0x7F,0x7F,0x20,0x7F,0x78,0x20,
    0x3F,0x60,0x00,0x7F,0x7F,0x7F,0x7F,0x71,0x7D,0x35,0x7F,0x01,0x00,0x3F,0x78,0x5A,
    0x7E,0x3E,0x7E,0x7E,0x7F,0x7F,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xE0,0xE0,0xF0,
    0xFC,0xFC,0xFE,0x7F,0x1F,0x0F,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x01,0x07,0x0F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x80,0x00,0x00,
    0x80,0x80,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,
    0x80,0x80,0x01,0x81,0x81,0x83,0x03,0x07,0x07,0x0F,0x0F,0x0F,0x1F,0x3F,0x3F,0x7F,
    0x7F,0x3F,0x1F,0x8F,0xC3,0xE1,0xF8,0xFC,0x7E,0x1F,0x07,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x80,0x90,0x90,0x90,0x98,0x98,0x98,0x8C,0x8C,0xCC,0xCE,0xCE,0xCE,
    0xCE,0xCE,0xCE,0xCF,0xC7,0xC7,0xC7,0xC7,0xC7,0xC7,0xC7,0x07,0x07,0x07,0x07,0x07,
    0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x0F,0x0F,0x0F,
    0x0F,0x0F,0x0F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x3F,0x3F,0x3F,0x7F,0x7F,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0x1F,0x87,0xE3,0xF0,0x3C,0x0F,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0xC0,0xC0,0xC0,0xC0,0xC0,
    0xC0,0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF8,0xF8,
    0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,
    0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,
    0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0x7C,0x3C,0x00,0x80,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

};
const uint8 F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 },   // horiz lines
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }    // cursor
};

void OLED_WrLogo(uint8 data) //��logoר�ú���
{
       uint8 i=8;	
       DC_HIGH;
        asm("nop"); 
        asm("nop"); 
       D0_LOW;
        asm("nop");
        asm("nop"); 
  while(i--)
  {
    if(data&0x01)
    {
      D1_HIGH;
    }
    else
    {
      D1_LOW;
    }
    D0_HIGH;
      asm("nop");  
      asm("nop"); 
    D0_LOW;  
    data>>=1;    
  }
}


void OLED_WrDat(uint8 data, uint8 revs)
{
       uint8 i=8;	
       DC_HIGH;
        asm("nop"); 
        asm("nop"); 
       D0_LOW;
        asm("nop");
        asm("nop"); 
       if(revs) data=~data;
  while(i--)
  {
    if(data&0x80)
    {
      D1_HIGH;
    }
    else
    {
      D1_LOW;
    }
      asm("nop");  
      asm("nop"); 
    D0_HIGH;
      asm("nop");  
      asm("nop"); 
    D0_LOW;  
    data<<=1;    
  }
}
void OLED_WrCmd(uint8 cmd)
{
    uint8 i=8;
    DC_LOW;
    asm("nop"); 
    asm("nop"); 
    D0_LOW;   
    asm("nop"); 
    asm("nop"); 
  while(i--)
  {
    if(cmd&0x80)
    {
      D1_HIGH;
    }
    else
    {
     D1_LOW; 
    }
    D0_HIGH;
    asm("nop");   
    asm("nop");  
    D0_LOW;  
    cmd<<=1;   
  } 	
}
void OLED_Set_Pos(uint8 x, uint8 y)
{ 
  OLED_WrCmd(0xb0+y);
  OLED_WrCmd(((x&0xf0)>>4)|0x10);
  OLED_WrCmd(x&0x0f);//|0x01);
} 
void OLED_Draw_Logo()
{
	uint8 y,x;
	int i;

	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);   //0xb0+0~7��ʾҳ0~ҳ7?
		OLED_WrCmd(0x00);       //0x00+0~16��ʾ��128�зֳ�16�����ַ��ĳ���еĵڼ���
		OLED_WrCmd(0x10);      //0x10+0~16��ʾ��128�зֳ�16�����ַ���ڵڼ���
		for(x=0;x<128;x++)
			OLED_WrLogo(mykc_logo[i++]);
	}
}

void  OLED_Draw_camera()
{
  uint8 x,y,i,temp;
  
  
      
  
  for(x=0;x<60;x++)
  {

             
        if(LMR[0][x]!=0)
       {
         img[x*80+LMR[0][x]+1]=0;
         
        }
          if(LMR[1][x]!=0)
       {
         img[x*80+LMR[1][x]]=0;
         
        }
          if(LMR[2][x]!=0)
       {
         img[x*80+LMR[2][x]-2]=0;
         
        }

  }
        
  
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);   //0xb0+0~7��ʾҳ0~ҳ7?
    OLED_WrCmd(0x00);       //0x00+0~16��ʾ��128�зֳ�16�����ַ��ĳ���еĵڼ���
    OLED_WrCmd(0x10);      //0x10+0~16��ʾ��128�зֳ�16�����ַ���ڵڼ���
    
     for(x=0;x<80;x++)
     {
       temp=0;
       for(i=0;i<8;i++)
       {
         temp+=img[y*640+i*80+x]<<(7-i); //��ת��Ϊ��

         
         
     
         
         
         
        if(y==7&&i==3) break;
       }

       OLED_WrLogo(temp);
     }
  }
}

void OLED_Fill(uint8 bmp_data)
{
	uint8 y,x;
	
	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);   //0xb0+0~7��ʾҳ0~ҳ7?
		OLED_WrCmd(0x00);       //0x00+0~16��ʾ��128�зֳ�16�����ַ��ĳ���еĵڼ���
		OLED_WrCmd(0x10);      //0x10+0~16��ʾ��128�зֳ�16�����ַ���ڵڼ���
		for(x=0;x<128;x++)
			OLED_WrDat(bmp_data,0);
	}
}

void OLED_CLS(void)
{
	uint8 y,x;	
	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);
		OLED_WrCmd(0x01);
		OLED_WrCmd(0x10); 
		for(x=0;x<128;x++)
	        OLED_WrDat(0,0);
	}
}
void OLED_DLY_ms(int ms)
{                         
  int a;
  while(ms)
  {
    a=13350;
    while(a--);
    ms--;
  }
  return;
}

void OLED_Init(void)        
{
  gpio_init (RST, GPO,0); //DC
  gpio_init (DC, GPO,0); //D1
  gpio_init (D0, GPO,0); //D0
  gpio_init (D1, GPO,0); //RST
  OLED_DLY_ms(50);
 
  D0_HIGH;
  RST_LOW;
  OLED_DLY_ms(50);
  RST_HIGH;


  OLED_WrCmd(0xae);//--turn off oled panel
  OLED_WrCmd(0x00);//---set low column address
  OLED_WrCmd(0x10);//---set high column address
  OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  OLED_WrCmd(0x81);//--set contrast control register
  OLED_WrCmd(0xcf); // Set SEG Output Current Brightness
  OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
  OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
  OLED_WrCmd(0xa6);//--set normal display
  OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
  OLED_WrCmd(0x3f);//--1/64 duty
  OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WrCmd(0x00);//-not offset
  OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
  OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WrCmd(0xd9);//--set pre-charge period
  OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WrCmd(0xda);//--set com pins hardware configuration
  OLED_WrCmd(0x12);
  OLED_WrCmd(0xdb);//--set vcomh
  OLED_WrCmd(0x40);//Set VCOM Deselect Level
  OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WrCmd(0x02);//
  OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
  OLED_WrCmd(0x14);//--set(0x10) disable
  OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
  OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7) 
  OLED_WrCmd(0xaf);//--turn on oled panel
  OLED_Fill(0);  //��ʼ����
  OLED_Set_Pos(0,0);  
	
} 
//==============================================================
//��������OLED_P6x8Str(uint8 x,uint8 y,uint8 *p)
//����������д��һ���׼ASCII�ַ���
//��������ʾ��λ�ã�x,y����yΪҳ��Χ0��7��Ҫ��ʾ���ַ���
//���أ���
//==============================================================  
void OLED_P6x8Str(uint8 x,uint8 y,char ch[])
{
  uint8 c=0,i=0,j=0,rs=0;      
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(reverse&&c) rs=1;  //�Ƿ�ת��ʾ
                else rs=0;      
    if(x>126){x=0;y++;}
    OLED_Set_Pos(x,y);    
  	for(i=0;i<6;i++)     
  	  OLED_WrDat(F6x8[c][i],rs);  
    x+=6;
    j++;
  }
}

//��ʾһ��6x8��׼ASCII�ַ�
void OLED_P6x8Char(char ch)
{
	 unsigned char c=0,i=0,rs=0;       
	c =ch-32;
         if(reverse&&c) rs=1;
	for(i=0;i<6;i++)
	{     
              OLED_WrDat(F6x8[c][i],rs);  
	}
}

//��һ��char����ת����3λ��������ʾ
void OLED_PrintValueC(unsigned char x, unsigned char y, int data)
{
        OLED_Set_Pos(x,y); 
	unsigned char i,j,k;
	if(data<0)
	{
		OLED_P6x8Char('-');
		data = - data;	
	}
	else
	{
		OLED_P6x8Char('+');
	}
	i = data/100;
	j = (data%100)/10;
	k = data%10;
	OLED_P6x8Char(i+48);
	OLED_P6x8Char(j+48);
	OLED_P6x8Char(k+48);		
}
//������������һ��int����ת����5λ��������ʾ
void OLED_PrintValueI(unsigned char x, unsigned char y, int data)
{       
        OLED_Set_Pos(x,y); 
	unsigned char i,j,k,l,m,fn=0;  
        if(data < 0)
	{
		OLED_P6x8Char('-');
		data = - data;	
	}
	l  = data/10000;
	m= (data%10000)/1000;
	i = (data%1000)/100;
	j = (data%100)/10;
	k = data%10;
	
        if(l)
        {
          OLED_P6x8Char(l+48);
          fn=1;
        }
	if(m||fn)
        {
          OLED_P6x8Char(m+48);
          fn=1;
        }
	if(i||fn)
        {
          OLED_P6x8Char(i+48);
          fn=1;
        }
	if(j||fn)
          OLED_P6x8Char(j+48);
	OLED_P6x8Char(k+48);	
}
//��ʾunsigned int��
 void OLED_PrintValueFP(unsigned int data, unsigned char num)
 {     unsigned char m,i,j,k;   	
 	OLED_P6x8Char('.');
        
	m= data/1000;
	i = (data%1000)/100;
	j = (data%100)/10;
	k = data%10;
	switch(num)
	{

		case 1:  	OLED_P6x8Char(k+48);
				break;
		case 2:  	OLED_P6x8Char(j+48);
				OLED_P6x8Char(k+48);
				break;
		case 3:	OLED_P6x8Char(i+48);
				OLED_P6x8Char(j+48);
				OLED_P6x8Char(k+48);
				break;
		case 4: 	OLED_P6x8Char(m+48);
				OLED_P6x8Char(i+48);
				OLED_P6x8Char(j+48);
				OLED_P6x8Char(k+48);
				break;	
	}
        OLED_P6x8Char(' ');     //��պ�һλ
 }

//������������һ��float����ת����2λ�������ִ�4λС���ͷ��ŵ����ݲ�������ʾ
 void OLED_PrintValueF(unsigned char x, unsigned char y, float data, unsigned char num)
 {
 	unsigned char l,m,i,j,k;  //wan
 	unsigned char Integer_Len = 6; //����λ��         
        
        if(data>0)       
        data=data+0.00001;
  	int Integer_Part = (int)data; //��������
 	long int Float_Part = (long int)((data - (int)data)*100000); //ȡС��λ��5λ
        OLED_Set_Pos(x,y); 
 	//����������ʾ
         
     
        
        if(data<-0.00001)
        OLED_P6x8Char('-'); 

	if(Integer_Part < 0)Integer_Part = - Integer_Part;  //ȥ���������ָ���
 	l  = Integer_Part/10000;
	m= (Integer_Part%10000)/1000;
	i = (Integer_Part%1000)/100;
	j = (Integer_Part%100)/10;
	k = Integer_Part%10;
        
        
        
        
 	if (l != 0)  //��λ
 	{
                Integer_Len = 5;
 		OLED_P6x8Char(l+48);
 		OLED_P6x8Char(m+48);
		OLED_P6x8Char(i+48);
		OLED_P6x8Char(j+48);
		OLED_P6x8Char(k+48);
 	}
 	else if(m != 0) //��λ
 	{
 		Integer_Len = 4;
 		OLED_P6x8Char(m+48);
 		OLED_P6x8Char(i+48);
		OLED_P6x8Char(j+48);
		OLED_P6x8Char(k+48);
 	}
  	else if(i != 0) //��λ
  	{
  		Integer_Len = 3;
  	 	OLED_P6x8Char(i+48);
 		OLED_P6x8Char(j+48);
		OLED_P6x8Char(k+48);
  	}
  	else if(j != 0) //��λ
  	{
    		Integer_Len = 2;	
  		OLED_P6x8Char(j+48);
 		OLED_P6x8Char(k+48);
  	}
	else 	
	{
		Integer_Len = 1;
		OLED_P6x8Char(k+48);
	}	
        if(Float_Part < 0)Float_Part = -Float_Part;
	switch(num-Integer_Len)
	{
		case 0:   OLED_P6x8Char(' '); break;
		case 1:  OLED_PrintValueFP((unsigned int)(Float_Part /10000),num-Integer_Len);break;
		case 2:  OLED_PrintValueFP((unsigned int)(Float_Part /1000),num-Integer_Len);break;
		case 3:  OLED_PrintValueFP((unsigned int)(Float_Part /100),num-Integer_Len);break;
		case 4:  OLED_PrintValueFP((unsigned int)(Float_Part/10),num-Integer_Len);break;					
	}
 }