#include "include.h"
uint8 Next_Valid_Block=0;//现在数据所在的块，一共1024/BLOCK_SIZE块



/****************       flash模拟EEPROM   把最后一个扇区分成 16块 ，循环写值，防止对flash的擦写次数过多造成损坏（当然一般情况下是多虑了） 

掉电后值可保存


*****************/


void EEPROM_init()
{
  int i;
  uint8 temp;
  uint8 find=0;
  flash_init();
  temp=flash_read(SECTOR_NUM,0,uint8);
  DELAY_MS(100);
  if(temp!=0xaa)//说明该该扇区没有存储数据
  {
     flash_erase_sector(SECTOR_NUM);                     //擦除扇区
     Next_Valid_Block=0;
     EEPROM_Save();
  }
  else
  { 
    Next_Valid_Block=0;
    for(i=BLOCK_NUM-1;i>=0;i--)
    {
      temp=flash_read(SECTOR_NUM,i*BLOCK_SIZE,uint8);
      if(temp==0xaa)  //说明该该扇区有存储数据
      {
        Next_Valid_Block=i+1;    //找到下一块没有存储的地址
        find=1;
        break;
      }
    }
    if(find==1)
    {
      EEPROM_Read();
      Para_Update();
    } 
  }
}

void EEPROM_Save()
{
  uint8 i,addr=0;
  uint8 buff[60]={0};
  uint8 Parameter_num=15;  //14个可改参数
  buff[0]=0xaa;    //这两个用来注明该块已经有数据了
  buff[1]=(uint8)(Next_Valid_Block%16+1);
  addr=2;
  
  /**************                           *************/
  
  for(i=0;i<Parameter_num;i++)
  {    
    buff[addr]=BYTE0(Control_Para[i]);
    addr++;
    buff[addr]=BYTE1(Control_Para[i]);
    addr++;
    buff[addr]=BYTE2(Control_Para[i]);
    addr++;
    buff[addr]=BYTE3(Control_Para[i]);
    addr++;
  }
   if(Next_Valid_Block<BLOCK_NUM) //32
   {
     flash_write_buf(SECTOR_NUM,Next_Valid_Block*BLOCK_SIZE,sizeof(buff),buff);
     if(flash_read(SECTOR_NUM,Next_Valid_Block*BLOCK_SIZE,uint8)!=0xaa)
     {
       uint32 temp=*(uint32 *)buff;
       flash_write(SECTOR_NUM,Next_Valid_Block*BLOCK_SIZE,temp);
     }
     Next_Valid_Block++;
   }
   else //没有新的空间了
   {
     flash_erase_sector(SECTOR_NUM);                     //擦除扇区
     Next_Valid_Block=0;
     flash_write_buf(SECTOR_NUM,Next_Valid_Block*BLOCK_SIZE,sizeof(buff),buff);
     if(flash_read(SECTOR_NUM,Next_Valid_Block*BLOCK_SIZE,uint8)!=0xaa)
     {
       uint32 temp=*(uint32 *)buff;
       flash_write(SECTOR_NUM,Next_Valid_Block*BLOCK_SIZE,temp);
     }
     Next_Valid_Block++;
   }
}
void EEPROM_Read()
{
   uint8 i=0,addr=2;
   float temp;
   uint8 Parameter_num=15;  //14个可改参数
   for(i=0;i<Parameter_num;i++)
  {
       BYTE0(temp)=flash_read(SECTOR_NUM,(Next_Valid_Block-1)*BLOCK_SIZE+addr,uint8);
       addr++;
       BYTE1(temp)=flash_read(SECTOR_NUM,(Next_Valid_Block-1)*BLOCK_SIZE+addr,uint8);
       addr++;
       BYTE2(temp)=flash_read(SECTOR_NUM,(Next_Valid_Block-1)*BLOCK_SIZE+addr,uint8);
       addr++;
       BYTE3(temp)=flash_read(SECTOR_NUM,(Next_Valid_Block-1)*BLOCK_SIZE+addr,uint8);
       addr++;
       Control_Para[i]=temp;
   }
    Para_Update();
}
