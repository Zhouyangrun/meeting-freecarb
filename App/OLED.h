#ifndef __K60_OLED_H__
#define __K60_OLED_H__

#include "include.h"
#define byte  char
#define uchar unsigned char
#define uint unsigned int
#define SCL_PIN PTA14
#define SDA_PIN PTA13
#define RST_PIN PTA15
#define DC_PIN PTA29

#define INIT 0
#define LED_IMAGE_WHITE 0   //图像白点

#define LED_SCL_Init  gpio_init(PTB9,GPO,INIT) // 时钟初始化定义  

#define LED_SDA_Init  gpio_init(PTB8,GPO,INIT)//数据口D0   

#define LED_RST_Init  gpio_init(PTB17,GPO,INIT)//复位低能电平

#define LED_DC_Init   gpio_init(PTB16,GPO,INIT)//偏置常低

#define LED_SCLH  gpio_set (PTB9, 1)// 时钟定义
#define LED_SCLL  gpio_set (PTB9, 0)

#define LED_SDAH  gpio_set (PTB8, 1)//数据口D0
#define LED_SDAL  gpio_set (PTB8, 0)

#define LED_RSTH  gpio_set (PTB17, 1)//复位低能电平
#define LED_RSTL  gpio_set (PTB17, 0)

#define LED_DCH   gpio_set (PTB16, 1)
#define LED_DCL   gpio_set (PTB16, 0)//偏置常低

/************************************************/

void  LEDPIN_Init(void);   //LED控制引脚初始化

void  OLED_Init(void);

 void LED_CLS(void);

 void LED_Set_Pos(byte x, byte y);//设置坐标函数
 void LED_WrDat(uint8_t data);   //写数据函数

 void LED_P6x8Char(byte x,byte y,byte ch);//发送字符
 void LED_P6x8Str(byte x,byte y,byte ch[]);//发送字符串
 void LED_P8x16Char(byte x,byte y,byte ch);//发送字符
 void LED_P8x16Str(byte x,byte y,byte ch[]);//发送字符串

void  LED_P14x16Str(byte x,byte y,byte ch[]);//发送汉字

 void LED_PrintBMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]);//发送图片

 void LED_Fill(byte dat);//清屏

 void LED_PrintValueC(uint8_t x, uint8_t y,char data);//发送字符型数据
 void LED_PrintValueI(uint8_t x, uint8_t y, int data);//发送整型数据
 void LED_PrintValueF(uint8_t x, uint8_t y, float data, uint8_t num);//发送浮点型数据
void LED_PrintShort(uchar ucIdxX, uchar ucIdxY, int sData);
 void LED_Cursor(uint8_t cursor_column, uint8_t cursor_row);
 void Printf_oled();
  void prif_oled_init();
 extern char logo[];
 void DM_LCD_ShowIntS(uint16_t row,uint16_t num);
void DM_LCD_ShowSTD(void);
 void UART5_send(void );
 extern int i0;
#endif

