/*!
 * @file       test.h
 * @brief      各类函数
 * @author     
 * @version    B车
 * @date       
 */

#ifndef __WIRELESS_H__
#define __WIRELESS_H__

/**************************  声明函数体  **************************************/
unsigned short CRC_CHECK(unsigned char *databuf,unsigned char CRC_CNT);
void OutPut_Data_test(void);
void OutPut_Data_test_sscom(void);

void push(uint8 chanel,uint16 data);
void sendDataToScope(void);

void Freecars_scope(void);

/****************************  宏定义  ****************************************/
//#define UartDataNum  11  //FreeCars上位机 串口数据显示框 接收通道数，按照上位机设置改变  在System_Initialization.h里
#endif  //__WIRELESS_H__  