/*!
 * @file       MK60_it.h
 * @brief      中断服务函数
 * @author     
 * @version    B车
 * @date       
 */

#ifndef __MK60_IT_H__
#define __MK60_IT_H__


/**************************  声明函数体  **************************************/
void PIT0_IRQHandler(void);  //PIT0中断服务函数
void PIT1_IRQHandler(void);  //PIT1中断服务函数
void PIT2_IRQHandler(void);  //PIT2中断服务函数
void PIT3_IRQHandler(void);  //PIT3中断服务函数
void uart4_test_handler(void);//蓝牙接收中断服务函数


/****************************  宏定义  ****************************************/

#endif  //__MK60_IT_H__