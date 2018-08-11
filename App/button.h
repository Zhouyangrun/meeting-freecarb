/*!
 * @file       button.h
 * @brief      中断服务函数
 * @author     
 * @version    B车
 * @date       
 */

#ifndef __BUTTON_H__
#define __BUTTON_H__


/**************************  声明函数体  **************************************/
void PORTE_IRQHandler(void);  //PORTE的参考中断服务函数
void PORTC_IRQHandler(void);
void PORTB_IRQHandler(void);
void PORTA_IRQHandler(void);

/****************************  宏定义  ****************************************/

#endif  //__BUTTON_H__