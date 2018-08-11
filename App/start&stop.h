/*!
 * @file       start&stop.h
 * @brief      各类函数
 * @author     
 * @version    B车
 * @date       
 */

#ifndef __STARTSTOP_H__
#define __STARTSTOP_H__

/**************************  声明函数体  **************************************/
void start_car(void);
void stop_car(void);
void turn_car(void);

/****************************  宏定义  ****************************************/
#define start_Seconds (400)  //开车靠边所用的秒数 1seconds=5ms 400=2s

#endif  //__STARTSTOP_H__  