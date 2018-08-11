/*!
 * @file       AllFunction.h
 * @brief      所有函数头文件函数(自己写的文件)
 * @author     
 * @version    B车
 * @date       
 */

#ifndef __ALLFUNCTION_H__
#define __ALLFUNCTION_H__

/**************************  包含头文件  **************************************/
#include    "MK60_it.h"
#include    "System_Initialization.h"
#include    "Function.h"
#include    "GetMessage.h"
#include    "AngleControl.h"
#include    "SpeedControl.h"
#include    "test.h"
#include    "wireless.h"
#include    "start&stop.h"
#include    "button.h"
#include    "mag3110.h"
/***************************  更新日志  ***************************************/
/*************************  看最新的就好  *************************************/
//双车代码更新日志                                                               
//测试版 v1.0                                                                   
//function里放一些函数                                                          
//anglecontrol放角度控制pid                                                     
//speedcontrol放速度控制（pid）                                                 
//mk60-it放定时器和按键（外部中断）                                             
//system-init放初始化
//getmessage放路况处理和adc采集处理
//allfunction放头文件定义
//（以后将按键控制放成一个文件夹）

//测试版 v2.0
//1.将每个文件都创建好了 
//2.将测试的代码变成函数放在了function里
//3.将一代一些错误引用改正

//测试版 v3.0
//1.加入了舵机模糊pid（参考binary队，并修改了） 在anglecontrol里
//2.加入了电感采集和普通处理 在get message里
//3.main函数里是测试getmessage里的代码，可删
//4.每个函数都加入了写代码日期的备注
//注：代码还没有检查充分 可能会有错

//测试版 v3.1
//1.改正了原来第三代getmessage里的错误
//2.增加了大量的注释
//3.增加了test文件，里面存放前几个版本使用过的测试方案
//4.更新日志放在了ALLFunction里，方便查看

//测试版 v4.0
//1.增加了test_steering函数，可以直接通过按键测试舵机转角
//2.增加了speed_fuzzy_mem_cal_forecast(void);
//        speed_fuzzy_query_forecast(void);
//        speed_fuzzy_solve_forecast(void);  三个函数，放在了SpeedControl函数
//3.修改了test里的test_motor(void) 函数，即增加了模糊速度，但未闭环
//4.注意！！speedcontrol后面的函数是未测试的，该代码是闭环的代码，且有个别参数重
//  复定义，编译时会有 （1）warning ，暂时不理


#endif  //__ALLFUNCTION_H__