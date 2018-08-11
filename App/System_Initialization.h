/*!
 * @file       System_Initialization.c
 * @brief      初始化函数
 * @author     
 * @version    B车
 * @date       
 */

#ifndef __SYSTEM_INITIALIZATION_H__
#define __SYSTEM_INITIALIZATION_H__

/*
 * 系统初始化头文件
 */
/**************************  声明函数体  **************************************/
void PIT_Initialization(void); //初始化周期定时器
void FTM_Initialization(void); //初始化FTM时钟
void ISR_Initialization(void); //开启所有中断，这个要在初始化之后
void GPIO_Initialization(void); //各个Io端口初始化
void ADC_Initialization(void); //初始化ADC模块
void OLED_Initialization(void); //OLED初始化
void System_Initialization(void); //总初始化

/****************************  宏定义  ****************************************/
#define  PIT0_TIMER  (5)  //定时器 0 所用的时间
#define  PIT1_TIMER  (5)  //定时器 1 所用的时间
#define  PIT2_TIMER  (1000)  //定时器 2 所用的时间
#define  PIT3_TIMER  (4000)  //定时器 3 所用的时间 

#define Midsteering 672   // 768  1380
#define Minsteering 596  //right  680   1540 
#define Maxsteering 748 //left 左转  855   1715

#define  S3010_FTM   FTM1
#define  S3010_CH    FTM_CH0 //FTM1_CH0_PIN   PTB0
#define  S3010_HZ    (50)

#define MOTOR_HZ    (13*1000)

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH4  //FTM0_CH4_PIN   PTD4   
#define MOTOR2_PWM  FTM_CH5  //FTM0_CH5_PIN   PTD5 
#define MOTOR3_PWM  FTM_CH6  //FTM0_CH6_PIN   PTD6 
#define MOTOR4_PWM  FTM_CH0  //FTM0_CH0_PIN   PTC1 

#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)    //尽量用最后面的扇区，确保安全

#define UartDataNum  11  //FreeCars上位机 串口数据显示框 接收通道数，按照上位机设置改变

#endif  //__SYSTEM_INITIALIZATION_H__