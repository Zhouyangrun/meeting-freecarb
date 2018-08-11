/*!
 * @file       System_Initialization.c
 * @brief      初始化函数
 * @author     
 * @version    B车
 * @date       
 */
   
/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"


/**************************  全局变量   ***************************************/
extern struct _MAG mag_read;
    
/*!
 *  @brief     ADC模块初始化 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void ADC_Initialization(void)
{
    //电感采集初始化
    adc_init (ADC1_SE10); //PTB4
    adc_init (ADC1_SE11); //PTB5
    adc_init (ADC1_SE12); //PTB6
    adc_init (ADC1_SE13); //PTB7
    adc_init (ADC1_SE14); //PTB10
    adc_init (ADC1_SE15); //PTB11
    //电源检测初始化
  //  adc_init (ADC1_SE9); //PTB1
}

/*!
 *  @brief     PIT模块初始化 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void PIT_Initialization(void)
{    
    //初始化PIT0
    pit_init_ms(PIT0, PIT0_TIMER);                                
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); //设置PIT0的中断服务函数为 PIT0_IRQHandler
    //初始化PIT1
   // pit_init_ms(PIT1, PIT1_TIMER);                                
    //set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler); //设置PIT1的中断服务函数为 PIT1_IRQHandler
    //初始化PIT2
   // pit_init_ms(PIT2, PIT2_TIMER);                                
   // set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler); //设置PIT2的中断服务函数为 PIT2_IRQHandler
    //初始化PIT3
    //pit_init_ms(PIT3, PIT3_TIMER);                                
    //set_vector_handler(PIT3_VECTORn ,PIT3_IRQHandler); //设置PIT3的中断服务函数为 PIT3_IRQHandler
}

/*!
 *  @brief     FTM模块初始化 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void FTM_Initialization(void)
{
    //正交解码初始化
    //ftm_quad_init(FTM1); //FTM1 正交解码初始化( PTA8、PIA9 )   FTM1_QDPHA_PIN
    ftm_quad_init(FTM2); //FTM2 正交解码初始化( PTA10、PIA11 )
    //舵机初始化
    ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ,Midsteering); //初始化 舵机 PWM
    //电机初始化 第一代车只需控制 MOTOR2 和 MOTOR3 即可都正转
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 0); //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 0); //初始化 电机 PWM 
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0); //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 0); //初始化 电机 PWM
    //lptmr初始化-编码器
    lptmr_pulse_init(LPT0_ALT1,0xFFFF,LPT_Rising); 
}

/*!·    
 *  @brief     GPIO模块初始化 
 *  @param      
 *  @param      
 *  @since      v5.0          
 *  @warning    
 *  Sample usage:       
 */
void GPIO_Initialization(void)
{
    //拨码器初始化
    gpio_init (PTA28, GPI,0);  //初始化 PTB20 管脚，方向输入
    gpio_init (PTA29, GPI,0);  //初始化 PTB21 管脚，方向输入
    gpio_init (PTA26, GPI,0);  //初始化 PTB22 管脚，方向输入
    gpio_init (PTA27, GPI,0);  //初始化 PTB23 管脚，方向输入
    //按键初始化
    port_init(PTA24, ALT1 | IRQ_FALLING | PULLUP ); //初始化 PTE0管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    port_init(PTB2, ALT1 | IRQ_FALLING | PULLUP ); //初始化 PTE1管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    port_init(PTB3, ALT1 | IRQ_FALLING | PULLUP ); //初始化 PTE2管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    port_init(PTA25, ALT1 | IRQ_FALLING | PULLUP ); //初始化 PTE3管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    //干簧管初始化
    port_init(PTE10, ALT1 | IRQ_FALLING | PULLUP ); //初始化 PTE10管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    //超声波模块测距
    port_init( PTC4, ALT1 | IRQ_RISING | PULLUP ); //初始化 PTB18管脚，复用功能为GPIO
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler); //设置PORTE的中断服务函数为 PORTE_IRQHandler
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler); //设置PORTA的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler); //设置PORTB的中断服务函数为 PORTB_IRQHandler
    set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler); //设置PORTC的中断服务函数为 PORTC_IRQHandler
    //蜂鸣器初始化
    gpio_init (PTE1, GPO, 0); //初始化 PTC8管脚，输出，初始低电平
    
 //   MAG3110_Init(); //磁力计初始化
   // mag_read.mag_x_offset = -885;
   // mag_read.mag_y_offset = 680;
    
}

/*!
 *  @brief     OLED模块初始化 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void OLED_Initialization(void)
{
    //OLED初始化
    OLED_Init();
}

/*!
 *  @brief     总初始化 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void System_Initialization(void)
{
    PIT_Initialization();
    GPIO_Initialization();
    FTM_Initialization();
    ADC_Initialization(); 
    OLED_Initialization();
    uart_init (UART4, 9600);//串口初始化
}

/*!
 *  @brief     中断启动函数
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void ISR_Initialization(void)
{
    
    enable_irq(PIT0_IRQn); // 使能PIT0中断
   // enable_irq(PIT1_IRQn); // 使能PIT1中断
   // enable_irq(PIT2_IRQn); // 使能PIT2中断
   // enable_irq(PIT3_IRQn); // 使能PIT3中断
    enable_irq (PORTE_IRQn); //使能PORTE中断
    enable_irq (PORTA_IRQn); //使能PORTE中断
    enable_irq (PORTB_IRQn); //使能PORTE中断
    enable_irq (PORTC_IRQn); //使能PORTE中断
   // DisableInterrupts;
   // beep_on();  //初始化成功响一下
   // DELAY();
   // beep_off();
    DELAY_MS(500);
   // EnableInterrupts; //同时启动中断
    
}
