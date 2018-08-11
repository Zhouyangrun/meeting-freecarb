/*!
 * @file       Function.c
 * @brief      各类函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
uint8 write_flash_flag = 0; //写入重要数据标志位
uint8 read_flash_flag = 0; //读取重要数据标志位
uint16 turn_car_dis = 4000; //倒车距离
uint16 last_start_flag = 300; //最后会车摆动时间
uint8 switch_mode = 100; //显示屏页码
uint8 page_line = 1; //行数
byte hello[][28] = {

{0x00,0xC0,0xC0,0xE0,0xB0,0x40,0xC0,0x50,0x60,0x40,0x40,0xC0,0xC0,0x40,
0x00,0x00,0x00,0x3F,0x00,0x0D,0x1E,0x17,0x17,0x16,0x3E,0x00,0x00,0x00},/*"信",0*/

{0x00,0x00,0x60,0x20,0x20,0x20,0xE0,0x60,0x20,0x20,0x20,0x00,0x00,0x00,
0x10,0x18,0x08,0x08,0x08,0x08,0x0F,0x08,0x08,0x18,0x18,0x18,0x18,0x08},/*"工",1*/

{0x00,0x80,0x80,0xB0,0xF0,0xA0,0xA0,0x60,0xE0,0x20,0x20,0xE0,0x00,0x00,
0x00,0x02,0x02,0x03,0x1F,0x1A,0x1B,0x2A,0x2B,0x3F,0x01,0x01,0x00,0x00},/*"智",2*/

{0x00,0x40,0x60,0xB0,0x50,0x60,0x40,0xF0,0xD0,0xA0,0xB0,0x80,0x00,0x00,
0x10,0x30,0x19,0x07,0x0B,0x3F,0x00,0x0F,0x1C,0x14,0x12,0x30,0x30,0x08},/*"能",3*/

{0x00,0x00,0x00,0x20,0x20,0xA0,0x70,0xB0,0x20,0x20,0x20,0x20,0x00,0x00,
0x00,0x0C,0x0C,0x04,0x07,0x05,0x05,0x3F,0x05,0x05,0x0D,0x0C,0x0C,0x04},/*"车",4*/

};
extern uint16 ADC_Value[5];
extern uint16 SUM_ADC_GetMessage[5];
extern uint16 ADC_Maxing[5];
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float ADC_Normal[5];
extern float fe,fec;
extern float speed_fec,speed_fe;
extern float steer_P;
extern float speed_Rule[5];
extern float speed_error_Rule[5];
extern int16 steerctrl;
extern float speed_forecast;
extern int16 steerctrl_error;
extern int16 speed_forecast_left ;  //左轮车速
extern int16 speed_forecast_right ;  //右轮车速
extern int8 tab;
extern float speed_fec_max;
extern int16 speed_now_left,speed_now_right;
extern uint8 level;
extern uint16 cross;
extern uint16 cross_pass;
extern uint16 car_dis;
extern int16 dis_left,dis_right;
extern char bluetooth_data;
extern uint8 shizi;
extern uint8 avoid_flag_shizi;
extern uint8 last_flag_shizi;
extern uint8 go_flag_shizi;
extern uint8 turn_left_flag;
extern uint8 turn_right_flag;
extern float steer_D;
extern float last_speed_power;
extern uint16 max_PWM;
extern struct _MAG mag_read;
extern float eRule[5];
/////////////////////////////////////////////////////////////////////////////// 
extern uint16 round_vaule;// round_vaule=0       不入环
                       // round_vaule=1       环在左边
                       // round_vaule=2       环在右边
//识别阈值
extern float  round_up_vaule;
extern float round_down_vaule;
//刹车强度
extern uint8 round_stop_vaule;
extern float round_shizi;
/*******************************************************************************
 *  @brief      beep_on函数
 *  @note       蜂鸣器一直响
   
                响一下的函数为
                beep_on();
                DELAY(); //延时500ms
                beep_off();
 *  @warning
 ******************************************************************************/
void beep_on(void)
{
    gpio_set(PTE1,1);
}

/*******************************************************************************
 *  @brief      beep_off函数
 *  @note       蜂鸣器关闭
 *  @warning
 ******************************************************************************/
void beep_off(void)
{
    gpio_set(PTE1,0);
}

/*******************************************************************************
 *  @brief      oled_view函数
 *  @note       oled显示
 *  @warning    18/3/25
 ******************************************************************************/
void oled_view(void)
{
     /********* 拨码器 0000 *********/
     /*********  查询电感值 归一化值 误差 左右轮子车速 *********/
     if( gpio_get(PTA28) == 0 && gpio_get(PTA29) == 0 && gpio_get(PTA26) == 0 && gpio_get(PTA27) == 0 )
     {   
         if( switch_mode != 0) //清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(40,0,"normal");
         LED_PrintValueF(4,1,ADC_Normal[0],3);  //显示绿色电感归一化后值
         LED_PrintValueF(4,2,ADC_Normal[1],3);  //显示蓝色电感归一化后值
         LED_PrintValueF(4,3,ADC_Normal[2],3);  //显示褐色电感归一化后值
         LED_PrintValueF(4,4,ADC_Normal[3],3);  //显示橙色电感归一化后值
         LED_PrintValueF(4,5,ADC_Normal[4],3);  //显示归一化后值
         LED_PrintShort(90,1,ADC_Value[0]); 
         LED_PrintShort(90,2,ADC_Value[1]); 
         LED_PrintShort(90,3,ADC_Value[2]); 
         LED_PrintShort(90,4,ADC_Value[3]); 
         LED_PrintShort(90,5,ADC_Value[4]); 
         LED_PrintShort(40,6,level); 
         LED_PrintValueF(40,4,fe,3);  //显示输入的误差
         LED_PrintShort(90,7,speed_now_right); 
         LED_PrintShort(4,7,speed_now_left); 
         switch_mode = 0;
     }
     /********* 拨码器 1000 *********/
     /*********  flash中的最大电感值 *********/
     else if( gpio_get(PTA28) == 1 && gpio_get(PTA29) == 0 && gpio_get(PTA26) == 0 && gpio_get(PTA27) == 0 )
     {
         if( switch_mode != 1)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(4,0,"stop message");  
         
         LED_P6x8Str(4,2,"how many dis:");
         LED_P6x8Str(4,3,"turn_car_dis=");
         LED_PrintShort(4,4,turn_car_dis);  
         LED_P6x8Str(80,4,"(up-d)");
         LED_P6x8Str(4,6,"last_start_flag=");
         LED_PrintShort(4,7,last_start_flag);  
         LED_P6x8Str(80,7,"(left-r)");
         switch_mode = 1;
     }
     /********* 拨码器 1100 *********/
     /*********  会车数据 *********/
     else if( gpio_get(PTA28) == 1 && gpio_get(PTA29) == 1 && gpio_get(PTA26) == 0 && gpio_get(PTA27) == 0 )
     {
         if( switch_mode != 2)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(4,0,"meeting message");  
         LED_P6x8Str(4,1,"meeting"); 
         LED_P6x8Str(4,2,"which shizi meeting:");
         LED_P6x8Str(4,3,"avoid_flag_shizi="); 
         LED_PrintShort(4,4,avoid_flag_shizi);  
         LED_P6x8Str(80,4,"(up-d)");
         LED_P6x8Str(4,6,"go_flag_shizi=");
         LED_PrintShort(4,7,go_flag_shizi);  
         LED_P6x8Str(80,7,"(left-r)");
         switch_mode = 2;
     }
     /********* 拨码器 1110 *********/
     /*********  会车数据 *********/
     else if( gpio_get(PTA28) == 1 && gpio_get(PTA29) == 1 && gpio_get(PTA26) == 1 && gpio_get(PTA27) == 0 )
     {
         if( switch_mode != 3)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(4,0,"meeting message"); 
         LED_P6x8Str(4,1,"speed reduction");   
         LED_P6x8Str(4,2,"which shizi reduce:");
         LED_P6x8Str(4,3,"last_flag_shizi=");
         LED_PrintShort(4,4,last_flag_shizi);  
         LED_P6x8Str(80,4,"(up-d)");
         LED_P6x8Str(4,5,"reduce how many:");
         LED_PrintValueF(4,6,last_speed_power,3);
         LED_P6x8Str(80,7,"(left-r)");
         switch_mode = 3;
     }
     /********* 拨码器 1111 *********/
     /*********  环 *********/
     else if( gpio_get(PTA28) == 1 && gpio_get(PTA29) == 1 && gpio_get(PTA26) == 1 && gpio_get(PTA27) == 1 )
     {
         if( switch_mode != 4)//清屏  round_vaule round_up_vaule round_down_vaule round_stop_vaule  
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(20,0,"round"); 
         LED_P6x8Str(15,2,"round_vaule"); 
         LED_PrintShort(85,2,round_vaule); 
         LED_P6x8Str(15,3,"round_up"); 
         LED_PrintValueF(85,3,round_up_vaule,3); 
         LED_P6x8Str(15,4,"round_down"); 
         LED_PrintValueF(85,4,round_down_vaule,3);
         LED_P6x8Str(15,5,"round_stop"); 
         LED_PrintShort(85,5,round_stop_vaule); 
         LED_P6x8Str(15,6,"round_shizi");
         LED_PrintValueF(85,6,round_shizi,3); 
         if( page_line == 1)
         {
            LED_P6x8Str(4,2,"o");
            LED_P6x8Str(4,3,"x");
            LED_P6x8Str(4,4,"x");
            LED_P6x8Str(4,5,"x");
            LED_P6x8Str(4,6,"x");
         }
         else if( page_line == 2)
         {
            LED_P6x8Str(4,2,"x");
            LED_P6x8Str(4,3,"o");
            LED_P6x8Str(4,4,"x");
            LED_P6x8Str(4,5,"x");
            LED_P6x8Str(4,6,"x");
         }
         else if( page_line == 3)
         {
            LED_P6x8Str(4,2,"x");
            LED_P6x8Str(4,3,"x");
            LED_P6x8Str(4,4,"o");
            LED_P6x8Str(4,5,"x");
            LED_P6x8Str(4,6,"x");
         }
         else if( page_line == 4)
         {
            LED_P6x8Str(4,2,"x");
            LED_P6x8Str(4,3,"x");
            LED_P6x8Str(4,4,"x");
            LED_P6x8Str(4,5,"o");
            LED_P6x8Str(4,6,"x");
         }
         else if( page_line == 5)
         {
            LED_P6x8Str(4,2,"x");
            LED_P6x8Str(4,3,"x");
            LED_P6x8Str(4,4,"x");
            LED_P6x8Str(4,5,"x");
            LED_P6x8Str(4,6,"o");
         }
         switch_mode = 4;
     }
     /********* 拨码器 0111 *********/
     /*********  速度最大PWM *********/
     else if( gpio_get(PTA28) == 0 && gpio_get(PTA29) == 1 && gpio_get(PTA26) == 1 && gpio_get(PTA27) == 1 )
     {
         if( switch_mode != 5)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(20,0,"max_PWM");    
         LED_PrintShort(5,1,max_PWM);
         LED_P6x8Str(80,1,"(up-d)");     
         LED_P6x8Str(40,2,"the max ADC");
         LED_PrintShort(4,3,ADC_Maxing[0]); 
         LED_PrintShort(4,4,ADC_Maxing[1]); 
         LED_PrintShort(4,5,ADC_Maxing[2]); 
         LED_PrintShort(4,6,ADC_Maxing[3]); 
         LED_PrintShort(4,7,ADC_Maxing[4]); 
         switch_mode = 5;
     }
     /********* 拨码器 0011 *********/
     /*********  舵机PID *********/
     else if( gpio_get(PTA28) == 0 && gpio_get(PTA29) == 0 && gpio_get(PTA26) == 1 && gpio_get(PTA27) == 1 )
     {
         if( switch_mode != 6)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(20,0,"change pid"); 
         LED_P6x8Str(5,1,"P");
         LED_PrintValueF(4,2,Rule_kp[0],2);
         LED_PrintValueF(4,3,Rule_kp[1],2);
         LED_PrintValueF(4,4,Rule_kp[2],2);
         LED_PrintValueF(4,5,Rule_kp[3],2);
         LED_PrintValueF(4,6,Rule_kp[4],2);
         LED_P6x8Str(10,7,"(up-d)");                 
         LED_P6x8Str(65,1,"D");   
         LED_PrintValueF(60,2,steer_D,2);
         LED_P6x8Str(60,7,"(left-r)");
         switch_mode = 6;
     }
     /********* 拨码器 0001 *********/
     /*********  存读数据 *********/
     else if( gpio_get(PTA28) == 0 && gpio_get(PTA29) == 0 && gpio_get(PTA26) == 0 && gpio_get(PTA27) == 1 )
     {
         if( switch_mode != 7)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(20,0,"flash order");
         LED_P6x8Str(4,2,"read flash");
         LED_P6x8Str(80,2,"(down)");
         if( read_flash_flag == 1)
         {
            LED_P6x8Str(20,3,"ok");
         }
         LED_P6x8Str(4,4,"write flash");
         LED_P6x8Str(80,4,"(left)");        
         switch_mode = 7;
         if( write_flash_flag == 1)
         {
            LED_P6x8Str(20,5,"ok");
         }
     }
     /********* 拨码器 1010 *********/
     /*********   p范围   *********/
     else if( gpio_get(PTA28) == 1 && gpio_get(PTA29) == 0 && gpio_get(PTA26) == 1 && gpio_get(PTA27) == 0 )
     {
         if( switch_mode != 8)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(20,0,"eRule"); 
         LED_PrintValueF(4,2,eRule[0],2);
         LED_PrintValueF(4,3,eRule[1],2);
         LED_PrintValueF(4,4,eRule[2],2);
         LED_PrintValueF(4,5,eRule[3],2);
         LED_PrintValueF(4,6,eRule[4],2);
         LED_P6x8Str(10,7,"(up-d)"); 
         switch_mode = 8;
     }
      /********* 拨码器 1001 *********/
     /*********  调整速度 *********/
     else if( gpio_get(PTA28) == 1 && gpio_get(PTA29) == 0 && gpio_get(PTA26) == 0 && gpio_get(PTA27) == 1 )
     {
         if( switch_mode != 10)//清屏
         {
            LED_Fill(0x00);
         }
         LED_P6x8Str(20,0,"change speed"); 
         LED_P6x8Str(4,1,"speed");
         LED_PrintValueF(4,2,speed_Rule[4],2);
         LED_PrintValueF(4,3,speed_Rule[3],2);
         LED_PrintValueF(4,4,speed_Rule[2],2);
         LED_PrintValueF(4,5,speed_Rule[1],2);
         LED_PrintValueF(4,6,speed_Rule[0],2);
         LED_P6x8Str(10,7,"(up-d)");                 
         LED_P6x8Str(60,1,"speederr");   
         LED_PrintValueF(60,2,speed_error_Rule[0],2);
         LED_PrintValueF(60,3,speed_error_Rule[1],2);
         LED_PrintValueF(60,4,speed_error_Rule[2],2);
         LED_PrintValueF(60,5,speed_error_Rule[3],2);
         LED_PrintValueF(60,6,speed_error_Rule[4],2);
         LED_P6x8Str(60,7,"(left-r)");
         switch_mode = 10;
     }
}
/*******************************************************************************
 *  @brief      write_flash函数
 *  @note       
 *  @warning    18/7/15
 ******************************************************************************/
void write_flash(void)
{
    flash_erase_sector(SECTOR_NUM);                     //擦除扇区
                                                       //写入flash数据前，需要先擦除对应的扇区(不然数据会乱)
    flash_write(SECTOR_NUM, 0, ADC_Maxing[0] );   //写入数据到扇区，偏移地址为0，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 4, ADC_Maxing[1] );   //写入数据到扇区，偏移地址为4，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 8, ADC_Maxing[2] ) ;  //写入数据到扇区，偏移地址为8，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 12, ADC_Maxing[3] ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 16, ADC_Maxing[4] ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 20, (uint16)avoid_flag_shizi ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 24, (uint16)go_flag_shizi ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 28, (uint16)last_flag_shizi ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 32, (uint16)(last_speed_power * 100) ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 36, max_PWM ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 40, turn_car_dis ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 44, last_start_flag ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 48, round_vaule ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 52, (uint16)(round_up_vaule * 100) ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 56, (uint16)(round_down_vaule * 100) ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 60, (uint16) round_stop_vaule ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 64, (uint16)(round_shizi * 100) ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
   
    
    write_flash_flag = 1;
}

/*******************************************************************************
 *  @brief      read_flash函数
 *  @note       
 *  @warning    18/7/15
 ******************************************************************************/
void read_flash(void)
{
   /* ADC_Maxing[0] = flash_read(SECTOR_NUM, 0, uint16);  //读取16位
    ADC_Maxing[1] = flash_read(SECTOR_NUM, 4, uint16);  //读取16位
    ADC_Maxing[2] = flash_read(SECTOR_NUM, 8, uint16);  //读取16位
    ADC_Maxing[3] = flash_read(SECTOR_NUM, 12, uint16);  //读取16位 2字节
    ADC_Maxing[4] = flash_read(SECTOR_NUM, 16, uint16);  //读取16位 2字节*/
    avoid_flag_shizi = flash_read(SECTOR_NUM, 20, uint16); 
    go_flag_shizi    = flash_read(SECTOR_NUM, 24, uint16); 
    last_flag_shizi  = flash_read(SECTOR_NUM, 28, uint16); 
    last_speed_power = ((float)(flash_read(SECTOR_NUM, 32, uint16))) / 100; 
    max_PWM          = flash_read(SECTOR_NUM, 36, uint16); 
    turn_car_dis     = flash_read(SECTOR_NUM, 40, uint16); 
    last_start_flag  = flash_read(SECTOR_NUM, 44, uint16); 
    round_vaule      = flash_read(SECTOR_NUM, 48, uint16);
    round_up_vaule   = ((float)(flash_read(SECTOR_NUM, 52, uint16))) / 100;
    round_down_vaule = ((float)(flash_read(SECTOR_NUM, 56, uint16))) / 100;
    round_stop_vaule = flash_read(SECTOR_NUM, 60, uint16);
    round_shizi = ((float)(flash_read(SECTOR_NUM, 64, uint16))) / 100;
    read_flash_flag  = 1;
}











