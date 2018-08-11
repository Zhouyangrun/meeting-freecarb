/*!
 * @file       MK60_it.c
 * @brief      中断服务函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
int16 times = 0; //定时停车标志位 PIT0定时器
uint8 car_dis_flag = 0; //高电平开始标记位
uint16 car_dis = 0;  //超声波测距距离 单位cm
uint8 car_dis_ms = 0; //超声波测高电平的时间 单位ms
uint8 car_dis_times = 0; //（用于超声波）
uint8 now_vol = 0;//现在的电平
uint8 last_vol = 0;//上次的电平
uint16 delay_flag = 0; //用于周期延时使用
uint8 wait_flag = 0;//终点等待标志位
uint8 shizi = 0; //经过十字的个数
uint8 is_shizi = 0;//疑是十字判断，用于排除圆环
uint8 avoid_flag_shizi = 0;//第几个十字另一辆车避开，如果为0，则执行第二种会车方式
uint8 go_flag_shizi = 3; //第几个十字另一辆车继续行动
uint8 last_flag_shizi = 6;//第几个十字减速
uint16 gameover = 0;//level为100时使用，会车结束后一段时间flag无法置1，用于防止出赛道
uint8 round_times = 0;
char bluetooth_data=0;//接收到的数据存在这个变量
float last_speed_power = 0.3;//最后一个十字减速的幅度
extern uint16 start_flag;
extern uint8 level;
extern uint16 last_stop; 
extern uint16 flag;
extern uint16 dis_right;
extern float ADC_Normal[5];
extern uint16 dis_back;
extern float speed_power;
extern uint8 left_flag;
extern uint8 right_flag;
extern uint16 clj;
extern struct _MAG mag_read;
extern uint16 turn_car_dis;
extern uint16 last_start_flag;
extern int8 ones;
extern uint16 round_is,round_in,round_over,round_num,round_stop_flag,max_PWM,max_PWM_new,round_lr,round_vaule;

/******************************************************************************* 
 *  @brief      PIT0中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
   if(times > 0)  //检测该十字是否为十字
    {
       times--;
       if(times == 25 && is_shizi == 1) //第15个周期后进行判断，若前15个周期没有检测到环标志，则为十字
       {
            shizi++;//十字数量加1
            beep_on(); 
            if( shizi == avoid_flag_shizi) //第n个十字发信号
            {    
                uart_putchar (UART4,'3');
                uart_putchar (UART4,'3');
                uart_putchar (UART4,'3');
                uart_putchar (UART4,'3');
                speed_power = 0.5;
                  
            }
            if( shizi == go_flag_shizi )  //第n个十字发信号
            {    
                uart_putchar (UART4,'4');
                uart_putchar (UART4,'4');
                uart_putchar (UART4,'4');
                uart_putchar (UART4,'4');
                if( avoid_flag_shizi == 0)//新策略，第二种会车方式
                {
                    speed_power = 0.1;
                }
                else
                {
                    speed_power = 1;
                }
            }   
            if(shizi == go_flag_shizi + 1)//新策略，第二种会车方式
            {
                if( avoid_flag_shizi == 0)//新策略
                {
                    speed_power = 1;
                    times = 100;//滤去多余十字
                    is_shizi = 0;
                }
            }
            if( shizi == last_flag_shizi)//判断是否为最后一个十字
            {
                speed_power = last_speed_power; //最后一个十字减速
            }
        }
        if( ADC_Normal[4] > 2) //检测到环标志，is_shizi置0，表示不是十字
        {
            is_shizi = 0;
        }
        if(times < 1)
        {
            beep_off(); 
        }
    }
    else //检测是否有十字
    {
        if(level == 1) // 正常模式
        {
            if(ADC_Normal[0] > 0.5 && ADC_Normal[3] > 0.5)//如果直电感大于一定值，则判断疑是十字
            {
                times = 40;//进入200ms的判断时间，一个times是5ms      
                is_shizi = 1;//疑是十字
            }
        }
    }
    
    PIT_Flag_Clear(PIT0);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT1中断服务函数
 *  @note       
 ******************************************************************************/
void PIT1_IRQHandler(void)
{
    if( start_flag > 0 ) //发车函数，优先级最高，一个start_flag为5ms
    {
        start_car();
        delay_flag = 120; //发车之后用于防止赛道出界，防止因为电感偏小而flag置1
    }
     ///////////////////////////////////////////////////////////////////////////
    else if( level == 40 )
    {
        if(last_stop <= 130)  stop_car(); //刹车函数
        else  //刹车结束后进行倒车
        {
            if(wait_flag == 0) 
            {
                turn_car(); //倒车函数
                if( speed_power > 0.5 && wait_flag == 0 ) // 高速情况直接发信号 确认是否可以会车
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    speed_power = 0.1;
                }
                if( flag == 1 )  // 倒车完毕 发信号
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    wait_flag = 1;
                }
            }
            else //另一辆车出界的情况，直接冲线
            {
                test_motor();
                flag = 1;
            }
        } 
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 100 ) //终点回赛道停车，防止出界
    {
        gameover++;
        speed_power = 0.1;
        test_motor();
        if(gameover > 200)  flag = 1;
    }
     ///////////////////////////////////////////////////////////////////////////
    else 
    {
       test_motor();//正常行驶程序
    } 
    PIT_Flag_Clear(PIT1);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT2中断服务函数
 *  @note       超声波测距（注释的代码），后期用于清除环标志
 *  @warning
 ******************************************************************************/
void PIT2_IRQHandler(void)
{/*
    ///////////////////////////超声波测距//////////////////////////////////////
    last_vol = now_vol;
    if( gpio_get(PTC4) == 1 )
        now_vol = 1;
    else 
        now_vol = 0;
    if(last_vol == 0 && now_vol == 1)  //有上升沿
    {
        car_dis_flag = 1;
        car_dis_ms = 0;
    }
      
    if(car_dis_flag == 1)  //超声波测试标记位
    {
        if( car_dis_ms > 150)  //超过 15ms 即为无效数据
            car_dis_flag = 0;
        else
        {
            if( gpio_get(PTC4) == 1 ) //高电平使计数增加5 即高电平时间增加0.5ms
                car_dis_ms += 10;
            else
            {   */
              /*  car_dis_value[car_dis_times] = 34 * car_dis_ms;
                if(car_dis_times == 4) car_dis_times = 0;
                car_dis_times++;
              //  car_dis = 34 * car_dis_ms; //算出超声波距离  mm
                car_dis = car_dis_value[0] + car_dis_value[1] + car_dis_value[2] + car_dis_value[3] + car_dis_value[4];
                car_dis_flag = 0; //重置标记位  */
            /*    if( car_dis < 1500 )   
		{
                   // if( start_flag < 5) start_flag = 10;
                    //else if(start_flag < 600) start_flag = start_flag * 3;
                   // start_flag = 600;
                    //beep_on();
                    level = 40;
		}
                if( car_dis > 1500 )   
		{
                   // if( start_flag < 5) start_flag = 10;
                    //else if(start_flag < 600) start_flag = start_flag * 3;
                   // start_flag = 600;
                    //beep_off();
		}
                car_dis_flag = 0;
                car_dis = 34 * car_dis_ms; 
            }
        }        
    } */
  //////////////////////////////清除环标志////////////////////////////////////
    if( round_times != 0)
    {
        round_times--;
        if( round_times == 0)
        {
            round_in=0;
            round_is=0;
            round_over=0;
            if(level == 4) //误判十字
                level = 88;
            else if(level == 5) //误判十字
                level = 1;
            round_stop_flag=1;
            max_PWM=max_PWM_new;//恢复pwm限制
            if((round_lr==1)||(round_lr==0))
              round_lr=!round_lr;
        }
    }  
    else if(round_is != 0 && round_vaule != 0)
    {
        round_times = 8;
    }
    PIT_Flag_Clear(PIT2);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT3中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT3);       //清中断标志位
}

/*!
 *  @brief      UART3测试中断服务函数
 *  @since      v5.0
 *  @warning    蓝牙通讯函数，使用必须开中断才能使用 ,在你需要发送的地方用uart_putchar函数，如果uart_putchar(UART4,'1');
 *  Sample usage:  set_vector_handler(UART3_RX_TX_VECTORn , uart3_test_handler);    //把 uart3_handler 函数添加到中断向量表，不需要我们手动调用
 */
void uart4_test_handler(void)
{
 
    if(uart_query(UART4) == 1)   //接收数据寄存器满
    {
      uart_getchar(UART4, &bluetooth_data);//等待接收完//*bluetooth_data = UART_D_REG(UARTN[uratn]);///////////bluetooth_data是一个char型变量，你喜欢干啥就干啥
      if(bluetooth_data ==  '1') 
      {
        if( wait_flag == 1 )//两车已经做好准备会车
         {
              uart_putchar (UART4,'2'); 
              uart_putchar (UART4,'2'); 
              uart_putchar (UART4,'2'); 
              uart_putchar (UART4,'2'); 
              flag = 0;
              wait_flag = 0;
              start_flag = last_start_flag;
              level = 100;
         }
        // bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
      if(bluetooth_data ==  '2') //两车已经做好准备会车
      {
         flag = 0;
         wait_flag = 0;
         start_flag = last_start_flag;
         level = 100;
         //bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
      if(bluetooth_data ==  '8') //一辆车出界，收到信号后本车直接跑
      {
         level = 88;
         flag = 0;
         speed_power = 1;
         //bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
    }
}



