/*!
 * @file       start&stop.c
 * @brief      存放各类之前测试的函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
uint16 start_flag = 0;//开车标记位，1个单位为5ms
                      //例如start_flag = 300，意义为1.5s进入发车程序（ start_car(void) )
int16 dis_left = 0;//左轮走的总距离
int16 dis_right = 0;//右轮走的总距离
uint16 dis_back = 3000;//倒车后退的距离，会有惯性滑行一下
extern float ADC_Normal[5];
extern uint16 ADC_Value[5];
extern float fe,fec;
extern int16 steerctrl;
extern int16 speedctrl_left,speedctrl_right; 
extern uint8 flag;
extern int16 speed_now_left,speed_now_right;
extern float speed_power;
extern uint8 none_steerctrl; 
extern int16 speedctrl_left,speedctrl_right; 
extern int16 speedctrl_left_opp,speedctrl_right_opp; 
extern uint16 last_stop;  //停车计算用，停车时由于两轮计数方式不一样
                          //因此需要计时再将轮子PWM为0，才不会出现车抖动
extern float speed_forecast; //预测将要达到的速度（PWM）
extern float speed_forecast_error; //预测将要达到的速度的偏差（差速）
extern float fe,fe1,fe2,fe_last;
extern float P_power;
extern float D_power;


/*******************************************************************************
 *  @brief      start_car(void) 
 *  @note       车向右偏
 *  @warning    
 ******************************************************************************/
void start_car(void)
{           
            start_flag--;
            speed_now_right = ftm_quad_get(FTM2);  //right轮
            speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
            ftm_quad_clean(FTM2);
            lptmr_pulse_clean();    
            /*****  Part 1 信息采集 舵机速度PID *****/
            MessageProcessing(); //信息采集
            ADCnormal(); //采集的信息归一化  
            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
            speed_fuzzy_mem_cal_forecast(); //对输入的 fe（误差）的绝对值 和 fec（误差变化率）的绝对值 查询隶属度
            speed_fuzzy_query_forecast(); //查询模糊规则表（速度）
            speed_fuzzy_solve_forecast(); //解模糊 输出 speed_forecast 为预期的速度
            speedcontrol_forecast(); //速度预测
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();//左轮速度
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); //右轮速度
            
            if( ADC_Normal[2] < 0.4 &&  ADC_Normal[1] < 0.2)  steerctrl = Maxsteering; //发车会车程序
            else steerctrl = Minsteering + 15;
            
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////          
            ///////////////////////////左轮速度/////////////////////////////////////  
            ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,3000); //输出电机PWM  
            ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM 
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
            ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,3000); //输出电机PWM  
            ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM 
}

/*******************************************************************************
 *  @brief      stop_car(void) 
 *  @note       刹车
 *  @warning    
 ******************************************************************************/
void stop_car(void)
{
            last_stop++;//刹车计数          
            speed_now_right = ftm_quad_get(FTM2);  //right轮
	    speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
	    if(speed_now_right < 0) speed_now_right = -speed_now_right;//右轮不算负的，用于快速反转
            ftm_quad_clean(FTM2);
	    lptmr_pulse_clean();      
	    /*****  Part 1 信息采集 舵机速度PID *****/
	    MessageProcessing(); //信息采集
    	    ADCnormal(); //采集的信息归一化
	    ADCerror_diff(); //偏差法计算 误差 和 误差的变化率		
	    if(none_steerctrl == 0)
	    {
		fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
		fuzzy_query(); //查询模糊规则表
		fuzzy_solve(); //解模糊
		steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
	    }		
            speed_forecast = 0;  //预测速度为0
            speed_forecast_error = 0;  //预测速度为0      
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();//左轮速度
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); //右轮速度       
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////           
            ///////////////////////////左轮速度/////////////////////////////////////
            if(speedctrl_left < 0)  //左轮速度溢出
            {
                if(speedctrl_left < -9500) speedctrl_left_opp = 9500;
                else speedctrl_left_opp = -speedctrl_left;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,9000); //输出电机PWM  
            }
            else  //左轮速度没溢出  
            {
                if(speedctrl_left > 100) speedctrl_left = 100;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,9000); //输出电机PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
            if(speedctrl_right < 0)  //右轮速度溢出
            {
                if(speedctrl_right < -9500) speedctrl_right_opp = 9500;
                else speedctrl_right_opp = -speedctrl_right;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,9000); //输出电机PWM  
            }
            else  //右轮速度没溢出
            {
                if(speedctrl_right > 100) speedctrl_right = 100;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,9000); //输出电机PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            ////////////////////////////停车///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
            {                                                                                                 
            //    flag = 1;                                                                                       
            }
            if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
            } 
            if( last_stop == 100 ) { flag = 1; } //停车周期  大于x个周期强制给0PWM，防止抖动
}  

/*******************************************************************************
 *  @brief      turn_car(void)  
 *  @note       倒车
 *  @warning    
 ******************************************************************************/
void turn_car(void)
{
            speed_now_right = ftm_quad_get(FTM2);  //right轮
	    speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
            ftm_quad_clean(FTM2);
	    lptmr_pulse_clean();	  
	    /*****  Part 1 信息采集 舵机速度PID *****/
	    MessageProcessing(); //信息采集
    	    ADCnormal(); //采集的信息归一化
/**/        fe_last = fe;  //记录上一次的值  (ADC_Normal[0] * ADC_Normal[0])
/**/        fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );
/**/        fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
/**/        fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);      //更改算法，左变成右，右变成左，用于倒车
/**/        fe = -fe;
/**/        fec = fe - fe_last;  //算出变化率
	    if(none_steerctrl == 0)
	    {
		fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
		fuzzy_query(); //查询模糊规则表
		fuzzy_solve(); //解模糊
		steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
	    }

            ///////////////倒车计数/////////////////////
            flag = 0;
            dis_right += speed_now_right;
            if( dis_right <  - dis_back ) //倒车后退的距离
            {
                beep_off();
                flag = 1;
            }
            ////////////////////////////////////
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();//左轮速度
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); //右轮速度
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////           
            ///////////////////////////左轮速度/////////////////////////////////////
            if(1)  //左轮速度溢出
            {
            //    if(speedctrl_left < -2000) speedctrl_left_opp = 2000;
            //    else speedctrl_left_opp = -speedctrl_left;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,1250); //输出电机PWM  
            }
            else  //左轮速度没溢出  
            {
                if(speedctrl_left > 1000) speedctrl_left = 1000;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,speedctrl_left); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
            if(1)  //右轮速度溢出
            {
            //    if(speedctrl_right < -8000) speedctrl_right_opp = 8000;
            //    else speedctrl_right_opp = -speedctrl_right;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1300); //输出电机PWM  
            }
            else  //右轮速度没溢出
            {
                if(speedctrl_right > 1000) speedctrl_right = 1000;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,speedctrl_right); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM 
            }
            ////////////////////////////////////////////////////////////////////////       
}    



