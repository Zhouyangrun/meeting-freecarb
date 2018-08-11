/*!
 * @file       GetMessage.c
 * @brief      数据采集函数
 * @author     
 * @version    B车
 * @date       
 */
   
/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/

uint16 ADC_GetMessage[5][SamplingNum]; //采集回来的电感值，一个电感共 SamplingNum 次
uint16 ADC_Value[5] = {0,0,0,0,0}; //滤波取平均后的电感值
uint16 SUM_ADC_GetMessage[5] = {0,0,0,0,0}; 
uint16 ADC_Maxing[5] = {3300,3300,3300,3300,3300}; //电感的最大值（后期需要测）
uint16 i,j,k;
uint16 jishu=0;
uint16 change; //用于交换参数时所用的中介
uint8 cross_rank = 0,line_rank = 0;
uint8 flag = 0;    //用于停车的标志位 
                  //如果电感值都变小 则 flag 变 1，进入while（flag == 1）死循环中 
                 //按下按键即可将 flag 变 0，跳出死循环重新启动
uint8 level = 1; //level = 1 普通模式
                //level = 40 停车模式
                //level = 88 另一辆车出界，直接冲线
                //level = 4/5 圆环过渡
                //level = 100 冲线模式
uint8 huandao_flag_a=0,huandao_flag_b=0,huandao_flag_c=0,huandao_flag_d=0,huandao_flag_e=0,huandao_flag_f=0;//识别入环点
uint16 ruhuandao_jishu_a=0,ruhuandao_jishu_b=0;//入环打角计数
uint16 chuhuandao_jishu_a=0,chuhuandao_jishu_b=0;//出环打角计数
float ADC_Yuanhuan_L1=0.000,ADC_Yuanhuan_L2=0.000,ADC_Yuanhuan_L3=0.000,ADC_Yuanhuan_L4=0.000,ADC_Yuanhuan_L5=0.000;
float dreams = 0.07;
uint16 cross = 0;
uint16 cross_pass = 0;
uint8 none_steerctrl = 0; 
uint16 cross_left = 0;
uint cross_right = 0;
float ADC_Normal[5] = {0,0,0,0}; //电感归一化后的值（范围 0~1 ）
float fe = 0; //输出的误差
float fe1,fe2;
float fe_last; //上一次的误差
float fec; //误差的变化率
float error_diff_c;
float error_diff;
float error_diff_last;
extern float P_power;
extern float D_power;
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_pp,speed_round,speed_power;
extern int16 steerctrl;
extern int16 last_steerctrl;
extern int16 speed_forecast;
//环岛相关变量
uint16 round_is=0,round_in=0,round_out=0,round_over=0,round_num=0,round_stop=0,max_PWM_new=0,round_in_count=0,round_stop_flag=1;//round_vaule[3]={0},round_average[2],
//环岛调整参数
/////////////////////////////////////////////////////////////////////////////// 
uint16 round_vaule=0;// round_vaule=0       不入环
                       // round_vaule=1       环在左边
                       // round_vaule=2       环在右边
//识别阈值
float  round_up_vaule=2.32;
float round_down_vaule=2.05;
//刹车强度
uint8 round_stop_vaule=35,round_lr=2;

//////////////////////////////////////////////////////////////////////////////
//十字相关变量
uint16 cross_up=0,crossroad=0,crossroads=0;
extern uint16 max_PWM;
extern uint8 is_shizi;
extern int16 times;
extern float last_speed_power;
extern uint8 last_flag_shizi;
float round_shizi = 0.7;
/*******************************************************************************
 *  @brief      MessageProcessing函数
 *  @note       ADC信息采集处理，无归一化 
                结果为每个电感采集SamplingNum次后的平均值（去掉首尾）
 *  @warning    18/3/9 v3.1
 ******************************************************************************/
void MessageProcessing(void)
{  
    for(i = 0;i < SamplingNum; i++)//采集电感SamplingNum次
    {   
        //var_test1 = adc_once(ADC1_SE10, ADC_12bit);
        ADC_GetMessage[0][i] = adc_once(ADC1_SE10, ADC_12bit); //Green
        ADC_GetMessage[1][i] = adc_once(ADC1_SE12, ADC_12bit); //blue
        //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
        ADC_GetMessage[2][i] = adc_once(ADC1_SE13, ADC_12bit); //orange
        ADC_GetMessage[3][i] = adc_once(ADC1_SE15, ADC_12bit);  //brown
        ADC_GetMessage[4][i] = adc_once(ADC1_SE11, ADC_12bit);  //new
        
    }
    
    for(i = 0;i < (SamplingNum - 1); i++)  //冒泡法排序 从小到大
        for(j = i + 1;j < SamplingNum; j++)
            for(k = 0;k < 5; k++)  //选择电感
            {
                if( ADC_GetMessage[k][i] >= ADC_GetMessage[k][j] )//交换两个数
                {
                    change = ADC_GetMessage[k][i];
                    ADC_GetMessage[k][i] = ADC_GetMessage[k][j];
                    ADC_GetMessage[k][j] = change;
                }
                
            }
    
    for(i = Min_SamplingNum;i < SamplingNum - Min_SamplingNum; i++)//电感求和
        for(k = 0;k < 5; k++)
        {
            SUM_ADC_GetMessage[k] += ADC_GetMessage[k][i];
        }
    
    for(k = 0;k < 5; k++)//取平均
    {
        ADC_Value[k] = SUM_ADC_GetMessage[k] / (SamplingNum - 2 * Min_SamplingNum);
    }
    
    for(k = 0;k < 5; k++)//清空求和的数组，以便下一次使用
    {
        SUM_ADC_GetMessage[k] = 0;
    }
    
}

/*******************************************************************************
 *  @brief      ADCnormal函数
 *  @note       归一化函数，用每个电感除以电感的最大值
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCnormal(void)
{
    for(k = 0;k < 5; k++)
    {
        ADC_Normal[k] = (float)ADC_Value[k] / (float)ADC_Maxing[k];
    }
    if(ADC_Normal[1] <= 0.001) ADC_Normal[1] = 0.001;
    if(ADC_Normal[2] <= 0.001) ADC_Normal[2] = 0.001;
    if(ADC_Normal[3] <= 0.001) ADC_Normal[3] = 0.001;
    if(ADC_Normal[0] <= 0.001) ADC_Normal[0] = 0.001;
    if(ADC_Normal[4] <= 0.001) ADC_Normal[4] = 0.001;
    
}


/*******************************************************************************
 *  @brief      ADCerror_diff函数
 *  @note       偏差法求出误差
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCerror_diff(void)
{
      fe_last = fe;  //记录上一次的值  (ADC_Normal[0] * ADC_Normal[0])
      fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );
      fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
      fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);   //偏差算法
      if(fe > 400) fe = 400; //误差保护
      if(fe < -400) fe = -400; //误差保护
      fec = fe - fe_last;  //算出变化率
}
/*******************************************************************************
 *  @brief      Road_Id_Get函数
 *  @note       偏差法求出误差(未被使用）
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void Road_Id_Get()
{
   /*****  Part 1 丢线判断 *****/
        if( ((ADC_Normal[0] <= 0.500) || (ADC_Normal[1] <= 0.500)) && ((ADC_Normal[0] >= 0.020) || (ADC_Normal[1] >= 0.020)) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //如果右边两个电感均偏大，左边偏小，则角度打死
        {                                                                                                        //  待改
            steerctrl = Minsteering;//last_steerctrl; //舵机输出变最小，向右偏
           // beep_on(); //如果打死则蜂鸣器响一下
           // DELAY_MS(20);
           // beep_off();
        }
        
        if( ((ADC_Normal[2] <= 0.500) || (ADC_Normal[3] <= 0.500)) && ((ADC_Normal[2] >= 0.020) || (ADC_Normal[3] >= 0.020)) && (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) )//如果左边两个电感均偏大，右边偏小，则角度打死
        {                                                                                                        //  待改
             
            steerctrl = Maxsteering;//last_steerctrl; //舵机输出变最大，向左偏
          //  beep_on(); //如果打死则蜂鸣器响一下
          //  DELAY_MS(20);
          //  beep_off();
        }
                if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //如果四个电感都偏小，并且是初始状态，则将flag变成1，然后进入下面的死循环
        {                                                                                                 
          jishu++;
          steerctrl = last_steerctrl;
        }
   
         if(jishu >= 10) //几十毫秒后，还是那么小，flag=1，下面进入停车
        {
          flag = 1;
         //speed_forecast = 0;                  
        }
        
        if(( (ADC_Normal[0] >= 0.005) || (ADC_Normal[1] >= 0.005) || (ADC_Normal[2] >= 0.005) || (ADC_Normal[3] >= 0.005))&&(jishu < 10) ) //电感值恢复，则将jishu清空，继续正常跑
        {                                                                                                 
          jishu = 0;
        }        
          
        /*****  Part 2  路况识别*****/
        
///////////////////////////////////////////环岛逆向入环////////////////////////////////
       
        if((ADC_Normal[1] >= 0.750) && (ADC_Normal[3] >= 0.750)&&(ADC_Normal[2]>=0.600)&&((ADC_Normal[2]-ADC_Normal[0])>=0.3)&&(huandao_flag_a==0)&&(huandao_flag_c==0))
        {
          huandao_flag_a=1;         //环岛识别点，否则下面的条件均不成立
          beep_on();   //DELAY_MS(30); beep_off();
        }  
              if((huandao_flag_a==1)&&(ADC_Normal[2]<=0.170)&&(huandao_flag_b==0))
              {
                 huandao_flag_b=1;  //电感的低谷值识别点
                 beep_off();
              } 
              if((huandao_flag_b==1)&&(ADC_Normal[2]>=0.480))  //入岛点
              {
                steerctrl = Maxsteering;
                ruhuandao_jishu_a++;
              } 
              if((ruhuandao_jishu_a>0)&&(ruhuandao_jishu_a<=150))  //入环打角时间
              {
                steerctrl = Maxsteering;
                ruhuandao_jishu_a++;
              }
              if(ruhuandao_jishu_a>150)  //入环成功
              {
                ruhuandao_jishu_a = 0;
                huandao_flag_a=0;
                huandao_flag_b=0;
                huandao_flag_c=1; 
              }
              if((huandao_flag_c==1)&&(ADC_Normal[0]>=0.900))  //出环识别点
              {
                steerctrl = Maxsteering;
                speed_forecast = 2500;    //出环减速
                
                beep_on();
                chuhuandao_jishu_a++;
              }
              if((chuhuandao_jishu_a>0)&&(chuhuandao_jishu_a<=100)) //出环打角时间
              {
                steerctrl = Maxsteering;
                chuhuandao_jishu_a++;
              }
              if((chuhuandao_jishu_a>100)&&(chuhuandao_jishu_a<=(100+200)))  //清出环标志位    
              {
                chuhuandao_jishu_a++;
              }
              if(chuhuandao_jishu_a>100+200)
              {
                chuhuandao_jishu_a = 0;
                huandao_flag_c = 0;  
                beep_off();
              }
                            
        
 //////////////////////////////////////////环岛顺向入环/////////////////////////////////////      
        
        if((ADC_Normal[1] >= 0.670) && (ADC_Normal[3] >= 0.670)&&(ADC_Normal[0]>=0.600)&&((ADC_Normal[0]-ADC_Normal[2])>=0.3)&&(huandao_flag_d==0))
        {
          huandao_flag_d=1;  //环岛识别点，否则下面的条件均不成立
          beep_on();DELAY();beep_off();
        }  
              if((huandao_flag_d==1)&&(ADC_Normal[0]<=0.270))
              {
                 huandao_flag_e=1;  //电感的低谷值识别点
                 beep_off();
              } 
              if((huandao_flag_e==1)&&(ADC_Normal[0]>=0.480))  //入岛点
              {
                steerctrl = Minsteering;
                ruhuandao_jishu_b++;
              } 
              if((ruhuandao_jishu_b>0)&&(ruhuandao_jishu_b<=150))  //入环打角时间
              {
                steerctrl = Minsteering;
                ruhuandao_jishu_b++;
              }
              if(ruhuandao_jishu_b>150)  //入环成功
              {
                ruhuandao_jishu_b = 0;
                huandao_flag_f=1; 
                huandao_flag_d=0;
                huandao_flag_e=0;
              }
              if((huandao_flag_f==1)&&(ADC_Normal[2]>=0.900))  //出环识别点
              {
                steerctrl = Minsteering;
                speed_forecast = 2500;    //出环减速
                beep_on();
                chuhuandao_jishu_b++;
              }
              if((chuhuandao_jishu_b>0)&&(chuhuandao_jishu_b<=100))  //出环打角时间
              {
                steerctrl = Minsteering;
                chuhuandao_jishu_b++;
              }
              if((chuhuandao_jishu_b>100)&&(chuhuandao_jishu_b<=100+200))  //清出环标志位   
              {
                chuhuandao_jishu_b++;
              }
              if(chuhuandao_jishu_b>100+200)
              {
                chuhuandao_jishu_b = 0;
                huandao_flag_f = 0; 
                beep_off();
              } 
          
     
        
        
        /*****  Part 3 停车判断  *****/
       /* if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
        {                                                                                                 
          flag = 1;                                                                                       
        }*/     
        if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
        {
         speed_forecast = 0;
         
         //steerctrl = 768;
         
           // ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //电机输出 0
            //ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //电机输出 0
       //     LED_PrintShort(50,7,speed_forecast); //显示电机PWM
        //  LED_PrintValueF(50,5,speed_fec_max,2); //显示最大误差变化率的绝对值
        //     LED_PrintValueF(50,3,speed_min,2); //显示最小速度
        //     DELAY_MS(100);
             
        }
}
/*******************************************************************************
 *  @brief      road_check函数
 *  @note       路况
                
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void road_check(void)
{
    D_power = 1;
    if( (cross_pass > 1) || ( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ((ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5)) ) )
    {
        if( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ( (ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5) ))   //判断环岛
        {
            speed_power = 0.1;
            P_power = 2;
            cross = cross + 2;
            if( ADC_Normal[3] > ADC_Normal[0] )  
                cross_left++;
            if( ADC_Normal[0] > ADC_Normal[3] ) 
                cross_right++;
            if( (cross > 250) && (cross_pass < 50) )  
                cross_pass = cross_pass + 100;          
        }
        if( (cross_pass > 1) )
        {
            speed_power = 0.1;
            level = 31;
            if(cross > 0) cross--;
            if(cross_pass > 0) cross_pass--;
            if( cross_left > cross_right )  
            {
                fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe
                fe2 =  sqrt( ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
               // none_steerctrl = 1; steerctrl =  Maxsteering;
            }//左转
            else                           
            {
                //none_steerctrl = 1; steerctrl = Minsteering; 
            }//右转
        }   
    } 
    else 
    {       
            if(cross > 0) cross--;
            if(cross_left > 0) cross_left--;
            if(cross_right > 0) cross_right--;
            none_steerctrl = 0;
            if(ADC_Normal[0] < 0.05 && ADC_Normal[3] < 0.05 )   // 直道 
            {
                if( P_power > 0.5 ) P_power = P_power - 0.8 * dreams;   //缓和减少P值 
                fe1 =  sqrt( 2 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe
                fe2 =  sqrt( 2 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) )  * 100);
                D_power = 0.4;
                speed_power = 1.2;
                level = 1;
            }
            else
            {
               if( ( ADC_Normal[0] < 0.4 || ADC_Normal[3] < 0.4 ) ) //在弯道范围内
               {
                  if( (ADC_Normal[0] > ADC_Normal[3] && ADC_Normal[2] > ADC_Normal[1] && ADC_Normal[0] > 0.05) || ( ADC_Normal[3] > ADC_Normal[0] && ADC_Normal[1] > ADC_Normal[2] && ADC_Normal[3] > 0.05 ) )  //判断是否内切
                  {
                        fe = (int)( (sqrt( ADC_Normal[2] * ADC_Normal[2] ) - sqrt( ADC_Normal[1] * ADC_Normal[1] ) ) * 100 ); // 系数待定
                        level = 11;
                        if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                        speed_power = 0.8;
                  }
                  else
                  {      
                        if( ADC_Normal[0] < 0.05 || ADC_Normal[3] < 0.05 )  //判断是否有一个直的偏小
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2 )  //一个直的偏小，一个直的偏中
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 2; 
                            }             
                            else     //一个直的偏小，一个直的偏大 
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe *0.5会使误差变大
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                        }
                        else 
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2)   //电感都大于小 且有一个偏中
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] +  ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe  *0.5会使误差变大
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] +  ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                            
                            else     //两个电感偏大
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 2;
                            }
                        }
                  }
               }
                
            }
            if(ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.4 )   //判断十字
            {
                if( P_power < 1.5 ) P_power = P_power + dreams;   //缓和增加P值
                D_power = 0.6;
                speed_power = 0.5;
                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] );  //重新计算fe
                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                level = 21;
            }

    }
    
    if(fe > 100) fe = 100; //误差保护
    if(fe < -100) fe = -100; //误差保护
}


/*******************************************************************************
 *  @brief      Road_Message()函数
 *  @note       路况
                1、环位置判断 ；round_is，round_vaule，round_average，
                                round_left，round_right               
                2、十字判断  ：cross_up，cross。crossrosd
                
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void Road_Message()
{
    //速度调整
   
    if(ADC_Normal[4]<=1.6)
    {
        speed_pp=1.0;//正常速度
    }
    else 
    {
        speed_pp=0.8;//中间电感>1.2，减速
    }
     if(round_is!=0)
    {
        speed_pp=1;//环岛速度
    }
    
    //环位置判断      中间电感值>于2.00，说明是环交点处，，，，高进低出，防止电感跃变，产生误判
    //1.6(电感最大值2700.最小值1500)
    //1.95(电感最大值3400.最小值1500)

    
    if((ADC_Normal[4]>=round_up_vaule)&&(round_is==0))  // round_up_vaule=2.15
    {
        round_is=1;
        if( level == 88)
          level = 4;
        else
          level = 5;
    }
    else if((ADC_Normal[4]>=1.8)&&(round_is==0))///刹车入环
    {
        if(round_stop_flag==1)
        {    
            round_stop=round_stop_vaule;//刹车强度///round_stop_vaule
            round_stop_flag=! round_stop_flag;
        }
        if(max_PWM_new<max_PWM)//
            max_PWM_new=max_PWM;
        max_PWM=2500;       //改变max_PWM，防止刹车后瞬间加速度过大
    }
    
    if(round_is==1)//在环交点处
    {    
         if(ADC_Normal[4]<=round_down_vaule)   //打角标志
        {
            round_is=2;
        }    
    }
    //十字判断     两个直的>0.8+两个直的<0.1   -->过了一次十字，crossroad=1;    总数+1，crossroads+=1；
    else if((ADC_Normal[0] >= 0.800)&&(ADC_Normal[3] >= 0.800)&&(round_is==0)&&(crossroad==0))
    {
        crossroad=1;
    }
    
    if((crossroad==1)&&(ADC_Normal[0] <= 0.750)&&(ADC_Normal[3] <= 0.750))
    {
        crossroad=0;
        crossroads+=1;
      
    }
    Round_about();
}

/*******************************************************************************
 *  @brief      Road_about()函数
 *  @note       环岛进出
                1、入环岛点判断 ； round_is,  round_left，round_right
                                 环内标志   round_in   

                2、出环岛点判断  ：round_out,   round_left，round_right
                                    结束打角标志   round_over
                                  round_num
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void Round_about()
{
  if((round_vaule==3)&&(round_lr==2))
  {
    round_lr=0;
  }
  else if((round_vaule==4)&&(round_lr==2))
  {
    round_lr=1;
  }
  
  //入环判断       round_is==2
  if(round_is==2)
  {
    //环在右侧 
    if((round_vaule==2)||((round_lr==1)&&((round_vaule==3)||(round_vaule==4))))
    {
      none_steerctrl=1;//关闭模糊pid
      steerctrl=601;   //大死角
      // speed_round= -13;//      强制差数
      
      
      //  if((fe>20)||(fe< -20))//////取决于偏差变化范围
      round_in_count+=1;
      if(speed_power<0.5)
      {
        
        if(round_in_count==120)
        {
          none_steerctrl=0; //开启模糊pid
          
          round_in=1;   //
          //   speed_pp=0.2;
          round_is=3;
          //round_right=0; //
          speed_round=0; //差速清零
          round_in_count=0;
          
          
        }
      }
      else if(round_in_count==80)
      {
        none_steerctrl=0; //开启模糊pid
        
        round_in=1;   //
        //   speed_pp=0.2;
        round_is=3;
        //round_right=0; //
        speed_round=0; //差速清零
        round_in_count=0;
        
        
      }
      //round_right=0;
    }
    
    //环在左侧
    if((round_vaule==1)||((round_lr==0)&&((round_vaule==3)||(round_vaule==4))))
    {
      
      none_steerctrl=1;//关闭模糊pid
      steerctrl=742;//大死角
      //speed_round=-16;//      强制差数
     // speed_pp=0.5;
      round_in_count+=1;
       if(speed_power<0.5)
       {
         if(round_in_count==120)
         {
           none_steerctrl=0;
           round_in=1;
           
           
           // round_left=0;
           round_is=3;
           //speed_pp=0.1;
           speed_round=0;
           round_in_count=0;
           
         }
       }
      else if(round_in_count==80)
      {
        none_steerctrl=0;
        round_in=1;
        
      
        // round_left=0;
        round_is=3;
        //speed_pp=0.1;
        speed_round=0;
        round_in_count=0;
        
      }
      //round_left=0; 
    }
    // round_is=0
  }
  
  //出环操作        
  //环内标志   round_in=1   
  //结束标志   round_over=1;
  if(round_in==1)
  {
    //
    if((ADC_Normal[0] >= round_shizi)&&(ADC_Normal[3] >= round_shizi))
    {
      round_out=1;//出环标志
    }
    
    if(round_over==1)
    {
      if(ADC_Normal[4]<=1.1)//清除环标志位
      {
        round_in=0;
        round_is=0;
        round_over=0;
        round_num+=1;
        if(round_num == 2)
        {
            round_vaule = 0;
        }
        round_stop_flag=1;
        if(level == 4) //误判十字
            level = 88;
        else           //误判十字
            level = 1;
        times = 200; //误判十字
        is_shizi = 0; //误判十字
        max_PWM=max_PWM_new;//恢复pwm限制
        if((round_lr==1)||(round_lr==0))
          round_lr=!round_lr;
        
      }
    }
    
    //出环动作
    if(round_out==1)
    {
      //环在右侧 
      if((round_vaule==2)||((round_lr==1)&&((round_vaule==3)||(round_vaule==4))))
      {
        none_steerctrl=1;//关闭模糊pid
        steerctrl=602;   //大死角
        //speed_round= -13;//      强制差数
        
        
        if(ADC_Normal[4]>=1.2)//////取决于偏差变化范围
        {
          none_steerctrl=0; //开启模糊pid
          
          round_over=1;   //结束标志
          //speed_pp=0.2;
          
          //round_right=0; //
          round_out=0;
          speed_round=0; //差速清零
          
        }
        //round_right=0;
      }
      
      //环在左侧
      else if((round_vaule==1)||((round_lr==0)&&((round_vaule==3)||(round_vaule==4))))
      {
        
        none_steerctrl=1;//关闭模糊pid
        steerctrl=748;//大死角
       // speed_round=-16;//      强制差数
       // speed_pp=0.1;
        
        if(ADC_Normal[4]>=1.2)
        {
          none_steerctrl=0;
          round_over=1;//结束标志
          
          //speed_pp=0.2;
         // round_left=0;
          round_out=0;
          speed_round=0;
        }
      }
    }
  }
  if( level == 1 )
  {
      if( round_num == 1 )
          if( last_flag_shizi == 11)
              speed_power = last_speed_power;
      if( round_num == 2 )
          if( last_flag_shizi == 12)
              speed_power = last_speed_power;
  }
}
