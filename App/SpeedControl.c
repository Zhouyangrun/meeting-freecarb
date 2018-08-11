/*!
 * @file       SpeedControl.c
 * @brief      速度控制函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"


/**************************  全局变量   ***************************************/
int8 m;
extern int8 pe,pec,num; 
extern float fe,fe_last,fec;
extern float eRule[5],ecRule[5];
extern float eFuzzy[2]; 
extern float ecFuzzy[2]; 
extern int16 speed_now_left,speed_now_right; 
float speed_fe = 0;  //将输入回来的fe变成绝对值，赋值进speed_fe
float speed_fec = 0; //将输入回来的fec变成绝对值，赋值进speed_fec
float speed_Fuzzy[6] = {0,0,0,0,0,/*末尾为0,用来查询模糊表步骤*/ 0};
float speed_forecast; //预测将要达到的速度（PWM）
float speed_forecast_error; //预测将要达到的速度的偏差（差速）
int8 speed_pe,speed_pec;
float speed_forecast_left = 0;  //左轮预测车速
float speed_forecast_right = 0;  //右轮预测车速
float speed_round=0;  //入环强制差速参数
/*需要调节的参数*/
float speed_power = 1,speed_pp=1;
float speed_eRule[5] = {0,4,8,12,16}; //输入误差（speed_fe）的范围                                  
float speed_ecRule[5] = {0,10,20,30,40}; //输入误差的变化率（speed_fec）的范围                                    //
float speed_Rule[5] = {38,44,47,48,49}; //输出预测速度（speed_forecast）的范围 
float speed_error_Rule[5] = {35,24,16,4,0};  //预测速度偏差的范围
int speed_rule[6][6] =   //速度规则表
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0    
      {4,3,3,2,1,5},//1   
      {4,3,2,1,0,5},//2    
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4
      {5,5,5,5,5,5} //
};
/****************  以下参数为速度闭环所用，函数未测试且有重复定义，暂时不理 ************/
int16 speedctrl_left,speedctrl_right; 
int16 speedctrl_error_left,speedctrl_error_right; 
int16 last_PWM_right,last_PWM_left;
float speed_fe_last_left = 0;
float speed_fe_last_right = 0;
float speed_fe_left = 0;
float speed_fe_right = 0;
float speed_fec_left = 0;
float speed_fec_right = 0;
float speed_P,speed_D;  // 输出的 P 值 和 D 值
float speed_Fuzzy_kp[6] = {0,0,0,0,0,/*末尾为0,用来查询模糊表步骤*/ 0};
float speed_Fuzzy_kd[6] = {0,0,0,0,0,/*末尾为0,用来查询模糊表步骤*/ 0};

/*******************  需要调节的参数  ******************************************/
float speed_eRule_err[5] = {-4,-2,0,2,4}; //输入误差的范围                                  
float speed_ecRule_err[5] = {-5,-2,0,2,5}; //输入误差的变化率的范围
float speed_Rule_kp[5] = {-4,-2,0,2,4};  //输出的P值的范围                                      
float speed_Rule_kd[5] = {0,0,0,0,0};  //输出的D值的范围
int speed_rule_kp[6][6]=  //p值规则表
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0  
      {4,3,3,2,1,5},//1   
      {4,3,2,1,0,5},//2    
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4
      {5,5,5,5,5,5} //
};
int speed_rule_kd[6][6]=  //d值规则表
{
  //ec 0 1 2 3 4  //e
      {3,1,0,0,3,5},//0    
      {2,1,1,1,2,5},//1   
      {2,1,1,1,2,5},//2    
      {3,2,2,2,3,5},//3         
      {4,3,3,3,4,5},//4
      {5,5,5,5,5,5} //
};
uint8 speed_error_power = 1;
float DDD = 0;

/*******************************************************************************
 *  @brief      speed_fuzzy_mem_cal_forecast函数
 *  @note       隶属度计算函数
                输出结果放在eFuzzy[]和ecfuzzy[]，等级为pe、pec
 *  @warning    18/3/14 代码参考binary-star队  v4.0
 ******************************************************************************/
void speed_fuzzy_mem_cal_forecast(void)//隶属度计算
{
    if(fe < 0) speed_fe = -fe;  //将输入回来的fe变成绝对值，赋值speed_fe
    else speed_fe = fe;
    if(fec < 0) speed_fec = -fec; //将输入回来的fe变成绝对值，赋值speed_fec
    else speed_fec = fec;
    
    
  /*-----误差隶属函数描述-----*/
    if(speed_fe < speed_eRule[0])     //用来确定隶属度		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      speed_pe= 0;          //?
    }
    else if(speed_fe < speed_eRule[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (speed_eRule[1]-speed_fe)/(speed_eRule[1]-speed_eRule[0]);
      speed_pe = 0;
    }
    else if(speed_fe < speed_eRule[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule[2] -speed_fe)/(speed_eRule[2]-speed_eRule[1]);
      speed_pe =1;
    }
    else if(speed_fe < speed_eRule[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule[3] -speed_fe)/(speed_eRule[3]-speed_eRule[2]);
      speed_pe =2;
    }
    else if(speed_fe < speed_eRule[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (speed_eRule[4]-speed_fe)/(speed_eRule[4]-speed_eRule[3]);
      speed_pe=3;
    }	
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      speed_pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;

/**/speed_pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
      
}
    

/*******************************************************************************
 *  @brief      speed_fuzzy_query_forecast函数
 *  @note       查询模糊规则表，算出输出的各占比
                
 *  @warning    18/3/14 代码参考binary-star队  v4.0
 ******************************************************************************/
void speed_fuzzy_query_forecast(void)//查询模糊规则表
{
    for(m = 0;m <= rank; m++) //清空数组以便累加
    {
        speed_Fuzzy[m] = 0;
    }
      /*查询kp模糊规则表*/  
    num = speed_rule[speed_pe][speed_pec];
    speed_Fuzzy[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule[speed_pe][speed_pec+1];
    speed_Fuzzy[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule[speed_pe+1][speed_pec];
    speed_Fuzzy[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule[speed_pe+1][speed_pec+1];
    speed_Fuzzy[num] += eFuzzy[1] * ecFuzzy[1];
}

/*******************************************************************************
 *  @brief      speed_fuzzy_solve_forecast函数
 *  @note       重心法解模糊
                
 *  @warning    18/3/14 代码参考binary-star队 v4.0
 ******************************************************************************/
void speed_fuzzy_solve_forecast(void)//解模糊得到pd值
{
    speed_forecast = 0; //清空P和D值以便累加
    speed_forecast_error = 0;
    /*面积中心法解模糊*/
    for(m = 0;m < rank; m++)
    {
      speed_forecast += (speed_Fuzzy[m] * speed_Rule[m]);
      speed_forecast_error += (speed_Fuzzy[m] * speed_error_Rule[m]);
    }
   // if( speed_forecast > 55) speed_forecast = 55;
   // if( speed_forecast_error > 30) speed_forecast_error = 30;
    speed_forecast = speed_power * speed_forecast*speed_pp;
    speed_forecast_error = speed_power * speed_forecast_error*speed_pp;
}

/*******************************************************************************
 *  @brief      speedcontrol_forecast函数
 *  @note       速度预测函数，算出预测要达到的速度的偏差，及其偏差变化率
                
 *  @warning    18/3/14 v4.0  未测试，待定
 ******************************************************************************/
void speedcontrol_forecast(void)
{
     if(fe < 0)
        {
            speed_forecast_right = (speed_forecast - speed_forecast_error)+speed_round;
            speed_forecast_left = speed_forecast + speed_forecast_error / 2 - speed_round;
        }
        else
        {
            speed_forecast_left = (speed_forecast - speed_forecast_error)-speed_round;
            speed_forecast_right = speed_forecast + speed_forecast_error / 2  + speed_round;
        }
    
    speed_fe_last_left = speed_fe_left;
    speed_fe_left = speed_now_left - speed_forecast_left;
    speed_fec_left = speed_fe_left - speed_fe_last_left;
    
    speed_fe_last_right = speed_fe_right;
    speed_fe_right = speed_now_right - speed_forecast_right;
    speed_fec_right = speed_fe_right - speed_fe_last_right;
    
    
}



/*******************************************************************************
 *  @brief      speed_fuzzy_mem_cal函数
 *  @note       隶属度计算函数
                输出结果放在eFuzzy[]和ecfuzzy[]，等级为pe、pec
 *  @warning    18/3/14 代码参考binary-star队  v4.0 未测试，待定
 ******************************************************************************/
void speed_fuzzy_mem_cal_left(void)//隶属度计算
{
    //////////////////////////左轮//////////////////////////////////
  /*-----误差隶属函数描述-----*/
    if(speed_fe_left < speed_eRule_err[0])     //用来确定隶属度		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      pe= 0;          //?
    }
    else if(speed_fe_left < speed_eRule_err[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (speed_eRule_err[1]-speed_fe_left) / (speed_eRule_err[1]-speed_eRule_err[0]);
      pe = 0;
    }
    else if(speed_fe_left < speed_eRule_err[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[2] -speed_fe_left) / (speed_eRule_err[2]-speed_eRule_err[1]);
      pe =1;
    }
    else if(speed_fe_left < speed_eRule_err[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[3] -speed_fe_left) / (speed_eRule_err[3]-speed_eRule_err[2]);
      pe =2;
    }
    else if(speed_fe_left < speed_eRule_err[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (speed_eRule_err[4]-speed_fe_left) / (speed_eRule_err[4]-speed_eRule_err[3]);
      pe=3;
    }
   
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;
 
/**/pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
    
   
}

/*******************************************************************************
 *  @brief      speed_fuzzy_query函数
 *  @note       查询模糊规则表，算出输出的各占比
                
 *  @warning    18/3/14 代码参考binary-star队  v4.0 未测试，待定
 ******************************************************************************/
void speed_fuzzy_query_left(void)//查询模糊规则表
{
    for(m = 0;m <= rank; m++) //清空数组以便累加
    {
        speed_Fuzzy_kp[m] = 0;
        speed_Fuzzy_kd[m] = 0;
    }
      /*查询kp模糊规则表*/  
    num = speed_rule_kp[pe][pec];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule_kp[pe][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule_kp[pe+1][pec];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule_kp[pe+1][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[1];

}

/*******************************************************************************
 *  @brief      speed_fuzzy_solve函数
 *  @note       重心法解模糊
                
 *  @warning    18/3/14 代码参考binary-star队 v4.0 未测试，待定
 ******************************************************************************/
void speed_fuzzy_solve_left(void)//解模糊得到pd值
{
    speed_P = 0; //清空P和D值以便累加
    //speed_D = 0;
    /*面积中心法解模糊*/
    for(m = 0;m < rank; m++)
    {
      speed_P += speed_Fuzzy_kp[m] * speed_Rule_kp[m];
    //  speed_D += speed_Fuzzy_kd[m] * speed_Rule_kd[m];   
    }
}

/*******************************************************************************
 *  @brief      speedcontrol函数
 *  @note       舵机转角函数
                
 *  @warning    18/3/14 v4.0 未测试，待定
 ******************************************************************************/
void speedcontrol_left(void) 
{
    last_PWM_left = speedctrl_left;
    if(speed_P > 0)  speed_P = -speed_P; //将输出的 P 值变为正数，乘上输入的误差恰好为舵机转角的增量
    //if(speed_D > 0)  speed_D = -speed_D; //将输出的 D 值变为正数
    speedctrl_error_left = (int)( speed_P * speed_fe_left + 0.35 * (speed_fe_left + speed_fe_last_left) + DDD * (speed_fe_left - speed_fe_last_left) );//舵机转角增量
    speedctrl_left = (int)(last_PWM_left + speedctrl_error_left); //舵机转角PWM
}

/*******************************************************************************
 *  @brief      speed_fuzzy_mem_cal函数
 *  @note       隶属度计算函数
                输出结果放在eFuzzy[]和ecfuzzy[]，等级为pe、pec
 *  @warning    18/3/14 代码参考binary-star队  v4.0 未测试，待定
 ******************************************************************************/
void speed_fuzzy_mem_cal_right(void)//隶属度计算
{
     //////////////////////右轮/////////////////////////////////////
  /*-----误差隶属函数描述-----*/
    if(speed_fe_right < speed_eRule_err[0])     //用来确定隶属度		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      pe= 0;          //?
    }
    else if(speed_fe_right < speed_eRule_err[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (speed_eRule_err[1] -speed_fe_right) / (speed_eRule_err[1]-speed_eRule_err[0]);
      pe = 0;
    }
    else if(speed_fe_right < speed_eRule_err[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[2] -speed_fe_right) / (speed_eRule_err[2]-speed_eRule_err[1]);
      pe =1;
    }
    else if(speed_fe_right < speed_eRule_err[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[3] -speed_fe_right) / (speed_eRule_err[3]-speed_eRule_err[2]);
      pe =2;
    }
    else if(speed_fe_right < speed_eRule_err[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (speed_eRule_err[4] -speed_fe_right) / (speed_eRule_err[4]-speed_eRule_err[3]);
      pe=3;
    }
   
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;

/**/pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
}

/*******************************************************************************
 *  @brief      speed_fuzzy_query函数
 *  @note       查询模糊规则表，算出输出的各占比
                
 *  @warning    18/3/14 代码参考binary-star队  v4.0 未测试，待定
 ******************************************************************************/
void speed_fuzzy_query_right(void)//查询模糊规则表
{
    for(m = 0;m <= rank; m++) //清空数组以便累加
    {
        speed_Fuzzy_kp[m] = 0;
        speed_Fuzzy_kd[m] = 0;
    }
      /*查询kp模糊规则表*/  
    num = speed_rule_kp[pe][pec];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule_kp[pe][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule_kp[pe+1][pec];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule_kp[pe+1][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[1];

}

/*******************************************************************************
 *  @brief      speed_fuzzy_solve函数
 *  @note       重心法解模糊
                
 *  @warning    18/3/14 代码参考binary-star队 v4.0 未测试，待定
 ******************************************************************************/
void speed_fuzzy_solve_right(void)//解模糊得到pd值
{
    speed_P = 0; //清空P和D值以便累加
    //speed_D = 0;
    /*面积中心法解模糊*/
    for(m = 0;m < rank; m++)
    {
      speed_P += speed_Fuzzy_kp[m] * speed_Rule_kp[m];
   //   speed_D += speed_Fuzzy_kd[m] * speed_Rule_kd[m];   
    }
}

/*******************************************************************************
 *  @brief      speedcontrol函数
 *  @note       舵机转角函数
                
 *  @warning    18/3/14 v4.0 未测试，待定
 ******************************************************************************/
void speedcontrol_right(void) 
{
    last_PWM_right = speedctrl_right;
    if(speed_P > 0)  speed_P = -speed_P; //将输出的 P 值变为正数，乘上输入的误差恰好为舵机转角的增量
   // if(speed_D > 0)  speed_D = -speed_D; //将输出的 D 值变为正数
    speedctrl_error_right = (int)( speed_P * speed_fe_right + 0.35 * (speed_fe_right + speed_fe_last_right) + DDD * (speed_fe_right - speed_fe_last_right));//舵机转角增量
    speedctrl_right = (int)(last_PWM_right + speedctrl_error_right); //舵机转角PWM
}
    
    
