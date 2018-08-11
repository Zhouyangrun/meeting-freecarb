/*!
 * @file       AngleControl.c
 * @brief      角度控制函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
int8 l;
int8 pe,pec,num; 
int16 steerctrl; // 输出的舵机转角PWM
int16 last_steerctrl; // 上次输出的舵机转角PWM
int16 steerctrl_error; //舵机转角的增量（可正可负） 加上舵机中间值即为舵机需要转角的PWM
float steer_P;
float eFuzzy[2] = {0,0}; 
float ecFuzzy[2] = {0,0}; 
float Fuzzy_kp[6] = {0,0,0,0,0,/*末尾为0,用来查询模糊表步骤*/ 0};
float Fuzzy_kd[6] = {0,0,0,0,0,/*末尾为0,用来查询模糊表步骤*/ 0};
/*需要调节的参数*/
float P_power = 1;
float D_power = 1;
float eRule[5] = {-25,-16,0,16,25}; //输入误差（fe）的范围，由负到正 
                                            //如归一化偏差法，输入的误差为-0.9 到 0.9，乘以100即为-90到90，再分成7份
float ecRule[5] = {-25,-15,0,15,25}; //输入误差的变化率（fec）的范围，由负到正
float Rule_kp[5] = {-10.8,-6,0,6,10.8};  //  输出的P值的范围   
float steer_D = 70;  // 输出的 P 值 和 D 值                                                 //  
float Rule_kd[5] = {0,0,0,0,0};  //输出的D值的范围
int rule_kp[6][6]=  //p值规则表
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0          
      {4,3,3,2,1,5},//1       
      {4,3,2,1,0,5},//2     
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4           
      {5,5,5,5,5,5} //
};
int rule_kd[6][6]=  //p值规则表
{
  //ec 0 1 2 3 4  //e
      {3,1,4,0,3,5},//0    
      {2,1,3,1,2,5},//1   
      {2,1,2,1,2,5},//2    
      {3,2,1,2,3,5},//3         
      {4,3,0,3,4,5},//4
      {5,5,5,5,5,5} //
};
extern float ADC_Normal[5];
extern float fe,fec,fe_last; 
/*******************************************************************************
 *  @brief      fuzzy_mem_cal函数
 *  @note       隶属度计算函数
                输出结果放在eFuzzy[]和ecfuzzy[]，等级为pe、pec
 *  @warning    18/3/10 代码参考binary-star队  v3.0
 ******************************************************************************/
void fuzzy_mem_cal(void)//隶属度计算
{
  /*-----误差隶属函数描述-----*/
    if(fe < eRule[0])     //用来确定隶属度		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      pe= 0;          //?
    }
    else if(fe < eRule[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (eRule[1]-fe)/(eRule[1]-eRule[0]);
      pe = 0;
    }
    else if(fe < eRule[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (eRule[2] -fe)/(eRule[2]-eRule[1]);
      pe =1;
    }
    else if(fe < eRule[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (eRule[3] -fe)/(eRule[3]-eRule[2]);
      pe =2;
    }
    else if(fe < eRule[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (eRule[4]-fe)/(eRule[4]-eRule[3]);
      pe=3;
    }	
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;
    
    /*-----误差变化隶属函数描述-----*/
/**/pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
      
}

/*******************************************************************************
 *  @brief      fuzzy_query函数
 *  @note       查询模糊规则表，算出输出的各占比
                
 *  @warning    18/3/10 代码参考binary-star队  v3.0
 ******************************************************************************/
void fuzzy_query(void)//查询模糊规则表
{
    for(l = 0;l <= rank; l++) //清空数组以便累加
    {
        Fuzzy_kp[l] = 0;
    }
      /*查询kp模糊规则表*/  
    num = rule_kp[pe][pec];
    Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[0];

    num = rule_kp[pe][pec+1];
    Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = rule_kp[pe+1][pec];
    Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = rule_kp[pe+1][pec+1];
    Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[1];
}

/*******************************************************************************
 *  @brief      fuzzy_solve函数
 *  @note       重心法解模糊
                
 *  @warning    18/3/10 代码参考binary-star队 v3.0
 ******************************************************************************/
void fuzzy_solve(void)//解模糊得到pd值
{
    steer_P = 0; //清空P和D值以便累加
    //steer_D = 0;
    /*面积中心法解模糊*/
    for(l = 0;l < rank; l++)
    {
      steer_P += Fuzzy_kp[l] * Rule_kp[l];
      //steer_D += Fuzzy_kd[l] * Rule_kd[l];   
    }
}

/*******************************************************************************
 *  @brief      steercontrol函数
 *  @note       舵机转角函数
                
 *  @warning    18/3/10 v3.0
 ******************************************************************************/
void steercontrol(void) 
{
    if(steer_P < 0)  steer_P = -steer_P; //将输出的 P 值变为正数，乘上输入的误差恰好为舵机转角的增量
   // if(steer_D < 0)  steer_D = -steer_D; //将输出的 D 值变为正数
    steerctrl_error = (int)( steer_P * fe + steer_D * (fe - fe_last) );//舵机转角增量
    last_steerctrl = steerctrl;
  //  steerctrl_error = (int)(10 * ( 0.2 * fe ));
    steerctrl = Midsteering + ( steerctrl_error / 10 ); //舵机转角PWM
}
    


    