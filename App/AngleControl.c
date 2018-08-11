/*!
 * @file       AngleControl.c
 * @brief      �Ƕȿ��ƺ���
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
int8 l;
int8 pe,pec,num; 
int16 steerctrl; // ����Ķ��ת��PWM
int16 last_steerctrl; // �ϴ�����Ķ��ת��PWM
int16 steerctrl_error; //���ת�ǵ������������ɸ��� ���϶���м�ֵ��Ϊ�����Ҫת�ǵ�PWM
float steer_P;
float eFuzzy[2] = {0,0}; 
float ecFuzzy[2] = {0,0}; 
float Fuzzy_kp[6] = {0,0,0,0,0,/*ĩβΪ0,������ѯģ��������*/ 0};
float Fuzzy_kd[6] = {0,0,0,0,0,/*ĩβΪ0,������ѯģ��������*/ 0};
/*��Ҫ���ڵĲ���*/
float P_power = 1;
float D_power = 1;
float eRule[5] = {-25,-16,0,16,25}; //������fe���ķ�Χ���ɸ����� 
                                            //���һ��ƫ�����������Ϊ-0.9 �� 0.9������100��Ϊ-90��90���ٷֳ�7��
float ecRule[5] = {-25,-15,0,15,25}; //�������ı仯�ʣ�fec���ķ�Χ���ɸ�����
float Rule_kp[5] = {-10.8,-6,0,6,10.8};  //  �����Pֵ�ķ�Χ   
float steer_D = 70;  // ����� P ֵ �� D ֵ                                                 //  
float Rule_kd[5] = {0,0,0,0,0};  //�����Dֵ�ķ�Χ
int rule_kp[6][6]=  //pֵ�����
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0          
      {4,3,3,2,1,5},//1       
      {4,3,2,1,0,5},//2     
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4           
      {5,5,5,5,5,5} //
};
int rule_kd[6][6]=  //pֵ�����
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
 *  @brief      fuzzy_mem_cal����
 *  @note       �����ȼ��㺯��
                ����������eFuzzy[]��ecfuzzy[]���ȼ�Ϊpe��pec
 *  @warning    18/3/10 ����ο�binary-star��  v3.0
 ******************************************************************************/
void fuzzy_mem_cal(void)//�����ȼ���
{
  /*-----���������������-----*/
    if(fe < eRule[0])     //����ȷ��������		        // |x_x_x_x_x_x_x_
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
    
    /*-----���仯������������-----*/
/**/pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
      
}

/*******************************************************************************
 *  @brief      fuzzy_query����
 *  @note       ��ѯģ����������������ĸ�ռ��
                
 *  @warning    18/3/10 ����ο�binary-star��  v3.0
 ******************************************************************************/
void fuzzy_query(void)//��ѯģ�������
{
    for(l = 0;l <= rank; l++) //��������Ա��ۼ�
    {
        Fuzzy_kp[l] = 0;
    }
      /*��ѯkpģ�������*/  
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
 *  @brief      fuzzy_solve����
 *  @note       ���ķ���ģ��
                
 *  @warning    18/3/10 ����ο�binary-star�� v3.0
 ******************************************************************************/
void fuzzy_solve(void)//��ģ���õ�pdֵ
{
    steer_P = 0; //���P��Dֵ�Ա��ۼ�
    //steer_D = 0;
    /*������ķ���ģ��*/
    for(l = 0;l < rank; l++)
    {
      steer_P += Fuzzy_kp[l] * Rule_kp[l];
      //steer_D += Fuzzy_kd[l] * Rule_kd[l];   
    }
}

/*******************************************************************************
 *  @brief      steercontrol����
 *  @note       ���ת�Ǻ���
                
 *  @warning    18/3/10 v3.0
 ******************************************************************************/
void steercontrol(void) 
{
    if(steer_P < 0)  steer_P = -steer_P; //������� P ֵ��Ϊ������������������ǡ��Ϊ���ת�ǵ�����
   // if(steer_D < 0)  steer_D = -steer_D; //������� D ֵ��Ϊ����
    steerctrl_error = (int)( steer_P * fe + steer_D * (fe - fe_last) );//���ת������
    last_steerctrl = steerctrl;
  //  steerctrl_error = (int)(10 * ( 0.2 * fe ));
    steerctrl = Midsteering + ( steerctrl_error / 10 ); //���ת��PWM
}
    


    