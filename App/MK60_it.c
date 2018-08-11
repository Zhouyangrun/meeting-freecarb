/*!
 * @file       MK60_it.c
 * @brief      �жϷ�����
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
int16 times = 0; //��ʱͣ����־λ PIT0��ʱ��
uint8 car_dis_flag = 0; //�ߵ�ƽ��ʼ���λ
uint16 car_dis = 0;  //������������ ��λcm
uint8 car_dis_ms = 0; //��������ߵ�ƽ��ʱ�� ��λms
uint8 car_dis_times = 0; //�����ڳ�������
uint8 now_vol = 0;//���ڵĵ�ƽ
uint8 last_vol = 0;//�ϴεĵ�ƽ
uint16 delay_flag = 0; //����������ʱʹ��
uint8 wait_flag = 0;//�յ�ȴ���־λ
uint8 shizi = 0; //����ʮ�ֵĸ���
uint8 is_shizi = 0;//����ʮ���жϣ������ų�Բ��
uint8 avoid_flag_shizi = 0;//�ڼ���ʮ����һ�����ܿ������Ϊ0����ִ�еڶ��ֻᳵ��ʽ
uint8 go_flag_shizi = 3; //�ڼ���ʮ����һ���������ж�
uint8 last_flag_shizi = 6;//�ڼ���ʮ�ּ���
uint16 gameover = 0;//levelΪ100ʱʹ�ã��ᳵ������һ��ʱ��flag�޷���1�����ڷ�ֹ������
uint8 round_times = 0;
char bluetooth_data=0;//���յ������ݴ����������
float last_speed_power = 0.3;//���һ��ʮ�ּ��ٵķ���
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
 *  @brief      PIT0�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
   if(times > 0)  //����ʮ���Ƿ�Ϊʮ��
    {
       times--;
       if(times == 25 && is_shizi == 1) //��15�����ں�����жϣ���ǰ15������û�м�⵽����־����Ϊʮ��
       {
            shizi++;//ʮ��������1
            beep_on(); 
            if( shizi == avoid_flag_shizi) //��n��ʮ�ַ��ź�
            {    
                uart_putchar (UART4,'3');
                uart_putchar (UART4,'3');
                uart_putchar (UART4,'3');
                uart_putchar (UART4,'3');
                speed_power = 0.5;
                  
            }
            if( shizi == go_flag_shizi )  //��n��ʮ�ַ��ź�
            {    
                uart_putchar (UART4,'4');
                uart_putchar (UART4,'4');
                uart_putchar (UART4,'4');
                uart_putchar (UART4,'4');
                if( avoid_flag_shizi == 0)//�²��ԣ��ڶ��ֻᳵ��ʽ
                {
                    speed_power = 0.1;
                }
                else
                {
                    speed_power = 1;
                }
            }   
            if(shizi == go_flag_shizi + 1)//�²��ԣ��ڶ��ֻᳵ��ʽ
            {
                if( avoid_flag_shizi == 0)//�²���
                {
                    speed_power = 1;
                    times = 100;//��ȥ����ʮ��
                    is_shizi = 0;
                }
            }
            if( shizi == last_flag_shizi)//�ж��Ƿ�Ϊ���һ��ʮ��
            {
                speed_power = last_speed_power; //���һ��ʮ�ּ���
            }
        }
        if( ADC_Normal[4] > 2) //��⵽����־��is_shizi��0����ʾ����ʮ��
        {
            is_shizi = 0;
        }
        if(times < 1)
        {
            beep_off(); 
        }
    }
    else //����Ƿ���ʮ��
    {
        if(level == 1) // ����ģʽ
        {
            if(ADC_Normal[0] > 0.5 && ADC_Normal[3] > 0.5)//���ֱ��д���һ��ֵ�����ж�����ʮ��
            {
                times = 40;//����200ms���ж�ʱ�䣬һ��times��5ms      
                is_shizi = 1;//����ʮ��
            }
        }
    }
    
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT1�жϷ�����
 *  @note       
 ******************************************************************************/
void PIT1_IRQHandler(void)
{
    if( start_flag > 0 ) //�������������ȼ���ߣ�һ��start_flagΪ5ms
    {
        start_car();
        delay_flag = 120; //����֮�����ڷ�ֹ�������磬��ֹ��Ϊ���ƫС��flag��1
    }
     ///////////////////////////////////////////////////////////////////////////
    else if( level == 40 )
    {
        if(last_stop <= 130)  stop_car(); //ɲ������
        else  //ɲ����������е���
        {
            if(wait_flag == 0) 
            {
                turn_car(); //��������
                if( speed_power > 0.5 && wait_flag == 0 ) // �������ֱ�ӷ��ź� ȷ���Ƿ���Իᳵ
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    speed_power = 0.1;
                }
                if( flag == 1 )  // ������� ���ź�
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    wait_flag = 1;
                }
            }
            else //��һ��������������ֱ�ӳ���
            {
                test_motor();
                flag = 1;
            }
        } 
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 100 ) //�յ������ͣ������ֹ����
    {
        gameover++;
        speed_power = 0.1;
        test_motor();
        if(gameover > 200)  flag = 1;
    }
     ///////////////////////////////////////////////////////////////////////////
    else 
    {
       test_motor();//������ʻ����
    } 
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT2�жϷ�����
 *  @note       ��������ࣨע�͵Ĵ��룩�����������������־
 *  @warning
 ******************************************************************************/
void PIT2_IRQHandler(void)
{/*
    ///////////////////////////���������//////////////////////////////////////
    last_vol = now_vol;
    if( gpio_get(PTC4) == 1 )
        now_vol = 1;
    else 
        now_vol = 0;
    if(last_vol == 0 && now_vol == 1)  //��������
    {
        car_dis_flag = 1;
        car_dis_ms = 0;
    }
      
    if(car_dis_flag == 1)  //���������Ա��λ
    {
        if( car_dis_ms > 150)  //���� 15ms ��Ϊ��Ч����
            car_dis_flag = 0;
        else
        {
            if( gpio_get(PTC4) == 1 ) //�ߵ�ƽʹ��������5 ���ߵ�ƽʱ������0.5ms
                car_dis_ms += 10;
            else
            {   */
              /*  car_dis_value[car_dis_times] = 34 * car_dis_ms;
                if(car_dis_times == 4) car_dis_times = 0;
                car_dis_times++;
              //  car_dis = 34 * car_dis_ms; //�������������  mm
                car_dis = car_dis_value[0] + car_dis_value[1] + car_dis_value[2] + car_dis_value[3] + car_dis_value[4];
                car_dis_flag = 0; //���ñ��λ  */
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
  //////////////////////////////�������־////////////////////////////////////
    if( round_times != 0)
    {
        round_times--;
        if( round_times == 0)
        {
            round_in=0;
            round_is=0;
            round_over=0;
            if(level == 4) //����ʮ��
                level = 88;
            else if(level == 5) //����ʮ��
                level = 1;
            round_stop_flag=1;
            max_PWM=max_PWM_new;//�ָ�pwm����
            if((round_lr==1)||(round_lr==0))
              round_lr=!round_lr;
        }
    }  
    else if(round_is != 0 && round_vaule != 0)
    {
        round_times = 8;
    }
    PIT_Flag_Clear(PIT2);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT3�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT3);       //���жϱ�־λ
}

/*!
 *  @brief      UART3�����жϷ�����
 *  @since      v5.0
 *  @warning    ����ͨѶ������ʹ�ñ��뿪�жϲ���ʹ�� ,������Ҫ���͵ĵط���uart_putchar���������uart_putchar(UART4,'1');
 *  Sample usage:  set_vector_handler(UART3_RX_TX_VECTORn , uart3_test_handler);    //�� uart3_handler ������ӵ��ж�����������Ҫ�����ֶ�����
 */
void uart4_test_handler(void)
{
 
    if(uart_query(UART4) == 1)   //�������ݼĴ�����
    {
      uart_getchar(UART4, &bluetooth_data);//�ȴ�������//*bluetooth_data = UART_D_REG(UARTN[uratn]);///////////bluetooth_data��һ��char�ͱ�������ϲ����ɶ�͸�ɶ
      if(bluetooth_data ==  '1') 
      {
        if( wait_flag == 1 )//�����Ѿ�����׼���ᳵ
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
        // bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
      if(bluetooth_data ==  '2') //�����Ѿ�����׼���ᳵ
      {
         flag = 0;
         wait_flag = 0;
         start_flag = last_start_flag;
         level = 100;
         //bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
      if(bluetooth_data ==  '8') //һ�������磬�յ��źź󱾳�ֱ����
      {
         level = 88;
         flag = 0;
         speed_power = 1;
         //bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
    }
}



