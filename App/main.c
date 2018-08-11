/*!
 * @file       main.c
 * @brief      main 
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
uint8 adc_test = 0;
uint16 clj = 0;//���ڴ����Ʊ�־
extern uint16 flag ;
extern int length;
extern uint8 chaoshengbotime;
extern uint8 flag_csb;
extern uint32 timevar;
extern byte bmp[];
extern void  OutPut_Data_test_sscom(void);
extern struct _MAG mag_read;
extern uint16 last_stop;//�յ�ͣ����� ����1Ϊͣ��
extern uint8 level;
extern uint16 dis_right,dis_left;
extern uint16 dis_back;
extern uint8 wait_flag;
extern uint16 start_flag;
extern uint16 turn_car_dis;
/*!
 *  @brief      main����
 *  @since      
 *  @note       
 */
void main()
{   
    System_Initialization(); //��ʼ��
    ISR_Initialization();  //�жϳ�ʼ��
    
    adc_test = 0; 
    while(adc_test == 0) //�����°���downʱ���Ż���1������һֱ��ִ�вɼ�������
    {
        test_max_ADC();
    }   
    
    //��ʼ��PIT1    
    pit_init_ms(PIT1, PIT1_TIMER);                                
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler); //����PIT1���жϷ�����Ϊ PIT1_IRQHandler
    enable_irq(PIT1_IRQn); // ʹ��PIT1�ж�,���ӿ�ʼ��
    set_vector_handler(UART4_RX_TX_VECTORn , uart4_test_handler);//�����жϼ���
    uart_rx_irq_en(UART4);//���������ж�ʹ��

   // �����ж����ȼ�  ԽСԽ���� 15������
    set_irq_priority(PIT0_IRQn,6);
    set_irq_priority(PIT1_IRQn,4);
    set_irq_priority(PORTC_IRQn,0);
    set_irq_priority(PORTB_IRQn,1);
    set_irq_priority(PORTA_IRQn,2);
    set_irq_priority(PORTE_IRQn,3);
    DisableInterrupts;
    DELAY_MS(10);
    EnableInterrupts; //ͬʱ�����ж�   
        
    while(1)
    {
       oled_view(); //oled��ʾ         
     //test_motor(); //�ܶ�����
     //test_steering();//�������
     //OutPut_Data_test();//ʾ��������        
     //Freecars_scope();//��ͨ��ʾ����
     //OutPut_Data_test_sscom();//�������ֵ��� 
     //test_nrf_re();//nrf����  
     //////////////////////////������/////////////////////////////////
   /*     MAG3110_Read(&mag_read);//�����ƶ�ȡ
        mag_read.mag_y = 0;
        mag_read.mag_x = 0;
        
        LED_PrintShort(45,2,mag_read.mag_x);//��ʾ�����Ʋ���
        LED_PrintShort(45,4,mag_read.mag_y); //��ʾ�����Ʋ���
       if( (mag_read.mag_y < -2000 || mag_read.mag_y > 3000 ) && (start_flag == 0) && (level != 40) && (level!= 100) && (ones == 2)  &&(level != 88)) //��������ͣ��
       {
          clj = 1000;
          level = 40;
          dis_back = turn_car_dis;
          dis_right = 0;
          last_stop = 0;
          wait_flag = 0;
          xxxxxx = mag_read.mag_x;
          yyyyyy = mag_read.mag_y;
       }
       else
       {
          clj = 0;
       } */
     ////////////////////////////////////////////////////////////////
    }
}