#include    "include.h"
#include    "AllFunction.h"

/**************************  定义变量  **************************************/
extern uint16 ADC_Value[5];
extern float ADC_Normal[5]; 
//蓝牙模块发送的数据数组
int16 OutData[10]={0};
int16 send_b=10000;//10000对应万位，以此类推
/************以下是FreeCars2.0协议变量**************/
extern uint8 uSendBuf[UartDataNum*2]={0};                //****UartDataNum是上位机设置通道数目，需保持一致
extern uint8 FreeCarsDataNum=UartDataNum*2;

extern float speed_forecast_left;
extern float speed_forecast_right;
extern int16 speedctrl_left;
extern int16 speedctrl_right;
extern int16 speed_now_left,speed_now_right;
extern float speed_fe; 
extern float fe,fec,fe_last; 
extern int16 steerctrl; 
extern float steer_P;
extern float steer_D;
extern int16 first_steerctrl;
extern uint8 flag;
extern int16 speedctrl_left;
extern float speed_Rule[5];
extern struct _MAG mag_read;
/*
extern float speed_forecast_left;
extern float speed_forecast_right;
extern int16 speedctrl_left;
extern int16 speedctrl_right;
extern int16 speed_now_left,speed_now_right;
extern float speed_fe; 
*/
extern uint16 clj;

extern uint16 round_left,round_right,cross_up,crossroad,crossroads,round_is,round_in,round_over,round_out,round_stop_flag;
extern uint8 none_steerctrl;
/*******************************************************************************
 *  @brief      CRC_CHECK函数
 *  @note       直接放入main中while（1)里执行               
      
                对发送的数据惊醒crc校验：用于虚拟示波器

 *  @warning    18/3/20  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义
 ******************************************************************************/
unsigned short CRC_CHECK(unsigned char *databuf,unsigned char CRC_CNT)
{
  unsigned short CRC_Temp;
  unsigned char k,j;
  CRC_Temp = 0xffff;
  
  for(k=0;k<CRC_CNT;k++)
  {
    CRC_Temp^=databuf[k];
    for(j=0;j<8;j++)
    {if(CRC_Temp&0x01)
      CRC_Temp=(CRC_Temp>>1)^0xa001;
     else CRC_Temp=CRC_Temp>>1;
    }
  }
  return(CRC_Temp);
}

/*
2. * 功能说明： SCI 示波器发送函数，发送一个字节
3. * 参数说明：
4. OutData[] 需要发送的数值赋予该数组
5. * 函数返回：无符号结果值
6. * 修改时间： 2013-2-10
7.  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义*/
 void OutPut_Data(void)
 {
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
 {

  temp[i] = (int)OutData[i];
  temp1[i] = (unsigned int)temp[i];

  }
  for(i=0;i<4;i++)
 {
 databuf[i*2] = (unsigned char)(temp1[i]%256);
 databuf[i*2+1] = (unsigned char)(temp1[i]/256);
 }

 CRC16 = CRC_CHECK(databuf,8);
 databuf[8] = CRC16%256;
 databuf[9] = CRC16/256;

 for(i=0;i<10;i++)
 {
 uart_putchar (UART4,(char)databuf[i]);
 }
 }

/*
2. * 功能说明： SCI 示波器调试函数
3. * 参数说明：
4. OutData[] 需要发送的数值赋予该数组
5. * 函数返回：无符号结果值
6. * 修改时间： 2013-2-10
7.  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义*/
 void OutPut_Data_test(void)
 {
  OutData[0] = (int)(mag_read.mag_x / 10);//adc_once(ADC1_SE10, ADC_12bit);
  OutData[1] = (int)(mag_read.mag_y / 10);//adc_once(ADC1_SE12, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
  OutData[2] = (int)(50);//adc_once(ADC1_SE14, ADC_12bit);
  OutData[3] = (int)(clj);//adc_once(ADC1_SE15, ADC_12bit);
  //OutPut_Data();
  //printf("ADC_FroBack_6[1]=%d\n",OutData[0]);

  OutPut_Data();
 }

/*******************************************************************************
 *  @brief      串口调试助手函数
 *  @note       直接放入main中while（1)里执行               
      
                将需要的数据变成字符串发送回来：用于sscom

 *  @warning   2018.3.20 4.1  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义
 ******************************************************************************/
void OutPut_Data_test_sscom(void)
{
  int16 databuff[20];
  unsigned char l;
  send_b=10000;//给这个数赋值
  OutData[0] = (int)(ADC_Normal[0]*100);//adc_once(ADC1_SE10, ADC_12bit);
  OutData[1] = (int)(100*ADC_Normal[1]);//adc_once(ADC1_SE12, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
  OutData[2] = (int)(100*ADC_Normal[2]);//adc_once(ADC1_SE14, ADC_12bit);
  OutData[3] = (int)(100*ADC_Normal[3]);//adc_once(ADC1_SE15, ADC_12bit);
  //printf("ADC_FroBack_6[1]=%d\n",OutData[0]);
  for(l=0;l<5;l++)
  {  

   databuff[l*4] = (OutData[0]/send_b);//首先存入高位，依次存入
   databuff[l*4+1] = (OutData[1]/send_b);
   databuff[l*4+2] = (OutData[2]/send_b);
   databuff[l*4+3] = (OutData[3]/send_b);
   OutData[0]=OutData[0]%send_b;//将最高位去掉
   OutData[1]=OutData[1]%send_b;
   OutData[2]=OutData[2]%send_b;
   OutData[3]=OutData[3]%send_b;
   send_b=send_b/10;//除以10，以便下次得到低位
  }
  send_b=10000;//重新给这个数赋值
  
  for(l=0;l<20;l++) //将数据变为对应ASC码
   databuff[l] = databuff[l] + 48;
  
  
 /* databuff[1]=50;
  databuff[2]=50;
  databuff[3]=50;
 */ 
  uart_putchar (UART4,'a');//在串口助手上显示第数据的识别符，例如：发送12345，串口上显示a12345
   for(l=0;l<5;l++)//发送OutData[]第一个数据//此段位发送储存的数据
   {
     uart_putchar (UART4,(char)databuff[l*4]);
    }
   uart_putchar (UART4,'b');
   for(l=0;l<5;l++)//发送OutData[]第二个数据
   {
     uart_putchar (UART4,(char)databuff[l*4+1]);
    }
   uart_putchar (UART4,'c');
   for(l=0;l<5;l++)//发送OutData[]第三个数据
   {
     uart_putchar (UART4,(char)databuff[l*4+2]);
    }
   uart_putchar (UART4,'d');
   for(l=0;l<5;l++)//发送OutData[]第四个数据
   {
     uart_putchar (UART4,(char)databuff[l*4+3]);
    }
}

/*******************************************************************************
 *  @brief     push()
 *  @note       向某个通道缓冲区填充数据               
      
                chanel：通道     对应上位机显示通道
                data  ：数据-32768~32767

 *  @warning   2018/5/08  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义
 ******************************************************************************/

void push(uint8 chanel,uint16 data)
{
    uSendBuf[chanel*2]=data/256;
    uSendBuf[chanel*2+1]=data%256;
}
/*******************************************************************************
* @brief              sendDataToScope    
* @note            轮询法发送一帧数据, 消耗时间与数据长度有关
                   消耗时间计算看帮助文档

* @warning        不可以放在中断里面周期性调用                
******************************************************************************/
void sendDataToScope(void)
{
  uint8 i,sum=0; 
  //使用轮询的方式发送数据，当数据未发送，程序停在此处直到发送完成
  uart_putchar_toscope(UART4,251) ;//USendOneByte(FreeCarsUARTPort,251);
  uart_putchar_toscope(UART4,109) ;//USendOneByte(FreeCarsUARTPort,109);
  uart_putchar_toscope(UART4,37) ;//USendOneByte(FreeCarsUARTPort,37);

  sum+=(251);      //全部数据加入校验
  sum+=(109);
  sum+=(37);
  for(i=0;i<FreeCarsDataNum;i++)
  {
     uart_putchar_toscope(UART4,uSendBuf[i]) ;    //USendOneByte(FreeCarsUARTPort,uSendBuf[i]);
    sum+=uSendBuf[i];         //全部数据加入校验
  }
   uart_putchar_toscope(UART4,sum);// USendOneByte(FreeCarsUARTPort,sum);
}
/*******************************************************************************
 *  @brief      Freecars_scope
 *  @note       直接放入main中while（1)里执行   Freecars上位机示波器函数            
      
                将需要的数据变成字符发送到PC端

 *  @warning   2018/5/08  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义
 ******************************************************************************/
void Freecars_scope(void)
{
  int i;
   //100ms发送一次数据（所有通道）到示波器，也可以使用delayms(100)的方式
     // DELAY_MS(5);
       OutData[0] = (int)(100 * ADC_Normal[0]);//ADC_Value[0];//(int)speed_forecast_left;//ADC_Value[0];//adc_once(ADC1_SE10, ADC_12bit);
       OutData[1] = (int)(100 * ADC_Normal[1]);//ADC_Value[1];//speedctrl_left;//ADC_Value[1];//;
       OutData[2] = (int)(100 * ADC_Normal[2]);//ADC_Value[2];//speed_now_left;//ADC_Value[2];//;
       OutData[3] = (int)(100 * ADC_Normal[3]);//ADC_Value[3];
       OutData[4] = (int)(100 * ADC_Normal[4]);
        if(round_is!=0)
        {
          OutData[5] = 600;
         // round_is=!round_is;
        }
  
        if(round_in==1)
        {
          OutData[6] = 500;
          //round_right=!round_right;
        }
        if(round_out==1)
        {
          OutData[7] = 400;
        //  crossroad=!crossroad;
        }
        if(round_stop_flag==1)
        {
          OutData[8] = 300;
        }
        if( crossroad==1)
        {
          OutData[9] = 650;
        }
       for(i=1;i<UartDataNum;i++)
       {
         push(i,OutData[i-1]);
       
       }
       sendDataToScope();		     //把缓冲区里的数据发送到示波器
       push(0,10);         //获取发送一次数据的时间,放到示波器0通道缓冲区，单位为：100us（如：10表示1ms）.
   // }
      OutData[5] =0;
      OutData[6] =0;
      OutData[7] =0;
      OutData[8] =0;
      OutData[9] =0;
}