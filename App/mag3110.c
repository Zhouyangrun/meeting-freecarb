#include "mag3110.h"
//PB7接SDA
//PB6接SCL
#define get_x_y_offset_cnt  200 //xy采集数据数 x = -885 y = 680

/**************************  包含头文件  **************************************/
#include    "common.h"
#include "include.h"

struct _MAG mag_read;

void Get_touch_parastruct(struct _MAG *p)
{
		p->mag_x_offset =(s16) (*(volatile u32*)0x08007c00);
		p->mag_y_offset =(s16) (*(volatile u32*)0x08007c04);			
}

void I2C_Mag_GPIO_Config_moni(void)
{

    gpio_init (PTC4, GPO,1); 
    gpio_init (PTC5, GPO,1);
    
    SCL1_H;
    SDA1_H;    //总线释放
}

void I2C_delay(void)
{
    u16 i = 1000; //这里可以优化速度	，经测试最低到80还能写入
    while(i)
    {
        i--;
    }

}

void delay(u32 i)
{
    while(i)
    {
        i--;
    }
}

u8 I2C1_Start(void)
{
    SDA1_H;
    SCL1_H;
    I2C_delay();
  //  if(!SDA1_read) return 0;	//SDA线为低电平则总线忙,退出
    SDA1_L;
    I2C_delay();
  //  if(SDA1_read) return 0;	//SDA线为高电平则总线出错,退出
    SDA1_L;
    I2C_delay();
    return 1;
}

void I2C1_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i = 8;
    while(i--)
    {
        SCL1_L;
        I2C_delay();
        if(SendByte & 0x80)
            SDA1_H;
        else
            SDA1_L;
        SendByte <<= 1;
        I2C_delay();
        SCL1_H;
        I2C_delay();
    }
    SCL1_L;
		I2C_delay();
}

u8 I2C1_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
    SCL1_L;
    I2C_delay();
    SDA1_H;
    I2C_delay();
    SCL1_H;
    I2C_delay();
  /*  if(SDA1_read)
    {
        SCL1_L;
        I2C_delay();
        return 0;
    }*/
    SCL1_L;
    I2C_delay();
    return 0;
}


u8 I2C1_RadeByte(void)  //数据从高位到低位//
{
    u8 i = 8;
    u8 ReceiveByte = 0;
    SDA1_H;
	  I2C_delay();
    while(i--)
    {
        ReceiveByte <<= 1;
        SCL1_L;
        I2C_delay();
        SCL1_H;
        I2C_delay();
        gpio_ddr(PTC5,GPI);
        if(SDA1_read)
        {
            ReceiveByte |= 0x01;
            I2C_delay();
        }
        gpio_ddr(PTC5,GPO);
    }
    SCL1_L;
		I2C_delay();
    return ReceiveByte;
}

void I2C1_NoAck(void)
{
    SCL1_L;
    I2C_delay();
    SDA1_H;
    I2C_delay();
    SCL1_H;
    I2C_delay();
    SCL1_L;
    I2C_delay();
}

void I2C1_Stop(void)
{
    SCL1_L;
    I2C_delay();
    SDA1_L;
    I2C_delay();
    SCL1_H;
    I2C_delay();
    SDA1_H;
    I2C_delay();
}

//单字节写入*******************************************

void Single1_Write(u8 SlaveAddress, u8 REG_Address, u8 REG_data)		   //void
{
    I2C1_Start();
    I2C1_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址
    I2C_delay();
	  I2C1_WaitAck();
//    I2C1_Stop();
    I2C1_SendByte(REG_Address );   //设置低起始地址
    I2C1_WaitAck();
    I2C1_SendByte(REG_data);
    I2C1_WaitAck();
    I2C1_Stop();
}
//单字节读取*****************************************
u8 Single1_Read(u8 SlaveAddress, u8 REG_Address)
{
    u8 REG_data;
    I2C1_Start();
    I2C1_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址
	  I2C_delay();
    I2C1_WaitAck();
    I2C1_SendByte((u8) REG_Address);   //设置低起始地址
    I2C1_WaitAck();
    I2C1_Start();
	  I2C_delay();
    I2C1_SendByte(SlaveAddress + 1);
    I2C1_WaitAck();
    REG_data = I2C1_RadeByte();
	  I2C_delay();
    I2C1_NoAck();
    I2C1_Stop();
	  I2C_delay();
    return REG_data;
}

void MAG3110_Init(void)
{
     I2C_Mag_GPIO_Config_moni();
     Single1_Write(MAG_Addr,CTRL_REG2, 0x80); //自动转换
	   Single1_Write(MAG_Addr,0x0E, 0x00);      //offest
	   Single1_Write(MAG_Addr,0x0D, 0x1f);			//offest
//	   Single1_Write(MAG_Addr,CTRL_REG1, 0x29);//20hz
	    Single1_Write(MAG_Addr, CTRL_REG1, 0x19);//10hz
			//Get_touch_parastruct(&mag_read);
}

void GET_OFFSET(struct _MAG *p)
{
	s32 x_offset=0,y_offset=0;
	s16 offset_tmp[2][get_x_y_offset_cnt]={0},temp;	
  u16 cnt=0,i,j;
	for(cnt=0;cnt<get_x_y_offset_cnt;cnt++)//读取xy值个数
	{
		while(!(Single1_Read(MAG_Addr, 0x00)& 0x08));							
		offset_tmp[0][cnt] = (s16)((Single1_Read(MAG_Addr, OUT_X_MSB_REG) << 8) | Single1_Read(MAG_Addr, OUT_X_LSB_REG)); //x offset
		offset_tmp[1][cnt] = (s16)((Single1_Read(MAG_Addr, OUT_Y_MSB_REG) << 8) | Single1_Read(MAG_Addr, OUT_Y_LSB_REG)); //y ofsset
		DELAY_MS(15);//延时时间，适当延长转的时间肯以慢一点
	}
		
 	for(j=0;j<get_x_y_offset_cnt;j++)  //x最小值?判?
	{ 
		for (i=0;i<get_x_y_offset_cnt-j;i++) 
		{
			if (offset_tmp[0][i]>offset_tmp[0][i+1]) 
			{ 
				temp=offset_tmp[0][i]; 
				offset_tmp[0][i]=offset_tmp[0][i+1]; 
				offset_tmp[0][i+1]=temp;
			} 
		}
	}
 	for(j=0;j<get_x_y_offset_cnt;j++)  //x最小值排序
	{ 
		for (i=0;i<get_x_y_offset_cnt-j;i++) 
		{
			if (offset_tmp[1][i]>offset_tmp[1][i+1]) 
			{ 
				temp=offset_tmp[1][i]; 
				offset_tmp[1][i]=offset_tmp[1][i+1]; 
				offset_tmp[1][i+1]=temp;
			} 
		}
	}
 
	for(i=5;i<15;i++)
	{
		x_offset+=offset_tmp[0][i]+offset_tmp[0][get_x_y_offset_cnt-15-1];
		y_offset+=offset_tmp[1][i]+offset_tmp[1][get_x_y_offset_cnt-15-1];
	}
	

 	p->mag_x_offset=x_offset/2/10;
 	p->mag_y_offset=y_offset/2/10;
	
	//Keep_touch_para(p);
}
void MAG3110_Read(struct _MAG *p)
{
	 	u8 ID = Single1_Read(MAG_Addr,WHO_AM_I_REG);		  
    u8 ready = Single1_Read(MAG_Addr, 0x00);
	  u8   ID1 = Single1_Read(MAG_Addr,CTRL_REG2);
		if(ID==0xC4)
		{
			if(ready & 0x08)
			{							
// 					MAG_buf[0] = Single1_Read(MAG_Addr, 9);
// 					MAG_buf[1] = Single1_Read(MAG_Addr, 10);
// 					MAG_buf[2] = Single1_Read(MAG_Addr, 11);
// 					MAG_buf[3] = Single1_Read(MAG_Addr, 12);
// 					MAG_buf[0] = Single1_Read(MAG_Addr, 13);
// 					MAG_buf[1] = Single1_Read(MAG_Addr, 14);
//					Single1_Write(MAG_Addr, CTRL_REG1, 0x2b);
//				Single1_Write(MAG_Addr, CTRL_REG1, 0x1b);
//				  p->mag_x = (s16)((MAG_buf[0] << 8) | MAG_buf[1]); //Minus the offset determined according to the practical situation
//          p->mag_y = (s16)((MAG_buf[2] << 8) | MAG_buf[3]);
//          p->mag_z = (s16)((MAG_buf[4] << 8) | MAG_buf[5]);
					p->mag_x = (s16)((Single1_Read(MAG_Addr, OUT_X_MSB_REG) << 8) | Single1_Read(MAG_Addr, OUT_X_LSB_REG)) ; //Minus the offset determined according to the practical situation
          p->mag_y = (s16)((Single1_Read(MAG_Addr, OUT_Y_MSB_REG) << 8) | Single1_Read(MAG_Addr, OUT_Y_LSB_REG)) ;
          p->mag_z = (s16)((Single1_Read(MAG_Addr, OUT_Z_MSB_REG) << 8) | Single1_Read(MAG_Addr, OUT_Z_LSB_REG));
					
					p->mag_x=p->mag_x- p->mag_x_offset;
					p->mag_y=p->mag_y- p->mag_y_offset;
//            //992，1290，1793就是偏移量
// 				  Magnetic_X = ysdat + 1290 ;
// 					Magnetic_Y = -(xsdat - 992) ;
//          Magnetic_Z = -(zsdat + 1793) ;
			}
	  }
}
/*
u16 MAG3110_DataProcess (s16 MAG3110_XData,s16 MAG3110_YData)
{
		u16 MAG3110_Ang;
// 		MAG3110_XData -= MAG3110_XOFF;
// 		MAG3110_YData -= MAG3110_YOFF;
		if (MAG3110_XData == 0)
		{
			if (MAG3110_YData>0)
			{
				MAG3110_Ang
				= 90;
			}
			else
			{
				MAG3110_Ang
				= 270;
			}
		}
		else if (MAG3110_YData == 0)
		{
			if (MAG3110_XData>0)
			{
				MAG3110_Ang
				= 0;
			}
			else
			{
				MAG3110_Ang
				= 180;
			}
		}
		else if ((MAG3110_XData > 0) && (MAG3110_YData > 0))
		{
			MAG3110_Ang = (atan ( ( (float)MAG3110_YData) / ( (float) MAG3110_XData ) ) )
			* 180 / 3.14;
		}
		else if ((MAG3110_XData < 0) && (MAG3110_YData > 0))
		{
			MAG3110_XData = -MAG3110_XData;
			MAG3110_Ang = 180
			-
			(atan ( ( (float)MAG3110_YData) / ( (float)
			MAG3110_XData ) ) ) * 180 / 3.14;
		}
		else if ((MAG3110_XData < 0) && (MAG3110_YData < 0))
		{
			MAG3110_XData = -MAG3110_XData;
			MAG3110_YData = -MAG3110_YData;
			MAG3110_Ang = (atan ( ( (float)MAG3110_YData) / ( (float) MAG3110_XData ) ) )
			* 180 / 3.14 + 180;
		}
		else if ((MAG3110_XData > 0) && (MAG3110_YData < 0))
		{
			MAG3110_YData = -MAG3110_YData;
			MAG3110_Ang = 360
			-
			(atan ( ( (float)MAG3110_YData) / ( (float)
			MAG3110_XData ) ) ) * 180 / 3.14;
		}
		return 	 MAG3110_Ang;
}*/

