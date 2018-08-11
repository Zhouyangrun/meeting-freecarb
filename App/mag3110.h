#ifndef __mag3110_H
#define __mag3110_H

typedef short int s16;
typedef long int s32;
typedef char s8;

typedef unsigned short int u16;
typedef unsigned long int u32;
typedef unsigned char u8;

#define I2C1_SLAVE_ADDRESS7    0x1C
#define MAG_Addr              0x1C
#define STATUS_REG            0x00		/*STATUS Registers*/
#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06  
#define WHO_AM_I_REG          0x07
#define STANDBY_MASK          0x00
#define ACTIVE_MASK           0x01
#define DR1_MASK              0x00
#define DATA_RATE_80HZ        DR1_MASK

#define CTRL_REG1             0x10		 /*CTRL_REG1 System Control 1 Register*/
#define CTRL_REG2             0x11	

/*模拟MAG3110的IIC接口*/   
#define SCL1_H         gpio_set(PTC4,1)
#define SCL1_L         gpio_set(PTC4,0) 
   
#define SDA1_H         gpio_set(PTC5,1)
#define SDA1_L         gpio_set(PTC5,0)

#define SCL1_read      gpio_get(PTC4)
#define SDA1_read      gpio_get(PTC5)
struct _MAG
{	
	long long int mag_x;
	int mag_x_offset;
	long long int mag_y;
	int mag_y_offset;
	int mag_z;
	int ang;
};
void MAG3110_Init(void);//引脚设置，请看此函数。
void MAG3110_Read(struct _MAG *p);
unsigned int MAG3110_DataProcess (int MAG3110_XData,int MAG3110_YData);
void GET_OFFSET(struct _MAG *p);

#endif
