/***************************************************************
*   File name    :  SGP30.h  
*****************************************************************/

/*****************Function Declaration*************************/
/*----define to easier to use-----*/
        #define uchar unsigned char         
        #define uint  unsigned int
        #define ulong unsigned long
					
				#define SGP30_read  0xb1           
				#define SGP30_write 0xb0

/*================================================================
【  Name  】void Delay(uint t)
【Function】delay
【  Notes 】
================================================================*/
void Delay(uint t);

/*================================================================
【  Name  】void I2CDelay (uchar t)
【Function】模拟IIC用的短延时
【  Notes 】
================================================================*/
void I2CDelay (uchar t);

/*================================================================
【  Name  】void I2CStart(void)
【Function】I2C起始信号
【  Notes 】SCL、SDA同为高，SDA跳变成低之后，SCL跳变成低
================================================================*/
void I2CStart(void);

/*================================================================
【名 称】void I2CStop(void)
【功 能】I2C停止信号
【备 注】SCL、SDA同为低，SCL跳变成高之后，SDA跳变成高
================================================================*/
void I2CStop(void);

/*================================================================
【  Name  】uchar I2C_Write_Byte(uchar WRByte)
【Function】I2C写一个字节数据，返回ACK或者NACK
【  Notes 】从高到低，依次发送
================================================================*/
uchar I2C_Write_Byte(uchar Write_Byte);

/*================================================================
【  Name  】uchar I2C_Read_Byte(uchar AckValue)
【Function】I2C读一个字节数据，入口参数用于控制应答状态，ACK或者NACK
【  Notes 】从高到低，依次接收
================================================================*/
uchar I2C_Read_Byte(uchar AckValue);

void ReadShtc3(void);

void SGP30_Init(void);

void SGP30_ad_write(uchar, uchar);

unsigned long SGP30_ad_read(void);











