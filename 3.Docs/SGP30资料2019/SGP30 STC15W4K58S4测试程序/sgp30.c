/***************************************************************
*   File name    :  sgp30.c  
*****************************************************************/
#include "STC15W4K32S4.h"
#include "sgp30.H" //SHTC3 head file 
#include "delay.h"
void delay_ms(uint);
/*================================================================
【  Name  】Pin Description
【Function】模拟IIC使用的引脚定义
【  Notes 】
================================================================*/
/*************Pin Description*************
   SCL -P1.0     
   SDA -P1.1
*****************************************/
sbit SCL = P1^0;
sbit SDA = P1^1;

/*================================================================
【  Name  】
【Function】应答位电平定义
【  Notes 】
================================================================*/
#define ACK         0             //应答信号
#define NACK        1             //非应答信号
/*------------------------------------------------------------*/
/*================================================================
【  Name  】void I2CDelay (uchar t)
【Function】模拟IIC用的短延时 us
【  Notes 】
================================================================*/
void I2CDelay (uchar t)          
{
        while(t--);
}

/*================================================================
【  Name  】void I2CStart(void)
【Function】I2C起始信号
【  Notes 】SCL、SDA同为高，SDA跳变成低之后，SCL跳变成低          
================================================================*/
void I2CStart(void)
{        
          SDA = 1;                            //发送起始条件的数据信号--ZLG
          SCL = 1;
          I2CDelay(50);                    //起始条件建立时间大于4.7us,延时--ZLG
          SDA = 0;                            //发送起始信号-ZLG
					I2CDelay(50);                    //起始条件锁定时间大于4μs--ZLG
          SCL = 0;                            //钳住I2C总线，准备发送或接收数据--ZLG
					I2CDelay(50);         
}

/*================================================================
【  Name  】void I2CStop(void)
【Function】I2C停止信号
【  Notes 】SCL、SDA同为低，SCL跳变成高之后，SDA跳变成高           
================================================================*/
void I2CStop(void)
{        
          SDA = 0;                        //发送结束条件的数据信号--ZLG
          SCL = 0;
					I2CDelay(50);
          SCL = 1;                        //发送结束条件的时钟信号--ZLG
          I2CDelay(50);                //结束条件建立时间大于4μs--ZLG
          SDA = 1;                        //发送I2C总线结束信号--ZLG
          I2CDelay(50); 
}

/*================================================================
【  Name  】uchar I2C_Write_Byte(uchar WRByte)
【Function】I2C写一个字节数据，返回ACK或者NACK
【  Notes 】从高到低，依次发送
================================================================*/
uchar I2C_Write_Byte(uchar Write_Byte)  //Sendbyte
{
        uchar i;   
				SCL=0;	
				I2CDelay(10); 
        for(i=0;i<8;i++)              //要传送的数据长度为8位--ZLG
        {         
							if(Write_Byte&0x80)   //判断发送位--ZLG
							{
											SDA = 1;        
							}
							else
							{
											SDA = 0;
							}
							I2CDelay(5);                
							SCL=1;                //输出SDA稳定后，拉高SCL给出上升沿，从机检测到后进行数据采样      
							I2CDelay(5);         //保证时钟高电平周期大于4μs--ZLG
							SCL=0;
							I2CDelay(5);
							Write_Byte <<= 1;
        } 
        I2CDelay(1);
        SDA = 1;                      //8位发送完后释放数据线，准备接收应答位-ZLG
				I2CDelay(40);
				SCL = 1;                      //MCU告知SHT2X数据发送完毕，等待从机的应答信号        
        I2CDelay(40);                   
        /*以下是判断I2C总线接收应到应答信号是ACK还是NACK*/
        if(SDA==1)                                   //SDA为高，收到NACK
				{
					  SCL=0;
						return NACK;   
					  I2CDelay(40); 
				}        
        else                                         //SDA为低，收到ACK 
				{
					  SCL=0;
						return ACK;  
					  I2CDelay(40); 
				}	 
}

/*================================================================
【  Name  】uchar I2C_Read_Byte(uchar AckValue)
【Function】I2C读一个字节数据，入口参数用于控制应答状态，ACK或者NACK
【  Notes 】从高到低，依次接收
================================================================*/
uchar I2C_Read_Byte(uchar AckValue)//receivebyte
{
        uchar i,RDByte=0;
        SCL=0;                                   //置时钟线为低，准备接收数据位--ZLG
				I2CDelay(40);  
        SDA = 1;                                 //释放总线,置数据线为输入方式--ZLG        
        for (i=0;i<8;i++) 
        {        
                SCL = 1;                          //SCL高电平期间，采集SDA信号，并作为有效数据 //置时钟线为高使数据线上数据有效--ZLG 
                I2CDelay(20);
                RDByte <<= 1;                  //移位
                if(SDA==1)                           //采样获取数据
                {
                        RDByte |= 0x01;
                }
                else
                {
                        RDByte &= 0xfe;
                }
                I2CDelay(10);
                SCL = 0;                             //下降沿，从机给出下一位值
                I2CDelay(60);
        }   
        /*以下是I2C总线发送应答信号ACK或者NACK*/
        SDA = AckValue;                      //应答状态        
        I2CDelay(30);
        SCL = 1;                         
        I2CDelay(50);                  //时钟低电平周期大于4μs--ZLG
        SCL = 0;                                  //清时钟线，钳住I2C总线以便继续接收--ZLG               
        I2CDelay(150);       
        return RDByte;
}


//初始化IIC接口
void SGP30_Init(void)
{
	SGP30_ad_write(0x20,0x03);
//	SGP30_ad_write(0x20,0x61);
//	SGP30_ad_write(0x01,0x00);
}

void SGP30_ad_write(uchar a, uchar b)
{
  	I2CStart();
  	I2C_Write_Byte(SGP30_write); //发送器件地址+写指令
  	I2C_Write_Byte(a);		//发送控制字节
		I2C_Write_Byte(b);
		I2CStop();
		delay_ms(100);
}

unsigned long SGP30_ad_read(void)
{
  	unsigned long dat;
		int crc;
  	I2CStart();
  	I2C_Write_Byte(SGP30_read); //发送器件地址+读指令
  	dat = I2C_Read_Byte(ACK);
		dat <<= 8;
		dat += I2C_Read_Byte(ACK);
		crc = I2C_Read_Byte(ACK); //check数据，舍去
		crc = crc;             //避免编译产生警告，这句可有可无
		dat <<= 8;
		dat += I2C_Read_Byte(ACK);
		dat <<= 8;
		dat += I2C_Read_Byte(NACK);
  	I2CStop();
  	return(dat);
}
