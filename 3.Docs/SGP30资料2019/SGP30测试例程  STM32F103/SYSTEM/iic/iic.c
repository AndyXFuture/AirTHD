#include "iic.h"
#include "delay.h"
#include "usart.h"
 
 
//初始化IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11); 	//PB10,PB11 输出高
}

//SDA PB11
//SCL PB10

void SDA_OUT(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     	//sda线输出
	SDA_DATA(1);	
	SCL_OUT(1);  	  
	delay_us(20);

	SDA_DATA(0);	//START:when CLK is high,DATA change form high to low 
	delay_us(20);
	SCL_OUT(0); 	//钳住I2C总线，准备发送或接收数据 
}	
  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();		//sda线输出
	SCL_OUT(0);  
	SDA_DATA(0);	//STOP:when CLK is high DATA change form low to high
 	delay_us(20);
 	SCL_OUT(1);  
	SDA_DATA(1);	//发送I2C总线结束信号
	delay_us(20);
}  

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      		//SDA设置为输入  
	SDA_DATA(1);
	delay_us(10);
	SCL_OUT(1);  	   
	delay_us(10);
	while(SDA_IO_IN)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			printf("no ack!\r\n");
			IIC_Stop();
			return 1;
		}
	}
	SCL_OUT(0);  		//时钟输出0 	
	return 0;  
} 

//产生ACK应答
void IIC_Ack(void)
{
	SCL_OUT(0);  	
	SDA_OUT();
	SDA_DATA(0);
	delay_us(20);
	SCL_OUT(1); 
	delay_us(20);
	SCL_OUT(0); 
} 

//不产生ACK应答		    
void IIC_NAck(void)
{
	SCL_OUT(0); 
	SDA_OUT();
	SDA_DATA(1);
	delay_us(20);
	SCL_OUT(1); 
	delay_us(20);
	SCL_OUT(0); 
}		
			 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 
		SCL_OUT(0); 	    	//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
			if((txd&0x80)>>7)
				SDA_DATA(1); 
			else
				SDA_DATA(0); 
			txd<<=1; 	  
			delay_us(20);   
			SCL_OUT(1); 
			delay_us(20); 
			SCL_OUT(0); 	
			delay_us(20);
    }	
		delay_us(20);
		
} 	    

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u16 IIC_Read_Byte(u8 ack)
{
		u8 i;
		u16 receive=0;
		SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        SCL_OUT(0);  
        delay_us(20);
        SCL_OUT(1); 
        receive<<=1;
        if(SDA_IO_IN)	
					receive++;   
				delay_us(20); 
				//printf("%d,%d ",i,receive);
    }		
		//printf("\n");
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}



