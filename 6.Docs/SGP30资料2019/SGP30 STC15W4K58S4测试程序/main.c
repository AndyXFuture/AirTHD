#include "STC15W4K32S4.h"
#include <intrins.h>
#include "sgp30.H" //SHTC3 head file 
#include "delay.h"
#include <stdio.h>
/*************Pin Description*************
   SCL -P1.0     
   SDA -P1.1   
   波特率：115200bps@22.184MHz
*****************************************/
sbit LED = P5^5; //指示灯 
void UartInit(void) //115200bps@22.184MHz
{
	SCON = 0x50;		//8???,?????
	AUXR |= 0x40;		//???1???Fosc,?1T
	AUXR &= 0xFE;		//??1?????1???????
	TMOD &= 0x0F;		//?????1?16???????
	TL1 = 0xD0;		//??????
	TH1 = 0xFF;		//??????
	ET1 = 0;		//?????1??
	TR1 = 1;		//?????1
  
	REN=1;
  EA = 1 ; //?????
  ES = 1 ; //?????
  TI = 1;		//发送中断标志位，必须设置
}
 
void main( void )
{
	unsigned long dat;
	uchar flag = 0;
	uint co2Data,TVOCData;
	UartInit() ;
	SGP30_Init(); 
  delay_ms(100);
	printf("Start!\r\n");
  while(1) 
  { 
			SGP30_ad_write(0x20,0x08);//Measure_air_quality
			dat = SGP30_ad_read();
			co2Data = (dat & 0xffff0000) >> 16;
			TVOCData = dat & 0x0000ffff;		
			if(co2Data == 400 && TVOCData == 0 && flag == 0)
			{
				printf("SGP sensor probing!\r\n");
				delay_ms(200);
			}
				else
			{
				printf("TVOC:%d Ppb , co2:%d Ppm\r\n",TVOCData,co2Data);
				delay_ms(800);
				flag = 1;
			}
   }
} 