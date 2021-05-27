/************************************************
 SGP30���Գ���
//SDA PB11
//SCL PB10
������ 115200 
************************************************/
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "SGP30.h"

 int main(void)
 {		
		u32 dat;
		u16 co2Data,TVOCData;
		delay_init();	    	 //��ʱ������ʼ��	  
		uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
		SGP30_Init(); 
    delay_ms(800);
	  printf("Start!\r\n");
		while(1)
		{		
			SGP30_ad_write(0x20,0x08);
			dat = SGP30_ad_read();
			co2Data = (dat & 0xffff0000) >> 16;
			TVOCData = dat & 0x0000ffff;		
			printf("TVOC:%d Ppb,co2:%d Ppm\r\n",TVOCData,co2Data);
			delay_ms(800);
		
		}	 
}
  

