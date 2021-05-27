/*
	SGP30 ²âÊÔ³ÌÐò
	MCU£ºSTM32F051C8T6
	´®¿Ú£ºUSART1 ²¨ÌØÂÊ115200
	SGO30 I2C½Ó¿Ú: SDA=PF7,SCL=PF6
	×¢ÒâSGP30¹¤×÷µçÑ¹ÊÇ1.8V ÓÐLDOµÄÄ£¿éÍâ½Ó3.3V£¨µ¥Æ¬»ú±¾ÉíÊÇ3.3V£©£¬Èç¹ûÊÇ5Vµ¥Æ¬»ú¿ÉÒÔÍâ½Ó5V
*/
#include <stm32f0xx.h>
#include "sgp30.h"
#include <stdio.h>
#include "usart.h"
static volatile uint32_t TimingDelay; 
void Delay(uint32_t nTime);
int fputc(int ch,FILE *f);//USART1 printf()ÖØ¶¨Ïò
int fgetc(FILE *f);//USART1 scanf()ÖØ¶¨Ïò

int fputc(int ch,FILE *f)//ÖØ¶¨ÏòC¿âº¯Êý    printf    µ½USART1
{
	//USART1->SR;//½â¾öµÚÒ»¸ö×Ö·û·¢ËÍÊ§°ÜµÄÎÊÌâ
	USART_SendData(USART1,(uint8_t)ch);//·¢ËÍÒ»¸ö×Ö½ÚÊý¾Ýµ½USART1
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//µÈ´ý·¢ËÍÍê±Ï,·¢ËÍÍê³É±êÖ¾Î»£¬RESET±íÃ÷»¹Ã»ÓÐ·¢ËÍÍê³É
	return ch;
}

//ÖØ¶¨ÏòC¿âº¯Êý    scanf    µ½USART1
int fgetc(FILE *f)//USART1½ÓÊÕº¯Êý¯Êý
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==RESET);//µÈ´ý´®¿Ú1ÊäÈëÊý¾Ý,½ÓÊÕÊý¾Ý¼Ä´æÆ÷·Ç¿Õ±êÖ¾Î»,RESET±íÃ÷»¹Ã»ÓÐ½ÓÊÕÍê³É
	return (int)USART_ReceiveData(USART1);
}


int main(void)
{
    uint16_t i = 0;
    int16_t err;
    uint16_t tvoc_ppb, co2_eq_ppm;
    uint32_t iaq_baseline;
    uint16_t ethanol_signal, h2_signal;

	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure the LED_pin as output push-pull for LD3 & LD4 usage*/
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13;						  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	  RCC_ClocksTypeDef RCC_Clocks;
		/* Configure SysTick IRQ and SysTick Timer to generate interrupts every 500*/
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
		
		USART_Configuration();
		NVIC_USART_Config();
	  printf("SGP sensor probing start...\n");
	  Delay(1000);
		
    /* Busy loop for initialization. The main loop does not work without a sensor. */
    while (sgp_probe() != STATUS_OK) {
         printf("SGP sensor probing failed...\r\n"); 
			   Delay(0x400000); 
    }
    printf("SGP sensor probing successful!\r\n"); 


    /* Read gas signals */
    err = sgp_measure_signals_blocking_read(&ethanol_signal,&h2_signal);
    if (err == STATUS_OK) {
         //Print ethanol signal and h2 signal 
         printf("Ethanol signal: %u\t", ethanol_signal); 
         printf("H2 signal: %u\r\n", h2_signal); 
    } else {
         printf("error reading signals\r\n"); 
    }


    /* Consider the two cases (A) and (B):
     * (A) If no baseline is available or the most recent baseline is more than
     *     one week old, it must discarded. A new baseline is found with
     *     sgp_iaq_init() */
    err = sgp_iaq_init();
		
		
    /* (B) If a recent baseline is available, set it after sgp_iaq_init() for
     * faster start-up */
    /* IMPLEMENT: retrieve iaq_baseline from presistent storage;
     * err = sgp_set_iaq_baseline(iaq_baseline);
     */

    /* Run periodic IAQ measurements at defined intervals */
    while (1) {
        err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
        if (err == STATUS_OK) {
             printf("tVOC  : %dppb\t", tvoc_ppb);
             printf("CO2eq : %dppm\r\n", co2_eq_ppm);
             
        } else {
             printf("error reading IAQ values\r\n"); 
        }

        /*
        * IMPLEMENT: get absolute humidity to enable humidity compensation
        * u32 ah = get_absolute_humidity(); // absolute humidity in mg/m^3
        * sgp_set_absolute_humidity(ah);
        */

        /* Persist the current baseline every hour */
        if (++i % 3600 == 3599) {
            err = sgp_get_iaq_baseline(&iaq_baseline);
            if (err == STATUS_FAIL) {
                printf("Failed to get baseline readings\r\n");
            }
						printf("****Baseline values: eCO2: 0x%d\r\n",iaq_baseline>>16); 
					  printf(" & TVOC: 0x%d \r\n",iaq_baseline & 0x0000ffff);
        }

        /* The IAQ measurement must be triggered exactly once per second (SGP30)
         * to get accurate values.
         */
         Delay(0x500000);  // SGP30 */
    }
}

void  Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{

if (TimingDelay != 0x00)
{ 
TimingDelay--;
}

}


//	while (1)//LEDÉÁË¸Ö¸Ê¾µÆ
//	{	
//		printf("SGP sensor probing start1...\n");
//		GPIO_SetBits(GPIOC, GPIO_Pin_13);
//		
//		Delay(0x400000);
//		
//		printf("SGP sensor probing start2...\n");
//		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
//		Delay(0x400000); 
//	}

/****************************END OF FILE****/
