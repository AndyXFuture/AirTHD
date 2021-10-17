#include "stm32f0xx.h"
#include "usart.h"
//	#ifdef __GNUC__
//	/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//	set to 'Yes') calls __io_putchar() */
//	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//	#else
//	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//	#endif /* __GNUC__ */


//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART */
//  USART_SendData(USART1, (uint8_t) ch);

//  /* Loop until the end of transmission */
//  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
//  {}

//  return ch;
//}

void USART_Configuration(void )
{
     USART_InitTypeDef USART_InitStructure;
     
   GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Enable USART1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

 /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_1);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
  
  /* Configure USART Tx / Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
//  /* Configure USART Rx as alternate function push-pull */
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10;
//  GPIO_Init(GPIOA,&GPIO_InitStructure);

  /* USART configuration */

  USART_Init(USART1,&USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE,ENABLE);
  
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
	

}
void NVIC_USART_Config()
{
	NVIC_InitTypeDef NVIC_InitStructure;
 /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	

}

