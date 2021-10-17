/**
  ******************************************************************************
  * @file    USART/USART_8xUsartsOneBoard/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_8xUsartsOneBoard
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t aRxBuffer[8][BUFFER_SIZE];
__IO uint8_t RxCounter= 0, ReceiveState = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    aRxBuffer[0][RxCounter++] = USART_ReceiveData(USART1);

    if(RxCounter == BUFFER_SIZE)
    {
      ReceiveState = 1;
      RxCounter = 0;
    }
  }
}

/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    aRxBuffer[1][RxCounter++] = USART_ReceiveData(USART2);

    if(RxCounter == BUFFER_SIZE)
    {
      ReceiveState = 1;
      RxCounter=0;
    }
  }
}

/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USART3_8_IRQHandler(void)
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    aRxBuffer[2][RxCounter++] = USART_ReceiveData(USART3);
    
    if(RxCounter == BUFFER_SIZE)
    {
      ReceiveState = 1;
      RxCounter = 0;
    }
  }
  
  if(USART_GetITStatus(USART4, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    aRxBuffer[3][RxCounter++] = USART_ReceiveData(USART4);
    
    if(RxCounter == BUFFER_SIZE)
    {
      ReceiveState = 1;
      RxCounter = 0;
    }
  }
  
  if(USART_GetITStatus(USART5, USART_IT_RXNE) != RESET)
  {
      
    /* Read one byte from the receive data register */
    aRxBuffer[4][RxCounter++] = USART_ReceiveData(USART5);
    
    if(RxCounter == BUFFER_SIZE)
    {  
      ReceiveState = 1;
      RxCounter = 0;
    }
  }
  
  if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    aRxBuffer[5][RxCounter++] = USART_ReceiveData(USART6);
    
    if(RxCounter == BUFFER_SIZE)
    {
      ReceiveState = 1;      
      RxCounter = 0;
    }
  }
  
  if(USART_GetITStatus(USART7, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    aRxBuffer[6][RxCounter++] = USART_ReceiveData(USART7);
    
    if(RxCounter == BUFFER_SIZE)
    { 
      ReceiveState = 1;
      RxCounter = 0;
    }
  }
  
  if(USART_GetITStatus(USART8, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    aRxBuffer[7][RxCounter++] = USART_ReceiveData(USART8);
    
    if(RxCounter == BUFFER_SIZE)
    { 
      ReceiveState = 1;
      RxCounter = 0;
    }
  }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
