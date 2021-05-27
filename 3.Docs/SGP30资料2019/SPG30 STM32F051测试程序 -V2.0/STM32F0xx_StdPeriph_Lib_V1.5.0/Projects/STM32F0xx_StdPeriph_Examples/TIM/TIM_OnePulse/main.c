/**
  ******************************************************************************
  * @file    TIM/TIM_OnePulse/main.c  
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main program body
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
#include "main.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_OnePulse
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t PrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
static void TIM_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */ 

  /* TIM Configuration */
  TIM_Config();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Configure the TIM2 Pins.
  * @param  None
  * @retval None
  */
static void TIM_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* TIM2_CH1 pin (PA.05) and TIM2_CH2 pin (PA.01) configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_2); 

  /* --------------------------------------------------------------------------
    TIM2 configuration: One Pulse mode
    The external signal is connected to TIM2_CH2 pin (PA.01), 
    The Rising edge is used as active edge,
    The One Pulse signal is output on TIM2_CH1 pin (PA.05)
    The TIM_Pulse defines the delay value 
    The (TIM_Period -  TIM_Pulse) defines the One Pulse value.
     
    TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1)   
      TIM2CLK = PCLK1  
      PCLK1 = HCLK  
      => TIM2CLK = HCLK  = SystemCoreClock 

    TIM2CLK = SystemCoreClock, we want to get TIM2 counter clock at 24 MHz:
     Prescaler = (TIM2CLK / TIM2 counter clock) - 1
     Prescaler = (SystemCoreClock  /24 MHz) - 1
     
    The Autoreload value is 65535 (TIM2->ARR), so the maximum frequency value 
    to trigger the TIM2 input is 24000000/65535 = 366.2 Hz.

    The TIM_Pulse defines the delay value, this value is fixed to: 
       delay =  CCR1/TIM2 counter clock = 682.6 us. 
    
    The (TIM_Period - TIM_Pulse) defines the One Pulse value, this value is fixed to: 
       One Pulse value = (TIM_Period - TIM_Pulse) / TIM2 counter clock = 2.04 ms.

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    

  --------------------------------------------------------------------------- */

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock ) / 24000000) - 1;
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM2 PWM2 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 16383;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  /* TIM4 configuration in Input Capture Mode */

  TIM_ICStructInit(&TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;

  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* One Pulse Mode selection */
  TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);

  /* Input Trigger selection */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

  /* Slave Mode selection: Trigger Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Trigger);
    
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
