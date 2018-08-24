#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_gpio.h"

#include "pulse_trigg.h"

void TI_TriggerInit(void)
{
	uint32_t TIM2_clk;

  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the peripheral clock of GPIOs */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  /* Enable the peripheral clock of TIM2 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /*************************/
  /* GPIO AF configuration */
  /*************************/
  /* GPIO TIM2_CH1 configuration */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_1);

  /* GPIO TIM2_CH2 configuration */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_1, LL_GPIO_AF_1);
  
  /*******************************/
  /* Input trigger configuration */
  /*******************************/
  /* Map TI2FP2 on TI2 */
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  
  /* TI2FP2 must detect a rising edge */
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  
  /* Configure TI2FP2 as trigger */
  LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_TI2FP2);
  
  /* Enable the slave mode controller: TI2FP2 is used to start the counter */
  LL_TIM_SetSlaveMode(TIM2, LL_TIM_SLAVEMODE_TRIGGER);
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Select counter mode: counting up */
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
  
  /* Set the one pulse mode: generate only 1 pulse */
  LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
  
  /* In this example TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1), */
  /* since APB1 pre-scaler is equal to /2*2 = 1.                             */
  /*    TIM2CLK = PCLK1                                                     */
  /*    PCLK1 = HCLK                                                          */
  /*    => TIM2CLK = SystemCoreClock (32 MHz)                               */  
  TIM2_clk = SystemCoreClock/1;

  /* Set the TIM2 prescaler to get counter clock frequency at 2 kHz */ 
  LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(TIM2_clk, 2000));
  
  /* Set the capture/compare register to get a pulse delay of 2s (2000000 us)*/
  LL_TIM_OC_SetCompareCH1(TIM2, __LL_TIM_CALC_DELAY(TIM2_clk, LL_TIM_GetPrescaler(TIM2), 2000000));
  
  /* Set the autoreload register to get a pulse length of 3s (3000000 us)*/
  LL_TIM_SetAutoReload(TIM2, __LL_TIM_CALC_PULSE(TIM2_clk, LL_TIM_GetPrescaler(TIM2), 2000000, 3000000));
  
  /* Set output channel 1 in PWM2 mode */
  LL_TIM_OC_SetMode(TIM2,  LL_TIM_CHANNEL_CH1,  LL_TIM_OCMODE_PWM2);
  
  /* Configure output channel 1 configuration */
  LL_TIM_OC_ConfigOutput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  
  /**************************/
  /* Start pulse generation */
  /**************************/
  /* Enable channel 1 */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  
  /* Enable auto-reload register preload */
  LL_TIM_EnableARRPreload(TIM2);

  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM2);
}

void LED_Init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
	
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);

}