#include "USART_service.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_bus.h"

static usart_recv_callback USART_CALLBACK;

void USART_GPIO_Init (void) {
	/*GPIO USART Cofig*/
	USART_GPIO_TX_PIN_CLK_ENABLE();
	USART_GPIO_RX_PIN_CLK_ENABLE();
	/*TX Pin*/
	LL_GPIO_SetPinMode(USART_TX_GPIO_PORT, USART_TX_PIN, LL_GPIO_MODE_ALTERNATE);
	USART_SET_TX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USART_TX_GPIO_PORT, USART_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USART_TX_GPIO_PORT, USART_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USART_TX_GPIO_PORT, USART_TX_PIN, LL_GPIO_PULL_UP);
	
	/*RX Pin*/
	LL_GPIO_SetPinMode(GPIOA, USART_RX_PIN, LL_GPIO_MODE_ALTERNATE);
	USART_SET_RX_GPIO_AF();
  LL_GPIO_SetPinSpeed(USART_RX_GPIO_PORT, USART_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USART_RX_GPIO_PORT, USART_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USART_RX_GPIO_PORT, USART_RX_PIN, LL_GPIO_PULL_UP);
}

void USART_Init (void) {
	USART_GPIO_Config();
  /* Enable USART peripheral clock and clock source ***********************/
  USART_CLK_ENABLE();

  /* Configure USART functional parameters ********************************/
  LL_USART_SetTransferDirection(USART_INSTANCE, LL_USART_DIRECTION_TX_RX);
  LL_USART_ConfigCharacter(USART_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  LL_USART_SetBaudRate(USART_INSTANCE, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200); 
  
	/* NVIC Configuration for USART interrupts */
  NVIC_SetPriority(USART_IRQn, 0);  
  NVIC_EnableIRQ(USART_IRQn);
  /* Enable USART *********************************************************/
  LL_USART_Enable(USART1);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART_INSTANCE);
  LL_USART_EnableIT_ERROR(USART_INSTANCE);
}

uint8_t USART_SendATCmd(const uint8_t* cmd, uint8_t len) {
	uint8_t i;
	for(i = 0; i < len; ++i)
	{
		while(LL_USART_IsActiveFlag_TXE(USART1) == RESET);
		LL_USART_TransmitData8(USART1, cmd[i]);
		cmd++;
	}
	return 1;
}

void USART_Service_Init(void* callback_func) {
	USART_Config();
	USART_CALLBACK = (usart_recv_callback)callback_func;
}

void USART_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_RXNE(USART_INSTANCE) == SET)
	{
		LL_USART_ClearFlag_RXNE(USART_INSTANCE);
		USART_CALLBACK();
	}
}