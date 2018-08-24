#ifndef __USART_SERVICE_H__
#define __USART_SERVICE_H__

#include "stm32l1xx.h"

/*Peripheral related definition*/
#define USART_INSTANCE               				USART1
#define USART_CLK_ENABLE()           				LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define USART_IRQn                   				USART1_IRQn
#define USART_IRQHandler             				USART1_IRQHandler

#define USART_GPIO_TX_PIN_CLK_ENABLE()      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define USART_GPIO_RX_PIN_CLK_ENABLE()			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)
#define USART_TX_PIN                 				LL_GPIO_PIN_9
#define USART_TX_GPIO_PORT           				GPIOA
#define USART_SET_TX_GPIO_AF()       				LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7)
#define USART_RX_PIN                 				LL_GPIO_PIN_10
#define USART_RX_GPIO_PORT           				GPIOA
#define USART_SET_RX_GPIO_AF()       				LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_7)

#define USART_BAUD_RATE											115200
#define MAX_DATA_LENGTH											255

typedef void (*usart_recv_callback) (void);

void USART_GPIO_Config(void);
void USART_Config(void);
uint8_t USART_SendATCmd(const uint8_t*, uint8_t);

void USART_Service_Init(void*);




#endif