/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l1xx_ll_exti.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l152_glass_lcd.h"

#include <String.h>

#define MAX_RING_BUFFER_SIZE			8	/*MUST BE POWER OF 2*/
#define MAX_RESP_BUFFER_SIZE			256 

typedef struct
{
	uint8_t head;
	uint8_t tail;
	uint8_t nbr_element;
	uint8_t buffer[MAX_RING_BUFFER_SIZE];
}rb_t;

enum prog_state
{
	ESP_IDLE = 0,
	ESP_RESET = 1,
	ESP_TEST = 2,
	ESP_FIND_AP = 3,
	ESP_CONN_AP = 4,
	ESP_ROUTINE = 5
};

void SystemClock_Config(void);

/* ESP Communication Relate ***************************************************************************************/
uint8_t ESP_IsRecvComp(uint8_t* buf, uint16_t* idx, rb_t* rb);
void ESP_SendATCmd(uint8_t* cmd);
void ESP_ClearBuffer(uint8_t* buf, uint16_t* idx);
//Var
uint8_t prog_state = 0;
uint8_t resp_buf[MAX_RESP_BUFFER_SIZE] = {0};
uint16_t resp_buffer_idx = 0;


/* Buffer Mangagement Relate *************************************************************************************/
void rb_init(rb_t* b);
uint8_t rb_isfull(rb_t* b);
uint8_t rb_isempty(rb_t* b);
void rb_put_char(rb_t* b, uint8_t data);
uint8_t rb_get_char(rb_t* b);
//Var
rb_t uart_buf;

/*USART Relate **************************************************************************************************/
void USART_GPIO_Config(void);
void USART_Config(void);
void USART_SendCmd(uint8_t* cmd, uint8_t len);


/************************************************   MAIN FUNCTION ***********************************************/
int main(void)
{
  SystemClock_Config();
	USART_Config();
	rb_init(&uart_buf);

	prog_state = ESP_RESET;
	
	while (1)
	{
		switch(prog_state)
		{
			case ESP_IDLE: 
			break;
			
			case ESP_RESET:
				ESP_SendATCmd((uint8_t*)"AT+RST\r\n");
				while(ESP_IsRecvComp(resp_buf, &resp_buffer_idx, &uart_buf));
				
			if(strstr((const char*)resp_buf, "OK") != NULL)
			{
					prog_state = ESP_TEST;
				  ESP_ClearBuffer(resp_buf, &resp_buffer_idx);
			}
			break;
			
			case ESP_TEST:
				ESP_SendATCmd((uint8_t*)"AT\r\n");
				while(ESP_IsRecvComp(resp_buf, &resp_buffer_idx, &uart_buf));
				
			if(strstr((const char*)resp_buf, "OK") != NULL)
			{
					prog_state = ESP_FIND_AP;
				  ESP_ClearBuffer(resp_buf, &resp_buffer_idx);
			}
			break;
			
			case ESP_FIND_AP:
				/*Set ESP MODE 3*/
			  do
				{
					ESP_SendATCmd((uint8_t*)"AT+CWMODE:3\r\n");
					while(ESP_IsRecvComp(resp_buf, &resp_buffer_idx, &uart_buf));
				}while(strstr((const char*)resp_buf, "OK") == NULL);
				
				/* Find your ACCESS POINT*/
				ESP_SendATCmd((uint8_t*)"AT+CWLAP\r\n");
				while(ESP_IsRecvComp(resp_buf, &resp_buffer_idx, &uart_buf));
				
				if(strstr((const char*)resp_buf, "<your ssid>"))
				{
					prog_state = ESP_CONN_AP;
				  ESP_ClearBuffer(resp_buf, &resp_buffer_idx);
				}
			break;
			
			case ESP_CONN_AP:
				ESP_SendATCmd((uint8_t*)"AT+CWJAP:\"<your_ssid>\",\"<psw>\"\r\n");
				while(ESP_IsRecvComp(resp_buf, &resp_buffer_idx, &uart_buf));
				
				if(strstr((const char*)resp_buf, "CONNECTED"))
				{
					prog_state = ESP_ROUTINE;
				  ESP_ClearBuffer(resp_buf, &resp_buffer_idx);
				}
			break;
			
			case ESP_ROUTINE:
			break;
			default:
			break;
			
		}
	} 
}
/******************************************************* END ****************************************************/

/*********************************************************************************************************
**																						1.ESP LAYER																	     					**
----------------------------------------------------------------------------------------------------------
** At this laye, program is going to handle USART-ESP communication and read out response for process	  **
** 1. AT Command Sending		          																																	**
** 2. ESP response management (OK/ERROR)																																**
** 3. Buffer cleaning																																										**
----------------------------------------------------------------------------------------------------------
Author : Goragod P.
Version 1.0
*********************************************************************************************************/
void ESP_SendATCmd(uint8_t* cmd)
{
	uint8_t cmd_len = 0;
	while(cmd[cmd_len++] != '\n');
	USART_SendCmd(cmd, cmd_len);
}

uint16_t test;
uint8_t ESP_IsRecvComp(uint8_t* buf, uint16_t* idx, rb_t* rb)
{
	/* Process resp by getting data from ring buffer */
	test = *idx + 1;
	if(rb->nbr_element > 0)
	{
		buf[*idx] = rb_get_char(rb);
		*idx += 1 ;
	}
	
	if(strstr((const char*)buf, "OK") != NULL || strstr((const char*)buf, "ERROR") != NULL)
		return 0;
	else
		return 1;
}

void ESP_ClearBuffer(uint8_t* buf, uint16_t* idx)
{
	memset(buf, 0, MAX_RESP_BUFFER_SIZE);
	*idx = 0;
}
/*********************************************************************************************************
**																				2.Buffer MANAGEMENT LAYER												     					**
----------------------------------------------------------------------------------------------------------
** At this layer, incomming data is pre-process by implementation of ring-buffer to prevent drop-out	  **
** 1. Ring buffer initialize          																																	**
** 2. Put single character to ring buffer																																**
** 3. Get single character from ring buffer																															**
----------------------------------------------------------------------------------------------------------
Author : Goragod P.
Version 1.0
*********************************************************************************************************/
void rb_init(rb_t* b)
{
	b->head = 0;
	b->tail = 0;
	b->nbr_element = 0;
	memset(b->buffer, 0, MAX_RESP_BUFFER_SIZE);
}

uint8_t rb_isfull(rb_t* b)
{
	if(b->head - b->tail == b->nbr_element || b->tail - b->head == b->nbr_element)
		return 1;
	else
		return 0;
}

uint8_t rb_isempty(rb_t* b)
{
	if(b->head - b->tail == 0U || b->tail - b->head == 0)
		return 1;
	else
		return 0;
}

void rb_put_char(rb_t* b, uint8_t data)
{
	if(rb_isfull(b) != 0)
	{
		if(b->tail > MAX_RING_BUFFER_SIZE - 1)
			b->tail = 0;
		if(b->head > MAX_RING_BUFFER_SIZE - 1)
			b->head = 0;
		
		b->buffer[b->tail++] = data;
		if(b->head < b->tail)
			b->nbr_element = b->tail - b->head;
		else
			b->nbr_element = (MAX_RING_BUFFER_SIZE - b->tail) - b->head; 
	}
}

uint8_t rb_get_char(rb_t* b)
{
	if(rb_isempty(b) != 1)
	{
		b->nbr_element -= 1;
		return b->buffer[b->head++];	
	}
}
/*********************************************************************************************************
**																				3.USART COMMUNICATION LAYER																		**
----------------------------------------------------------------------------------------------------------
** - This layer handle in hardware related usart peripheral such as																			**
** 1. USART1 Peripheral Initialization																																	**
** 2. Transimit single character using polling mode (8b)																								**
** 3. Receive singlle character using interrupt mode (8b)																								**
----------------------------------------------------------------------------------------------------------
Author : Goragod P.
Version 1.0
*********************************************************************************************************/
void USART_SendCmd(uint8_t* cmd, uint8_t len)
{
	uint8_t i;
	for(i = 0; i < len; ++i)
	{
		while(LL_USART_IsActiveFlag_TXE(USART1) == RESET);
		LL_USART_TransmitData8(USART1, cmd[i]);
	}
}


void USART_GPIO_Config(void)
{

	LL_GPIO_InitTypeDef gpio_initstruct;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	gpio_initstruct.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstruct.Pin = LL_GPIO_PIN_9;
	gpio_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstruct.Pull = LL_GPIO_PULL_UP;
	gpio_initstruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_initstruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOA, &gpio_initstruct);
	
	gpio_initstruct.Pin = LL_GPIO_PIN_10;
	LL_GPIO_Init(GPIOA, &gpio_initstruct);
}

void USART_Config(void)
{
	LL_USART_InitTypeDef usart_initstructure;
	USART_GPIO_Config();
	
	/*NVIC Config*/
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	
	usart_initstructure.BaudRate = 115200;
	usart_initstructure.DataWidth = LL_USART_DATAWIDTH_8B;
	usart_initstructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	usart_initstructure.Parity = LL_USART_PARITY_NONE;
	usart_initstructure.StopBits = LL_USART_STOPBITS_1;
	usart_initstructure.OverSampling = LL_USART_OVERSAMPLING_16;
	usart_initstructure.TransferDirection = LL_USART_DIRECTION_TX_RX;
	
	LL_USART_Init(USART1, &usart_initstructure);
	LL_USART_Enable(USART1);
	
	LL_USART_EnableIT_RXNE(USART1);
}

void USART1_IRQHandler()
{
	uint8_t temp;
	if(LL_USART_IsActiveFlag_RXNE(USART1) == SET)
	{
		temp = LL_USART_ReceiveData8(USART1);
		rb_put_char(&uart_buf, temp);
	}
}

/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}

/* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

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
