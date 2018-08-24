#include "stm32l1xx.h"

int value_sw;

int main() {
	GPIO_InitTypeDef  sw_pc1;
	GPIO_InitTypeDef  led_pb6;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

	GPIO_StructInit(&sw_pc1);
	sw_pc1.GPIO_Mode = GPIO_Mode_IN;
	sw_pc1.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC,&sw_pc1);

	GPIO_StructInit(&led_pb6);
	led_pb6.GPIO_Mode = GPIO_Mode_OUT;
	led_pb6.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB,&led_pb6);

	while(1) {
		value_sw = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1);
		if(value_sw == 0) {
			GPIO_SetBits(GPIOB,GPIO_Pin_6);
		} else {
			GPIO_ResetBits(GPIOB,GPIO_Pin_6);
		}
	}
}

