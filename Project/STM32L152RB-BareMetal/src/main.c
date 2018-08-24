#include "stm32l1xx.h"

int main()
{
	//Enable GPIOA and GPIOB Clock
	RCC->AHBENR |= 0x00000003;
	
	//Config GPB6/PB7 as OUTPUT
	GPIOB->MODER |= (0x00005000);
	//Config PB6/PB7 as push pull type
	GPIOB->OTYPER &= ~(0xFFFF);
	//Config PB6/PB7 push pull type to no-pull (not using internal pullup-down circuit)
	GPIOB->PUPDR &= ~(0xFFFFFFFF);
	//Config PB6/PB7 speed to be high speed
	GPIOB->OSPEEDR |= (0x0000C0000);
	
//	//Config PA0 as INPUT
//	GPIOA->MODER &= ~(0xFFFFFFFF);
//	//Config PA0 as push pull type
//	GPIOB->OTYPER &= ~(0xFFFF);
//	//Config PA0 push pull type to no-pull (not using internal pullup-down circuit)
//	GPIOB->PUPDR &= ~(0xFFFFFFFF);
//	//Config PA0 speed to be high speed
//	GPIOB->OSPEEDR |= (0x0000C0000);
	
	//Config EXTI
	
	while(1){

				GPIOB->ODR |= (0x00C0);

				GPIOB->ODR &= ~(0xFFFF);
	
	}
}

void EXTI0_IRQHandler(void)
{

}
