/*---------------------------------------------------------------------------
 *      RL-ARM - RTX  --  example code for "RTOS" lab, CPE-312
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include "stm32l1xx.h"
#include "stdio.h"

/* id1, id2 will contain task identifications at run-time. */
OS_TID id1, id2;

/* Forward declaration of tasks. */
__task void task1 (void);
__task void task2 (void);

/*---------------------------------------------------------------------------
  Tasks : Implement tasks
 *---------------------------------------------------------------------------*/
__task void task1 (void){
	/* Obtain own system task identification number. */
  id1 = os_tsk_self();

	/* Create task2 and obtain its task identification number. */
  id2 = os_tsk_create (task2, 0);
	
  for (;;) {
		/* task1 activity :-- turn on "PB7" */
		GPIO_SetBits(GPIOB,GPIO_Pin_7);
		
		/* Signal to task2 that task1 has completed. */
    os_evt_set(0x0004, id2);

    /* Wait for completion of task2 activity. */
    /*  0xFFFF makes it wait without timeout. */
    /*  0x0004 represents bit 2. */
    os_evt_wait_or(0x0004, 0xFFFF);

    /* Wait for 20 clock ticks before restarting task1 activity. */
    os_dly_wait(20);
  }
}

__task void task2 (void) {

  for (;;) {
		/* Wait for completion of task1 activity. */
		/*  0xFFFF makes it wait without timeout. */
		/*  0x0004 represents bit 2. */
    os_evt_wait_or(0x0004, 0xFFFF);

		/* Wait for 50 clock ticks before starting task2 activity. */
    os_dly_wait(50);

		/* task2 activity :-- turn off "PB7" */
		GPIO_ResetBits(GPIOB,GPIO_Pin_7);
		
		/* Signal to task1 that task2 has completed. */
    os_evt_set(0x0004, id1);
  }
}

/*----------------------------------------------------------------------------
  Additional configuration: Initialize and configure additional peripherals
 *---------------------------------------------------------------------------*/
void PB6_7_Configuration(void){
	GPIO_InitTypeDef pb6_7;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	
	GPIO_StructInit (&pb6_7);
		pb6_7.GPIO_Mode = GPIO_Mode_OUT;
		pb6_7.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
	GPIO_Init (GPIOB, &pb6_7);
}

/*----------------------------------------------------------------------------
  Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {
	
	PB6_7_Configuration();					/* configure PB6 and PB7 */
	
	/* Start the RTX kernel, and then create and execute task1. */
  os_sys_init(task1);

}
