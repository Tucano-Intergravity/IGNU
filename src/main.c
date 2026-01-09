/**
 * @file main.c
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0.0
 * @date 2025-11-07
 *
 * @copyright Danam Systems Copyright (c) 2025
 */



/***********************************************************
					Include Files
***********************************************************/
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Xilinx includes. */
#include "xil_printf.h"

/* User includes */
#include "siu/siu_task.h"
#include "opu/opu_task.h"
#include "common/common.h"
#include "scu/scu_task.h"
#include "dbg/dbg_task.h"
#include "IGNU/Inc/ignu_task.h"

/***********************************************************
					Gloabal Variables
***********************************************************/

/***********************************************************
					Local Variables
***********************************************************/

/***********************************************************
					Function Declarations
***********************************************************/

/***********************************************************
					Functions
***********************************************************/

/**
 * @mainpage SCDAU PS SW
 */

/**
 * @brief ?? ??
 *
 * @return int
 */

void test_thread(void *p);
static TaskHandle_t xTestTask;		// GPS task handler

int main()
{
	static TaskHandle_t xSiuTask;		// System Initialization Unit Task
	static TaskHandle_t xOpuTask;		// Operational Unit Task
	static TaskHandle_t xScuTask;		// System Control Unit Task
	static TaskHandle_t xDbgTask;		// Debug Unit Task
	static TaskHandle_t xIgnuTask;		// IGNU Task

	printf( "SCDAU Processing Module GINU v0.1.0\n" );
	gpioSetFunc();

	/* SIU Task ?? */
	xTaskCreate( SiuTask, 					/* The function that implements the task. */
				( const char * ) "SIU", 	/* Text name for the task, provided to assist debugging only. */
				SCDAU_STACK_SIZE, 			/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY+3,			/* The task runs at the idle priority. */
				&xSiuTask );

	/* OPU Task ?? */
	xTaskCreate( OpuTask, 					/* The function that implements the task. */
				( const char * ) "OPU", 	/* Text name for the task, provided to assist debugging only. */
				SCDAU_STACK_SIZE*10, 		/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY+2,			/* The task runs at the idle priority. */
				&xOpuTask );

	/* SCU Task ?? */
	xTaskCreate( ScuTask, 					/* The function that implements the task. */
				( const char * ) "SCU", 	/* Text name for the task, provided to assist debugging only. */
				SCDAU_STACK_SIZE, 			/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY+1,			/* The task runs at the idle priority. */
				&xScuTask );

	/* DBG Task ?? */
	xTaskCreate( DbgTask, 					/* The function that implements the task. */
				( const char * ) "DBG", 	/* Text name for the task, provided to assist debugging only. */
				SCDAU_STACK_SIZE, 			/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY+1,			/* The task runs at the idle priority. */
				&xDbgTask );

	/* IGNU Task ?? */
	xTaskCreate( IgnuTask, 					/* The function that implements the task. */
				( const char * ) "IGNU", 	/* Text name for the task, provided to assist debugging only. */
				SCDAU_STACK_SIZE, 			/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY+2,			/* The task runs at the idle priority. */
				&xIgnuTask );

	//xTaskCreate( test_thread, (const char*)"test_thread", SCDAU_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xTestTask );


	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

void test_thread(void *p)
{
	const TickType_t x1ms = pdMS_TO_TICKS( DELAY_1_MSECOND );

	while(1)
	{
		vTaskDelay( x1ms );
	}
	vTaskDelete( NULL );
}
