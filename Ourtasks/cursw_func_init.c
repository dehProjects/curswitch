/******************************************************************************
* File Name          : cursw_func_init.c
* Date First Issued  : 03/03/2021
* Description        : 
*******************************************************************************/

#include "contactor_func_init.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "MailboxTask.h"
#include "CurswTask.h"
#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "morse.h"

/* From 'main.c' */
extern TIM_HandleTypeDef htim4;

/* *************************************************************************
 * void contactor_func_init_init(struct CURSWFUNCTION* p, struct ADCFUNCTION* padc);
 *	@brief	: Initialize working struct for ContactorTask
 * @param	: p    = pointer to ContactorTask
 * @param	: padc = pointer to ADC working struct
 * *************************************************************************/

void contactor_func_init_init(struct CURSWFUNCTION* p, struct ADCFUNCTION* padc)
{
	int i;

	/* Pointer to ADC working parameters. */
	p->padc = padc;


	/* Convert ms to timer ticks. */


	return;
}
