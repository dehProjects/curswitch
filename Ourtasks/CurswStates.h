/******************************************************************************
* File Name          : CurswStates.h
* Date First Issued  : 03/03/2021
* Description        : States w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __CURSWSTATES
#define __CURSWSTATES

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"

/* Substate states for CONNECTING main state */
enum connecting_state
{
/* Two contactor mode */
	CONNECT_C1,  // Contactor #1 closure delay
	CONNECT_C2,  // Minimum pre-charge duration delay
	CONNECT_C3,  // Additional pre-charge delay, voltage test
	CONNECT_C4,  // Contactor #2 closure delay
/* One contactor mode: #1 is contactor, #2 pre-chg relay */
	CONNECT_C1B, // Contactor #1 closure delay
	CONNECT_C2B, // Minimum pre-charge duration delay
	CONNECT_C3B, // Additional pre-charge delay, voltage test
	CONNECT_C4B, // Contactor #2 closure delay
};

void ContactorStates_otosettling_init(struct CURSWFUNCTION* pcf);
void ContactorStates_disconnecting(struct CURSWFUNCTION* pcf);
void ContactorStates_disconnected(struct CURSWFUNCTION* pcf);
void ContactorStates_connecting(struct CURSWFUNCTION* pcf);
void ContactorStates_connected(struct CURSWFUNCTION* pcf);
void ContactorStates_faulting(struct CURSWFUNCTION* pcf);
void ContactorStates_faulted(struct CURSWFUNCTION* pcf);
void ContactorStates_reset(struct CURSWFUNCTION* pcf);
void transition_faulting(struct CURSWFUNCTION* pcf, uint8_t fc);

#endif

