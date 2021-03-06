/******************************************************************************
* File Name          : CurswTask.h
* Date First Issued  : 03/03/2021
* Description        : Current sense Switch w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __CURSWTASK
#define __CURSWTASK

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"

/* Task notification bit assignments. */
#define CNCTBIT00	(1 << 0)  // ADCTask has new readings
#define CNCTBIT01	(1 << 1)  // HV sensors usart RX line ready
#define CNCTBIT02	(1 << 2)  // spare
#define CNCTBIT03	(1 << 3)  // TIMER 3: uart RX keep-alive
#define CNCTBIT04	(1 << 4)  // TIMER 1: Command Keep Alive
#define CNCTBIT05	(1 << 5)  // TIMER 2: Multiple use delays

/* Event status bit assignments (CoNtaCTor EVent ....) */
#define CNCTEVTIMER1 (1 << 0) // 1 = timer1 timed out: command/keep-alive
#define CNCTEVTIMER2 (1 << 1) // 1 = timer2 timed out: delay
#define CNCTEVTIMER3 (1 << 2) // 1 = timer3 timed out: uart RX/keep-alive
#define CNCTEVCACMD  (1 << 3) // 1 = CAN rcv: general purpose command
#define CNCTEVCANKA  (1 << 4) // 1 = CAN rcv: Keep-alive/command
#define CNCTEVCAPOL  (1 << 5) // 1 = CAN rcv: Poll
#define CNCTEVCMDRS  (1 << 6) // 1 = Command to reset
#define CNCTEVCMDCN  (1 << 7) // 1 = Command to connect
#define CNCTEVHV     (1 << 8) // 1 = New HV readings
#define CNCTEVADC    (1 << 9) // 1 = New ADC readings

/* Output status bit assignments */
#define CNCTOUT00K1  (1 << 0) // 1 = contactor #1 energized
#define CNCTOUT01K2  (1 << 1) // 1 = contactor #2 energized
#define CNCTOUT02X1  (1 << 2) // 1 = aux #1 closed
#define CNCTOUT03X2  (1 << 3) // 1 = aux #2 closed
#define CNCTOUT04EN  (1 << 4) // 1 = DMOC enable FET
#define CNCTOUT05KA  (1 << 5) // 1 = CAN msg queue: KA status
#define CNCTOUT06KAw (1 << 6) // 1 = contactor #1 energized & pwm'ed
#define CNCTOUT07KAw (1 << 7) // 1 = contactor #2 energized & pwm'ed



enum CONTACTOR_STATE
{
	DISCONNECTED,   /*  0 */
	CONNECTING,     /*  1 */
	CONNECTED,      /*  2 */
	FAULTING,       /*  3 */
	FAULTED,        /*  4 */
	RESETTING,      /*  5 */
	DISCONNECTING,  /*  6 */
	OTOSETTLING,    /*  7 one time intializing. */
};

/* Function command response payload codes. */
enum CONTACTOR_CMD_CODES
{
	ADCRAW5V,         // PA0 IN0  - 5V sensor supply
	ADCRAWCUR1,       // PA5 IN5  - Current sensor: total battery current
	ADCRAWCUR2,       // PA6 IN6  - Current sensor: motor
	ADCRAW12V,        // PA7 IN7  - +12 Raw power to board
	ADCINTERNALTEMP,  // IN17     - Internal temperature sensor
	ADCINTERNALVREF,  // IN18     - Internal voltage reference
	UARTWHV1,
	UARTWHV2,
	UARTWHV3,
	CAL5V,
	CAL12V,
};


/* Working struct for Contactor function/task. */
// Prefixes: i = scaled integer, f = float
// Suffixes: k = timer ticks, t = milliseconds
struct CONTACTORFUNCTION
{
   // Parameter loaded either by high-flash copy, or hard-coded subroutine
	struct CONTACTORLC lc; // Parameters for contactors, (lc = Local Copy)

	struct ADCFUNCTION* padc; // Pointer to ADC working struct

	/* Events status */
	uint32_t evstat;

	/* Output status */
	uint32_t outstat;
	uint32_t outstat_prev;


	/* OTO settling */
	uint32_t otosw;


	uint32_t statusbits;
	uint32_t statusbits_prev;

	/* Setup serial receive for uart (HV sensing) */
	struct SERIALRCVBCB* prbcb3;	// usart3

	TimerHandle_t swtimer1; // Software timer1: command/keep-alive
	TimerHandle_t swtimer2; // Software timer2: multiple purpose delay
	TimerHandle_t swtimer3; // Software timer3: uart RX/keep-alive


	/* PWM struct */
	TIM_OC_InitTypeDef sConfigOCn; // 'n' - serves ch3 and ch4
	
	uint8_t state;      // Contactor main state
	uint8_t substateC;  // State within CONNECTING (0-15)
	uint8_t substateX;  // spare substate (0-15)

};

/* *************************************************************************/
osThreadId xContactorTaskCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: CurswitchTaskHandle
 * *************************************************************************/
void StartCurswitchTask(void const * argument);
/*	@brief	: Task startup
 * *************************************************************************/

extern struct CONTACTORFUNCTION curswfunction;
extern osThreadId CurswitchTaskHandle;

#endif

