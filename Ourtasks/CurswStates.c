/******************************************************************************
* File Name          : CurswStates.c
* Date First Issued  : 03/03/2021
* Description        : States w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "CurswStates.h"
#include "CurswTask.h"
#include "cursw_idx_v_struct.h"
#include "morse.h"
#include "adcparamsinit.h"

int dbgstmp;

static void transition_connecting(struct CURSWFUNCTION* pcf);
static void transition_disconnecting(struct CURSWFUNCTION* pcf);
static void new_state(struct CURSWFUNCTION* pcf, uint32_t newstate);
static void open_contactors(struct CURSWFUNCTION* pcf);

/* TIM4 CH3, CH4 drive Conatactor #1, #2 coils. */
extern TIM_HandleTypeDef htim4; // Needs this for autoreload period

/* *************************************************************************
 * @brief	: 
 * *************************************************************************/

void CurswStates_otosettling_init(struct CURSWFUNCTION* pcf)
{
	/* Check if uart hv readings timer timed out. */
	if ((pcf->evstat & CNCTEVTIMER3) != 0)
	{ // Here, keep-alive uart rx timer timed out
		transition_faulting(pcf, NO_UART3_HV_READINGS);
		return;
	}

	/* Delay using data for a few cycles of readings. */
	if (pcf->hvuartctr < 50) return;

	transition_disconnecting(pcf);
	return;
}


/* ==== DISCONNECTED ======================================== */
void CurswStates_disconnected(struct CURSWFUNCTION* pcf)
{
	uint32_t tmp;
	int16_t ret;

	if ((pcf->evstat & CNCTEVTIMER1) != 0)
	{ // Keep-alive timer timout
		transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);	
		return;	
	}

	/* Get rid of prior fault codes, mostly for display purposes. */
	pcf->faultcode = NOFAULT; // Reset code

	/* Update zero offset for Hall-effect current sensor. */
	if ((pcf->evstat & CNCTEVADC) != 0)
	{ // Here, new set of ADC readings
		ret = ratiometric_cal_zero_CURRENTTOTAL(pcf->padc);
		if (ret != 0)// morse_trap(55);
		{
			transition_faulting(pcf,HE_AUTO_ZERO_TOLERANCE_ERR);
		}
	}

	/* Install jumper to ignore HV readings. */
	// I/O pin shows '1' when jumper removed; '0' when present.
	if (HAL_GPIO_ReadPin(HVBYPASSPINPORT,  HVBYPASSPINPIN) != GPIO_PIN_SET)
	{ // Set configuration bit to skip HV reading logic
		pcf->lc.hwconfig |= PWMNOHVSENSOR;
	}
	else
	{
		pcf->lc.hwconfig &= ~PWMNOHVSENSOR;
	}

	/* Skip HV readings and battery string voltage check if no sensor. */
	if ((pcf->lc.hwconfig & PWMNOHVSENSOR) == 0)
	{ // Here, configuration: HV sensor is present

		if ((pcf->evstat & CNCTEVTIMER3) != 0)
		{ // Here: Not receiving readings from uart3 sensor
			transition_faulting(pcf,NO_UART3_HV_READINGS);
			return;
		}

		/* Check for battery string below threshold. */
		if (pcf->hv[IDXHV1].hv < pcf->ibattlow)
		{ // Here, battery voltage is too low (or readings missing!)
			transition_faulting(pcf,BATTERYLOW);
			return;
		}
	}

	/* Check if aux contacts match, if aux contacts present. */
	if ((pcf->lc.hwconfig & AUX1PRESENT) != 0)
	{ // Aux contacts are present
		tmp = HAL_GPIO_ReadPin(AUX1_GPIO_REG,AUX1_GPIO_IN);// read i/o pin
		if ((pcf->lc.hwconfig & AUX1SENSE) != 0)
		{ // Reverse sense of bit
			tmp ^= 0x1;
		}
		if (tmp != GPIO_PIN_RESET)
		{ // Transition to fault state; set fault code
			transition_faulting(pcf,CURSW1_OFF_AUX1_ON);
			return;			
		}
	}
	/* Check if aux contacts match, if aux contacts present. */
	if ((pcf->lc.hwconfig & AUX2PRESENT) != 0)
	{ // Aux contacts are present
		tmp = HAL_GPIO_ReadPin(AUX2_GPIO_REG,AUX2_GPIO_IN);// read i/o pin
		if ((pcf->lc.hwconfig & AUX2SENSE) != 0)
		{ // Reverse sense of bit
			tmp ^= 0x1;
		}
		if (tmp != GPIO_PIN_RESET)
		{ // Transition to fault state; set fault code
			transition_faulting(pcf,CURSW2_OFF_AUX2_ON); 
			return;			
		}
	}
	/* Keep-alive CAN msgs carry commands. */
	if ((pcf->evstat & CNCTEVCMDCN) != 0)
	{ // Here, ==> request to CONNECT <==
		transition_connecting(pcf);
		return;
	}
	/* JIC.  Be sure Updates have both coils de-energized. DMOC hw disabled */
	pcf->outstat &= ~(CNCTOUT00K1 | CNCTOUT01K2 | CNCTOUT06KAw | CNCTOUT07KAw |
        CNCTOUT04EN);
	pcf->outstat_prev |= (CNCTOUT00K1 | CNCTOUT01K2); // jic
	return;
}
/* *************************************************************************
 * void CurswStates_connecting(struct CURSWFUNCTION* pcf);
 * @brief	: CONNECTING state
 * *************************************************************************/
/* ===== xCONNECTING ==================================================== */
static void transition_connecting(struct CURSWFUNCTION* pcf)
{ // Intialize disconnected state

	/* Cursw configuration modes handled differently. */
	if ((pcf->lc.hwconfig & ONECURSW) == 0)
	{ // Here, TWO CURSW mode

		/* Reset sub-states for connecting in this mode */
		pcf->substateC = CONNECT_C1;

		/* Set one-shot timer for contactor #1 closure delay */
		if (pcf->close1_k == 0) morse_trap(81);
		xTimerChangePeriod(pcf->swtimer2,pcf->close1_k, 2); 

		/* Energize coil #1: Battery_string-to-DMOC+ */
		pcf->outstat |= CNCTOUT00K1; // Energize coil during update section
		pcf->outstat &= ~CNCTOUT06KAw; // No pwm, JIC
	}
	else
	{ // Here, ONE CURSW W PRE-CHG RELAY mode

		/* Reset sub-states for connecting in this mode */
		pcf->substateC = CONNECT_C1B;

		/* Set one-shot timer for pre-chg relay (#2) closure delay */
		if (pcf->close2_k == 0) morse_trap(81);
		xTimerChangePeriod(pcf->swtimer2,pcf->close2_k, 2); 

		/* Energize coil #2: (Pre-charge relay) */
		pcf->outstat |= CNCTOUT01K2; // Energize coil during update section
		pcf->outstat &= ~CNCTOUT07KAw; // No pwm, JIC
	}
	pcf->evstat &= ~CNCTEVTIMER2; // Reset sw2 timer timeout bit

	/* Update main state */
	new_state(pcf,CONNECTING);
	return;
}
/* ====== CONNECTING ==================================================== */
void CurswStates_connecting(struct CURSWFUNCTION* pcf)
{
	uint32_t tmp;
	int32_t stmp;

	/* Terminate CONNECTED if commands are disconnect or reset. */
	if ( ((pcf->evstat & CNCTEVCMDCN) == 0) | ((pcf->evstat & CMDRESET) != 0) ) 
	{ // 
		transition_disconnecting(pcf);
	}

	switch(pcf->substateC)
	{
/* ============= TWO CURSW MODE ===================================== */
	case CONNECT_C1:  // Cursw #1 closure delay
		if ((pcf->evstat & CNCTEVTIMER2) == 0) break;

		/* Here, timer timed out, so contactor #1 should be closed. */

		/* Check if aux contacts match, if aux contacts present. */
		if ((pcf->lc.hwconfig & AUX1PRESENT) != 0)
		{ // Aux contacts are present
			tmp = HAL_GPIO_ReadPin(AUX1_GPIO_REG,AUX1_GPIO_IN);// read i/o pin
			if ((pcf->lc.hwconfig & AUX1SENSE) == 1)
			{ // Reverse sense of bit
				tmp ^= 0x1;
			}
			if (tmp != GPIO_PIN_SET)
			{ // Transition to fault state; set fault code
				/* Aux contact says it did not close. */
				transition_faulting(pcf,CURSW1_ON_AUX1_OFF);
//morse_trap(66);
				return;			
			}
		}

		/* For two contactor config, we can check if it looks closed. */
		if ((pcf->lc.hwconfig & PWMNOHVSENSOR) == 0)
		{ // Here, configuration: HV sensor is present

			if ((pcf->lc.hwconfig & ONECURSW) == 0)
			{ // Here, two contactor config, so voltage should jump up
//				if ((pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc) < pcf->ihv1mhv2max) // 
				stmp = (pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc);
				if (stmp < 0) stmp = -stmp;
				if ( stmp > pcf->idiffafter ) 
				{
					transition_faulting(pcf,CURSW1_DOES_NOT_APPEAR_CLOSED); 
//morse_trap(67);
					return;
				}
			}
			else
			{ // One contactor configuration
				// Voltage across contacts should be very small unless it didn't close
				// In case calibration makes diff negative, use absolute diff
				stmp = (pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc);
				if (stmp < 0) stmp = -stmp;
				if ( stmp > pcf->idiffafter ) 
				{ // Here, something not right with contactor closing
					transition_faulting(pcf,CURSW1_CLOSED_VOLTSTOOBIG);
					break;
				}	
			}
		}

		/* Here, looks good, so start a minimum pre-charge delay. */

		/* If this contactor is to be PWM'ed drop down from 100%. */
		if ((pcf->lc.hwconfig & PWMCURSW1) != 0)
		{ // TIM4 CH3 Lower PWM from 100%
			pcf->outstat |= CNCTOUT06KAw; // Switch pwm during update section
		}

		/* Set one-shot timer for a minimum pre-charge duration. */
if (pcf->prechgmin_k == 0) morse_trap(82); // Oops! Bad initialization
		xTimerChangePeriod(pcf->swtimer2, pcf->prechgmin_k, 2); 
		pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 

		pcf->substateC = CONNECT_C2;
		break;
/* ...................................................................... */
	case CONNECT_C2:  // Minimum pre-charge duration delay
		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Minimum pre-charge time has expired.
			if (pcf->prechgmax_k == 0) morse_trap(83); // Oops! Bad initialization
			xTimerChangePeriod(pcf->swtimer2, pcf->prechgmax_k, 2); 
			pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 
			pcf->substateC = CONNECT_C3;
			break;
		}
		/* Check that we are getting new hv readings. */
		if ((pcf->evstat & CNCTEVTIMER3) != 0)
		{ // Here, not receiving readings from uart3 sensor
			transition_faulting(pcf,NO_UART3_HV_READINGS);
			return;
		}

		/* Check that we are still getting keep-alive msgs. */
		if ((pcf->evstat & CNCTEVTIMER1) != 0)
		{ // Keep-alive timer timout
			transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);	
			return;	
		}

		break;

/* ...................................................................... */
	case CONNECT_C3: /* Check if voltage has reached cutoff. */

		/* Check that we are still getting keep-alive msgs. */
		if ((pcf->evstat & CNCTEVTIMER1) != 0)
		{ // Keep-alive timer timout
			transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);	
			return;	
		}

// This may not be useful.
		if ((pcf->evstat & CNCTEVHV) != 0)
		{ // Here, new readings available
			pcf->evstat &= ~CNCTEVHV; // Clear new reading bit
		}

		/* Check timeout waiting for voltage to reach threshold */
		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Maximum pre-charge time has expired.
			if ((pcf->lc.hwconfig & PWMNOHVSENSOR) == 0)
			{ // Here, configuration: HV sensor is present
				transition_faulting(pcf,PRECHGVOLT_NOTREACHED);
				return;
			}
			else
			{ // Here, no HV sensor and timer timere out. Continue
if (pcf->close2_k == 0) morse_trap(88); // Initialization mistake
				xTimerChangePeriod(pcf->swtimer2,pcf->close2_k, 2); 
				pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 
				pcf->outstat |= CNCTOUT01K2;  // Energize #2 during update section
				pcf->substateC = CONNECT_C4;  // Next substate
				return;
			}
		}

		/* Here, timer is still timing. Check if cutoff voltage reached */
		if ((pcf->lc.hwconfig & PWMNOHVSENSOR) != 0)
		{ // Here, configuration: HV sensor is NOT present
			return; // Run until timer times out
		}

		if (pcf->hv[IDXHV3].hv < pcf->iprechgendv)
		{ // Here, end of pre-charge. Energize contactor 2
			pcf->outstat |= CNCTOUT01K2; // Energize #2 during update section

			/* Set one-shot timer for contactor 2 closure duration. */
if (pcf->close2_k == 0) morse_trap(88); // Initialization mistake
			xTimerChangePeriod(pcf->swtimer2,pcf->close2_k, 2); 
			pcf->evstat &= ~CNCTEVTIMER2;	 // Clear timedout status bit 
			pcf->substateC = CONNECT_C4;   // Next substate
			return;
		}
		break;
/* ...................................................................... */
	case CONNECT_C4:  // Cursw #2 close

		/* Check that we are still getting keep-alive msgs. */
		if ((pcf->evstat & CNCTEVTIMER1) != 0)
		{ // Keep-alive timer timout
			transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);	
			return;	
		}

		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Timer2 timed out: Cursw #2 should be closed

			if ((pcf->lc.hwconfig & PWMNOHVSENSOR) == 0)
			{ // Here, configuration: HV sensor is present

				// Voltage across contacts should be very small unless it didn't close
				if ( pcf->hv[IDXHV3].hvc > pcf->idiffafter ) 
				{ // Here, something not right with contactor closing
					transition_faulting(pcf,CURSW2_CLOSED_VOLTSTOOBIG);
					break;
				}
			}
					
			/* If this contactor is to be PWM'ed drop down from 100%. */
			if ((pcf->lc.hwconfig & PWMCURSW1) != 0)
			{ // TIM4 CH3 Lower PWM from 100%
				pcf->outstat |= CNCTOUT07KAw; // Switch to lower pwm in update section
			}

			new_state(pcf,CONNECTED);
		}
		/* event not relevant. Continue waiting for timer2 */
		break;
/* ============= ONE CURSW MODE ===================================== */
	case CONNECT_C1B:  // Pre-charge relay (#2) closure delay
		if ((pcf->evstat & CNCTEVTIMER2) == 0) break;

		/* Here, timer timed out, so pre-chg relay #2 is assumed closed. */

		/* Check if aux contacts match, if aux contacts present. */
		if ((pcf->lc.hwconfig & AUX2PRESENT) != 0)
		{ // Aux contacts are present
			tmp = HAL_GPIO_ReadPin(AUX2_GPIO_REG,AUX2_GPIO_IN);// read i/o pin
			if ((pcf->lc.hwconfig & AUX2SENSE) == 1)
			{ // Reverse sense of bit
				tmp ^= 0x1;
			}
			if (tmp != GPIO_PIN_SET)
			{ // Transition to fault state; set fault code
				/* Aux contact says it did not close. */
				transition_faulting(pcf,CURSW2_ON_AUX2_OFF);
//morse_trap(66);
				return;			
			}
		}

		/* Start a minimum pre-charge delay. */

		/* If this relay is to be PWM'ed, drop down from 100%. */
		if ((pcf->lc.hwconfig & PWMCURSW2) != 0)
		{ // TIM4 CH2 Lower PWM from 100%
			pcf->outstat |= CNCTOUT07KAw; // Switch pwm during update section
		}

		/* Set one-shot timer for a minimum pre-charge duration. */
if (pcf->prechgmin_k == 0) morse_trap(82); // Oops! Bad initialization
		xTimerChangePeriod(pcf->swtimer2, pcf->prechgmin_k, 2); 
		pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 

		pcf->substateC = CONNECT_C2B;
		break;
/* ...................................................................... */
	case CONNECT_C2B:  // Minimum pre-charge duration delay
		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Minimum pre-charge time has expired.
if (pcf->prechgmax_k == 0) morse_trap(83); // Oops! Bad initialization
			xTimerChangePeriod(pcf->swtimer2, pcf->prechgmax_k, 2); 
			pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 
			pcf->substateC = CONNECT_C3B;
			break;
		}
		/* Check that we are getting new hv readings. */
		if ((pcf->evstat & CNCTEVTIMER3) != 0)
		{ // Here, not receiving readings from uart3 sensor
			transition_faulting(pcf,NO_UART3_HV_READINGS);
			return;
		}
		break;

/* ...................................................................... */
	case CONNECT_C3B: /* Check if voltage has reached cutoff. */

		/* Check that we are still getting keep-alive msgs. */
		if ((pcf->evstat & CNCTEVTIMER1) != 0)
		{ // Keep-alive timer timout
			transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);	
			return;	
		}

// This may not be useful.
		if ((pcf->evstat & CNCTEVHV) != 0)
		{ // Here, new readings available
			pcf->evstat &= ~CNCTEVHV; // Clear new reading bit
		}

		/* Check timeout waiting for voltage to reach threshold */
		if ((pcf->evstat & CNCTEVTIMER2) != 0)
		{ // Maximum pre-charge time has expired.
			if ((pcf->lc.hwconfig & PWMNOHVSENSOR) == 0)
			{ // Here, configuration: HV sensor is present
				transition_faulting(pcf,PRECHGVOLT_NOTREACHED);
				return;
			}
			else
			{
if (pcf->close1_k == 0) morse_trap(85); // Initialization mistake
				xTimerChangePeriod(pcf->swtimer2,pcf->close1_k, 2); 
				pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 
				pcf->outstat |= CNCTOUT00K1; // Energize #1 during update section
				pcf->substateC = CONNECT_C4B; // Next substate
			}
		}

		/* Here, timer is still timing. Check if cutoff voltage reached */
		if ((pcf->lc.hwconfig & PWMNOHVSENSOR) != 0)
		{ // Here, configuration: HV sensor is NOT present
			return; // Run until timer times out
		}

		stmp = (pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc);
		if (stmp < 0) stmp = -stmp; // JIC HV2 calibration makes difference negative
dbgstmp = stmp;
		if (stmp < pcf->iprechgendvb)
		{ // Here, end of pre-charge.  Energize contactor 1
			pcf->outstat |= CNCTOUT00K1; // Energize #1 during update section

			/* Set one-shot timer for contactor #1 closure duration. */
if (pcf->close1_k == 0) morse_trap(85); // Initialization mistake
			xTimerChangePeriod(pcf->swtimer2,pcf->close1_k, 2); 
			pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 

			pcf->substateC = CONNECT_C4B; // Next substate
			return;			
		}
		break;
/* ...................................................................... */
	case CONNECT_C4B:  // Cursw #1 close

		/* Check that we are still getting keep-alive msgs. */
		if ((pcf->evstat & CNCTEVTIMER1) != 0)
		{ // Keep-alive timer timout
			transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);	
			return;	
		}

		if ((pcf->evstat & CNCTEVTIMER2) == 0) break;

		// Timer2 timed out: Cursw #1 assumed to be closed

		if ((pcf->lc.hwconfig & PWMNOHVSENSOR) == 0)
		{ // Here, configuration: HV sensor is present

			// Voltage across contacts should be very small unless it didn't close
			// In case calibration makes diff negative, use absolute diff
			stmp = (pcf->hv[IDXHV1].hvc - pcf->hv[IDXHV2].hvc);
			if (stmp < 0) stmp = -stmp; // jic HV2 slightly larger than HV1
			if ( stmp > pcf->idiffafter ) 
			{ // Here, something not right with contactor closing
				transition_faulting(pcf,CURSW1_CLOSED_VOLTSTOOBIG);
			}
		}
					
		/* If this contactor is to be PWM'ed drop down from 100%. */
		if ((pcf->lc.hwconfig & PWMCURSW1) != 0)
		{ // TIM4 CH3 Lower PWM from 100%
			pcf->outstat |= CNCTOUT06KAw; // Switch to lower pwm in update section
		}

		/* Open pre-chg relay, to prevent failure from toasting pre-chg resistor. */
		pcf->outstat &= ~CNCTOUT01K2;  // De-energize coil during update section
		pcf->outstat &= ~CNCTOUT07KAw; // No pwm, JIC			

		new_state(pcf,CONNECTED);
		break;

/* LAST OF SWITCH STATEMENT */
default: morse_trap(69);break; // JIC bug trap
	}
}
		
/* ======= CONNECTED ==================================================== */
void CurswStates_connected(struct CURSWFUNCTION* pcf)
{
	/* Check that we are still getting keep-alive msgs. */
	if ((pcf->evstat & CNCTEVTIMER1) != 0)
	{ // Keep-alive timer timout
		transition_faulting(pcf,KEEP_ALIVE_TIMER_TIMEOUT);	
		return;	
	}

	/* Terminate CONNECTED if commands are disconnect or reset. */
	if ( ((pcf->evstat & CNCTEVCMDCN) == 0) | ((pcf->evstat & CMDRESET) != 0) ) 
	{ // 
		transition_disconnecting(pcf);
	}
	/* Continue connection. */
	pcf->outstat |= CNCTOUT04EN; // DMOC enabled
	return;
}
/* ===== xDISCONNECTING ================================================= */
static void transition_disconnecting(struct CURSWFUNCTION* pcf)
{
		/* Prevent opening contactors when battery current exceeds threshold. */
#ifdef WAITFORCURRENTTOSUBSIDE
/*
This code is by-passed becasue an open connection from the current sensor will
prevent the contactor from ever opening.

Better would be to sense the the open circuit value (which should(!) show up as
a negative pcf->padc->cur1.irk0 (the zero offset). The 'if' below with
// *** HAS NOT BEEN verified as correct!

The expedient solution is to simply allow opening the contactors regardless of
the current being drawn.
*/
		if (pcf->padc->cur1.iI >= 0)
		{ // Positive current
			if (pcf->padc->cur1.iI > pcf->icurrentdisconnect)
			{
				return;
			}
		}
		else
		{ // Negative current
			if (pcf->padc->cur1.iI < -pcf->icurrentdisconnect)
			{
				if (pcf->padc->cur1.iI > -pcf->padc->cur1.irk0) // ***
				return;
			}
		}
#endif
		open_contactors(pcf);
		new_state(pcf,DISCONNECTING);	
		return;	
}
/* ===== DISCONNECTING ================================================== */
void CurswStates_disconnecting(struct CURSWFUNCTION* pcf)
{
	if ((pcf->evstat & CNCTEVTIMER2) != 0)
	{
			pcf->state = DISCONNECTED;
	}		
	/* Here, still waiting for TIMER2 to time out. */
	return;
}
/* ===== xFAULTING ====================================================== */
void transition_faulting(struct CURSWFUNCTION* pcf, uint8_t fc)
{
		open_contactors(pcf);     // Be sure to open contactors, set timer2
		pcf->faultcode = fc;	     // Set fault code
		new_state(pcf,FAULTING);	
		return;	
}
/* ===== FAULTING ======================================================= */
void CurswStates_faulting(struct CURSWFUNCTION* pcf)
{
	if ((pcf->evstat & CNCTEVTIMER2) != 0)
	{ // Timer 2 timed out, and contactors should be open
			new_state(pcf,FAULTED);
	}		
	/* Here, still waiting for TIMER2 to time out. */
	return;
}
/* ===== FAULTED ======================================================= */
void CurswStates_faulted(struct CURSWFUNCTION* pcf)
{
	if ((pcf->evstat & CNCTEVCMDRS) != 0)
	{ // Command to RESET
		transition_disconnecting(pcf);
	}
	/* Stuck in this state until Command to Reset */
	return;
}
/* ===== RESET ========================================================= */
void CurswStates_reset(struct CURSWFUNCTION* pcf)
{
	if ((pcf->evstat & CNCTEVCMDRS) != 0)
	{ // Command to RESET
		pcf->faultcode = NOFAULT; // Clear fault code.
		transition_disconnecting(pcf);
	}
	return;
}
/* *************************************************************************
 * static void open_contactors(struct CURSWFUNCTION* pcf);
 * @brief	: De-energize contactors and set time delay for opening
 * @param	: pcf = pointer to struct with "everything" for this function
 * *************************************************************************/
static void open_contactors(struct CURSWFUNCTION* pcf)
{
	/* Set one-shot timer for contactors opening duration. */
	if (pcf->open2_k > pcf->open1_k)
	{
		if (pcf->open2_k == 0) morse_trap(86);
		xTimerChangePeriod(pcf->swtimer2,pcf->open2_k, 2);
		pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 
	} 
	else
	{
		if (pcf->open1_k == 0) morse_trap(87);
		xTimerChangePeriod(pcf->swtimer2,pcf->open1_k, 2); 
		pcf->evstat &= ~CNCTEVTIMER2;	// Clear timedout status bit 
	}

	pcf->evstat &= ~CNCTEVTIMER2;	// Reset timeout bit 

	/* De-engerize both contactors and pwm'ing if on */
	pcf->outstat      &= ~(CNCTOUT00K1 | CNCTOUT01K2 | CNCTOUT06KAw | CNCTOUT07KAw);
	pcf->outstat_prev |= (CNCTOUT00K1 | CNCTOUT01K2); // jic
	return;
}
/* *************************************************************************
 * static void new_state(struct CURSWFUNCTION* pcf, uint32_t newstate);
 * @brief	: When there is a state change, do common things
 * @param	: pcf = pointer to struct with "everything" for this function
 * @param	: newstate = code number for the new state
 * *************************************************************************/
static void new_state(struct CURSWFUNCTION* pcf, uint32_t newstate)
{
	if (pcf->faultcode != pcf->faultcode_prev)
	{
		pcf->faultcode_prev = pcf->faultcode;
		pcf->outstat |= CNCTOUT05KA;	// Queue keep-alive status CAN msg
	}
	pcf->state = newstate;
	return;
}

