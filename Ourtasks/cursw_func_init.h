/******************************************************************************
* File Name          : cursw_func_init.h
* Date First Issued  : 03/03/2021
* Description        : 
*******************************************************************************/

#ifndef __CURSWFUNCINIT
#define __CURSWFUNCINIT

#include "iir_filter_lx.h"
#include "adcparams.h"

/* *************************************************************************/
void contactor_func_init_init(struct CURSWFUNCTION* p, struct ADCFUNCTION* padc);
/*	@brief	: Initialize working struct for ContactorTask
 * @param	: p    = pointer to ContactorTask
 * @param	: padc = pointer to ADC working struct
 * *************************************************************************/
void contactor_func_init_canfilter(struct CURSWFUNCTION* p);
/*	@brief	: Setup CAN hardware filter with CAN addresses to receive
 * @param	: p    = pointer to ContactorTask
 * *************************************************************************/

#endif

