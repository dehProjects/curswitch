/******************************************************************************
* File Name          : cursw_idx_v_struct.c
* Date First Issued  : 03/03/2021
* Board              :
* Description        : Load parameter struct 
*******************************************************************************/

#include "cursw_idx_v_struct.h"
#include "SerialTaskReceive.h"

/* *************************************************************************
 * void cursw_idx_v_struct_hardcode_params(struct struct CURSWLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void cursw_idx_v_struct_hardcode_params(struct CURSWLC* p)
{
	p->size       = 47;
	p->crc        = 0;   // TBD
   p->version    = 1;   // 

	
	// Battery_minus-to-cursw #1
	p->calhv[IDXHV1].iir.k     = 3;
	p->calhv[IDXHV1].iir.scale = 2;
 	p->calhv[IDXHV1].dvcal  = 13.93; // Applied voltage
	p->calhv[IDXHV1].adchv  = 1396;  // ADC reading (received from uart)
	p->calhv[IDXHV1].offset =  3;    // ADC reading zero volts

	// Battery_minus-to-cursw #1 DMOC_plus
	p->calhv[IDXHV2].iir.k     = 3;
	p->calhv[IDXHV2].iir.scale = 2;
 	p->calhv[IDXHV2].dvcal  = 13.93; // Applied voltage
	p->calhv[IDXHV2].adchv  = 1442;  // ADC reading (received from uart)
	p->calhv[IDXHV2].offset =  7;    // ADC reading zero volts

	// Battery_minus-to-cursw #1 DMOC_minus
	p->calhv[IDXHV3].iir.k     = 3;
	p->calhv[IDXHV3].iir.scale = 2;
 	p->calhv[IDXHV3].dvcal  = 13.93; // Applied voltage
	p->calhv[IDXHV3].adchv  = 1436;  // ADC reading (received from uart)
	p->calhv[IDXHV3].offset =  3;    // ADC reading zero volts

	return;
}
