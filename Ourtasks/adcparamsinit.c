/******************************************************************************
* File Name          : adcparamsinit.c
* Date First Issued  : 03/09/2019
* Board              : DiscoveryF4
* Description        : Initialization of parameters for ADC app configuration
*******************************************************************************/
/* 
This is where hard-coded parameters for the ADC are entered.

Later, this may be replaced with a "copy" of the flat file in high flash, generated
by the java program from the sql database.
*/
#include "adcparamsinit.h"
#include "adcparams.h"
#include "ADCTask.h"
#include "morse.h"

static void ratiometric_cal(struct ADCRATIOMETRIC* p, struct ADCCALHE* plc);

/*                        Min  Typ  Max 
Internal reference F103: 1.16 1.20 1.26 V 
Internal reference F407: 1.18 1.20 1.24 V 
*/
// Define limits for initialization check
#define VREFMIN (1.15)
#define VREFMAX (1.27)

/* *************************************************************************
void adcparamsinit_init(struct ADCFUNCTION* p);
 *	@brief	: Load structs for compensation, calibration and filtering all ADC channels
 * @param	: p = Pointer to struct "everything" for this ADC module
 * *************************************************************************/

/* Reproduced for convenience

#define ADC1IDX_5VOLTSUPPLY   0   // PA0 IN0  - 5V sensor supply
#define ADC1IDX_CURRENTTOTAL  1   // PA5 IN5  - Current sensor: total battery current
#define ADC1IDX_CURRENTMOTOR1 2   // PA6 IN6  - Current sensor: motor #1
#define ADC1IDX_12VRAWSUPPLY  3   // PA7 IN7  - +12 Raw power to board
#define ADC1IDX_INTERNALTEMP  4   // IN17     - Internal temperature sensor
#define ADC1IDX_INTERNALVREF  5   // IN18     - Internal voltage reference

#define ADC1PARAM_COMPTYPE_NONE      0     // No supply or temp compensation applied
#define ADC1PARAM_COMPTYPE_RATIOVDD  1     // Vdd (3.3v nominal) ratiometric
#define ADC1PARAM_COMPTYPE_RATIO5V   2     // 5v ratiometric with 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_RATIO5VNO 3     // 5v ratiometric without 5->Vdd measurement
#define ADC1PARAM_COMPTYPE_VOLTVDD   4     // Vdd (absolute), Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTVDDNO 5     // Vdd (absolute), no Vref compensation applied
#define ADC1PARAM_COMPTYPE_VOLTV5    6     // 5v (absolute), with 5->Vdd measurement applied
#define ADC1PARAM_COMPTYPE_VOLTV5NO  7     // 5v (absolute), without 5->Vdd measurement applied

#define ADC1PARAM_CALIBTYPE_RAW_F  0    // No calibration applied: FLOAT
#define ADC1PARAM_CALIBTYPE_OFSC   1    // Offset & scale (poly ord 0 & 1): FLOAT
#define ADC1PARAM_CALIBTYPE_POLY2  2    // Polynomial 2nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_POLY3  3    // Polynomial 3nd ord: FLOAT
#define ADC1PARAM_CALIBTYPE_RAW_UI 4    // No calibration applied: UNSIGNED INT
*/
double adcdtmp;

void adcparamsinit_init(struct ADCFUNCTION* p)
{
/* Reproduced for convenience 
struct ADCFUNCTION
{
	struct ADCCONTACTORLC lc;    // Local Copy of parameters
	struct ADCINTERNAL    intern;// Vref & temperature
	struct ADCABSOLUTE    v12;   // Supply: raw 12v
	struct ADCABSOLUTE    v5;    // Supply: regulated 5v
	struct ADCRATIOMETRIC cur1;  // Current sensor #1
   struct ADCRATIOMETRIC cur2;  // Current sensor #2
	struct ADCCHANNEL	 chan[ADC1IDX_ADCSCANSIZE]; // ADC sums, calibrated endpt
	uint32_t ctr; // Running count of updates.
};
struct ADCINTERNAL
{
	struct IIRFILTERL iiradcvref; // Intermediate filter params: vref 
	struct IIRFILTERL iiradctemp; // Intermediate filter params: temperature sensor

	uint32_t adcfilvref;  // Filtered ADC[Vref]
	uint32_t adcfiltemp;  // Filtered ADC[temperature]

	uint32_t adcvref;    // Do I need this?
	uint32_t adccmpvref; // scaled vref compensated for temperature

	double dvref;        // (double) vref computed from calibration params
	uint32_t vref;       // (scaled) vref computed from calibration params

	uint32_t iRslope;    // (scaled) Reciprocal of temperature sensor slope
	uint32_t iv25s;      // (scaled) (V25 * iRslope)
	double   V25;        // (double) Computed V25 (no)
	uint32_t vrefRs;     // (scaled) Vref / slope
	uint32_t irmtemp;    // (scaled) calibration temperature
	uint32_t itemp;      // (scaled) temperature (degC)
};
*/

/* Internal sensors. */
	// Pointers to filter constants 
	p->intern.iiradcvref.pprm = &p->lc.calintern.iiradcvref;
	p->intern.iiradctemp.pprm = &p->lc.calintern.iiradctemp;

	// Compute a scaled integer vref from measurements
	double dadc  = p->lc.calintern.adcvdd; // ADC reading (~27360)
	p->intern.dvref = p->lc.calintern.dvdd * (dadc / 65520.0);
	p->intern.vref  = (p->intern.dvref * (1 << ADCSCALEbits) ); // Scaled uint32_t; 
	p->intern.adcvref = (65520.0 * p->intern.dvref) / p->lc.calintern.dvdd;

	// Check for out-of-datasheet Vref spec 
	if ((p->intern.dvref < (VREFMIN)) || (p->intern.dvref > (VREFMAX))) 
	{
		morse_trap(81);
	}
	p->chan[ADC1IDX_INTERNALTEMP].dscale = p->lc.calintern.dvdd / 65520.0;
	p->chan[ADC1IDX_INTERNALVREF].dscale = 1.0;

	// Reciprocal of temperature sensor slope ( ~65536/4.3E-3 = (232.55 << 16) )
	p->intern.iRslope  = (double)((1 << ADCSCALEbits) * (1000)) / p->lc.calintern.dslope;

	// Pre-compute the (V25 / slope) into a scaled integer.
//	double dtmp = ((1 << ADCSCALEbits) * (1000)) * p->lc.calintern.dvtemp / p->lc.calintern.dslope;
//	p->intern.iv25s = dtmp;
//adcdtmp = dtmp;

	// Pre-compute (Vref / slope) to scaled int (~65536*1.20/4.3E-3 = (279.07 << 16) = 18289116)
//	dtmp = ((1 << ADCSCALEbits) * (1000)) * p->intern.dvref / p->lc.calintern.dslope;
//	p->intern.vrefRs = dtmp;
//adcdtmp = dtmp;

	// Room temp calibration offset (7/17/19) Is this needed?)
	p->intern.irmtemp = ((double)(1 << ADCSCALEbits) * (double)(p->lc.calintern.drmtemp));// / p->lc.calintern.adcrmtmp);
	
	p->intern.yRs = ((double)((1 << ADCSCALEbitsy) * 1000) / p->lc.calintern.dslope);

	p->intern.iv25 = (uint32_t)((double)(1 << ADCSCALEbitsy) * p->lc.calintern.dvtemp);

	p->intern.dx25 = p->lc.calintern.dvtemp * (1.0/4.3E-3);

	p->intern.dxdvref = p->intern.dvref * (1.0/4.3E-3);

/* Reproduced for convenience
struct ADCABSOLUTE
{
	struct IIRFILTERL iir;// Intermediate filter params
	double dscale;        // Computed from measurements
	uint32_t adcfil;      // Filtered ADC reading
	uint32_t ival;        // scaled int computed value (not divider scaled)
}; */	

/* Absolute: 12v supply. */
	p->v12.iir.pprm = &p->lc.cal_12v.iir; // Filter param pointer
	p->v12.k   = (p->lc.cal_12v.dvn / p->intern.dvref) * (dadc / p->lc.cal_12v.adcvn);
	p->v12.dscale = p->v12.k * p->intern.dvref;
	p->chan[ADC1IDX_12VRAWSUPPLY].dscale = p->v12.k;

/* Absolute:  5v supply. */
	p->v5.iir.pprm = &p->lc.cal_5v.iir; // Filter param pointer
	p->v5.k   = (p->lc.cal_5v.dvn / p->intern.dvref) * (dadc / p->lc.cal_5v.adcvn);
	p->v5.dscale = p->v5.k * p->intern.dvref;
	p->chan[ADC1IDX_5VOLTSUPPLY].dscale = p->v5.dscale;

/* Ratiometric: battery string current. */
	ratiometric_cal(&p->cur1, &p->lc.cal_cur1);
	p->chan[ADC1IDX_CURRENTTOTAL].dscale = p->cur1.dscale; // For convenient access

/* Ratiometric: spare current. */
	ratiometric_cal(&p->cur2, &p->lc.cal_cur2);
	p->chan[ADC1IDX_CURRENTMOTOR].dscale = p->cur2.dscale; // For convenient access

	return;
}
/* *************************************************************************
static int16_t void ratiometric_cal_zero(struct ADCFUNCTION* p, struct ADCRATIOMETRIC* pcur, uint16_t idx);
 *	@brief	: Adjust no-current ratio for a Hall-effect sensor
 * @param	: p = Pointer to struct "everything" for this ADC module
 * @param	: pcur = Pointer to struct with values for the ratiometric sensor
 * @param	: idx = index into ADC sum array for the sensor measurement
 * @return	: 0 = no fault; -1 = out of tolerance
 * *************************************************************************/
static int16_t ratiometric_cal_zero(struct ADCFUNCTION* p, struct ADCRATIOMETRIC* pcur, uint16_t idx)
{
	double dtmp;

	// Check that re-zero'ing is not some crazy value
	dtmp  = ((double)p->chan[idx].sum / (double)p->chan[ADC1IDX_5VOLTSUPPLY].sum) ;
	if ( (dtmp > (pcur->drko * (1+ZTOLERANCE))) || (dtmp < (pcur->drko * (1-ZTOLERANCE))) )
	{
		return -1;
	}
	else
	{ // Here adjustment is considered reasonable.
		pcur->drko = dtmp;
		pcur->irko  =dtmp * (1 << ADCSCALEbits);
	}
	return 0;
}

/* *************************************************************************
int16_t ratiometric_cal_zero_CURRENTTOTAL(struct ADCFUNCTION* p);
int16_t ratiometric_cal_zero_CURRENTMOTOR(struct ADCFUNCTION* p);
 *	@brief	: Adjust no-current ratio for Hall-effect sensors
 * @param	: p = Pointer to struct "everything" for this ADC module
 * @return	: 0 = no fault; -1 = out of tolerance
 * *************************************************************************/
int16_t ratiometric_cal_zero_CURRENTTOTAL(struct ADCFUNCTION* p)
{
	return ratiometric_cal_zero(p, &p->cur1, ADC1IDX_CURRENTTOTAL);
}
int16_t ratiometric_cal_zero_CURRENTMOTOR(struct ADCFUNCTION* p)
{
	return ratiometric_cal_zero(p, &p->cur1, ADC1IDX_CURRENTMOTOR);
}

/* *************************************************************************
static void ratiometric_cal(struct ADCRATIOMETRIC* p, struct ADCCALHE* plc);
 *	@brief	: Compute calibration constants for ratiometric sensor
 * @param	: p = points to struct with computed results
 * @param	: plc = points to parameter struct for this sensor
 * *************************************************************************/
static void ratiometric_cal(struct ADCRATIOMETRIC* p, struct ADCCALHE* plc)
{
/* Reproduced for convenience
struct ADCRATIOMETRIC
{
	struct IIRFILTERL iir;    // Intermediate filter params
	double drko;      // Offset ratio: double (~0.5)
	double dscale;    // Scale factor
	uint32_t adcfil;  // Filtered ADC reading
	int32_t irko;     // Offset ratio: scale int (~32768)
	int32_t iI;       // integer result w offset, not final scaling
}; */

	p->iir.pprm = &plc->iir; // Filter param pointer
	
	// Sensor connected, no current -> offset ratio (~ 0.50)
	p->drko  = ((double)plc->zeroadcve / (double)plc->zeroadc5) ;
	p->irko  = (p->drko * (1 << ADCSCALEbits) );

	// Sensor connected, test current applied with maybe more than one turn through sensor
	// dscale = amp-turns / ((calibration ADC ratio - offset ratio) * divider ratio);
	double dtmp = ( (double)plc->caladcve / (double)plc->zeroadc5 ) - p->drko ;
	p->dscale = plc->dcalcur / dtmp;

	return;
}
