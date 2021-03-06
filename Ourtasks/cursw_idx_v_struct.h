/******************************************************************************
* File Name          : cursw_idx_v_struct.h
* Date First Issued  : 03/03/2021
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
#include "iir_filter_lx.h"
#include "CurswTask.h"

#ifndef __CURSW_IDX_V_STRUCT
#define __CURSW_IDX_V_STRUCT

/* Hardware configuration option bit assignments. */
#define AUX1PRESENT   (1 << 0)  // 1 = Contactor #1 has auxilary contacts
#define AUX1SENSE     (1 << 1)  // 1 = Aux I/O bit with Contactor #1 closed
#define AUX2PRESENT   (1 << 2)  // 1 = Contactor #2 has auxilary contacts
#define AUX2SENSE     (1 << 3)  // 1 = Aux I/O bit with Contactor #2 closed
#define ONECURSW  (1 << 4)  // 1 = One contactor; one small pre-charge relay
#define PWMCURSW1 (1 << 5)  // 1 = PWM'ing is used on coil #1
#define PWMCURSW2 (1 << 6)  // 1 = PWM'ing is used on coil #2
#define PWMNOHVSENSOR (1 << 7)  // 1 = No high voltage sensor, (ignore hv readings)

/* High Voltage readings arrive in on uart line. */
#define NUMHV 3       // Number of hv readings
#define	IDXHV1  0 // High voltage reading: battery string side of contactor
#define	IDXHV2  1 // High voltage reading: DMOC+ side of contactor
#define	IDXHV3  2 // High voltage reading: across pre-charge resistor (two contactors)

#define HVSCALE (1 << 17) // High voltage integer scaling for hvcal


/* Calibration parameter, float */
// Use double for F103 to save float->double conversions
struct CNTCTCALHV
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	double dvcal;       // Calibration voltage applied
	uint32_t adchv;     // ADC reading for calibration voltage
	uint32_t offset;    // ADC reading for Sensor zero
};
struct CNTCTCALF
{
	double offset;
	double scale;
};
/* Calibration parameters, scaled integer */
struct CNTCTCALSI
{
	int32_t offset;
	int32_t scale;
};

/* Parameters contactor instance */
struct CURSWLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	double ddiffb4;      // hv1-hv2 voltage difference before closing (volts)

/* NOTE: 
   - all suffix _t parameters are times in milliseconds
   - all voltages are in volts; prefix 'f' designates float */
	
/* Message timings. */
	uint32_t keepalive_t;// keep-alive timeout (timeout delay ms)
	uint32_t hbct1_t;		// Heartbeat ct: ticks between sending msgs hv1:cur1
	uint32_t hbct2_t;		// Heartbeat ct: ticks between sending msgs hv2:cur2
	uint32_t hbct3_t;		// Heartbeat ct: ticks between sending msgs hv3 (if two contactors)

/* Calibrations (offset, scale) */

	// High voltage from uart
	struct CNTCTCALHV calhv[NUMHV]; 

/* Send CAN ids  */
 };

/* *************************************************************************/
void contactor_idx_v_struct_hardcode_params(struct CURSWLC* p);
/* @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
 
#endif

