/*
 * conf_PDU.h
 *
 * Created: 11/25/2017 6:10:15 PM
 *  Author: MizzouRacing
 */ 


#ifndef CONF_PDU_H_
#define CONF_PDU_H_

#include <asf.h>	
#include "../CAN/PDU_CAN_conf.h"

/*
* Bit position for individual enable channels
* Each value is the offset from the zero position for each channel
*/

typedef enum enable_index	{
	FAN_MASK = 0	,		
	IGNITION_MASK	,
	PUMP_MASK		,
	ECU_MASK		,	
	BRAKE_MASK		,	
	HORN_MASK		,	
	DATA_MASK		,	
	GEN_5V_MASK		,		
	GEN_12V_MASK	,	
	WAWA_PUMP_MASK	,		
	NUM_PDU_CHANNEL  //total number of PDM channels
	}				pdu_enable_index;

/*
 *	This is the maximum value that we can currently 
 *	support with the hardware/software setups
 *	The MoTeC ECU can only control up to a max of 32 channels
 *	Thus without some extra mucking about we cannot go over this limit
 *	To go over this limit we would need to modify software to handle 
 *	multiple ECU request
 */	
#define MAX_PDU_CHANNEL 32


/*
* IOPORT Definitions for PDU enables
* Assumes all channels exist on PORTD from PORTD0 to PORTD(NUM_PDM_CHANNEL)
* Individual Definitions for each channels name and offset in PORT
*/
#define	PDU_ENABLE_PORT			IOPORT_PIOD
#define PDU_OFF_STATE			IOPORT_PIN_LEVEL_LOW
#define PDU_ON_STATE			IOPORT_PIN_LEVEL_HIGH													
#define PDU_PORTD_CHANNEL_0		0
#define PDU_ENABLES_PORTD_MASK	((1<<NUM_PDU_CHANNEL) - 1	) << PDU_PORTD_CHANNEL_0	//generate bitmask for PORTD

/*
*---------------DEFAULT INIT VAL ------------
* The default value to be loaded into ioport for enabled channels
* This determines which channels will be enabled at startup and in 
* complicated error recovery modes defined later
*/
#define PDU_DEFAULT_MASK		((1<<ECU_MASK) | (1<<DATA_MASK) | (1<<GEN_5V_MASK) | (1<<GEN_12V_MASK))
#define PDU_TIMEOUT_MASK		((1<<ECU_MASK) | (1<<DATA_MASK))
//At a later date might be able to remove ECU_MASK from this list, make a "run" relay in ECU that toggles its own PDU bit
#define PDU_NON_ENGINE_MASK		((1<<ECU_MASK) | (1<<DATA_MASK) | (1<<GEN_5V_MASK) | (1<<GEN_12V_MASK) )
//These channels are configured to be "restarted" if they encounter a fault or over current
#define CONF_SOFT_RESTART_MASK	( (1<<ECU_MASK) | (1<<FAN_MASK) | (1<<PUMP_MASK) | (1<<WAWA_PUMP_MASK) )
#define NULL_MASK				0

/*
* Config for Current limits for each channel
* These values are derived from bench testing devices
* and some from manufacturer datasheets, these represent the
* max current we expect a device to pull during normal operation
* NOTE: These are listed as the ceiling of what we expect
*/
#define FAN_IS_MAX				26
#define IGNITION_IS_MAX			10
#define PUMP_IS_MAX				15
#define ECU_IS_MAX				12
#define BRAKE_IS_MAX			2
#define HORN_IS_MAX				6
#define DATA_IS_MAX				6
#define GEN_5V__IS_MAX			3
#define GEN_12V_IS_MAX			5
#define WAWA_PUMP_IS_MAX		15

static const uint32_t curr_limits[NUM_PDU_CHANNEL] = {
	FAN_IS_MAX, 
	IGNITION_IS_MAX, 
	PUMP_IS_MAX,
	ECU_IS_MAX,
	BRAKE_IS_MAX,
	HORN_IS_MAX,
	DATA_IS_MAX,
	GEN_5V__IS_MAX,
	GEN_12V_IS_MAX,
	WAWA_PUMP_IS_MAX
	
	};


/*
* AFEC Symbolic Names for each channel
* All channels are current connected to AFEC0
*/
#define IS_AFEC					AFEC0
#define FAN_IS_CH				AFEC_CHANNEL_0
#define IGNITION_IS_CH			AFEC_CHANNEL_2
#define PUMP_IS_CH				AFEC_CHANNEL_1
#define ECU_IS_CH				AFEC_CHANNEL_3
#define BRAKE_IS_CH				AFEC_CHANNEL_4
#define HORN_IS_CH				AFEC_CHANNEL_5
#define DATA_IS_CH				AFEC_CHANNEL_10
#define GEN_5V__IS_CH			AFEC_CHANNEL_9
#define GEN_12V_IS_CH			AFEC_CHANNEL_6
#define AUX2_IS_CH				AFEC_CHANNEL_11


/*
* List of the AFEC channels used by the PDU
* This will be used to sequence ADC reads 
* Where each listed AFEC channel will be read in sequentially
* when a AFEC conversion is started
*/
static const enum afec_channel_num PDU_AFEC_channel_list[NUM_PDU_CHANNEL] = {
	FAN_IS_CH,		
	IGNITION_IS_CH,	
	PUMP_IS_CH,	
	ECU_IS_CH,	
	BRAKE_IS_CH,		
	HORN_IS_CH,	
	DATA_IS_CH,	
	GEN_5V__IS_CH,	
	GEN_12V_IS_CH,	
	AUX2_IS_CH	
	};
	
/*
* Const Array of static offsets taken by each powerfet Is
* The Fets all have a typical offset value that we could use
* But due to variances in soldering and PCB traces, all are just 
* different enough to warrant an array of values gleaned from 
* bench test in debug mode and are all raw ADC values
*/
static const uint32_t PDU_AFEC_channel_offset[NUM_PDU_CHANNEL] = {
	715,720,730,740,690,660,660,850,710,630								//why the fuck didnt i make these mV instead raw ADC vals????
	}; //this shit is why I should have put NVSRAM on board
	
#define AFEC_conversion_factor				3300UL / (4095UL * 16)			//conversion factor for afec result to mV
#define PDU_SENSE_MV_TO_MA					220							//this is value of resistor transforming current feedback into voltage
#define PDU_FET_DIFFERENTIAL_RATIO			13							//define in BTS50060 Datasheet	
#define SCALE_FACTOR						5							//factor to multiple Amperage value by before stuffing into uint8 val
#define IS_FAULT_MIN						1320						//min number of mV that indicates fault condition
#define PDU_OVERTEMP						75							//75C, considering this value as too high of a temperature for the pdu to continue operation

#define INRUSH_AFEC_DELAY					20							//number of samples to delay fuse tripping by, to ride through inrush currents
#define OVERCURRENT_REJECTION_DELAY			5							//number of quantum that channel should be overcurrent before tripping fault
#define PRECHARGE_ATTEMPTS					10							//number of times to cycle powerfet to charge capacitors on bus
#define PRECHARGE_TIME						50							//ms to leave bus energized to attempt precharge
#define EFUSE_LATCH_RELEASE_T				85							//ms to release short circuit/fault latch

/************************************************************************/
/* Channel Status Codes                                                 */
/************************************************************************/

typedef enum {
	CHANNEL_OFF				= 0,
	CHANNEL_ON				= 1,
	CHANNEL_OVER_CURRENT	= 2,
	CHANNEL_FAULT			= 4
}status_code;
	
	



/************************************************************************/
/*           Timer Definitions for PDU                                */
/************************************************************************/

#define PDU_TC0_MODULE						TC0
#define PDU_TC1_MODULE						TC1

#define PDU_TC_COMPARE_INT					TC_IER_CPCS

#define TC_100HZ							TC0_Handler
#define TC_10HZ								TC1_Handler
#define TC_1HZ								TC2_Handler

typedef enum{
	TC_100HZ_CHANNEL	=	0,
	TC_10HZ_CHANNEL		=	1,
	TC_1HZ_CHANNEL		=	2,
	TC_NUM_CHANNEL
	}PDU_TC0_INDEX;
	
static const uint32_t PDU_TC_DESIRED_HZ[TC_NUM_CHANNEL] = {
	100,
	10,
	1,
	};

static const IRQn_Type PDU_TC_IRQ[TC_NUM_CHANNEL] = {
	TC0_IRQn,
	TC1_IRQn,
	TC2_IRQn,
	//TC3_IRQn,
	//TC4_IRQn,
	//TC5_IRQn,
	//TC6_IRQn,
	//TC7_IRQn,
	//TC8_IRQn
};

#endif /* CONF_PDU_H_ */