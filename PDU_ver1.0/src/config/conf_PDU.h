/*
 * conf_PDU.h
 *
 * Created: 11/25/2017 6:10:15 PM
 *  Author: MizzouRacing
 */ 


#ifndef CONF_PDU_H_
#define CONF_PDU_H_

#include <asf.h>	

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

//Assert((NUM_PDU_CHANNEL < MAX_PDU_CHANNEL));

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
#define FAN_IS_MAX				25 
#define IGNITION_IS_MAX			10
#define PUMP_IS_MAX				15
#define ECU_IS_MAX				10
#define BRAKE_IS_MAX			2
#define HORN_IS_MAX				13
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
	};
	
#define AFEC_conversion_factor				3300UL / (4095UL * 16)			//conversion factor for afec result to mV
#define PDU_SENSE_MV_TO_MA					220							//this is value of resistor transforming current feedback into voltage
#define PDU_FET_DIFFERENTIAL_RATIO			13							//define in BTS50060 Datasheet	
#define SCALE_FACTOR						10							//factor to multiple Amperage value by before stuffing into uint8 val
#define IS_FAULT_MIN						1320						//min number of mV that indicates fault condition
#define PDU_OVERTEMP						75							//75C, considering this value as too high of a temperature for the pdu to continue operation

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
/*           CAN BUS Definitions for PDU                                */
/************************************************************************/
#define PDU_CAN_SEL							0				//select CAN0 or CAN1 for preprocessor statements involving which can bus
#define PDU_CAN								CAN0
#define PDU_CAN_BAUD						CAN_BPS_1000K
#define PDU_CAN_DIAGNOSTIC_DELAY			50				//number of millisec until ECU can timeout
/*
 *	this is slightly higher than warning limit but below error passive limit for tx error cnt
 *	This makes it so that we don't keep waiting for a message to transmit
 *	in loops where we need contents of mailbox x to transmit before loading with next message
 */
#define PDU_CAN_TX_ERROR_CNT				100				

typedef enum{
	standard,
	extended
	}CAN_BUS_ID_TYPE;

/** CAN Bus Addresses */
#define PDU_ADDRES_TYPE						standard
#define PDU_ECU_REC_ADDRESS					0x11A			//this is hard coded into ECU
#define PDU_ECU_ENG_DATA_ADD				0x118
#define PDU_ECU_VEHICLE_DATA_ADD			0x119
#define PDU_BASE_TX_ADDRESS					0x500			//configured in ECU software


/** Specific definitions for CAN receive                                */
#define PDU_RECEIVE_MB						2
#define PDU_CAN_RECEIVE_IE_MASK				(CAN_IER_MB1 | CAN_IER_MB0)			//mask for interrupts to enable
#define PDU_ECU_GEN_MSG_ID_MASK				CAN_MAM_MIDvA_Msk - 1;


#define	PDU_ALL_ERR_MASK					(CAN_SR_CERR | CAN_SR_SERR | CAN_SR_AERR |CAN_SR_FERR | CAN_SR_BERR)
#define PDU_CAN_IE_MASK						(PDU_CAN_RECEIVE_IE_MASK | CAN_IER_BOFF)

/** Definitions for CAN TX												*/
#define PDU_TX_MB							5
#define PDU_TX_STATUS_CNT					3

#define PDU_TX_FIRST_MB						2

#define PDU_STATUS_TX_MB					PDU_TX_FIRST_MB
#define PDU_OUTPUT_TX_MB					PDU_TX_FIRST_MB + 4
#define PDU_AMP_TX_MB						PDU_TX_FIRST_MB + 1
/** TODO: MAke this less hardcoded: find way to build this mask bases on how many TX messags */
#define PDU_TX_MB_MASK						( CAN_TCR_MB3 | CAN_TCR_MB4 | CAN_TCR_MB5 | CAN_TCR_MB6 | CAN_TCR_MB7 )

/*
* "Magic Numbers" for CAN data
* These are constant values used when encoding/decoding ECU can messages
* Some values are scales, others are for unit conversions
*/

#define ECU_RPM_SCALE						600


/*
* Values to be or'd with Byte 0 to set compound ID
* Detailed in Excel sheet
*/
typedef enum{
	state_zero,
	state_one		= 1 << 4,
	state_two		= 2 << 4,
	state_three		= 3 << 4
	}PDU_state_compound_id;
	
static const PDU_state_compound_id state_id[PDU_TX_MB]= {
	state_zero,
	state_one,
	state_two,
	state_three
};
	
typedef enum{
	output_zero,
	output_one		= 1 << 6,
	output_two		= 2 << 6,
	output_three	= 3 << 6
}PDU_output_compound_id;


/** CAN frame max data length: helpful def */
#define MAX_CAN_FRAME_DATA_LEN				8

//Alias for CAN0_Handler for PDU
#define	ECU_RECEIVE							CAN0_Handler

typedef enum{
	no_receive, 
	rx_good,
	rx_timeout,
	rx_fault
	}can_receive_status;
	
typedef enum{
	tx_request,
	tx_good,
	tx_timeout,
	tx_fault
}can_transmit_status;


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