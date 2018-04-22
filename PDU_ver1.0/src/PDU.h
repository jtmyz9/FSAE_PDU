/*
 * PDU.h
 *
 * Created: 11/26/2017 5:40:22 PM
 *  Author: MizzouRacing
 */ 


#ifndef PDU_H_
#define PDU_H_

#include <asf.h>	
#include "conf_PDU.h"

typedef struct PDM_state{
	int16_t batt_volt;
	uint8_t chip_temp;	
	uint8_t error_flag;
	uint8_t total_curr;
	uint8_t rail_voltage_3_3;
}PDU_state_t;

typedef struct vehicle_data{
	uint32_t RPM;
	uint16_t coolant_temp;
	uint8_t  fuel_pressure;
	uint8_t  engine_state;
}vehicle_data_t;

typedef struct output_data{
	uint8_t current;
	uint8_t state;
	uint8_t last_curr;
	uint32_t inrush_delay;
}output_data_t;

#ifndef PDU_ENABLE_PORT
	#error "PDM_ENABLE_PORT not defined"
#endif



/************************************************************************/
/* PDU Variables */
/************************************************************************/

/* 
* Mask of channels that should currently be enabled
* Inital value supplied by conf_PDU header
* Running state value retreive from ECU via CAN message
*/
volatile extern ioport_port_mask_t	enable_mask;
volatile extern	ioport_port_mask_t	soft_restart_mask;
volatile extern	ioport_port_mask_t	error_mask;

volatile extern PDU_state_t			PDU;

extern	vehicle_data_t		vehicle;

extern output_data_t		outputs[MAX_PDU_CHANNEL];

extern uint32_t				ul_sysclk;

/*
* CAN Transfer mailbox structure 
* Array of structs to be used for all the mailboxes in CAN controller
*/
can_mb_conf_t can_mailbox[CANMB_NUMBER];

/*
 *	Basically all the following status variables act as bad 
 *	implementation of mutexes, this is meant to be quick and dirty
 *	to get PDU running and in car for testing and possibly for first couple months
 *	until have time to make work correctly with RTOS threads
 */

/** TX Status      */
extern volatile can_transmit_status g_tx_status;

/** Receive status */
extern volatile can_receive_status g_ul_recv_status;

extern volatile bool				update_outputs;

/**Time in ms, since last received message from ECU*/
extern volatile uint32_t g_recv_timeout_cnt;


/************************************************************************/
/* PDU AFEC Variables                                                   */
/************************************************************************/
struct afec_config afec_cfg;
struct afec_ch_config afec_ch_cfg;

/*
* Set the supplied mask channels to supply ioport val
* Mask: is bit mask of channels 
* enable:	is the port value to write to masked channels
* Used to either enable or disable supplied channels
*/
void set_enable(ioport_port_mask_t mask, enum ioport_value enable);

/*
* Set all channels in the PDU to off state
* Used during intializaiton to ensure low states are asserted
* Also in error state where all outputs must be disabled
*/
void all_off(void);


uint8_t get_is( Afec *const afec ,enum afec_channel_num pdu_channel, uint32_t offset);


void update_output_status(void);

void get_chip_temp(void);

/*
 *	Prod: soft_restart
 *	Will enact a software restart of channels
 *	will restart all the channels in the provided mask
 */
void soft_restart(void);

/*
* Initialize the PDU
* Setup the I/O port used for enabling FET's
*		I/O Port is defined as PDU_ENABLE_PORT in config header
* Setup the ADC for reading the current values from each FET
*/
void init_PDU(void);

void handle_CAN(can_mb_conf_t *mailbox);
void PDU_transmit_callback(void);
void ECU_timeout_callback(void);



#endif /* PDU_H_ */