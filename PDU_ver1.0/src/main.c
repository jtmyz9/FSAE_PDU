/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */



#include <asf.h>	
#include "PDU.h"

/*
* Global Variable Declarations
*/
uint32_t ul_sysclk = 0;

volatile ioport_port_mask_t	enable_mask = PDU_DEFAULT_MASK;
volatile ioport_port_mask_t	soft_restart_mask = NULL_MASK;
volatile ioport_port_mask_t	error_mask = NULL_MASK;


volatile PDU_state_t			PDU = {
	0,0,0,0,0
	};

vehicle_data_t		vehicle = {0,0,0,0};

output_data_t		outputs[MAX_PDU_CHANNEL];

volatile can_receive_status		g_ul_recv_status    = no_receive;
volatile can_transmit_status	g_tx_status			= tx_request;
volatile bool					update_outputs		= false;
/**Time in ms, since last received message from ECU*/
volatile uint32_t g_recv_timeout_cnt = 0;



int main (void)
{	
	/*
	* init system clock based on clock settings in conf_clock.h
	* if using external XTAL, settings are defined in user_board.h
	*/
	sysclk_init();
	ul_sysclk = sysclk_get_cpu_hz();
	
	/*
	* Initialize the board peripherials
	* Enables the peripherial clocks for all required peripherials
	* Which clocks to enable are currently hard coded in init()
	*/
	board_init();
		
	/*
	* Initialize the PDU, and and system parameters required
	*/
	init_PDU();	
	
	while(1){

		if(g_tx_status == tx_request)	PDU_transmit_callback();		
		
		if( update_outputs ){ 
			update_output_status();
			update_outputs = false;
		}
		
		
	}
}

void TC_100HZ(void){
	tc_get_status(PDU_TC0_MODULE, TC_100HZ_CHANNEL); 
	
	
	g_recv_timeout_cnt++;		//increment timeout value
	
	//this has to be here so that is an interrupt and works
	if(g_recv_timeout_cnt > PDU_CAN_DIAGNOSTIC_DELAY){
		g_ul_recv_status = rx_timeout;
		ECU_timeout_callback();
	}	
	
	tc_start(PDU_TC0_MODULE, TC_100HZ_CHANNEL);	
}

void TC_10HZ(void){
	tc_get_status(PDU_TC0_MODULE, TC_10HZ_CHANNEL);
	
	g_tx_status = tx_request;
	update_outputs = true;
	
	tc_start(PDU_TC0_MODULE, TC_10HZ_CHANNEL);
}


void TC_1HZ(void){
	tc_get_status(PDU_TC0_MODULE, TC_1HZ_CHANNEL);
	
	/*
	*	run software restart routine on selected channels
	*	Only run if there are channels that have encountered
	*	faults
	*/
	if ( soft_restart_mask > 0) soft_restart();
	
	//just shut everything down
	if ( PDU.chip_temp > PDU_OVERTEMP ){
		set_enable(PDU_ENABLES_PORTD_MASK, PDU_OFF_STATE);
		__disable_irq();
		while(1);
	}
	
	tc_start(PDU_TC0_MODULE, TC_1HZ_CHANNEL);
}