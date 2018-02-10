/*
 * PDU.c
 *
 * Created: 11/26/2017 5:50:23 PM
 *  Author: MizzouRacing
 */ 


#include "PDU.h"

/*
*	Array of Channel Mappings
*	Used by PDU functions to read all channels
*	The AVR 
*/
const uint32_t PDU_ALL_CHANNELS[] = {	FAN_MASK, IGNITION_MASK,
										PUMP_MASK, ECU_MASK, BRAKE_MASK, HORN_MASK,
										DATA_MASK,GEN_5V_MASK, GEN_12V_MASK,WAWA_PUMP_MASK		};

//TODO: Establish compile time assert to ensure all channels are used

//int size = sizeof(PDU_ALL_CHANNELS);
//assert(size, NUM_PDU_CHANNEL );
																				

void init_PDU(void){
	//setup the port for the PDU
	ioport_enable_port(PDU_ENABLE_PORT, PDU_ENABLES_PORTD_MASK);
	//set all the pins in the PORT MASK as outputs
	ioport_set_port_dir(PDU_ENABLE_PORT, PDU_ENABLES_PORTD_MASK, IOPORT_DIR_OUTPUT);
	
	//enable the default channels (ECU, etc)
	enable_mask = PDU_DEFAULT_MASK;
	set_enable(enable_mask, PDU_ON_STATE);
	
	/*
	* Initialize the CAN Bus for the PDU
	* Setup mailbox structs and set required interrupts
	*/
	//NOTE: CAN peripherial must first be initialized in board init()
	can_init(PDU_CAN, ul_sysclk, PDU_CAN_BAUD);
		
	/* Disable all CAN interrupts. */
	can_disable_interrupt(PDU_CAN, CAN_DISABLE_ALL_INTERRUPT_MASK);
	//can_disable_interrupt(CAN1, CAN_DISABLE_ALL_INTERRUPT_MASK);
	
	
	can_enable_autobaud_listen_mode(PDU_CAN);
	while( ( can_get_status(PDU_CAN) & PDU_ALL_ERR_MASK ) );
	can_disable_autobaud_listen_mode(PDU_CAN);
	
	/* 
	* Enable the CAN(N) interrupt in the Nested vector interrupt controller
	* Sets the correct enabling bit(s) in the interrupt controller
	* Links the CAN(N)_handler to any interrupts from the CAN peripheral
	*/
	NVIC_EnableIRQ(CAN0_IRQn);
	//NVIC_EnableIRQ(CAN1_IRQn);
	
	//make sure all mailbox structs are zeroed
	for( int i =0; i < CANMB_NUMBER; i ++){
		reset_mailbox_conf(&can_mailbox[i]);
	}
	
	//the first mailbox is always the ECU receive mailbox
	can_mailbox[0].ul_mb_idx = 0;
	can_mailbox[0].uc_obj_type = CAN_MB_RX_MODE;
	can_mailbox[0].ul_id_msk = CAN_MAM_MIDvA_Msk;		//this masks all id bits
	can_mailbox[0].ul_id = CAN_MID_MIDvA(PDU_ECU_REC_ADDRESS);
	can_mailbox_init(PDU_CAN, &can_mailbox[0]);

	/* second mailbox is always the ECU receive mailbox for vehicle data */
	can_mailbox[1].ul_mb_idx = 1;
	can_mailbox[1].uc_obj_type = CAN_MB_RX_MODE;
	can_mailbox[1].ul_id_msk = PDU_ECU_GEN_MSG_ID_MASK;		//this will allow all generic messages from ECU
	can_mailbox[1].ul_id = CAN_MID_MIDvA(PDU_ECU_ENG_DATA_ADD);
	can_mailbox_init(PDU_CAN, &can_mailbox[1]);
	
	//Enable mailbox interrupts in receive mailboxes
	can_enable_interrupt( CAN0, PDU_CAN_IE_MASK );
	
	/** Init CAN bus TX mailboxes */
	for( uint8_t i = PDU_TX_FIRST_MB; i < PDU_TX_MB + PDU_TX_FIRST_MB; i++){
		can_mailbox[i].ul_mb_idx = i;
		can_mailbox[i].uc_obj_type = CAN_MB_TX_MODE;
		can_mailbox[i].ul_id_msk = 0;
		can_mailbox[i].ul_id = CAN_MID_MIDvA(PDU_BASE_TX_ADDRESS + (i - PDU_TX_FIRST_MB));
		can_mailbox_init(PDU_CAN, &can_mailbox[i]);
	}
	
	
	/*
	* AFEC Enable  
	*/
	afec_enable(IS_AFEC);
	afec_get_config_defaults(&afec_cfg);	
	afec_cfg.resolution = AFEC_16_BITS;
	afec_cfg.anach = false;
	afec_init(IS_AFEC, &afec_cfg);
	afec_set_trigger(IS_AFEC, AFEC_TRIG_SW);
	
	//get default config for a channel
	afec_ch_get_config_defaults(&afec_ch_cfg);
	for(uint8_t i = 0; i < NUM_PDU_CHANNEL; i++){
		//set the config for the AFEC channel
		afec_ch_set_config(IS_AFEC, PDU_AFEC_channel_list[i], &afec_ch_cfg);
		//move offset to measure voltage from 0->Vref
		afec_channel_set_analog_offset(IS_AFEC, PDU_AFEC_channel_list[i],0x800);
		//enable the channel
		afec_channel_enable(IS_AFEC, PDU_AFEC_channel_list[i]);
	}
	
	/*
	 *	Use AFEC0 here because the temperature sensor is hardwired to AFEC0
	 */
	afec_ch_set_config(AFEC0, AFEC_TEMPERATURE_SENSOR, &afec_ch_cfg);	
	/*
	 * Because the internal ADC offset is 0x800, it should cancel it and shift
	 * down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_TEMPERATURE_SENSOR, 0x800);
	
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_cfg.rctc = false;
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);
	afec_channel_enable(AFEC0, AFEC_TEMPERATURE_SENSOR);
	
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_15,	get_chip_temp, 1);
	
	/*
	* Timer Counter Init
	* Following local variables( counts, ul_div, ul tccclks)
	* are used while calculating needed div and clock source for desired clock 
	* speed for this timer counter channel, where count is timer compare count register val
	* ul_div is the divider for Timer clock and ul_tcclks is clock source
	*/
	uint32_t counts;
	uint32_t ul_div;
	uint32_t ul_tcclks;
	
	/*
	* loop through all desired counter frequencies
	* each TC module has 3 channels for total of 9 channels in uC
	*/
	for(uint8_t i = 0; i < TC_NUM_CHANNEL ; i++){
		tc_find_mck_divisor(PDU_TC_DESIRED_HZ[i], ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
		counts = (ul_sysclk	/	ul_div)	/	PDU_TC_DESIRED_HZ[i];
		tc_init(PDU_TC0_MODULE, i, ul_tcclks | TC_CMR_CPCTRG);
		tc_write_rc(PDU_TC0_MODULE, i, counts);
		tc_enable_interrupt(PDU_TC0_MODULE, i, TC_IER_CPCS);
		tc_start(PDU_TC0_MODULE, i);
		NVIC_EnableIRQ(PDU_TC_IRQ[i]);		
	}
	
	//TODO:: ADD the inits for possible 6 more TC channels

	
	
}

void all_off(void){
	enable_mask = 0;
	set_enable(PDU_ENABLES_PORTD_MASK, PDU_OFF_STATE );
}

/*
* Set the supplied mask channels to supply ioport val
* Mask: is bit mask of channels
* enable:	is the port value to write to masked channels
* Used to either enable or disable supplied channels
*
*	Function is supplied with enable_mask value that is used to bit shift 
*	to index in the port
*/
void set_enable(ioport_port_mask_t mask, enum ioport_value enable){
	//bit shift to index in port, and shift over by index of PDU channel zero
	ioport_set_port_level(PDU_ENABLE_PORT, mask, enable );
}



void update_output_status(){
	//start the AFEC conversions for all enabled outputs
	afec_start_software_conversion(IS_AFEC);
	uint8_t loc_total = 0;
	bool	sys_error = false;
	for(uint8_t i = 0; i < NUM_PDU_CHANNEL; i++){		
		
		//if the channel is actually in faulty state
		if( outputs[i].current == UINT8_MAX ){
			outputs[i].current = 0;
			outputs[i].state = CHANNEL_FAULT;
			//attempt to soft restart channel and assess on next run through
			error_mask |= (1<<i);
			sys_error = true;
		}
		else if ( ( outputs[i].current / SCALE_FACTOR ) >= curr_limits[i])
		{
			outputs[i].current = 0;
			outputs[i].state = CHANNEL_OVER_CURRENT;
			//set_enable((1<<i), PDU_OFF_STATE );
			//attempt to soft restart channel and assess on next run through
			error_mask |= (1<<i);
			sys_error = true;
		}
		else{				
			//error_mask &= ~(1<<i);
			}		
		
		if( Tst_bits( enable_mask, (1<<i) ) ){
			outputs[i].state = CHANNEL_ON;
						
			outputs[i].last_curr = outputs[i].current;
		 	outputs[i].current = get_is(IS_AFEC, PDU_AFEC_channel_list[i], PDU_AFEC_channel_offset[i]);	
			 			 
		 	//limit the gradient(deltaI) for the current measurement to ignore 
			//inrush and random noise/spikes, i am fuckng dumb
		 	//if( outputs[i].last_curr && ((outputs[i].current - outputs[i].last_curr) / outputs[i].last_curr ) * 100 > 20) outputs[i].current = 0;
		}	
		else {
			 outputs[i].state = CHANNEL_OFF;
			 outputs[i].current = 0;
		}
		
		loc_total += (outputs[i].current/ SCALE_FACTOR );
	}
		
	PDU.total_curr = loc_total;
	PDU.error_flag = sys_error;

}

/*
 *	Procedure: get_is
 *	Get the current sense feedback for the selected pdu_channel
 *	Where pdu_channel is the requested channel, offset is the analog offset of the 
 *	sense feedback for that channel and afec is the PDU AFEC
 *
 *	Returns the sense feedback value in amps, but value is also multiplied by the configured
 *	scale factor to get the desired significant digit, ie scale factor of 10 encodes 7.3 as 73 
 *	for CAN Bus transmitting
 *
 *	If the current conversion detects a fault returns uint8_t_MAX
 */
uint8_t get_is( Afec *const afec ,enum afec_channel_num pdu_channel, uint32_t offset){	
	
	while(  !(afec_get_interrupt_status(IS_AFEC) & (1 << pdu_channel))  );
	float conversion_result = afec_channel_get_value(afec, pdu_channel);
	
	//transform AFEC result into mV
	conversion_result = (conversion_result - offset )* AFEC_conversion_factor;
	
	/*
	 *	If the converted amperage is at the minimun value for fault 
	 *  indication, this is either a short circuit event or fet is over temp
	 */
	if ( conversion_result > IS_FAULT_MIN) return UINT8_MAX;
	
	//transform into number of mA
	conversion_result = conversion_result / PDU_SENSE_MV_TO_MA;
	
	//multiply by differential ratio to find load A
	conversion_result = conversion_result * PDU_FET_DIFFERENTIAL_RATIO;
	conversion_result = conversion_result * SCALE_FACTOR;
	
	//if the converted amperage is 0 or negative due to not perfect offset calibration
	if( conversion_result < 1 ) return 0;
	
	/*
	 *	Else return the converted value
	 *	The value is multiplied by the desired scale factor
	 *	to transmit the desired significant digits
	 */
	else return (uint8_t)conversion_result;
	}
	
	
void get_chip_temp(void){

	volatile float ul_temp;

	ul_temp = afec_channel_get_value(AFEC0, AFEC_TEMPERATURE_SENSOR);	
	
	ul_temp  = ul_temp * 3300 / (4095 * 16);// AFEC_conversion_factor;
	/*
	* According to datasheet, The output voltage VT = 1.44V at 27C
	* and the temperature slope dVT/dT = 4.7 mV/C
	*/
	ul_temp =  (ul_temp - 1440)  * 100 / 470 + 27;
	
	//basically shit is fucked
	if( ul_temp > UINT8_MAX ) ul_temp = UINT8_MAX;
	
	PDU.chip_temp = (uint8_t) ul_temp;
}


void soft_restart(void)
{
	__disable_irq();
	ioport_set_port_level(PDU_ENABLE_PORT, (error_mask & CONF_SOFT_RESTART_MASK), PDU_OFF_STATE );	
	/*
	 *	Delay to allow latch in powerfet to reset
	 *	The latch reset time is specified in Table 6 of BTS50060 datasheet
	 *	The Typical reset time is ~50ms but the maximum is 80ms, so we will 
	 *	wait for 85ms to ensure wait is long enough
	 */
	delay_ms(85);
	ioport_set_port_level(PDU_ENABLE_PORT, (error_mask & CONF_SOFT_RESTART_MASK), PDU_ON_STATE  );
	//reset the mask, after all channels have been "restarted"
	//why did i do this like this?
	error_mask &= ~(error_mask & CONF_SOFT_RESTART_MASK);
	__enable_irq();
	__ISB();
}


/*
* Handle received CAN messages
* mailbox: Pointer to mailbox holding newest message

* This function is intrinsically hard coded; We have 3 messages that are valid to 
* Receive by the current PDU setup, but by hiding the handling inside this function
* adding new messages that need to be handled by the PDU is less messy
*
* TODO: Build constant array of IDs with function pointers for handling different messages
*/
void handle_CAN(can_mb_conf_t *mailbox){
	
	switch(mailbox->ul_id){
		case CAN_MID_MIDvA(PDU_ECU_REC_ADDRESS) :
		
			enable_mask = ((mailbox->ul_datal)  | PDU_NON_ENGINE_MASK) & ~error_mask;
			g_recv_timeout_cnt = 0;
			set_enable(enable_mask, PDU_ON_STATE);
			//do this to ensure off request are serviced
			//TODO: someday refactor the port write code to make more sense
			set_enable(~enable_mask, PDU_OFF_STATE);
			break;
		case CAN_MID_MIDvA(PDU_ECU_ENG_DATA_ADD) :
			vehicle.RPM = LSB0W(mailbox->ul_datah) * ECU_RPM_SCALE;
		
			break;
		case CAN_MID_MIDvA(PDU_ECU_VEHICLE_DATA_ADD) :
			vehicle.fuel_pressure	= LSB0W(mailbox->ul_datah);
			PDU.batt_volt			= ( LSB1W(mailbox->ul_datah) << 8 ) | LSB2W(mailbox->ul_datah);
			break;
		default:
			break;
	}
}

/*
* Handler for generic interrupts from PDU_CAN
* PDU is connected directly to vehicle ECU that requests PDU updates
* Interrupt source is detected and then handled as needed
*/
void ECU_RECEIVE(void){
	//can_disable_interrupt(PDU_CAN, CAN_DISABLE_ALL_INTERRUPT_MASK);
	
	/*
	* Local status variable for use in interrupt handler
	* Used to hold/ and store result of determining source of the 
	* Triggering interrupt
	*/
	uint32_t status;
	status = can_get_status(PDU_CAN);				//get the entire status register
	
	/*
	* If this is an interrupt trigger by a Mailbox
	* ie, if it was triggered by a CAN reception
	*/
	if( status & GLOBAL_MAILBOX_MASK){
		//figure out which mailbox had interrupt
		for (uint8_t i = 0; i < CANMB_NUMBER; i++) {
			status = can_mailbox_get_status(PDU_CAN, i);
			
			/*
			* If the CAN message ready bit is set in the status register for 
			* the current mailbox, and only the message ready bit
			*/
			if ((status & CAN_MSR_MRDY) == CAN_MSR_MRDY) {
				can_mailbox[i].ul_status = status;
				can_mailbox_read(PDU_CAN, &can_mailbox[i]);
				handle_CAN(&can_mailbox[i]);
				
				/*Reinitialize the mailbox, resets all  interrupt flags regarding this mailbox*/
				can_mailbox[i].uc_obj_type = CAN_MB_RX_MODE;
				can_mailbox_init(PDU_CAN, &can_mailbox[i]);
				g_ul_recv_status = 1;
				break;
			}
		}

	}
	
	
	/*
	* Interrupt Triggered by CAN bus off
	*/	
	else if ( status & CAN_SR_BOFF ){
		
		enable_mask = PDU_TIMEOUT_MASK;
		set_enable(enable_mask, PDU_ON_STATE);
		//do this to ensure off request are serviced
		//TODO: someday refactor the port write code to make more sense
		set_enable(~enable_mask, PDU_OFF_STATE);
		PDU.error_flag = true;
		
		
		can_init(PDU_CAN, ul_sysclk, PDU_CAN_BAUD);		
				
		/* Disable all CAN interrupts. */
		can_disable_interrupt(PDU_CAN, CAN_DISABLE_ALL_INTERRUPT_MASK);
		__disable_irq();
		can_enable_autobaud_listen_mode(PDU_CAN);
		while( ( can_get_status(PDU_CAN) & PDU_ALL_ERR_MASK ) );
		can_disable_autobaud_listen_mode(PDU_CAN);
		__enable_irq();
		
	}
}


/*
* Handler for transmitting required data to ECU/ Data logger
* Called By Timer/Counter Compare Match interrupt handler
*   The Timer Counter handler this is called from determines 
*	the frequency at which messages will be transmitted 
*
* CAN mailboxes and CAN bus should be first initialized by init
* This function will just load new data into mailbox and start 
* transmission to ECU and data logger
*/
void PDU_transmit_callback(void){
	
	/*
	* Index value used for indexing into enablemask
	*/
	uint8_t byte_index = 0;
	/*
	* Send the first PDU message block containing input states and
	* PDU global status
	*/
	for( uint8_t i = 0; i < PDU_TX_STATUS_CNT; i++){
		can_mailbox[PDU_STATUS_TX_MB].ul_datal = 0;
		can_mailbox[PDU_STATUS_TX_MB].ul_datah = 0;
		//TODO: MAKE THIS LESS AWFUL; Maybe make new struct that char array, and lines up when cast pointer?		
		for( uint8_t index = 0; index < 4; index++){
			if( Tst_bits(enable_mask, ( 1 << (index + byte_index) ) ) ) can_mailbox[PDU_STATUS_TX_MB].ul_datal |= (1<< (8 *index));
		}
		
		for( uint8_t index = 4; index < 8; index++){
			if( Tst_bits(enable_mask, ( 1 << (index + byte_index) ) ) ) can_mailbox[PDU_STATUS_TX_MB].ul_datah |= (1<< (8 * (index - 4 )));
		}
		can_mailbox[PDU_STATUS_TX_MB].ul_datal |= state_id[i];
		
		can_mailbox[PDU_STATUS_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;
		/* Send out the information in the mailbox. */
		can_mailbox_write(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]);
		can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]); 
		//wait for device to send current message; average blocking time here should be negligible
		while( !( can_mailbox_get_status(PDU_CAN, PDU_STATUS_TX_MB) & CAN_MSR_MRDY ) ){
			if (can_get_tx_error_cnt(PDU_CAN) > PDU_CAN_TX_ERROR_CNT)break;
		}
		
		byte_index +=8;

	}
	
	can_mailbox[PDU_STATUS_TX_MB].ul_datal = 0;
	can_mailbox[PDU_STATUS_TX_MB].ul_datah = 0;
	
	LSB1W(can_mailbox[PDU_STATUS_TX_MB].ul_datal) = PDU.chip_temp;
	LSB2W(can_mailbox[PDU_STATUS_TX_MB].ul_datal) = 7;
	LSB3W(can_mailbox[PDU_STATUS_TX_MB].ul_datal) = PDU.error_flag;
	can_mailbox[PDU_STATUS_TX_MB].ul_datal |= state_three;
	LSB0W(can_mailbox[PDU_STATUS_TX_MB].ul_datah) = PDU.total_curr;
	can_mailbox[PDU_STATUS_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;
	can_mailbox_write(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]);
	can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]);
	//while( !( can_mailbox_get_status(PDU_CAN, PDU_STATUS_TX_MB) & CAN_MSR_MRDY ) ){
		//if (can_get_tx_error_cnt(PDU_CAN) > PDU_CAN_TX_ERROR_CNT)break; 
		//}
	
	
	g_tx_status = tx_good;
	
	/*
	* Send the PDU message block containing output states
	* TODO: actually finish this block
	*/
	can_mailbox[PDU_OUTPUT_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;
	for ( int index = 0; index < NUM_PDU_CHANNEL / 7 + 1; index++)
	{
		LSB0W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = index;
		LSB1W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = outputs[ (index * 8) + 0].state;
		LSB2W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = outputs[ (index * 8) + 1].state;
		LSB3W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = outputs[ (index * 8) + 2].state;

		LSB0W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (index * 8) + 3].state;
		LSB1W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (index * 8) + 4].state;
		LSB2W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (index * 8) + 5].state;
		LSB3W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (index * 8) + 6].state;
		
		can_mailbox_write(PDU_CAN, &can_mailbox[PDU_OUTPUT_TX_MB]);
		can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_OUTPUT_TX_MB]);
		
		//wait for device to send current message; average blocking time here should be negligible
		while( !( can_mailbox_get_status(PDU_CAN, PDU_OUTPUT_TX_MB) & CAN_MSR_MRDY ) ){
			if (can_get_tx_error_cnt(PDU_CAN) > PDU_CAN_TX_ERROR_CNT)break;
		}
	}		

	/* Send out the information in the mailbox. */
	
	/*
	 *	Send out the curent measurements to datalogger and ECU
	 *	Messages are sent in message with ID of Base Address + 1
	 *	7 measurements are sent per message with compound ID in byte 0
	 *	all messages send max data length
	 */
		
	can_mailbox[PDU_AMP_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;	
	for ( int index = 0; index < NUM_PDU_CHANNEL / 7 + 1; index++)
	{
		LSB0W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = index;
		LSB1W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (index * 8) + 0].current;
		LSB2W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (index * 8) + 1].current;
		LSB3W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (index * 8) + 2].current;

		LSB0W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (index * 8) + 3].current;
		LSB1W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (index * 8) + 4].current;
		LSB2W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (index * 8) + 5].current;
		LSB3W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (index * 8) + 6].current;
		
		can_mailbox_write(PDU_CAN, &can_mailbox[PDU_AMP_TX_MB]);
		can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_AMP_TX_MB]);
		
		//wait for device to send current message; average blocking time here should be negligible
		while( !( can_mailbox_get_status(PDU_CAN, PDU_AMP_TX_MB) & CAN_MSR_MRDY ) ){
			if (can_get_tx_error_cnt(PDU_CAN) > PDU_CAN_TX_ERROR_CNT)break;
		}
	}	
	return;
}


void ECU_timeout_callback(void){
	
	enable_mask = PDU_TIMEOUT_MASK;
	set_enable(enable_mask, PDU_ON_STATE);
	//do this to ensure off request are serviced
	//TODO: someday refactor the port write code to make more sense
	set_enable(~enable_mask, PDU_OFF_STATE);
	PDU.error_flag = true;
}