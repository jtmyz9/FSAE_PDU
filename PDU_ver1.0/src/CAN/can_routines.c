/*
 * can_routines.c
 *
 * Created: 4/21/2018 3:39:38 PM
 *  Author: Jason
 */ 

#include <asf.h>
#include "PDU.h"

/*
* Service a State Request from Requesting ECU
*/
void state_request(uint64_t payload){
	enable_mask = ((payload)  | PDU_NON_ENGINE_MASK) & ~error_mask;
	g_recv_timeout_cnt = 0;
	set_enable(enable_mask, PDU_ON_STATE);
	//do this to ensure off request are serviced
	//TODO: someday refactor the port write code to make more sense
	set_enable(~enable_mask, PDU_OFF_STATE);
}

void ecu_eng_data(uint64_t payload){
	vehicle.RPM = LSB0W(payload) * ECU_RPM_SCALE;
}

void ecu_vehicle_data(uint64_t payload){
	vehicle.fuel_pressure	= LSB0W(payload);
	PDU.batt_volt			= ( LSB1W(payload) << 8 ) | LSB2W(payload);
}