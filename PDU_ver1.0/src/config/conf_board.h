/*
 * conf_board.h
 *
 * Created: 12/2/2017 3:55:21 PM
 *  Author: MizzouRacing
 */ 


#ifndef CONF_BOARD_H_
#define CONF_BOARD_H_

#ifndef BOARD
#define BOARD USER_BOARD
#endif


//!------   External Clock Config
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    (20000000U)
#define BOARD_FREQ_MAINCK_BYPASS  (20000000U)
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625

/*
* define CAN1 and CAN0 peripherals,
* leverages conditional compilation from ASF examples
*/
#define CONF_BOARD_CAN0
#define CONF_BOARD_CAN1


#endif /* CONF_BOARD_H_ */