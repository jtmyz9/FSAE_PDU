/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>
#include <ioport.h>

/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do {\
		ioport_set_port_mode(port, masks, mode);\
		ioport_disable_port(port, masks);\
	} while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)

/**
 * \brief Set input mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 * \param sense Sense for interrupt detection (\ref ioport_sense)
 */
#define ioport_set_pin_input_mode(pin, mode, sense) \
	do {\
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);\
		ioport_set_pin_mode(pin, mode);\
		ioport_set_pin_sense_mode(pin, sense);\
	} while (0)

void board_init(void)
{	
		
#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
#endif

#ifdef CONF_BOARD_UART_CONSOLE
	/* Configure UART pins */
	ioport_set_port_peripheral_mode(PINS_UART0_PORT, PINS_UART0,
			PINS_UART0_FLAGS);
#endif

#ifdef CONF_BOARD_USART_RXD
	/* Configure USART RXD pin */
	ioport_set_pin_peripheral_mode(PIN_USART1_RXD_IDX,
			PIN_USART1_RXD_FLAGS);
#endif

#ifdef CONF_BOARD_USART_TXD
	/* Configure USART TXD pin */
	ioport_set_pin_peripheral_mode(PIN_USART1_TXD_IDX,
			PIN_USART1_TXD_FLAGS);
#endif

#ifdef CONF_BOARD_USART_CTS
	/* Configure USART CTS pin */
	ioport_set_pin_peripheral_mode(PIN_USART1_CTS_IDX,
			PIN_USART1_CTS_FLAGS);
#endif

#ifdef CONF_BOARD_USART_RTS
	/* Configure USART RTS pin */
	ioport_set_pin_peripheral_mode(PIN_USART1_RTS_IDX,
			PIN_USART1_RTS_FLAGS);
#endif

#ifdef CONF_BOARD_USART_SCK
	/* Configure USART synchronous communication SCK pin */
	ioport_set_pin_peripheral_mode(PIN_USART1_SCK_IDX,
			PIN_USART1_SCK_FLAGS);
#endif

#ifdef CONF_BOARD_CAN0
	/* Configure the CAN0 TX and RX pins. */
	ioport_set_pin_peripheral_mode(PIN_CAN0_RX_IDX, PIN_CAN0_RX_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_CAN0_TX_IDX, PIN_CAN0_TX_FLAGS);
	/* Configure the transiver0 RS & EN pins. */
	ioport_set_pin_dir(PIN_CAN0_TR_RS_IDX, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_CAN0_TR_EN_IDX, IOPORT_DIR_OUTPUT);
#endif

#ifdef CONF_BOARD_CAN1
	/* Configure the CAN1 TX and RX pin. */
	ioport_set_pin_peripheral_mode(PIN_CAN1_RX_IDX, PIN_CAN1_RX_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_CAN1_TX_IDX, PIN_CAN1_TX_FLAGS);
	/* Configure the transiver1 RS & EN pins. */
	ioport_set_pin_dir(PIN_CAN1_TR_RS_IDX, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIN_CAN1_TR_EN_IDX, IOPORT_DIR_OUTPUT);
#endif


#ifdef CONF_BOARD_SPI
	ioport_set_pin_peripheral_mode(SPI_MISO_GPIO, SPI_MISO_FLAGS);
	ioport_set_pin_peripheral_mode(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
	ioport_set_pin_peripheral_mode(SPI_SPCK_GPIO, SPI_SPCK_FLAGS);
#endif

#if defined(CONF_BOARD_TWI0) 
	ioport_set_pin_peripheral_mode(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
	ioport_set_pin_peripheral_mode(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
#endif

#ifdef CONF_BOARD_TWI1
	ioport_set_pin_peripheral_mode(TWI1_DATA_GPIO, TWI1_DATA_FLAGS);
	ioport_set_pin_peripheral_mode(TWI1_CLK_GPIO, TWI1_CLK_FLAGS);
#endif

#ifdef CONF_BOARD_NAND
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDOE,   PIN_EBI_NANDOE_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDWE,   PIN_EBI_NANDWE_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDCLE,  PIN_EBI_NANDCLE_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDALE,  PIN_EBI_NANDALE_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_0, PIN_EBI_NANDIO_0_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_1, PIN_EBI_NANDIO_1_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_2, PIN_EBI_NANDIO_2_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_3, PIN_EBI_NANDIO_3_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_4, PIN_EBI_NANDIO_4_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_5, PIN_EBI_NANDIO_5_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_6, PIN_EBI_NANDIO_6_FLAGS);
	ioport_set_pin_peripheral_mode(PIN_EBI_NANDIO_7, PIN_EBI_NANDIO_7_FLAGS);
    ioport_set_pin_dir(PIN_NF_CE_IDX, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(PIN_NF_RB_IDX, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(PIN_NF_RB_IDX, IOPORT_MODE_PULLUP);
#endif



	/* Initialize IOPORTs */
	ioport_init();
	delay_init(sysclk_get_cpu_hz());
	
	/* Enable CAN0 & CAN1 clock. */
	pmc_enable_periph_clk(ID_CAN0);
	//pmc_enable_periph_clk(ID_CAN1);
	
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC1);
	pmc_enable_periph_clk(ID_TC2);
	
}