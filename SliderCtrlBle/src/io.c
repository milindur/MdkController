#include "asf.h"
#include "io.h"

#define ioPIN_IN0	PIO_PA23_IDX
#define ioPIN_IN1	PIO_PA22_IDX
#define ioPIN_IN2   PIO_PA16_IDX
#define ioPIN_IN3	PIO_PA24_IDX

void prvPinIntHdl(uint32_t id, uint32_t mask);

void vIoInit(void)
{
	ioport_set_pin_dir(ioPIN_IN0, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ioPIN_IN0, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(ioPIN_IN1, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ioPIN_IN1, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(ioPIN_IN2, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ioPIN_IN2, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(ioPIN_IN3, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ioPIN_IN3, IOPORT_MODE_PULLUP);
	
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);

	UNUSED(pio_get_interrupt_status(PIOA));

	pio_handler_set_pin(ioPIN_IN0, PIO_IT_EDGE, prvPinIntHdl);
	pio_enable_pin_interrupt(ioPIN_IN0);
	pio_handler_set_pin(ioPIN_IN1, PIO_IT_EDGE, prvPinIntHdl);
	pio_enable_pin_interrupt(ioPIN_IN1);
	pio_handler_set_pin(ioPIN_IN2, PIO_IT_EDGE, prvPinIntHdl);
	pio_enable_pin_interrupt(ioPIN_IN2);
	pio_handler_set_pin(ioPIN_IN3, PIO_IT_EDGE, prvPinIntHdl);
	pio_enable_pin_interrupt(ioPIN_IN3);

	NVIC_SetPriority(PIOA_IRQn, 15);
	NVIC_EnableIRQ(PIOA_IRQn);	
}

void prvPinIntHdl(uint32_t id, uint32_t mask)
{
	SEGGER_RTT_printf(0, "I/O ISR %d %d %d %d\n", ioport_get_pin_level(ioPIN_IN0), ioport_get_pin_level(ioPIN_IN1), ioport_get_pin_level(ioPIN_IN2), ioport_get_pin_level(ioPIN_IN3));
}
