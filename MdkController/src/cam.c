#include "asf.h"
#include "SEGGER_RTT.h"
#include "cam.h"

#define camPIN_FOCUS   PIO_PD8_IDX
#define camPIN_SHUTTER PIO_PD7_IDX

void vCamInit(void)
{
	ioport_set_pin_dir(camPIN_FOCUS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(camPIN_FOCUS, 0);
	ioport_set_pin_level(camPIN_FOCUS, false);

	ioport_set_pin_dir(camPIN_SHUTTER, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(camPIN_SHUTTER, 0);
	ioport_set_pin_level(camPIN_SHUTTER, false);
}

void vCamClear(void)
{
	SEGGER_RTT_printf(0, "Cam: Clear\n");
	ioport_set_pin_level(camPIN_FOCUS, false);
	ioport_set_pin_level(camPIN_SHUTTER, false);
}

void vCamFocus(void)
{
	SEGGER_RTT_printf(0, "Cam: Focus\n");
	ioport_set_pin_level(camPIN_FOCUS, true);
}

void vCamShutter(void)
{
	SEGGER_RTT_printf(0, "Cam: Shutter\n");
	ioport_set_pin_level(camPIN_SHUTTER, true);
}
