#include <string.h>
#include "sm.h"
#include "eep.h"

#define EEP_VERSION 0x0100
#define EEP_CHECK   0xaa55

const eep_params_t eep_params_def = {
	EEP_VERSION,
	EEP_CHECK,
	0xffff,
	0xffff,
	0,
	0,
	0,
	0,
	0,
	0xffff,
	0xffff,
	0xffff,
	0xffff,
	0xffff,
	0xffff,
	0xffff,
	6*SM_SPR,                   // accel steps/s^2
	6*SM_SPR,                   // decel steps/s^2
	4*SM_SPR,                   // speed steps/s
	1,							// power save
	0,							// motor reverse
	0xffff,
	0xffff,
	0xffff,
	0xffff,
	100,			// pre time
	100,			// focus time
	100,			// exposure time
	100,			// post time
	0xffff,
	0xffff,
	0xffff,
	0xff,
	1,				// optimize acceleration
    2000,           // interval
    300,            // count
    0,              // start position
    0,              // end position,
	0,              // ramp count
	0,              // stall count
	0xffff,
	0xffff,
	100,			// lcd backlight full brightness
	50,				// lcd backlight reduced brightness
	10,				// lcd backlight reduce time
	60				// lcd backlight off time
};
eep_params_t eep_params_ee;
eep_params_t eep_params;

void eep_init(void)
{
    eep_load();
}

void eep_save(void)
{
	//eeprom_write_block(&eep_params, &eep_params_ee, sizeof(eep_params_t));
}

void eep_load(void)
{
    //eeprom_read_block(&eep_params, &eep_params_ee, sizeof(eep_params_t));
	//if (eep_params.version != EEP_VERSION || eep_params.check != EEP_CHECK)
	{
		eep_load_default();
		eep_save();
	}
}

void eep_load_default(void)
{
	memcpy(&eep_params, &eep_params_def, sizeof(eep_params_t));
}
