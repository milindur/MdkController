/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Atmel library includes. */
#include "asf.h"

#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#include "SEGGER_RTT.h"
#include "eep.h"
#include "io.h"
#include "cam.h"
#include "sm.h"
#include "ble.h"
#include "slider.h"

#define mainCHECK_TIMER_RATE                    (500 / portTICK_RATE_MS)

static traceLabel check_user_event_channel;

static void prvSetupHardware(void);
static void prvCheckTimerCallback(void *pvParameters);
static void prvTestTimerCallback(void *pvParameters);

void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName);
void vApplicationTickHook(void);

/* Define how assert should function in the BLE library */
void __ble_assert(const char * file, uint16_t line)
{
	vAssertCalled(file, line);
}

int main(void)
{
	SEGGER_RTT_printf(0, "Startup...\n");
	
	prvSetupHardware();
	
	vTraceInitTraceData();
	if (!uiTraceStart())
	{
		SEGGER_RTT_printf(0, "Could not start recorder!\n");
	}
	
	check_user_event_channel = xTraceOpenLabel("Chk");

	vEepInit();
	vCamInit();
	vIoInit();
	vSmInit();
	vBleInit();
	vSliderInit();
	
	xTimerHandle xCheckTimer = xTimerCreate(
		(const char * const) "ChkTmr",
		mainCHECK_TIMER_RATE,
		pdTRUE,
		NULL,
		prvCheckTimerCallback);
	configASSERT(xCheckTimer);
	xTimerStart(xCheckTimer, 0);

	xTimerHandle xTestTimer = xTimerCreate(
		(const char * const) "TestTmr",
		(1500 / portTICK_RATE_MS),
		pdTRUE,
		NULL,
		prvTestTimerCallback);
	configASSERT(xTestTimer);
	//xTimerStart(xTestTimer, 0);
	
	vTaskStartScheduler();
	
	for (;;) {}
}

static void prvCheckTimerCallback(void *pvParameters)
{
	UNUSED(pvParameters);
	
	vTracePrintF(check_user_event_channel, "Check: %d", 0);
	
	ioport_toggle_pin_level(LED0_GPIO);
}

static void prvTestTimerCallback(void *pvParameters)
{
	static uint8_t state = 0;
	
	#define MOTOR 0
	
	UNUSED(pvParameters);
	
	switch (state)
	{
	case 0:
		bSmMoveContinuous(MOTOR, 1*SM_SPR);
		bSmMoveContinuous(MOTOR+1, 1*SM_SPR);
		bSmMoveContinuous(MOTOR+2, 1*SM_SPR);
		break;
	case 1:
		bSmMoveContinuous(MOTOR, 2*SM_SPR);
		bSmMoveContinuous(MOTOR+1, 2*SM_SPR);
		bSmMoveContinuous(MOTOR+2, 2*SM_SPR);
		break;
	case 2:
		bSmMoveContinuous(MOTOR, 3*SM_SPR);
		bSmMoveContinuous(MOTOR+1, 3*SM_SPR);
		bSmMoveContinuous(MOTOR+2, 3*SM_SPR);
		break;
	case 3:
		bSmMoveContinuous(MOTOR, -1*SM_SPR);
		bSmMoveContinuous(MOTOR+1, -1*SM_SPR);
		bSmMoveContinuous(MOTOR+2, -1*SM_SPR);
		break;
	case 4:
		bSmMoveContinuous(MOTOR, -2*SM_SPR);
		bSmMoveContinuous(MOTOR+1, -2*SM_SPR);
		bSmMoveContinuous(MOTOR+2, -2*SM_SPR);
		break;
	case 5:
		bSmMoveContinuous(MOTOR, 0);
		bSmMoveContinuous(MOTOR+1, 0);
		bSmMoveContinuous(MOTOR+2, 0);
		break;
	default:
		break;
	}
	
	/*if (state == 0)
	{
		bSmMoveContinuous(0, 1*SM_SPR);
		state++;
	}*/
	
	state++;
	state %= 15;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(0);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	
	SEGGER_RTT_printf(0, "ERROR: vApplicationMallocFailedHook\n");
	
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	
	SEGGER_RTT_printf(0, "ERROR: vApplicationStackOverflowHook (%s)\n", pcTaskName);
	
	for (;;) {}
}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}

/*-----------------------------------------------------------*/

void vAssertCalled(const char *file, uint32_t line)
{
	volatile uint32_t block_var = 0, line_in;
	const char *file_in;

	/* These assignments are made to prevent the compiler optimizing the
	values away. */
	file_in = file;
	line_in = line;
	(void) file_in;
	(void) line_in;
	
	SEGGER_RTT_printf(0, "ERROR %s: %d\n", file, line);	

	taskENTER_CRITICAL();
	{
		while (block_var == 0) {
			/* Set block_var to a non-zero value in the debugger to
			step out of this function. */
		}
	}
	taskEXIT_CRITICAL();
}
