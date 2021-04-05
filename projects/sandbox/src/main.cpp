
#define DEBUG_MODULE "SANDBOX_PROJECT"

// FreeRTOS imports
extern "C" {
#include <FreeRTOS.h>
#include <app.h> /* appMain */
#include <debug.h> /* DEBUG_PRINT */
}

// Crazyflie Firmware imports
extern "C" {
#include <led.h>
#include <task.h> /* vTaskDelay */
}

void appMain() {
	while (true) {
		DEBUG_PRINT("Setting all leds\n");
		ledSetAll();
		vTaskDelay(M2T(1000));
		DEBUG_PRINT("Clearing all leds\n");
		ledClearAll();
	}
}
