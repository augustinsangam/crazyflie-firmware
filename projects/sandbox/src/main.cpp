
#define DEBUG_MODULE "HELLOWORLD"

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
		ledClearAll();
		vTaskDelay(M2T(1000));
		DEBUG_PRINT("Clearing all leds\n");
		ledClearAll();
	}
}
