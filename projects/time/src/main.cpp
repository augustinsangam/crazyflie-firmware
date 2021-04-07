#define DEBUG_MODULE "TIME_PROJECT"

// FreeRTOS imports
extern "C" {
#include <FreeRTOS.h>
#include <app.h> /* appMain */
#include <debug.h> /* DEBUG_PRINT */
}

// Crazyflie Firmware imports
extern "C" {
#include <task.h> /* vTaskDelay */
}

void appMain() {
	while (true) {
		vTaskDelay(M2T(2000));
		auto us = usecTimestamp();
		DEBUG_PRINT("Time %u\n", static_cast<unsigned>(us / 1000000));
	}
}
