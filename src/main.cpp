#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <FreeRTOS.h>
#include <app.h>   /* appMain */
#include <debug.h> /* DEBUG_PRINT */
#include <led.h>
#include <task.h> /* vTaskDelay */
}

#include <string>

void appMain() {
	std::string s;
	DEBUG_PRINT("Waiting for activation ...\n");

	for (;;) {
		vTaskDelay(M2T(2000));
		ledSet(LED_BLUE_L, true);
		vTaskDelay(M2T(2000));
		ledSet(LED_BLUE_L, false);
	}
}
