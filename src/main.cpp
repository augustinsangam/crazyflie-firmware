#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <FreeRTOSConfig.h> /* M2T */
#include <app.h>            /* appMain */
#include <debug.h>          /* DEBUG_PRINT */
}

extern "C" {
/* error: "include FreeRTOS.h must appear in source files before include task.h"
 */
#include <FreeRTOS.h>
#include <task.h> /* vTaskDelay */
}

void appMain() {
	DEBUG_PRINT("Waiting for activation ...\n");

	for (;;) {
		vTaskDelay(M2T(2000));
		DEBUG_PRINT("hi\n");
	}
}
