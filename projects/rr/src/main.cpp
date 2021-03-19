#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <FreeRTOS.h>
#include <app.h> /* appMain */
#include <app_channel.h>
#include <debug.h> /* DEBUG_PRINT */
#include <task.h>  /* vTaskDelay */

#include <led.h>
#include <log.h>
#include <pm.h>
#include <radiolink.h>
}

#include <cstdint>
#include <ctime>

#include <exploration/state_machine.hpp>

static exploration::StateMachine sm; // NOLINT

static void p2pCB(P2PPacket *p) {
	sm.p2pCallbackHandler(reinterpret_cast<exploration::P2PPacket *>(p));
}

void appMain() {
	sm.startup();
	p2pRegisterCB(p2pCB);

	for (;;) {
		sm.iteration_loop();
	}
}
