#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <FreeRTOS.h>
#include <app.h>
#include <app_channel.h>
#include <debug.h>
#include <task.h>

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
	sm.p2p_callback_handler(reinterpret_cast<exploration::P2PPacket *>(p));
}

void appMain() {
	sm.startup();
	p2pRegisterCB(p2pCB);

	for (;;) {
		sm.iteration_loop();
	}
}
