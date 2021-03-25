#include "porting.hpp"
extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

extern "C" {
#include <app.h>
#include <radiolink.h>
}

#include "exploration/StateMachine.hpp"

static porting::DroneLayer d(nullptr);   // NOLINT
static exploration::StateMachine sm(&d); // NOLINT

static void p2pCB(P2PPacket *p) {
	sm.p2p_callback_handler(reinterpret_cast<exploration::P2PPacket *>(p));
}

void appMain() {
	sm.init();
	::p2pRegisterCB(p2pCB);
	for (;;) {
		::vTaskDelay(10);
		sm.step();
	}
}
