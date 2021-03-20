extern "C" {
#include <app.h>
#include <radiolink.h>
}

#include "StateMachine.hpp"

static exploration::StateMachine sm; // NOLINT

static void p2pCB(P2PPacket *p) {
	sm.p2p_callback_handler(reinterpret_cast<exploration::P2PPacket *>(p));
}

void appMain() {
	sm.init();
	::p2pRegisterCB(p2pCB);
	for (;;) {
		sm.step();
	}
}
