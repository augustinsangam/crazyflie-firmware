#define TYPES_H

//namespace cff {
extern "C" {
#include <radiolink.h>
}
//}

#include "StateMachine.hpp"

static StateMachine sm; // NOLINT

static void p2pCB(P2PPacket *p) {
	sm.p2p_callback_handler(reinterpret_cast<P2PPacket *>(p));
}

extern "C" void appMain() {
	sm.init();
	p2pRegisterCB(p2pCB);
	for (;;) {
		sm.step();
	}
}
