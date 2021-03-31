#include "porting.hpp"
extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

extern "C" {
#include <app.h>
#include <radiolink.h>
#include "estimator_kalman.h"
#include "debug.h"
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
	unsigned int i = 0;
	for (;;) {
		i++;
		::vTaskDelay(10);
		sm.step();
		point_t p;
		estimatorKalmanGetEstimatedPos(&p);
		if (i % 100 == 0) {
			DEBUG_PRINT("X %f \t Y %f \t Z %f \n", p.x, p.y, p.z);
		}

		// if ((i < 100) && (i % 10 == 0)) {
		// 	estimatorKalmanTaskInit();
		// }
	}
}
