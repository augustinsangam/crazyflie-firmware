#define DEBUG_MODULE "RR_PROJECT"

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

extern "C" {
#include "debug.h"
#include "estimator_kalman.h"
#include <app.h>
#include <app_channel.h>
#include <led.h>
#include <radiolink.h>
}

#include "communication.hpp"
#include <exploration/StateMachine.hpp>

static constexpr unsigned int DELAY_PER_FSM_LOOP = 10U;
static constexpr unsigned int DELAY_PER_STATE_UPDATE =
    500U / DELAY_PER_FSM_LOOP;

static porting::DroneLayer d(nullptr);   // NOLINT
static exploration::StateMachine sm(&d); // NOLINT

static void p2pCB(P2PPacket *p) {
	sm.p2p_callback_handler(reinterpret_cast<exploration::P2PPacket *>(p));
}

void appMain() {
	ledClearAll();
	Communication com(&sm, false);

	sm.init();
	::p2pRegisterCB(p2pCB);

	PacketRX rx_packet{};

	for (std::uint32_t i = 0;; ++i) {
		::vTaskDelay(M2T(DELAY_PER_FSM_LOOP));
		sm.step();
		com.update_speed();
		if (i % DELAY_PER_STATE_UPDATE == 0) {
			// DEBUG_PRINT("Sending all packets\n"); // NOLINT
			com.send_all_packets();
		}
		if (appchannelReceivePacket(&rx_packet, sizeof rx_packet, 0) != 0) {
			DEBUG_PRINT("App channel received code: %d\n", // NOLINT
			            (int)rx_packet.code);
			com.on_receive_packet(rx_packet);
		}
	}
}
