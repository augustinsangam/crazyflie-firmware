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

void onReceivePacket(const struct PacketRX &packet, bool *led_is_on) {

	switch (packet.code) {
	case RxPacketCode::start_mission: {
		DEBUG_PRINT("Start mission received\n"); // NOLINT
		sm.start_mission();
		break;
	}

	case RxPacketCode::end_mission: {
		DEBUG_PRINT("End mission received\n"); // NOLINT
		sm.end_mission();
		break;
	}

	case RxPacketCode::return_to_base: {
		DEBUG_PRINT("Return to base received\n"); // NOLINT
		sm.return_to_base();
		break;
	}

	case RxPacketCode::take_off: {
		DEBUG_PRINT("Take off received\n"); // NOLINT
		sm.take_off_robot();
		break;
	}

	case RxPacketCode::land: {
		DEBUG_PRINT("Land received\n"); // NOLINT
		sm.land_robot();
		break;
	}

	case RxPacketCode::set_led: {
		DEBUG_PRINT("Set Led received\n"); // NOLINT
		ledSetAll();
		*led_is_on = true;
		break;
	}

	case RxPacketCode::clear_led: {
		DEBUG_PRINT("Clear led received\n"); // NOLINT
		ledClearAll();
		*led_is_on = false;
		break;
	}

	default:
		break;
	}
}

void appMain() {
	ledClearAll();
	bool led_is_on = false;

	sm.init();
	::p2pRegisterCB(p2pCB);

	PacketRX rx_packet{};

	for (std::uint32_t i = 0;; ++i) {
		::vTaskDelay(M2T(DELAY_PER_FSM_LOOP));
		sm.step();
		if (i % DELAY_PER_STATE_UPDATE == 0) {
			// DEBUG_PRINT("Sending all packets\n"); // NOLINT
			send_all_packets(sm, led_is_on);
		}
		if (appchannelReceivePacket(&rx_packet, sizeof rx_packet, 0) != 0) {
			DEBUG_PRINT("App channel received code: %d\n", // NOLINT
			            (int)rx_packet.code);
			onReceivePacket(rx_packet, &led_is_on);
		}
	}
}
