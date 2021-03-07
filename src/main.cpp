#include <cstdint>
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
}

#include <cstdint>
#include <ctime>

constexpr static float bat671723HS25C[] = {
    3.00, // 00%
    3.78, // 10%
    3.83, // 20%
    3.87, // 30%
    3.89, // 40%
    3.92, // 50%
    3.96, // 60%
    4.00, // 70%
    4.04, // 80%
    4.10  // 90%
};

static std::int32_t pmBatteryChargeFromVoltage(float voltage) {
	if (voltage < bat671723HS25C[0]) {
		return 0;
	}

	if (voltage > bat671723HS25C[9]) {
		return 9;
	}

	std::int32_t charge = 0;
	while (voltage > bat671723HS25C[charge]) {
		++charge;
	}
	return charge;
}

struct testPacketRX {
	bool setLeds;
} __attribute__((packed));

struct testPacketTX {
	std::uint64_t timestemp;
	float speed, battery;
	float positionX, positionY, positionZ;
	bool flying, ledOn;

	std::uint16_t front, left, back, right, up;

} __attribute__((packed));

void appMain() {
	vTaskDelay(M2T(3000));

	ledClearAll();

	struct testPacketRX rxPacket;
	struct testPacketTX txPacket;

	logVarId_t idUp = logGetVarId("range", "up");
	logVarId_t idLeft = logGetVarId("range", "left");
	logVarId_t idRight = logGetVarId("range", "right");
	logVarId_t idFront = logGetVarId("range", "front");
	logVarId_t idBack = logGetVarId("range", "back");

	// DEBUG_PRINT("%i", idUp);

	DEBUG_PRINT("Waiting for activation ...\n");

	for (;;) {
		vTaskDelay(M2T(10));

		if (appchannelReceivePacket(&rxPacket, sizeof(rxPacket), 0)) {
			DEBUG_PRINT("App channel received setLeds: %d\n",
			            (int)rxPacket.setLeds);
			if (rxPacket.setLeds) {
				ledSetAll();
				txPacket.ledOn = true;
			} else {
				ledClearAll();
				txPacket.ledOn = false;
			}
		}

		txPacket.timestemp = std::time(nullptr);
		txPacket.speed = 0.0;
		txPacket.battery =
		    pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10;
		txPacket.positionX = 0.0;
		txPacket.positionY = 0.0;
		txPacket.positionZ = 0.0;
		txPacket.flying = false;

		// DEBUG_PRINT(".");
		uint16_t left = logGetUint(idLeft);
		uint16_t right = logGetUint(idRight);
		uint16_t front = logGetUint(idFront);
		uint16_t back = logGetUint(idBack);
		uint16_t up = logGetUint(idUp);

		txPacket.back = back;
		txPacket.front = front;
		txPacket.left = left;
		txPacket.right = right;
		txPacket.up = up;
		appchannelSendPacket(&txPacket, sizeof(txPacket));
		vTaskDelay(M2T(1000));
	}
}
