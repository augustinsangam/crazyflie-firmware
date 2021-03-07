/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * appchanel_test.c: Demonstrate the appchanel functionality
 */

#include "app.h"
#include "app_channel.h"
#include "debug.h"
#include "log.h"
#include "pm.h"

#define DEBUG_MODULE "HELLOWORLD"

const static float bat671723HS25C[10] = {
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

static int32_t pmBatteryChargeFromVoltage(float voltage) {
	int charge = 0;

	if (voltage < bat671723HS25C[0]) {
		return 0;
	}
	if (voltage > bat671723HS25C[9]) {
		return 9;
	}
	while (voltage > bat671723HS25C[charge]) {
		charge++;
	}

	return charge;
}

struct testPacketRX {
	float x;
	float y;
	float z;
} __attribute__((packed));

struct testPacketTX {

	unsigned long long timestemp;
	float speed;
	float battery;
	float positionX;

	uint16_t up;
	uint16_t left;
	uint16_t right;
	uint16_t front;
	uint16_t back;
} __attribute__((packed));

void appMain() {
	DEBUG_PRINT("Waiting for activation ...\n");

	struct testPacketRX rxPacket;
	struct testPacketTX txPacket;

	logVarId_t idUp = logGetVarId("range", "up");
	logVarId_t idLeft = logGetVarId("range", "left");
	logVarId_t idRight = logGetVarId("range", "right");
	logVarId_t idFront = logGetVarId("range", "front");
	logVarId_t idBack = logGetVarId("range", "back");

	while (1) {
		if (appchannelReceivePacket(&rxPacket, sizeof(rxPacket),
		                            APPCHANNEL_WAIT_FOREVER)) {

			DEBUG_PRINT("App channel received x: %f, y: %f, z: %f\n",
			            (double)rxPacket.x, (double)rxPacket.y,
			            (double)rxPacket.z);

			txPacket.timestemp = 0;
			txPacket.speed = 0;
			txPacket.battery =
			    pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10;
			txPacket.positionX = 0;

			txPacket.back = logGetUint(idBack);
			txPacket.front = logGetUint(idFront);
			txPacket.left = logGetUint(idLeft);
			txPacket.right = logGetUint(idRight);
			txPacket.up = logGetUint(idUp);

			appchannelSendPacket(&txPacket, sizeof(txPacket));
		}
	}
}
