#include <cstdint>
#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <FreeRTOS.h>
#include <app.h> /* appMain */
#include <app_channel.h>
#include <debug.h> /* DEBUG_PRINT */
#include <task.h>  /* vTaskDelay */
#include "stabilizer_types.h"
#include "estimator_kalman.h"

#include <led.h>
#include <log.h>
#include <pm.h>
}

#include <cstdint>
#include <ctime>


enum TxPacketCode{
	BATTERY=0, TIMESTEMP=1, SPEED=2,
	POSITION=3, SENSORS=4, OTHERS=5
};
enum RxPacketCode{
	START_MISSION=0, END_MISSION=1, RETURN_TO_BASE=2,
	TAKE_OFF=3, LANDING=4, LED_ON=5, LED_OFF=6
};

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

struct PacketRX {
	int code;
} __attribute__((packed));

struct TimestempPacket{
	short code;
	std::uint64_t timestemp;
} __attribute__((packed));

struct SpeedPacket{
	short code;
	float speed;
} __attribute__((packed));

struct PositionPacket{
	short code;
	float positionX, positionY, positionZ;
} __attribute__((packed));

struct SensorsPacket{
	short code;
	std::uint16_t front, left, back, right, up;
} __attribute__((packed));

struct BatteryPacket {
	short code;
	float battery;
} __attribute__((packed));

struct OtherPacket {
	short code;
	bool flying, ledOn;
} __attribute__((packed));

void onReceivePacket(const struct PacketRX& packet,
	struct OtherPacket otherPacket ){
	switch (packet.code) {
	case START_MISSION:
		DEBUG_PRINT("Start mission received\n");
		break;

	case END_MISSION:
		DEBUG_PRINT("End mission received\n");
		break;

	case RETURN_TO_BASE:
		DEBUG_PRINT("Return to base received\n");
		break;

	case TAKE_OFF:
		DEBUG_PRINT("Take off received\n");
		break;

	case LANDING:
		DEBUG_PRINT("Landing received\n");
		break;

	case LED_ON:
		DEBUG_PRINT("Led On received\n");
		ledSetAll();
		otherPacket.ledOn = true;
		break;

	case LED_OFF:
		DEBUG_PRINT("Led Off received\n");
		ledClearAll();
		otherPacket.ledOn = false;
		break;

	default:
		break;
	}


}

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

void setPosition(const point_t& p, struct PositionPacket& position ){
	position.code = POSITION;
	position.positionX = p.x;
	position.positionY = p.y;
	position.positionZ = p.z;
}

void appMain() {
	vTaskDelay(M2T(3000));

	ledClearAll();

	struct PacketRX rxPacket;
	struct TimestempPacket timestempPacket;
	struct SpeedPacket speedPacket;
	struct PositionPacket positionPacket;
	struct SensorsPacket sensorsPacket;
	struct BatteryPacket batteryPacket;
	struct OtherPacket otherPacket;

	logVarId_t idUp = logGetVarId("range", "up");
	logVarId_t idLeft = logGetVarId("range", "left");
	logVarId_t idRight = logGetVarId("range", "right");
	logVarId_t idFront = logGetVarId("range", "front");
	logVarId_t idBack = logGetVarId("range", "back");


	logVarId_t idPitch = logGetVarId("stabilizer", "pitch");
    logVarId_t idRoll = logGetVarId("stabilizer", "roll");
    logVarId_t idThrust = logGetVarId("stabilizer", "thrust");
	logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");



	// DEBUG_PRINT("%i", idUp);

	DEBUG_PRINT("Waiting for activation ...\n");

	for (;;) {
		vTaskDelay(M2T(10));

		if (appchannelReceivePacket(&rxPacket, sizeof(rxPacket), 0)) {
			DEBUG_PRINT("App channel received setLeds: %d\n",
			            (int)rxPacket.code);
			onReceivePacket(rxPacket, otherPacket);
		}


		// Others data
		otherPacket.code = OTHERS;
		otherPacket.flying = false;


		// speed data
		speedPacket.code = SPEED;
		speedPacket.speed = 0.0;

		// timestemp data

		timestempPacket.code = TIMESTEMP;
		timestempPacket.timestemp = std::time(nullptr);

		// Battery data
		batteryPacket.code = BATTERY;
		batteryPacket.battery =
		pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10;


		// Position data
		point_t p;
		estimatorKalmanGetEstimatedPos(&p);
		setPosition(p, positionPacket);

		// tilt data
		float pitch = logGetFloat(idPitch);
		float roll = logGetFloat(idRoll);
		float thrust = logGetFloat(idThrust);
		float yaw = logGetFloat(idYaw);


		// Sensors data
		uint16_t left = logGetUint(idLeft);
		uint16_t right = logGetUint(idRight);
		uint16_t front = logGetUint(idFront);
		uint16_t back = logGetUint(idBack);
		uint16_t up = logGetUint(idUp);

		sensorsPacket.code = SENSORS;
		sensorsPacket.back = back;
		sensorsPacket.front = front;
		sensorsPacket.left = left;
		sensorsPacket.right = right;
		sensorsPacket.up = up;

		appchannelSendPacket(&sensorsPacket, sizeof(sensorsPacket));
		appchannelSendPacket(&positionPacket, sizeof(positionPacket));
		appchannelSendPacket(&batteryPacket, sizeof(batteryPacket));
		appchannelSendPacket(&timestempPacket, sizeof(timestempPacket));
		appchannelSendPacket(&speedPacket, sizeof(speedPacket));
		appchannelSendPacket(&otherPacket, sizeof(otherPacket));
		vTaskDelay(M2T(1000));
		vTaskDelay(M2T(10));
	}
}
