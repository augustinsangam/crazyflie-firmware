#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include "estimator_kalman.h"
#include "stabilizer_types.h"
#include <FreeRTOS.h>
#include <app.h> /* appMain */
#include <app_channel.h>
#include <debug.h> /* DEBUG_PRINT */
#include <task.h> /* vTaskDelay */

#include <led.h>
#include <log.h>
#include <pm.h>
}

#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>

enum class TxPacketCode : std::uint8_t {
	battery = 0,
	timestamp,
	speed,
	position,
	sensors,
	others
};

enum RxPacketCode : uint8_t {
	start_mission,
	end_mission,
	return_to_base,
	take_off,
	land,
	set_led,
	clear_led
};

enum class DroneState : uint8_t {
	onTheGround = 0,
	takingOff,
	landing,
	crashed,
	exploring,
	standBy,
	returningToBase
};

const static std::array battery_thresholds = {
    3.00F,  // 00%
    3.60F,  // 05%
    3.78F,  // 10%
    3.805F, // 15%
    3.83F,  // 20%
    3.85F,  // 25%
    3.87F,  // 30%
    3.88F,  // 35%
    3.89F,  // 40%
    3.905F, // 45%
    3.92F,  // 50%
    3.94F,  // 55%
    3.96F,  // 60%
    3.98F,  // 65%
    4.00F,  // 70%
    4.02F,  // 75%
    4.04F,  // 80%
    4.07F,  // 85%
    4.10F   // 90%
};

struct PacketRX {
	uint8_t code;
} __attribute__((packed));

struct TimestempPacket {
	uint8_t code;
	int64_t timestamp;
} __attribute__((packed));

struct SpeedPacket {
	uint8_t code;
	float speed;
} __attribute__((packed));

struct PositionPacket {
	uint8_t code;
	float positionX, positionY, positionZ;
} __attribute__((packed));

struct SensorsPacket {
	uint8_t code;
	uint16_t front, left, back, right, up;
} __attribute__((packed));

struct BatteryPacket {
	uint8_t code;
	float battery;
} __attribute__((packed));

struct OtherPacket {
	uint8_t code;
	uint8_t state;
	bool ledOn;
} __attribute__((packed));

void onReceivePacket(const struct PacketRX &packet,
                     struct OtherPacket otherPacket) {
	switch (packet.code) {
	case RxPacketCode::start_mission:
		DEBUG_PRINT("Start mission received\n"); // NOLINT
		break;

	case RxPacketCode::end_mission:
		DEBUG_PRINT("End mission received\n"); // NOLINT
		break;

	case RxPacketCode::return_to_base:
		DEBUG_PRINT("Return to base received\n"); // NOLINT
		break;

	case RxPacketCode::take_off:
		DEBUG_PRINT("Take off received\n"); // NOLINT
		break;

	case RxPacketCode::land:
		DEBUG_PRINT("Land received\n"); // NOLINT
		break;

	case RxPacketCode::set_led:
		DEBUG_PRINT("Set Led received\n"); // NOLINT
		ledSetAll();
		otherPacket.ledOn = true;
		break;

	case RxPacketCode::clear_led:
		DEBUG_PRINT("Clear led received\n"); // NOLINT
		ledClearAll();
		otherPacket.ledOn = false;
		break;

	default:
		break;
	}
}

static float pmBatteryChargeFromVoltage(float voltage) {
	if (voltage < battery_thresholds[0]) {
		return 0.0F;
	}

	if (voltage > battery_thresholds.back()) {
		return 100.0F;
	}

	std::size_t charge = 0;
	while (voltage > battery_thresholds.at(charge)) {
		++charge;
	}
	return static_cast<float>(charge) * 10.0F / 2.0F;
}

static int64_t get_timestamp() {
	return time(nullptr);
}

void appMain() {
	vTaskDelay(M2T(3000));

	ledClearAll();

	PacketRX rx_packet{};
	TimestempPacket timestamp_packet{};
	SpeedPacket speed_packet{};
	PositionPacket position_packet{};
	SensorsPacket sensors_packet{};
	BatteryPacket battery_packet{};
	OtherPacket other_packet{};

	logVarId_t id_up = logGetVarId("range", "up");       // NOLINT
	logVarId_t id_left = logGetVarId("range", "left");   // NOLINT
	logVarId_t id_right = logGetVarId("range", "right"); // NOLINT
	logVarId_t id_front = logGetVarId("range", "front"); // NOLINT
	logVarId_t id_back = logGetVarId("range", "back");   // NOLINT

	logVarId_t id_pitch = logGetVarId("stabilizer", "pitch");   // NOLINT
	logVarId_t id_roll = logGetVarId("stabilizer", "roll");     // NOLINT
	logVarId_t id_thrust = logGetVarId("stabilizer", "thrust"); // NOLINT
	logVarId_t id_yaw = logGetVarId("stateEstimate", "yaw");    // NOLINT

	for (;;) {

		vTaskDelay(M2T(1000));

		if (appchannelReceivePacket(&rx_packet, sizeof(rx_packet), 0) != 0) {
			DEBUG_PRINT("App channel received code: %d\n", // NOLINT
			            rx_packet.code);
			onReceivePacket(rx_packet, other_packet);
		}

		// Others data
		other_packet.code = static_cast<uint8_t>(TxPacketCode::others);
		other_packet.state = static_cast<uint8_t>(DroneState::onTheGround);

		// speed data
		speed_packet.code = static_cast<uint8_t>(TxPacketCode::speed);
		speed_packet.speed = 0.0;

		// timestemp data
		timestamp_packet.code = static_cast<uint8_t>(TxPacketCode::timestamp);
		timestamp_packet.timestamp = get_timestamp();

		// Battery data
		battery_packet.code = static_cast<uint8_t>(TxPacketCode::battery);
		battery_packet.battery =
		    pmBatteryChargeFromVoltage(pmGetBatteryVoltage());

		// Position data
		point_t p;
		estimatorKalmanGetEstimatedPos(&p);
		position_packet.code = static_cast<uint8_t>(TxPacketCode::position);
		position_packet.positionX = p.x;
		position_packet.positionY = p.y;
		position_packet.positionZ = p.z;

		// TODO (sambabal19)
		// auto pitch = logGetFloat(id_pitch);
		// auto roll = logGetFloat(id_roll);
		// auto thrust = logGetFloat(id_thrust);
		// auto yaw = logGetFloat(id_yaw);

		// Sensors data
		auto left = logGetUint(id_left);
		auto right = logGetUint(id_right);
		auto front = logGetUint(id_front);
		auto back = logGetUint(id_back);
		auto up = logGetUint(id_up);

		sensors_packet.code = static_cast<uint8_t>(TxPacketCode::sensors);
		sensors_packet.back = static_cast<uint16_t>(back);
		sensors_packet.front = static_cast<uint16_t>(front);
		sensors_packet.left = static_cast<uint16_t>(left);
		sensors_packet.right = static_cast<uint16_t>(right);
		sensors_packet.up = static_cast<uint16_t>(up);

		appchannelSendPacket(&sensors_packet, sizeof(sensors_packet));
		appchannelSendPacket(&position_packet, sizeof(position_packet));
		appchannelSendPacket(&battery_packet, sizeof(battery_packet));
		// appchannelSendPacket(&timestamp_packet, sizeof(timestamp_packet));
		appchannelSendPacket(&speed_packet, sizeof(speed_packet));
		appchannelSendPacket(&other_packet, sizeof(other_packet));
	}
}
