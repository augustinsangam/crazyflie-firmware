extern "C" {
#include "estimator_kalman.h"
#include "stabilizer_types.h"
#include "usec_time.h"
#include <FreeRTOS.h>
#include <app.h> /* appMain */
#include <app_channel.h>
#include <debug.h> /* DEBUG_PRINT */
#include <led.h>
#include <log.h>
#include <pm.h>
#include <task.h> /* vTaskDelay */
}

#include "battery.hpp"
#include "communication.hpp"
#include <array>
#include <cstdint>

static int64_t get_timestamp() { return usecTimestamp() / 1000000; }

static logVarId_t id_up = logGetVarId("range", "up");       // NOLINT
static logVarId_t id_left = logGetVarId("range", "left");   // NOLINT
static logVarId_t id_right = logGetVarId("range", "right"); // NOLINT
static logVarId_t id_front = logGetVarId("range", "front"); // NOLINT
static logVarId_t id_back = logGetVarId("range", "back");   // NOLINT

static logVarId_t id_pitch = logGetVarId("stabilizer", "pitch");   // NOLINT
static logVarId_t id_roll = logGetVarId("stabilizer", "roll");     // NOLINT
static logVarId_t id_thrust = logGetVarId("stabilizer", "thrust"); // NOLINT
static logVarId_t id_yaw = logGetVarId("stateEstimate", "yaw");    // NOLINT

void send_all_packets(const exploration::StateMachine &sm) {

	TimestempPacket timestamp_packet{};
	SpeedPacket speed_packet{};
	PositionPacket position_packet{};
	SensorsPacket sensors_packet{};
	BatteryPacket battery_packet{};
	OtherPacket other_packet{};

	// Others data
	other_packet.code = static_cast<uint8_t>(TxPacketCode::others);
	other_packet.state = static_cast<uint8_t>(sm.get_state());

	// speed data
	speed_packet.code = static_cast<uint8_t>(TxPacketCode::speed);
	speed_packet.speed = 0.0;

	// timestemp data
	timestamp_packet.code = static_cast<uint8_t>(TxPacketCode::timestamp);
	timestamp_packet.timestamp = get_timestamp();

	// Battery data
	battery_packet.code = static_cast<uint8_t>(TxPacketCode::battery);
	battery_packet.battery = getBatteryPercentage();

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
