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

void send_all_packets(const exploration::StateMachine &sm) {

	SpeedPacket speed_packet{};
	PositionAndSensorsPacket position_and_sensors_packet{};
	BatteryPacket battery_packet{};
	OtherPacket other_packet{};

	// Others data
	other_packet.code = static_cast<uint8_t>(TxPacketCode::others);
	other_packet.state = static_cast<uint8_t>(sm.get_state());

	// speed data
	speed_packet.code = static_cast<uint8_t>(TxPacketCode::speed);
	speed_packet.speed = 0.0;

	// Battery data
	battery_packet.code = static_cast<uint8_t>(TxPacketCode::battery);
	battery_packet.battery = getBatteryPercentage();

	// Position data
	point_t p;
	estimatorKalmanGetEstimatedPos(&p);
	position_and_sensors_packet.code =
	    static_cast<uint8_t>(TxPacketCode::position_and_sensors);
	position_and_sensors_packet.positionX = p.x;
	position_and_sensors_packet.positionY = p.y;
	position_and_sensors_packet.positionZ = p.z;
	// Orientation data
	logVarId_t id_up = logGetVarId("range", "up");       // NOLINT
	logVarId_t id_left = logGetVarId("range", "left");   // NOLINT
	logVarId_t id_right = logGetVarId("range", "right"); // NOLINT
	logVarId_t id_front = logGetVarId("range", "front"); // NOLINT
	logVarId_t id_back = logGetVarId("range", "back");   // NOLINT

	logVarId_t id_pitch = logGetVarId("stabilizer", "pitch");   // NOLINT
	logVarId_t id_roll = logGetVarId("stabilizer", "roll");     // NOLINT
	logVarId_t id_thrust = logGetVarId("stabilizer", "thrust"); // NOLINT
	logVarId_t id_yaw = logGetVarId("stateEstimate", "yaw");    // NOLINT
	position_and_sensors_packet.yaw = logGetFloat(id_yaw);
	// Sensors data
	position_and_sensors_packet.back =
	    static_cast<uint16_t>(logGetUint(id_back));
	position_and_sensors_packet.front =
	    static_cast<uint16_t>(logGetUint(id_front));
	position_and_sensors_packet.left =
	    static_cast<uint16_t>(logGetUint(id_left));
	position_and_sensors_packet.right =
	    static_cast<uint16_t>(logGetUint(id_right));
	position_and_sensors_packet.up = static_cast<uint16_t>(logGetUint(id_up));

	// TODO (sambabal19)
	// auto pitch = logGetFloat(id_pitch);
	// auto roll = logGetFloat(id_roll);
	// auto thrust = logGetFloat(id_thrust);
	// auto yaw = logGetFloat(id_yaw);

	appchannelSendPacket(&position_and_sensors_packet,
	                     sizeof(position_and_sensors_packet));
	appchannelSendPacket(&position_and_sensors_packet,
	                     sizeof(position_and_sensors_packet));
	appchannelSendPacket(&battery_packet, sizeof(battery_packet));
	appchannelSendPacket(&speed_packet, sizeof(speed_packet));
	appchannelSendPacket(&other_packet, sizeof(other_packet));
}
