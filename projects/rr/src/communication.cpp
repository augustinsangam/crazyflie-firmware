#include <cmath>
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

void Communication::send_all_packets() {

	SpeedPacket speed_packet{};
	PositionAndSensorsPacket position_and_sensors_packet{};
	BatteryPacket battery_packet{};
	OtherPacket other_packet{};

	// Others data
	other_packet.code = static_cast<uint8_t>(TxPacketCode::others);
	other_packet.state = static_cast<uint8_t>(sm_->get_state());
	other_packet.ledOn = led_is_on_;

	// speed data
	speed_packet.code = static_cast<uint8_t>(TxPacketCode::speed);
	speed_packet.speed = speed_;

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

void Communication::on_receive_packet(const PacketRX &packet) {

	switch (packet.code) {
	case RxPacketCode::start_mission: {
		DEBUG_PRINT("Start mission received\n"); // NOLINT
		sm_->start_mission();
		break;
	}

	case RxPacketCode::end_mission: {
		DEBUG_PRINT("End mission received\n"); // NOLINT
		sm_->end_mission();
		break;
	}

	case RxPacketCode::return_to_base: {
		DEBUG_PRINT("Return to base received\n"); // NOLINT
		sm_->return_to_base();
		break;
	}

	case RxPacketCode::take_off: {
		DEBUG_PRINT("Take off received\n"); // NOLINT
		sm_->take_off_robot();
		break;
	}

	case RxPacketCode::land: {
		DEBUG_PRINT("Land received\n"); // NOLINT
		sm_->land_robot();
		break;
	}

	case RxPacketCode::set_led: {
		DEBUG_PRINT("Set Led received\n"); // NOLINT
		ledSetAll();
		led_is_on_ = true;
		break;
	}

	case RxPacketCode::clear_led: {
		DEBUG_PRINT("Clear led received\n"); // NOLINT
		ledClearAll();
		led_is_on_ = false;
		break;
	}

	default:
		break;
	}
}

void Communication::update_speed() {
	unsigned int motor1 = logGetUint(logGetVarId("motor", "m1")); // NOLINT
	unsigned int motor2 = logGetUint(logGetVarId("motor", "m2")); // NOLINT
	unsigned int motor3 = logGetUint(logGetVarId("motor", "m3")); // NOLINT
	unsigned int motor4 = logGetUint(logGetVarId("motor", "m4")); // NOLINT

	point_t pos;
	estimatorKalmanGetEstimatedPos(&pos);

	if ((motor1 + motor2 + motor3 + motor4) == 0) {
		speed_ = 0.0F;
		current_pos_ = pos;
		return;
	}

	float dx = pos.x - current_pos_.x;
	float dy = pos.y - current_pos_.y;
	speed_ = std::sqrt(dx * dx + dy * dy) / DELAY_PER_SPEED_UPDATE;
	current_pos_ = pos;
}
