#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include "stabilizer_types.h"
#include <array>
#include <cstdint>
#include <exploration/StateMachine.hpp>

enum class TxPacketCode : uint8_t {
	battery = 0,
	speed,
	position_and_sensors,
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

struct PacketRX {
	uint8_t code;
} __attribute__((packed));

struct SpeedPacket {
	uint8_t code;
	float speed;
} __attribute__((packed));

struct PositionAndSensorsPacket {
	uint8_t code;
	float positionX, positionY, positionZ;
	float yaw;
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

class Communication {

public:
	Communication(exploration::StateMachine *sm, bool led_is_on)
	    : sm_{sm}, led_is_on_{led_is_on} {}

	void send_all_packets();
	void on_receive_packet(const PacketRX &packet);
	void update_speed();

private:
	exploration::StateMachine *sm_;
	bool led_is_on_;
	point_t current_pos_;
	float speed_;
	static constexpr float DELAY_PER_SPEED_UPDATE = 0.01F;
};

#endif // !COMMUNICATION_HPP
