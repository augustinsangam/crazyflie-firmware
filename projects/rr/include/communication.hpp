#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <array>
#include <cstdint>
#include <exploration/StateMachine.hpp>

enum class TxPacketCode : uint8_t {
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

void send_all_packets(const exploration::StateMachine &sm);

#endif // !COMMUNICATION_HPP