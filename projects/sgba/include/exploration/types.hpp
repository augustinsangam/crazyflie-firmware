#ifndef EXPLORATION_TYPES_HPP
#define EXPLORATION_TYPES_HPP

#include <cstddef>
#include <cstdint>

namespace exploration {

/******************************** radiolink.h *********************************/

static constexpr std::size_t p2p_max_data_size{60};

using P2PPacket = struct P2PPacket {
	uint8_t size; //< Size of data
	uint8_t rssi; //< Received Signal Strength Intensity
	union {
		struct {          // NOLINT
			uint8_t port; //< Header selecting channel and port
			uint8_t data[p2p_max_data_size]; // NOLINT
		};
		uint8_t raw[p2p_max_data_size + 1]; // NOLINT
	};
} __attribute__((packed));

/***************************** stabilizer_types.h *****************************/

struct vec3_s {
	uint32_t timestamp; // Timestamp when the data was computed

	float x;
	float y;
	float z;
};

using vector_t = struct vec3_s;
using point_t = struct vec3_s;
using velocity_t = struct vec3_s;
using acc_t = struct vec3_s;

using attitude_t = struct attitude_s {
	uint32_t timestamp; // Timestamp when the data was computed

	float roll;
	float pitch;
	float yaw;
};

using quaternion_t = struct quaternion_s {
	uint32_t timestamp;

	union {
		struct { // NOLINT
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct { // NOLINT
			float x;
			float y;
			float z;
			float w;
		};
	};
};

using stab_mode_t = enum mode_e { modeDisable = 0, modeAbs, modeVelocity };

using setpoint_t = struct setpoint_s {
	uint32_t timestamp;

	attitude_t attitude;     // deg
	attitude_t attitudeRate; // deg/s
	quaternion_t attitudeQuaternion;
	float thrust;
	point_t position;    // m
	velocity_t velocity; // m/s
	acc_t acceleration;  // m/s^2
	bool velocity_body;  // true if velocity is given in body frame; false if
	                     // velocity is given in world frame

	struct {
		stab_mode_t x;
		stab_mode_t y;
		stab_mode_t z;
		stab_mode_t roll;
		stab_mode_t pitch;
		stab_mode_t yaw;
		stab_mode_t quat;
	} mode;
};

/******************************************************************************/

} // namespace exploration

#endif /* EXPLORATION_TYPES_HPP */
