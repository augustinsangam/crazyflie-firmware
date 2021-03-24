#ifndef EXPLORATION_TYPES_HPP
#define EXPLORATION_TYPES_HPP

#include <cstdint>

namespace exploration {

/******************************************************************************/

#define P2P_MAX_DATA_SIZE 60

typedef struct _P2PPacket {
	uint8_t size; //< Size of data
	uint8_t rssi; //< Received Signal Strength Intensity
	union {
		struct {
			uint8_t port; //< Header selecting channel and port
			uint8_t data[P2P_MAX_DATA_SIZE]; //< Data
		};
		uint8_t raw[P2P_MAX_DATA_SIZE + 1]; //< The full packet "raw"
	};
} __attribute__((packed)) P2PPacket;

/******************************************************************************/

struct vec3_s {
	uint32_t timestamp; // Timestamp when the data was computed

	float x;
	float y;
	float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

typedef struct attitude_s {
	uint32_t timestamp; // Timestamp when the data was computed

	float roll;
	float pitch;
	float yaw;
} attitude_t;

typedef struct quaternion_s {
	uint32_t timestamp;

	union {
		struct {
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct {
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;

typedef enum mode_e { modeDisable = 0, modeAbs, modeVelocity } stab_mode_t;

typedef struct setpoint_s {
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
} setpoint_t;

/******************************************************************************/

} // namespace exploration

#endif /* EXPLORATION_TYPES_HPP */
