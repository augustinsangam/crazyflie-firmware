#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

//#include "WallFollowing.hpp"
#include "median_filter.hpp"
#include "types.hpp"
#include <cstdint>

namespace exploration {

class StateMachine {
	static constexpr float nominal_height = 0.3F;

	uint8_t my_id;
	struct MedianFilterFloat medFilt, medFilt_2, medFilt_3;
	float rssi_angle;
	float front_range, right_range, left_range, back_range, up_range;
	uint8_t rssi_beacon;
	uint64_t takeoffdelaytime = 0;
	uint64_t time_array_other_drones[9] = {0};
	uint8_t rssi_array_other_drones[9] = {150, 150, 150, 150, 150,
	                                      150, 150, 150, 150};
	float rssi_angle_array_other_drones[9] = {500.0F};
	bool correctly_initialized = false;
	bool keep_flying = false, taken_off = false;
	int state = 0;

#if METHOD != 1
	uint64_t radioSendBroadcastTime = 0;
	P2PPacket p_reply;
#endif

#if METHOD == 1
	bool outbound = true;
#endif

#if METHOD == 1
	//WallFollowing exploration_controller_;
#endif

public:
	void init();
	void step();
	void p2p_callback_handler(P2PPacket *p);
};

} // namespace exploration

#endif /* STATE_MACHINE_HPP */
