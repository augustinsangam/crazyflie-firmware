#ifndef WALLFOLLOWINGWITHAVOID_HPP
#define WALLFOLLOWINGWITHAVOID_HPP

#include <cstdint>

#include "WallFollowing.hpp"

namespace exploration {

class WallFollowingWithAvoid {
public:
	void init(float new_ref_distance_from_wall, float max_speed_ref,
	          float starting_local_direction);
	int controller(float *vel_x, float *vel_y, float *vel_w, float front_range,
	               float left_range, float right_range, float current_heading,
	               uint8_t rssi_other_drone);

private:
	float state_start_time_{};
	float ref_distance_from_wall_{0.5};
	float max_speed_{0.5};
	float local_direction_{1};
	bool first_run_{true};
	exploration::WallFollowing wf_;
};

} // namespace exploration

#endif /* WALLFOLLOWINGWITHAVOID_HPP */
