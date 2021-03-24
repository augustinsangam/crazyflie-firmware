#ifndef WALLFOLLOWINGWITHAVOID_HPP
#define WALLFOLLOWINGWITHAVOID_HPP

#include <cstdint>

#include "WallFollowing.hpp"

namespace exploration {

class WallFollowingWithAvoid {
	float state_start_time_{};
	float ref_distance_from_wall_{0.5};
	float max_speed_{0.5};
	float local_direction_{1};
	bool first_run_{true};
	exploration::WallFollowing wf_;

public:
	void wall_follower_init(float new_ref_distance_from_wall,
	                        float max_speed_ref,
	                        float starting_local_direction);
	int wall_follower(float *vel_x, float *vel_y, float *vel_w,
	                  float front_range, float left_range, float right_range,
	                  float current_heading, uint8_t rssi_other_drone);
};

} // namespace exploration

#endif /* WALLFOLLOWINGWITHAVOID_HPP */
