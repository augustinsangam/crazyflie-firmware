#ifndef EXPLORATION_WALLFOLLOWING_HPP
#define EXPLORATION_WALLFOLLOWING_HPP

#include <cstdint>

namespace exploration {

class WallFollowing {
	static constexpr float max_rate_{0.5};

	float ref_distance_from_wall_{};
	float state_start_time_{};
	float direction_{1};
	float max_speed_{0.5};
	int state_{1};
	bool first_run_{};

public:
	int wall_follower(float *vel_x, float *vel_y, float *vel_w,
	                  float front_range, float side_range,
	                  float current_heading, float direction_turn);
	void adjustDistanceWall(float distance_wall_new);
	void wall_follower_init(float new_ref_distance_from_wall,
	                        float max_speed_ref, int init_state);
};

} // namespace exploration

#endif /* EXPLORATION_WALLFOLLOWING_HPP */
