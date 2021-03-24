#ifndef EXPLORATION_WALLFOLLOWING_HPP
#define EXPLORATION_WALLFOLLOWING_HPP

#include <cstdint>

namespace exploration {

class WallFollowing {
public:
	void init(float new_ref_distance_from_wall, float max_speed_ref,
	          int init_state);
	int controller(float *vel_x, float *vel_y, float *vel_w, float front_range,
	               float side_range, float current_heading,
	               float direction_turn);

private:
	void adjust_distance_wall(float distance_wall_new);

	static constexpr float max_rate_{0.5};

	float ref_distance_from_wall_{};
	float state_start_time_{};
	float direction_{1};
	float max_speed_{0.5};
	int state_{1};
	bool first_run_{};
};

} // namespace exploration

#endif /* EXPLORATION_WALLFOLLOWING_HPP */
