#ifndef WALLFOLLOWING_HPP
#define WALLFOLLOWING_HPP

#include <cstdint>

namespace exploration {

class WallFollowing {

public:
	void init(float new_ref_distance_from_wall, float max_speed_ref,
	          int init_state);

	int wall_follower(float *vel_x, float *vel_y, float *vel_w,
	                  float front_range, float side_range,
	                  float current_heading, float direction_turn);

private:
	void command_turn(float *vel_x, float *vel_w, float ref_rate) const;
	void command_align_corner(float *vel_y, float *vel_w, float ref_rate,
	                          float range,
	                          float wanted_distance_from_corner) const;
	void command_forward_along_wall(float *vel_x, float *vel_y,
	                                float range) const;
	void command_turn_around_corner_and_adjust(float *vel_x, float *vel_y,
	                                           float *vel_w, float radius,
	                                           float range) const;
	void command_turn_and_adjust(float *vel_y, float *vel_w, float rate,
	                             float range) const;
	int transition(int new_state);
	void adjust_distance_wall(float distance_wall_new);

	float ref_distance_from_wall = 0;
	float max_speed = 0.5;
	float max_rate = 0.5;
	float direction = 1;
	bool first_run = false;
	int state = 1;
	float state_start_time = 0;
};

} // namespace exploration

#endif /* WALLFOLLOWING_HPP */
