/*
 * wallfollowing_with_avoid.c
 *
 *  Created on: Nov 12, 2018
 *      Author: knmcguire
 */

#include "exploration/WallFollowingWithAvoid.hpp"
#include "porting.hpp"

static int transition(int new_state, float *state_start_time) {
	*state_start_time =
	    static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);
	return new_state;
}

// statemachine functions
void exploration::WallFollowingWithAvoid::init(float new_ref_distance_from_wall,
                                               float max_speed_ref,
                                               float starting_local_direction) {
	ref_distance_from_wall_ = new_ref_distance_from_wall;
	max_speed_ = max_speed_ref;
	local_direction_ = starting_local_direction;
	first_run_ = true;
}

int exploration::WallFollowingWithAvoid::controller(
    float *vel_x, float *vel_y, float *vel_w, float front_range,
    float left_range, float right_range, float current_heading,
    uint8_t rssi_other_drone) {

	// Initalize static variables
	static int state = 1;
	static int rssi_collision_threshold = 43;

	// if it is reinitialized
	if (first_run_) {
		state = 1;
		state_start_time_ =
		    static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);
		first_run_ = false;
	}

	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = wall_following
	// 3 = move_out_of_way

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) { // FORWARD
		// if front range is close, start wallfollowing
		if (front_range < ref_distance_from_wall_ + 0.2F) {
			wf_.init(ref_distance_from_wall_, max_speed_, 3);
			state = transition(2, &state_start_time_); // wall_following
		}
	} else if (state == 2) { // WALL_FOLLOWING
		if (rssi_other_drone < rssi_collision_threshold) {
			state = transition(3, &state_start_time_);
		}
	} else if (state == 3) { // MOVE_OUT_OF_WAY
		if (rssi_other_drone > rssi_collision_threshold) {
			state = transition(1, &state_start_time_);
		}
	}
	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x = 0;
	float temp_vel_y = 0;
	float temp_vel_w = 0;

	if (state == 1) { // FORWARD
		// forward max speed
		temp_vel_x = 0.5;

	} else if (state == 2) { // WALL_FOLLOWING
		// Get the values from the wallfollowing
		if (local_direction_ == 1) {
			wf_.controller(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range,
			               right_range, current_heading, local_direction_);
		} else if (local_direction_ == -1) {
			wf_.controller(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range,
			               left_range, current_heading, local_direction_);
		}
	} else if (state == 3) { // MOVE_OUT_OF_WAY

		float save_distance = 0.7F;
		if (left_range < save_distance) {
			temp_vel_y = temp_vel_y - 0.5F;
		}
		if (right_range < save_distance) {
			temp_vel_y = temp_vel_y + 0.5F;
		}
		if (front_range < save_distance) {
			temp_vel_x = temp_vel_x - 0.5F;
		}
	}

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

	return state;
}
