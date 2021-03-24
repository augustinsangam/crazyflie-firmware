#include "exploration/SGBA.hpp"
#include "math_supp.hpp"
#include "porting.hpp"

static int transition(int new_state, float *state_start_time) {
	*state_start_time =
	    static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);

	return new_state;
}

// Static helper functions
static bool logicIsCloseTo(float real_value, float checked_value,
                           float margin) {
	return real_value > checked_value - margin &&
	       real_value < checked_value + margin;
}

// Command functions
static void commandTurn(float *vel_w, float max_rate) { *vel_w = max_rate; }

uint8_t maxValue(uint8_t myArray[], int size) {
	/* enforce the contract */
	// assert(myArray && size);
	int i;
	uint8_t maxValue = myArray[0];

	for (i = 1; i < size; ++i) {
		if (myArray[i] > maxValue) {
			maxValue = myArray[i];
		}
	}
	return maxValue;
}

static float fillHeadingArray(uint8_t *correct_heading_array,
                              float rssi_heading, int diff_rssi,
                              int max_meters) {

	// Heading array of action choices
	static float heading_array[8] = {-135.0F, -90.0F, -45.0F, 0.0F,
	                                 45.0F,   90.0F,  135.0F, 180.0F};
	float rssi_heading_deg = rad_to_deg(rssi_heading);

	for (int it = 0; it < 8; it++) {

		// Fill array based on heading and rssi heading
		if ((rssi_heading_deg >= heading_array[it] - 22.5F &&
		     rssi_heading_deg < heading_array[it] + 22.5F && it != 7) ||
		    (it == 7 && (rssi_heading_deg >= heading_array[it] - 22.5F ||
		                 rssi_heading_deg < -135.0F - 22.5F))) {
			uint8_t temp_value_forward = correct_heading_array[it];
			uint8_t temp_value_backward = correct_heading_array[(it + 4) % 8];

			// if gradient is good, increment the array corresponding to the
			// current heading and decrement the exact opposite
			if (diff_rssi > 0) {
				correct_heading_array[it] =
				    temp_value_forward + 1; //(uint8_t)abs(diff_rssi);
				if (temp_value_backward > 0) {
					correct_heading_array[(it + 4) % 8] =
					    temp_value_backward - 1; //(uint8_t)abs(diff_rssi);
				}
				// if gradient is bad, decrement the array corresponding to the
				// current heading and increment the exact opposite

			} else if (diff_rssi < 0) {
				if (temp_value_forward > 0) {
					correct_heading_array[it] =
					    temp_value_forward - 1; //(uint8_t)abs(diff_rssi);
				}
				correct_heading_array[(it + 4) % 8] =
				    temp_value_backward + 1; //(uint8_t)abs(diff_rssi);
			}
		}
	}

	// degrading function
	//    If one of the arrays goes over maximum amount of points (meters), then
	//    decrement all values
	if (maxValue(correct_heading_array, 8) > max_meters) {
		for (int it = 0; it < 8; it++) {
			if (correct_heading_array[it] > 0) {
				correct_heading_array[it] = correct_heading_array[it] - 1;
			}
		}
	}

	// Calculate heading where the beacon might be
	int count = 0;
	float y_part = 0, x_part = 0;

	for (int it = 0; it < 8; it++) {
		if (correct_heading_array[it] > 0) {
			auto angle = deg_to_rad(heading_array[it]);
			x_part +=
			    static_cast<float>(correct_heading_array[it]) * std::cos(angle);
			y_part +=
			    static_cast<float>(correct_heading_array[it]) * std::sin(angle);

			// sum += heading_array[it];
			count = count + correct_heading_array[it];
		}
	}
	float wanted_angle_return = 0;
	if (count != 0) {
		wanted_angle_return = std::atan2(y_part / static_cast<float>(count),
		                                 x_part / static_cast<float>(count));
	}

	return wanted_angle_return;
}

void exploration::SGBA::init(float new_ref_distance_from_wall,
                             float max_speed_ref, float begin_wanted_heading) {
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
	wanted_angle = begin_wanted_heading;
	first_run = true;
}

int exploration::SGBA::controller(float *vel_x, float *vel_y, float *vel_w,
                                  float *rssi_angle, int *state_wallfollowing,
                                  float front_range, float left_range,
                                  float right_range, float back_range,
                                  float current_heading, float current_pos_x,
                                  float current_pos_y, uint8_t rssi_beacon,
                                  uint8_t rssi_inter, float rssi_angle_inter,
                                  bool priority, bool outbound) {

	// Initalize static variables
	static int state = 2;
	// static float previous_heading = 0;
	static int state_wf = 0;
	static float wanted_angle_dir = 0;
	static float pos_x_hit = 0;
	static float pos_y_hit = 0;
	static float pos_x_sample = 0;
	static float pos_y_sample = 0;
	// static float pos_x_move = 0;
	// static float pos_y_move = 0;
	static bool overwrite_and_reverse_direction = false;
	static float direction = 1;
	static bool cannot_go_to_goal = false;
	static uint8_t prev_rssi = 150;
	static int diff_rssi = 0;
	static bool rssi_sample_reset = false;
	static float heading_rssi = 0;
	static uint8_t correct_heading_array[8] = {0};

	static bool first_time_inbound = true;
	static float wanted_angle_hit = 0;

	// if it is reinitialized
	if (first_run) {

		wanted_angle_dir = wrap_to_pi(
		    current_heading -
		    wanted_angle); // to determine the direction when turning to goal

		overwrite_and_reverse_direction = false;
		state = 2;

		state_start_time =
		    static_cast<float>(static_cast<double>(timestamp_us()) / 1e6);
		first_run = false;
	}

	if (first_time_inbound) {
		wrap_to_pi(wanted_angle - pi<float>);
		wanted_angle_dir = wrap_to_pi(current_heading - wanted_angle);
		state = transition(2, &state_start_time);
		first_time_inbound = false;
	}

	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = rotate_to_goal
	// 3 = wall_following
	// 4 = move out of way

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) { // FORWARD
		if (front_range < ref_distance_from_wall + 0.2F) {

			// if looping is detected, reverse direction (only on outbound)
			if (overwrite_and_reverse_direction) {
				direction = -1.0F * direction;
				overwrite_and_reverse_direction = false;
			} else {
				if (left_range < right_range && left_range < 2.0F) {
					direction = -1.0F;
				} else if (left_range > right_range && right_range < 2.0F) {
					direction = 1.0F;

				} else if (left_range > 2.0F && right_range > 2.0F) {
					direction = 1.0F;
				} else {
				}
			}

			pos_x_hit = current_pos_x;
			pos_y_hit = current_pos_y;
			wanted_angle_hit = wanted_angle;

			wf_.wall_follower_init(0.4F, 0.5, 3);

			for (int it = 0; it < 8; it++) {
				correct_heading_array[it] = 0;
			}

			state = transition(3, &state_start_time); // wall_following
		}
	} else if (state == 2) { // ROTATE_TO_GOAL
		// check if heading is close to the preferred_angle
		bool goal_check =
		    logicIsCloseTo(wrap_to_pi(current_heading - wanted_angle), 0, 0.1F);
		if (front_range < ref_distance_from_wall + 0.2F) {
			cannot_go_to_goal = true;
			wf_.wall_follower_init(0.4F, 0.5, 3);

			state = transition(3, &state_start_time); // wall_following
		}
		if (goal_check) {
			state = transition(1, &state_start_time); // forward
		}
	} else if (state == 3) { // WALL_FOLLOWING

		// if another drone is close and there is no right of way, move out of
		// the way
		if (priority == false && rssi_inter < rssi_threshold) {
			if (outbound) {
				if ((rssi_angle_inter < 0 && wanted_angle < 0) ||
				    (rssi_angle_inter > 0 && wanted_angle > 0)) {
					wanted_angle = -1 * wanted_angle;
					wanted_angle_dir =
					    wrap_to_pi(current_heading - wanted_angle);
					// state= transition(2);
				}
			}
			if (rssi_inter < rssi_collision_threshold) {
				state = transition(4, &state_start_time);
			}
		}

		// If going forward with wall following and cannot_go_to_goal bool is
		// still on
		//    turn it off!
		if (state_wf == 5 && cannot_go_to_goal) {
			cannot_go_to_goal = false;
		}

		// Check if the goal is reachable from the current point of view of the
		// agent
		float bearing_to_goal = wrap_to_pi(wanted_angle - current_heading);
		bool goal_check_WF = false;
		if (direction == -1) {
			goal_check_WF = (bearing_to_goal < 0 && bearing_to_goal > -1.5F);
		} else {
			goal_check_WF = (bearing_to_goal > 0 && bearing_to_goal < 1.5F);
		}

		// Check if bug went into a looping while wall following,
		//    if so, then forse the reverse direction predical.
		float rel_x_loop =
		    current_pos_x -
		    pos_x_hit; //  diff_rssi = (int)prev_rssi - (int)rssi_beacon;
		float rel_y_loop = current_pos_y - pos_y_hit;
		float loop_angle = wrap_to_pi(std::atan2(rel_y_loop, rel_x_loop));

		if (std::abs(wrap_to_pi(wanted_angle_hit + pi<float> - loop_angle)) <
		    1) {
			overwrite_and_reverse_direction = true;
		}

		// if during wallfollowing, agent goes around wall, and heading is close
		// to rssi _angle
		//      got to rotate to goal
		if ((state_wf == 6 || state_wf == 8) && goal_check_WF &&
		    front_range > ref_distance_from_wall + 0.4F && !cannot_go_to_goal) {
			wanted_angle_dir = wrap_to_pi(
			    current_heading - wanted_angle); // to determine the direction
			                                     // when turning to goal
			state = transition(2, &state_start_time); // rotate_to_goal
		}

		// If going straight
		//    determine by the gradient of the crazyradio what the approx
		//    direction is.
		if (state_wf == 5) {

			if (!outbound) {
				// Reset sample gathering
				if (rssi_sample_reset) {
					pos_x_sample = current_pos_x;
					pos_y_sample = current_pos_y;
					rssi_sample_reset = false;
					prev_rssi = rssi_beacon;
				}

				// if the crazyflie traveled for 1 meter, than measure if it
				// went into the right path
				float rel_x_sample = current_pos_x - pos_x_sample;
				float rel_y_sample = current_pos_y - pos_y_sample;
				auto distance = std::sqrt(rel_x_sample * rel_x_sample +
				                          rel_y_sample * rel_y_sample);
				if (distance > 1.0F) {
					rssi_sample_reset = true;
					heading_rssi = current_heading;
					int diff_rssi_unf = static_cast<int>(prev_rssi) -
					                    static_cast<int>(rssi_beacon);

					// rssi already gets filtered at the radio_link.c
					diff_rssi = diff_rssi_unf;

					// Estimate the angle to the beacon
					wanted_angle = fillHeadingArray(correct_heading_array,
					                                heading_rssi, diff_rssi, 5);
				}
			}

		} else {
			rssi_sample_reset = true;
		}
	} else if (state == 4) { // MOVE_OUT_OF_WAY
		// once the drone has gone by, rotate to goal
		if (rssi_inter >= rssi_collision_threshold) {

			state = transition(2, &state_start_time); // rotate_to_goal
		}
	}

	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x = 0;
	float temp_vel_y = 0;
	float temp_vel_w = 0;

	if (state == 1) { // FORWARD
		// stop moving if there is another drone in the way
		// forward max speed
		if (left_range < ref_distance_from_wall) {
			temp_vel_y = -0.2F;
		}
		if (right_range < ref_distance_from_wall) {
			temp_vel_y = 0.2F;
		}
		temp_vel_x = 0.5;
		//}

	} else if (state == 2) { // ROTATE_TO_GOAL
		// rotate to goal, determined on the sign
		if (wanted_angle_dir < 0) {
			commandTurn(&temp_vel_w, 0.5);
		} else {
			commandTurn(&temp_vel_w, -0.5);
		}

	} else if (state == 3) { // WALL_FOLLOWING
		// Get the values from the wallfollowing
		if (direction == -1) {
			state_wf = wf_.wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w,
			                             front_range, left_range,
			                             current_heading, direction);
		} else {
			state_wf = wf_.wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w,
			                             front_range, right_range,
			                             current_heading, direction);
		}
	} else if (state == 4) { // MOVE_AWAY

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
		if (back_range < save_distance) {
			temp_vel_x = temp_vel_x + 0.5F;
		}
	}

	*rssi_angle = wanted_angle;
	*state_wallfollowing = state_wf;

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

	return state;
}
